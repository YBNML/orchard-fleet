"""
명령 감사 로그 — 누가 언제 무슨 명령을 내렸는지 파일에 남긴다.

사람을 다치게 할 수 있는 기계다. 사고가 나면 "그 순간에 무엇이 있었나" 를
재구성할 수 있어야 하는데, 지금 이벤트는 메모리 버퍼 50개뿐이라 재시작하면
사라진다. 정작 알아야 할 상황(비상정지·전복·통신두절)은 대개 재시작을 동반한다.

왜 이렇게 설계했는가
    JSON Lines      한 줄이 한 건이고 끝에 덧붙이기만 한다. 쓰는 도중에 프로세스가
                    죽어도 앞의 기록은 온전하다 (JSON 배열이면 닫는 괄호를 못 써서
                    파일 전체가 못 읽는 것이 된다). grep 한 줄로도 읽힌다.
                    ensure_ascii=False 로 한국어를 그대로 두되, 줄바꿈은 JSON 이
                    이스케이프하므로 **한 레코드가 두 줄로 쪼개지지 않는다**.

    비동기 기록      제어 루프는 20 Hz 로 돈다. 디스크가 잠깐 멎으면(회전·SD 카드
                    쓰기 지연·NFS) 기록하는 그 자리에서 블록되고, 그만큼 비상정지
                    처리가 늦어진다. 그래서 기록 요청은 큐에 넣고 즉시 돌아오며,
                    파일을 만지는 것은 백그라운드 스레드 하나뿐이다.
                    → log() 는 어떤 경우에도 블록하지 않는다.

    가득 차면 오래된 것부터 버린다
                    최신 기록(사고 직전)이 오래된 기록보다 중요하다. 다만 버린
                    사실 자체를 카운터로 세고 **파일에도 한 줄 남긴다** —
                    감사 로그에 생긴 구멍은 반드시 보여야 한다. 조용히 사라진
                    기록은 없는 것보다 나쁘다(있었던 것처럼 읽히므로).

    토큰은 절대 남기지 않는다
                    감사 로그는 대개 권한이 넓게 열리고 오래 보관된다. 로그가
                    자격증명 유출 경로가 되면 안 된다. 키 이름으로 마스킹하고,
                    문자열 안에 섞인 `?token=...` / `Bearer ...` 도 지운다.
                    (대시보드가 토큰을 질의문자열로 보내는 구조라 실제로 섞여
                    들어올 수 있다 — link/wsserver.py 참조)

    시각을 둘 적는다  벽시계(ISO8601 + 오프셋)는 사람이 읽고 다른 기록과 맞추는
                    용도다. 하지만 로봇 PC 는 부팅 직후 시계가 틀리고 NTP 보정으로
                    튀기도 한다. 그래서 단조시각을 같이 남긴다 — 벽시계가 뒤로
                    가도 **순서와 간격**은 단조시각으로 복원할 수 있다.

    회전을 직접 짠 이유
                    logging.handlers.RotatingFileHandler 로도 되지만, 포맷과 회전
                    시점이 핸들러 규칙에 묶인다. 여기서는 (1) 레코드 중간에서
                    절대 자르지 않고 줄 단위로만 회전하고, (2) 회전이 실패해도
                    던지지 않고 계속 쓰는 동작이 필요하다.

    못 열면 조용히 꺼진다
                    권한이 없거나 디스크가 없으면 예외를 던지지 않고 비활성으로
                    동작한다. 관제가 죽는 것이 로그를 못 남기는 것보다 훨씬 나쁘다
                    — 비상정지를 못 누르게 되기 때문이다 (registry 와 같은 원칙).
                    비활성 사유는 stats() 로 드러나므로 조용히 묻히지는 않는다.

쓰는 법 (호출부는 이 파일 밖에서 연결한다)
    audit = AuditLog("/var/log/orchard/audit.jsonl")
    audit.command("estop", {"reason": "관제 지시"}, role="operator",
                  addr=conn.addr, result=R_ACCEPT)
    audit.auth(False, addr=("10.0.0.9", 51234), why="토큰 불일치")
    audit.close()                      # 종료 시 (atexit 로도 한 번 더 걸린다)
"""
from __future__ import annotations

import atexit
import json
import math
import os
import queue
import re
import threading
import time
from datetime import datetime

# ── 기본값 ──────────────────────────────────────────────────────────────────
# 2 MB × (현재 + 보관 5) = 최대 12 MB. 임베디드 저장장치를 채우지 않을 정도로
# 작게, 사고 조사에 며칠치가 남을 정도로는 크게 잡았다.
DEFAULT_MAX_BYTES = 2_000_000
DEFAULT_KEEP = 5
DEFAULT_QUEUE = 2000            # 20 Hz 기준 100초치 — 디스크가 멎어도 버틴다

# 결과 어휘. 문자열을 여기저기서 직접 쓰면 표기가 갈려 나중에 못 센다.
R_ACCEPT = "accepted"           # 받아서 수행했다
R_REJECT = "rejected"           # 형식·권한·상태 때문에 거부했다
R_BLOCKED = "blocked"           # 안전장치(비상정지·데드맨 등)에 막혔다

KIND_COMMAND = "command"
KIND_EVENT = "event"
KIND_AUTH = "auth"
KIND_AUDIT = "audit"            # 감사 로그 자신에 대한 기록 (유실 표시 등)

# ── 민감값 제거 ─────────────────────────────────────────────────────────────
# 값이 아니라 **키 이름**으로 지운다. 페이로드 구조를 미리 알 필요가 없고,
# 새 명령이 생겨도 규칙이 따라간다.
_SECRET_HINTS = ("token", "secret", "password", "passwd", "pwd", "apikey",
                 "api_key", "auth", "credential", "cookie", "session",
                 "bearer", "signature", "privkey", "passphrase")

# 문자열 값에 섞여 들어오는 자격증명 (URL 질의문자열 / Authorization 헤더).
_RE_QTOK = re.compile(r"(?i)\b(token|access_token|auth|key)=[^&\s\"']+")
_RE_BEARER = re.compile(r"(?i)\bbearer\s+\S+")

MASK = "***"
MAX_STR = 120                   # 문자열 값 하나의 최대 길이
MAX_ITEMS = 8                   # 리스트에서 남길 앞쪽 개수
MAX_KEYS = 20                   # 딕셔너리에서 남길 키 개수
MAX_DEPTH = 3                   # 중첩 깊이
MAX_LINE = 2000                 # 한 줄(레코드)의 최대 길이


def _is_secret(name: str) -> bool:
    """키 이름이 자격증명으로 보이는가."""
    n = str(name).lower()
    if n == "key" or n.endswith("_key") or n.startswith("key_"):
        return True
    return any(h in n for h in _SECRET_HINTS)


def _scrub(s: str, limit: int = MAX_STR) -> str:
    """문자열에서 자격증명을 지우고 길이를 자른다."""
    s = str(s)
    if "=" in s:
        s = _RE_QTOK.sub(lambda m: f"{m.group(1)}={MASK}", s)
    if "earer" in s or "EARER" in s:
        s = _RE_BEARER.sub(f"Bearer {MASK}", s)
    if len(s) > limit:
        s = s[:limit] + "…"
    return s


def summarize(obj, depth: int = 0):
    """페이로드를 로그에 남길 형태로 줄인다 — 민감값 제거 + 크기 제한.

    JSON 으로 직렬화되지 않는 값(numpy 스칼라 등)이 섞여도 여기서 문자열로
    바뀌므로 기록이 통째로 날아가지 않는다. 2026-07-26 에 numpy int32 하나로
    노드가 종료된 적이 있어(link/wsserver.py 참조) 같은 실수를 반복하지 않는다.
    """
    if obj is None or isinstance(obj, bool):
        return obj
    if isinstance(obj, int):
        return obj
    if isinstance(obj, float):
        # NaN/Inf 는 표준 JSON 이 아니다 — 읽는 쪽이 깨진다
        return obj if math.isfinite(obj) else str(obj)
    if isinstance(obj, str):
        return _scrub(obj)
    if depth >= MAX_DEPTH:
        return _scrub(repr(obj), 60)
    if isinstance(obj, dict):
        out = {}
        for i, (k, v) in enumerate(obj.items()):
            if i >= MAX_KEYS:
                out["…"] = f"+{len(obj) - MAX_KEYS}개"
                break
            key = _scrub(k, 40)
            out[key] = MASK if _is_secret(k) else summarize(v, depth + 1)
        return out
    if isinstance(obj, (list, tuple, set)):
        seq = list(obj)
        out = [summarize(v, depth + 1) for v in seq[:MAX_ITEMS]]
        if len(seq) > MAX_ITEMS:
            out.append(f"…+{len(seq) - MAX_ITEMS}개")
        return out
    return _scrub(repr(obj), 60)


def _addr_str(a) -> str:
    """('10.0.0.9', 51234) → '10.0.0.9:51234'. Conn.addr 을 그대로 받기 위한 것."""
    if a is None:
        return ""
    if isinstance(a, (tuple, list)) and len(a) >= 2:
        return f"{a[0]}:{a[1]}"
    return _scrub(a, 64)


def _tail_lines(path: str, k: int, cap: int = 4 << 20):
    """파일 끝에서 최대 k줄을 읽는다. 파일 전체를 메모리에 올리지 않는다."""
    if k <= 0:
        return []
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            end = pos = f.tell()
            buf = b""
            while pos > 0 and buf.count(b"\n") <= k and (end - pos) < cap:
                step = min(65536, pos)
                pos -= step
                f.seek(pos)
                buf = f.read(step) + buf
    except OSError:
        return []
    # 맨 앞 줄은 잘려 있을 수 있다 — 파싱에 실패하면 호출부가 버린다
    lines = [ln for ln in buf.split(b"\n") if ln.strip()]
    return [ln.decode("utf-8", "replace") for ln in lines[-k:]]


# ═══════════════════════════════════════════════════════════════════════════
# 감사 로그
# ═══════════════════════════════════════════════════════════════════════════
class AuditLog:
    """명령·이벤트·인증을 JSON Lines 로 남기는 비동기 감사 로그.

    한 레코드의 모양:
        {"ts":"2026-07-30T22:13:05.123+09:00","mono":1234.567,"seq":42,
         "kind":"command","role":"operator","addr":"10.0.0.9:51234",
         "cmd":"estop","result":"accepted","why":"","arg":{"reason":"관제 지시"}}

    스레드 안전하다. 여러 워커 스레드(WebSocket)와 제어 루프가 동시에 불러도
    되고, 파일을 만지는 것은 내부 기록 스레드 하나뿐이다.
    """

    def __init__(self, path, max_bytes=DEFAULT_MAX_BYTES, keep=DEFAULT_KEEP,
                 queue_size=DEFAULT_QUEUE, fsync=False, atexit_close=True):
        self.path = os.path.abspath(str(path))
        self.max_bytes = max(4096, int(max_bytes))
        self.keep = max(0, int(keep))
        # fsync=True 면 디스크까지 밀어 넣는다. 기본값이 False 인 이유: 전원이
        # 끊기는 순간의 마지막 몇 건보다 쓰기 지연이 실제 위험에 가깝다.
        # 정전 조사가 목적이라면 켤 것 (그만큼 느려진다).
        self.fsync = bool(fsync)

        self.enabled = False
        self.disabled_reason = ""
        self.written = 0            # 파일에 실제로 쓴 줄 수
        self.dropped = 0            # 큐 포화로 버린 건수 (누적)
        self.rotations = 0
        self.errors = 0

        self._q = queue.Queue(maxsize=max(16, int(queue_size)))
        self._cv = threading.Condition()    # _pending / dropped / _gap 보호
        self._pending = 0
        self._gap = 0               # 아직 파일에 알리지 않은 유실 건수
        self._seq = 0
        self._seq_lock = threading.Lock()
        self._stop = threading.Event()
        self._f = None
        self._size = 0
        self._closed = False

        self._open()
        if self.enabled:
            self._thr = threading.Thread(target=self._run, name="audit",
                                         daemon=True)
            self._thr.start()
        else:
            self._thr = None        # 비활성 — 스레드도 만들지 않는다
        if atexit_close:
            # SIGINT 로 내려갈 때 destroy_node 를 못 타는 경로가 있어 한 겹 더 건다.
            # close() 는 여러 번 불러도 안전하다.
            atexit.register(self.close)

    # ── 파일 ────────────────────────────────────────────────────────────────
    def _open(self) -> bool:
        """이어쓰기로 연다. 실패하면 예외 대신 비활성 상태가 된다."""
        try:
            d = os.path.dirname(self.path)
            if d:
                os.makedirs(d, exist_ok=True)
            self._size = os.path.getsize(self.path) \
                if os.path.exists(self.path) else 0
            self._f = open(self.path, "ab")     # 바이트로 써야 크기가 정확하다
            self.enabled = True
            self.disabled_reason = ""
            return True
        except Exception as e:
            self._f = None
            self.enabled = False
            self.disabled_reason = f"{type(e).__name__}: {e}"
            return False

    def _disable(self, reason: str):
        self.enabled = False
        self.disabled_reason = reason
        try:
            if self._f is not None:
                self._f.close()
        except Exception:
            pass
        self._f = None

    def _rotate(self) -> bool:
        """path → path.1 → … → path.<keep>. 넘치는 것은 지운다.

        줄을 쓰기 **전에** 판단하므로 레코드가 두 파일에 걸쳐 쪼개지지 않는다.
        """
        try:
            self._f.flush()
            self._f.close()
        except Exception:
            pass
        self._f = None
        try:
            if self.keep <= 0:
                os.remove(self.path)
            else:
                oldest = f"{self.path}.{self.keep}"
                if os.path.exists(oldest):
                    os.remove(oldest)
                for i in range(self.keep - 1, 0, -1):
                    src = f"{self.path}.{i}"
                    if os.path.exists(src):
                        os.replace(src, f"{self.path}.{i + 1}")
                os.replace(self.path, f"{self.path}.1")
            self.rotations += 1
        except Exception:
            # 회전 실패는 치명적이지 않다 — 새로 열어 계속 쓴다 (파일이 커질 뿐)
            self.errors += 1
        return self._open()

    # ── 레코드 만들기 ───────────────────────────────────────────────────────
    def _next_seq(self) -> int:
        with self._seq_lock:
            self._seq += 1
            return self._seq

    def _make(self, kind, cmd="", payload=None, role="", addr="",
              result="", why=""):
        return {
            # 벽시계는 오프셋을 붙여 남긴다 — 로봇과 관제의 시간대가 다를 수 있다
            "ts": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "mono": round(time.monotonic(), 3),
            "seq": self._next_seq(),
            "kind": _scrub(kind, 32),
            "role": _scrub(role, 32),
            "addr": _addr_str(addr),
            "cmd": _scrub(cmd, 64),
            "result": _scrub(result, 32),
            "why": _scrub(why, 200),
            "arg": summarize(payload),      # 큰 값이라 맨 뒤에 둔다
        }

    def _encode(self, rec) -> bytes:
        """레코드 → 한 줄 바이트. 어떤 값도 기록을 통째로 날리지 못한다."""
        try:
            s = json.dumps(rec, ensure_ascii=False, separators=(",", ":"),
                           allow_nan=False)
        except Exception as e:
            rec = dict(rec)
            rec["arg"] = {"_error": f"직렬화 실패: {type(e).__name__}"}
            s = json.dumps(rec, ensure_ascii=False, separators=(",", ":"),
                           default=repr)
        if len(s) > MAX_LINE:
            # 잘라내면 JSON 이 깨진다 — 페이로드를 통째로 표식으로 바꾼다
            rec = dict(rec)
            rec["arg"] = {"_trunc": f"{len(s)}자 — 생략"}
            s = json.dumps(rec, ensure_ascii=False, separators=(",", ":"),
                           default=repr)
        return s.encode("utf-8") + b"\n"

    # ── 기록 스레드 ─────────────────────────────────────────────────────────
    def _run(self):
        while True:
            try:
                rec = self._q.get(timeout=0.2)
            except queue.Empty:
                if self._stop.is_set():
                    break
                continue
            batch = [rec]
            while len(batch) < 256:             # 몰아서 써서 flush 횟수를 줄인다
                try:
                    batch.append(self._q.get_nowait())
                except queue.Empty:
                    break
            self._write_batch(batch)
            with self._cv:
                self._pending -= len(batch)
                self._cv.notify_all()

    def _write_batch(self, batch):
        if not self.enabled or self._f is None:
            return
        lines = []
        gap = self._take_gap()
        if gap:
            # 구멍을 파일 안에서도 보이게 한다. 이 줄이 없으면 나중에 읽는 사람은
            # 그냥 그 시간에 아무 일도 없었다고 읽는다.
            lines.append(self._encode(self._make(
                KIND_AUDIT, cmd="queue_overflow", result="dropped",
                why=f"큐 포화로 {gap}건 유실")))
        lines.extend(self._encode(r) for r in batch)
        try:
            for ln in lines:
                if self._size > 0 and self._size + len(ln) > self.max_bytes:
                    if not self._rotate():
                        return          # 새 파일을 못 열었다 — 비활성
                self._f.write(ln)
                self._size += len(ln)
                self.written += 1
            self._f.flush()
            if self.fsync:
                os.fsync(self._f.fileno())
        except Exception as e:
            self._disable(f"쓰기 실패 — {type(e).__name__}: {e}")
            self.errors += 1

    def _take_gap(self) -> int:
        with self._cv:
            g, self._gap = self._gap, 0
            return g

    def _enqueue(self, rec) -> bool:
        """큐에 넣는다. 가득 차면 **가장 오래된 것**을 버린다. 절대 블록하지 않는다."""
        with self._cv:
            self._pending += 1
        try:
            self._q.put_nowait(rec)
            return True
        except queue.Full:
            pass
        try:
            self._q.get_nowait()                # 가장 오래된 한 건을 버린다
            with self._cv:
                self._pending -= 1
                self.dropped += 1
                self._gap += 1
                self._cv.notify_all()
        except queue.Empty:
            pass
        try:
            self._q.put_nowait(rec)
            return True
        except queue.Full:
            with self._cv:                      # 자리를 못 만들었다 — 새 것을 버린다
                self._pending -= 1
                self.dropped += 1
                self._gap += 1
                self._cv.notify_all()
            return False

    # ── 공개 API ────────────────────────────────────────────────────────────
    def log(self, kind, cmd="", payload=None, *, role="", addr="",
            result="", why="") -> bool:
        """한 건 기록. 제어 루프에서 불러도 되게 **절대 블록하지 않는다**.

        돌려주는 값은 '큐에 들어갔는가' 이지 '파일에 쓰였는가' 가 아니다.
        호출부는 이 값으로 분기하지 말 것 — 로그 때문에 동작이 갈리면 안 된다.
        """
        if not self.enabled or self._thr is None:
            return False
        try:
            rec = self._make(kind, cmd, payload, role, addr, result, why)
        except Exception:
            return False        # 요약 중 어떤 예외도 호출부로 새 나가지 않게
        return self._enqueue(rec)

    def command(self, cmd, payload=None, *, role="", addr="",
                result=R_ACCEPT, why="") -> bool:
        """관제가 내린 명령. 거부·차단도 반드시 남긴다 (거부가 더 중요하다)."""
        return self.log(KIND_COMMAND, cmd, payload, role=role, addr=addr,
                        result=result, why=why)

    def event(self, kind, msg="", level="info", *, role="", addr="") -> bool:
        """로봇이 낸 사건 (비상정지·링크두절·기능 오류 등)."""
        return self.log(KIND_EVENT, kind, None, role=role, addr=addr,
                        result=level, why=msg)

    def auth(self, ok: bool, cmd="connect", *, role="", addr="",
             why="") -> bool:
        """접속 인증 결과. **토큰 값은 넘기지 말 것** (넘겨도 마스킹되지만)."""
        return self.log(KIND_AUTH, cmd, None, role=role, addr=addr,
                        result=R_ACCEPT if ok else R_REJECT, why=why)

    def flush(self, timeout: float = 1.0) -> bool:
        """큐가 빌 때까지 기다린다. **제어 루프에서는 부르지 말 것** — 조회·종료용."""
        if self._thr is None:
            return True
        with self._cv:
            return self._cv.wait_for(lambda: self._pending <= 0,
                                     timeout=timeout)

    def tail(self, n: int = 50):
        """최근 n건을 오래된 순서로 돌려준다 (회전된 파일까지 거슬러 읽는다)."""
        self.flush(0.5)             # 방금 넣은 것이 안 보이면 조회가 쓸모없다
        out = []
        files = [self.path] + [f"{self.path}.{i}"
                               for i in range(1, self.keep + 1)]
        for p in files:             # 최신 파일부터 뒤로 거슬러 간다
            need = n - len(out)
            if need <= 0:
                break
            recs = []
            for raw in _tail_lines(p, need):
                try:
                    o = json.loads(raw)
                except Exception:
                    continue        # 잘린 첫 줄·손상된 줄은 버린다
                if isinstance(o, dict):
                    recs.append(o)
            out = recs[-need:] + out
        return out

    def stats(self) -> dict:
        """텔레메트리에 실어 보내기 좋은 요약. 비활성 사유가 여기 드러난다."""
        return dict(enabled=self.enabled, path=self.path, written=self.written,
                    dropped=self.dropped, rotations=self.rotations,
                    errors=self.errors, queued=self._q.qsize(),
                    reason=self.disabled_reason)

    def close(self, timeout: float = 2.0):
        """남은 기록을 파일에 밀어 넣고 닫는다. 여러 번 불러도 안전하다."""
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        t = self._thr
        if t is not None and t.is_alive():
            t.join(timeout=timeout)
        try:
            if self._f is not None:
                self._f.flush()
                if self.fsync:
                    os.fsync(self._f.fileno())
                self._f.close()
        except Exception:
            pass
        self._f = None
        self.enabled = False        # 이후 log() 는 조용한 무동작


# ═══════════════════════════════════════════════════════════════════════════
# 자체 시험 —  python3 ros2_ws/src/orchard_sim/orchard_sim/control/audit.py
# ═══════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    import shutil
    import sys
    import tempfile

    OK, NG = "\033[92m✔\033[0m", "\033[91m✗\033[0m"
    res = []

    def check(name, cond, detail=""):
        res.append(bool(cond))
        print(f"   {OK if cond else NG} {name}"
              + (f"  — {detail}" if detail else ""))

    tmp = tempfile.mkdtemp(prefix="audit_")
    SECRET = "SUPERSECRET_TOKEN_abcdef0123456789"

    # ── 1. 기본 기록 · 필드 · 순서 ──────────────────────────────────────────
    print("\n── 1. 기본 기록 ──")
    lg = AuditLog(os.path.join(tmp, "a.jsonl"), atexit_close=False)
    lg.command("estop", {"reason": "관제 지시"}, role="operator",
               addr=("10.0.0.9", 51234))
    lg.command("mission_start", {"alleys": [0, 1, 2], "mode": "mapping"},
               role="operator", addr=("10.0.0.9", 51234))
    lg.command("set_mode", {"mode": "teleop"}, role="viewer",
               addr="10.0.0.9:51234", result=R_REJECT, why="권한 없음")
    lg.event("estop", "비상정지 — 전복 감지 (기울기 41°)", "critical")
    lg.auth(False, addr=("10.0.0.4", 40001), why="토큰 불일치")
    got = lg.tail(10)
    check("5건 기록", len(got) == 5, f"{len(got)}건")
    check("순서 유지 (오래된 것부터)",
          [r["cmd"] for r in got][:2] == ["estop", "mission_start"])
    r0 = got[0]
    check("시각 두 종류", r0["ts"].startswith("20") and r0["mono"] > 0,
          f"{r0['ts']} / mono={r0['mono']}")
    check("역할·주소·결과", r0["role"] == "operator"
          and r0["addr"] == "10.0.0.9:51234" and r0["result"] == R_ACCEPT)
    check("거부 사유 보존",
          got[2]["result"] == R_REJECT and got[2]["why"] == "권한 없음")
    check("seq 증가", [r["seq"] for r in got] == sorted(r["seq"] for r in got))
    check("kind 구분", {r["kind"] for r in got}
          == {KIND_COMMAND, KIND_EVENT, KIND_AUTH})

    # ── 2. 민감값 제거 ─────────────────────────────────────────────────────
    print("\n── 2. 토큰은 남지 않는다 ──")
    lg.command("login", {"user": "kim", "token": SECRET,
                         "nested": {"auth_token": SECRET}},
               role="admin", addr=("10.0.0.9", 1))
    lg.auth(True, addr=("10.0.0.9", 1), why=f"GET /ws?token={SECRET} 통과")
    lg.event("hdr", f"Authorization: Bearer {SECRET}")
    lg.flush(2.0)
    with open(os.path.join(tmp, "a.jsonl"), encoding="utf-8") as f:
        blob = f.read()
    check("파일 어디에도 토큰 없음", SECRET not in blob)
    check("질의문자열 마스킹", "token=***" in blob)
    check("Bearer 마스킹", "Bearer ***" in blob)
    check("비밀 아닌 값은 남음", '"user":"kim"' in blob)
    check("중첩 키도 마스킹", blob.count(MASK) >= 4)

    # ── 3. 직렬화 안 되는 값 · 큰 페이로드 ─────────────────────────────────
    print("\n── 3. 이상한 페이로드에도 살아남는다 ──")
    lg.command("odd", {"obj": object(), "nan": float("nan"),
                       "big": list(range(1000)), "deep": {"a": {"b": {"c": 1}}}})
    lg.command("blob", {"blob": "가" * 50000})
    # 요약을 통과하고도 한 줄 상한을 넘는 경우 (키가 많은 페이로드)
    lg.command("huge", {f"k{i}": "가" * MAX_STR for i in range(20)})
    lg.flush(2.0)
    last = lg.tail(3)
    check("기록이 살아남음", len(last) == 3)
    check("긴 문자열은 잘려서 남음",
          len(last[1]["arg"]["blob"]) <= MAX_STR + 1)
    check("리스트는 앞쪽만", len(last[0]["arg"]["big"]) == MAX_ITEMS + 1)
    check("직렬화 불가 값도 문자열로", isinstance(last[0]["arg"]["obj"], str))
    check("한 줄 상한 넘으면 표식으로 대체", "_trunc" in json.dumps(last[-1]))
    check("모든 줄이 JSON 으로 읽힘",
          all(json.loads(x) for x in
              open(os.path.join(tmp, "a.jsonl"), encoding="utf-8")))
    lg.close()

    # ── 4. 회전 ────────────────────────────────────────────────────────────
    print("\n── 4. 크기 기반 회전 ──")
    rp = os.path.join(tmp, "rot.jsonl")
    lg2 = AuditLog(rp, max_bytes=8192, keep=3, atexit_close=False)
    for i in range(400):
        lg2.command("teleop", {"i": i, "v": 0.5, "w": 0.1}, role="operator")
    lg2.flush(3.0)
    rotated = [p for p in (f"{rp}.{i}" for i in range(1, 6))
               if os.path.exists(p)]
    check("회전이 일어남", lg2.rotations > 0, f"{lg2.rotations}회")
    check("보관 개수 제한", len(rotated) == 3, f"{len(rotated)}개 남음")
    check("각 파일이 상한 근처", all(os.path.getsize(p) <= 8192 + 512
                                     for p in [rp] + rotated))
    check("회전 파일까지 거슬러 조회", len(lg2.tail(120)) == 120)
    check("줄 경계가 안 깨짐",
          all(json.loads(x) for x in open(f"{rp}.1", encoding="utf-8")))
    lg2.close()

    # ── 5. 큐 포화 — 오래된 것을 버리고 구멍을 남긴다 ──────────────────────
    print("\n── 5. 큐가 가득 차도 제어 루프를 막지 않는다 ──")
    sp = os.path.join(tmp, "slow.jsonl")
    lg3 = AuditLog(sp, queue_size=32, atexit_close=False)
    lg3._write_batch = lambda batch: time.sleep(0.02)    # 느린 디스크 흉내
    t0 = time.monotonic()
    for i in range(3000):
        lg3.command("teleop", {"i": i})
    dt = time.monotonic() - t0
    check("3000건 요청이 블록되지 않음", dt < 1.0, f"{dt * 1000:.0f} ms")
    check("오래된 것부터 버림 + 카운트", lg3.dropped > 0, f"{lg3.dropped}건 유실")
    del lg3._write_batch                                # 정상 쓰기로 복귀
    lg3.command("estop", {"reason": "마지막 명령"})
    lg3.flush(3.0)
    txt = open(sp, encoding="utf-8").read()
    check("유실 사실이 파일에도 남음", "queue_overflow" in txt)
    check("가장 최근 명령은 살아남음", "마지막 명령" in txt)
    lg3.close()

    # ── 6. 파일을 못 열면 조용히 비활성 ────────────────────────────────────
    print("\n── 6. 못 쓰면 조용히 꺼진다 (관제는 계속 산다) ──")
    blocker = os.path.join(tmp, "notadir")
    open(blocker, "w").close()                  # 파일을 디렉터리처럼 쓰게 만든다
    try:
        lg4 = AuditLog(os.path.join(blocker, "sub", "x.jsonl"),
                       atexit_close=False)
        ok_no_raise = True
    except Exception as e:
        lg4, ok_no_raise = None, False
        print(f"      예외: {e}")
    check("생성에서 예외를 던지지 않음", ok_no_raise)
    if lg4 is not None:
        check("비활성 상태", not lg4.enabled, lg4.disabled_reason[:40])
        check("기록해도 예외 없음", lg4.command("estop", {"a": 1}) is False)
        check("조회는 빈 목록", lg4.tail(5) == [])
        check("사유가 stats 에 드러남", bool(lg4.stats()["reason"]))
        lg4.close()

    # ── 7. 종료 시 flush ───────────────────────────────────────────────────
    print("\n── 7. 종료 시 남김 없이 flush ──")
    cp = os.path.join(tmp, "close.jsonl")
    lg5 = AuditLog(cp, atexit_close=False)
    for i in range(500):
        lg5.command("ping", {"i": i})
    lg5.close()                                 # flush 를 따로 부르지 않는다
    n = sum(1 for _ in open(cp, encoding="utf-8"))
    check("큐에 남은 것까지 기록됨", n == 500, f"{n}/500 줄")
    check("닫은 뒤 기록은 무동작", lg5.command("x") is False)
    check("닫기를 두 번 불러도 안전", lg5.close() is None)

    print("\n" + "=" * 74)
    n_ok, n_all = sum(res), len(res)
    print(f"{n_ok}/{n_all} 통과   (시험 파일: {tmp})")
    shutil.rmtree(tmp, ignore_errors=True)
    sys.exit(0 if n_ok == n_all else 1)
