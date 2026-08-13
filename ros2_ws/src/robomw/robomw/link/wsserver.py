"""
의존성 없는 WebSocket + 정적파일 서버 (RFC 6455)

이 머신은 pip 가 PEP 668 로 잠겨 있고 sudo 도 없어서 fastapi/uvicorn/websockets 를
설치할 수 없다. venv 도 ensurepip 가 없어 실패한다. 그래서 표준 라이브러리만으로
짰다. 결과적으로 배포가 단순해지는 이점도 있다 — 로봇에 올릴 때 파이썬 표준
라이브러리 말고는 아무것도 필요 없다.

지원 범위는 우리가 실제로 쓰는 것까지만이다: 텍스트 프레임, ping/pong, close,
클라이언트 마스킹. 확장(permessage-deflate)·바이너리·조각난 프레임은 다루지 않는다.
프레임 코덱을 따로 떼어둔 이유는, 나중에 로봇이 관제로 **걸어 나가는**(아웃바운드)
연결이 필요해지면 같은 코덱으로 클라이언트를 붙이기 위해서다.
"""
from __future__ import annotations

import base64
import hashlib
import hmac
import json
import os
import socket
import ssl
import struct
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from robomw.link import protocol as P

GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

OP_CONT, OP_TEXT, OP_BIN, OP_CLOSE, OP_PING, OP_PONG = 0x0, 0x1, 0x2, 0x8, 0x9, 0xA

# 한 프레임을 이 시간 안에 소켓으로 밀어내지 못하면 그 관제를 끊는다.
#
# **시한이 없으면 안 된다.** 관제가 읽기를 멈추면(asyncio 루프 정체 등) 커널
# 송신 버퍼가 차고 sendall 이 무기한 막힌다. 브로드캐스트는 ROS 실행기
# 스레드에서 도니까, 그 순간 텔레메트리도 제어 틱도 통째로 선다.
#
# 프레임만 버리고 링크는 살리는 절충은 두지 않았다. 버퍼가 찬 상태에서
# 시한을 넘기면 프레임이 **반쯤 나간** 경우가 대부분인데(우리 프레임은
# 수십 KB 라 한 번에 안 들어간다), 반만 나간 WebSocket 프레임은 이어 붙일
# 수 없다 — 그 스트림은 이미 깨졌다. 그러니 정직하게 끊고 재접속시킨다.
# 건강한 관제는 랜/루프백에서 한 프레임을 밀리초 안에 받는다. 2초를 못
# 받는 관제는 이미 관제가 아니다.
SEND_TIMEOUT_S = 2.0

# 커널에 거는 send 한 번의 시한(SO_SNDTIMEO). 전체 시한을 이 조각으로 나눠
# 재시도한다 — 조각마다 돌아와야 SEND_TIMEOUT_S 를 시험이 갈아 끼워도 듣는다.
SEND_SLICE_S = 0.2

# 수신 루프가 PONG 을 보내려고 송신 락을 기다리는 한도. 링크 두절 판정
# (P.LINK_LOSS_STOP_MS=1.5초)보다 훨씬 짧아야 한다 — Conn.pong 주석 참조.
PONG_LOCK_WAIT_S = 0.2

MIME = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
        ".css": "text/css; charset=utf-8", ".json": "application/json",
        ".svg": "image/svg+xml", ".png": "image/png", ".ico": "image/x-icon"}


# ═══════════════════════════════════════════════════════════════════════════
# 프레임 코덱
# ═══════════════════════════════════════════════════════════════════════════
def encode_frame(payload: bytes, opcode: int = OP_TEXT, mask: bool = False) -> bytes:
    """서버→클라이언트는 마스킹하지 않는다 (RFC 6455 §5.1)."""
    n = len(payload)
    head = bytearray([0x80 | opcode])
    mbit = 0x80 if mask else 0x00
    if n < 126:
        head.append(mbit | n)
    elif n < (1 << 16):
        head.append(mbit | 126)
        head += struct.pack("!H", n)
    else:
        head.append(mbit | 127)
        head += struct.pack("!Q", n)
    if mask:
        key = os.urandom(4)
        head += key
        payload = bytes(b ^ key[i % 4] for i, b in enumerate(payload))
    return bytes(head) + payload


def _set_send_timeout(sock, seconds: float) -> bool:
    """소켓의 **송신에만** 시한을 건다 (SO_SNDTIMEO). 실패해도 조용히 넘어간다.

    못 걸면 예전 동작(무기한 차단)으로 되돌아갈 뿐이라 연결을 막지는 않는다 —
    이 옵션 하나 때문에 관제가 아예 안 붙는 쪽이 더 나쁘다.
    """
    try:
        sec = int(seconds)
        usec = int(round((seconds - sec) * 1_000_000))
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDTIMEO,
                        struct.pack("ll", sec, usec))
        return True
    except Exception:
        return False


def _recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("연결 종료")
        buf += chunk
    return buf


def decode_frame(sock):
    """한 프레임을 읽어 (opcode, payload) 로 돌려준다."""
    b0, b1 = _recv_exact(sock, 2)
    opcode = b0 & 0x0F
    masked = bool(b1 & 0x80)
    n = b1 & 0x7F
    if n == 126:
        n = struct.unpack("!H", _recv_exact(sock, 2))[0]
    elif n == 127:
        n = struct.unpack("!Q", _recv_exact(sock, 8))[0]
    if n > 8 << 20:                       # 8 MB 초과는 우리 규약에 없다 — 끊는다
        raise ConnectionError(f"프레임이 너무 크다: {n}")
    key = _recv_exact(sock, 4) if masked else None
    data = _recv_exact(sock, n) if n else b""
    if masked:
        data = bytes(b ^ key[i % 4] for i, b in enumerate(data))
    return opcode, data


# ═══════════════════════════════════════════════════════════════════════════
# 연결
# ═══════════════════════════════════════════════════════════════════════════
class Conn:
    """WebSocket 연결 하나. 송신은 락으로 직렬화한다.

    role 은 업그레이드 때 제시한 토큰으로 정해지고 연결이 사는 동안 바뀌지
    않는다. 승격하려면 다른 토큰으로 다시 붙어야 한다 — 세션 중 권한이 오르는
    경로를 아예 두지 않는 편이 검증하기 쉽다.

    명령을 실제로 막는 것은 이 파일이 아니라 control_agent 다. 여기서는
    역할을 붙이기만 한다 (판정은 protocol.authorize).
    """

    # 기본값이 최소 권한인 이유: 역할을 넘기는 것을 잊은 호출 경로가 생기면
    # 그 연결은 조용히 전권을 갖게 된다. 잊으면 닫히는 쪽이 기본이어야 한다.
    # (개방 모드에서는 _role_for 가 admin 을 **명시적으로** 넘긴다)
    def __init__(self, sock, addr, role=P.ROLE_FALLBACK):
        self.sock = sock
        self.addr = addr
        self.role = P.normalize_role(role)
        self.lock = threading.Lock()
        self.alive = True
        self.opened = time.time()
        self.sent = 0
        self.dropped = 0
        self.deferred_pongs = 0
        # 송신 락을 못 잡아 미뤄 둔 PONG (다음 송신에 얹어 보낸다 — Conn.pong 참조).
        self._pending_pong = None
        self._pong_lock = threading.Lock()
        _set_send_timeout(sock, SEND_SLICE_S)

    def _send_locked(self, data: bytes, timeout=None) -> None:
        """락을 쥔 채 한 프레임을 보낸다. 시한을 넘기면 TimeoutError.

        sock.sendall 도, settimeout 도, select 도 쓰지 않는다 — 셋 다 여기서는
        안 듣는다.

          sendall      : 상대가 안 읽으면 무기한 매달린다(이 결함의 원인).
          settimeout   : 소켓 하나에 양방향으로 걸린다. recv 는 다음 프레임을
                         무기한 기다려야 하는데(수신 루프) 시한이 걸리면
                         멀쩡한 연결이 주기적으로 끊긴다.
          select       : **차단 소켓의 send 는 사실상 sendall 이다** — 쓸 공간이
                         생길 때까지 기다렸다 전량을 큐잉하고 돌아온다. 그래서
                         "쓸 수 있을 때만 send" 로는 시한이 안 걸린다(실측).

        남는 것이 SO_SNDTIMEO 다. 방향별 옵션이라 recv 는 그대로 두고 send 만
        제한한다(실측: recv 는 1.5초 뒤에도 정상 대기). 커널은 조각 시한마다
        보낸 만큼을 돌려주거나 EAGAIN 을 던지므로, 전체 시한까지 재시도한다.
        """
        timeout = SEND_TIMEOUT_S if timeout is None else timeout
        view = memoryview(data)
        end = time.monotonic() + timeout
        while view:
            if time.monotonic() >= end:
                raise TimeoutError("송신 시한 초과 — 관제가 읽지 않는다")
            try:
                n = self.sock.send(view)
            except (BlockingIOError, TimeoutError):
                continue                    # 조각 시한 안에 한 바이트도 못 냈다
            except InterruptedError:
                continue
            if n <= 0:
                raise ConnectionError("송신 실패")
            view = view[n:]

    def _flush_pong_locked(self) -> None:
        """송신 락을 쥔 상태에서, 미뤄 둔 PONG 이 있으면 **먼저** 내보낸다."""
        with self._pong_lock:
            data, self._pending_pong = self._pending_pong, None
        if data is not None:
            self._send_locked(data)

    def send_text(self, s: str) -> bool:
        if not self.alive:
            return False
        try:
            with self.lock:
                self._flush_pong_locked()
                self._send_locked(encode_frame(s.encode("utf-8"), OP_TEXT))
            self.sent += 1
            return True
        except TimeoutError:
            # 시한 안에 못 밀어냈다. 프레임이 반쯤 나갔을 수 있어 스트림을
            # 믿을 수 없다 — 끊고 재접속시킨다(SEND_TIMEOUT_S 주석 참조).
            self.dropped += 1
            self.alive = False
            return False
        except Exception:
            self.alive = False
            return False

    def send_json(self, obj) -> bool:
        return self.send_text(json.dumps(obj, separators=(",", ":")))

    def pong(self, data: bytes) -> bool:
        """PING 에 답한다. **송신 락을 무기한 기다리지 않는다.**

        여기서 락을 그냥 기다리면(예전 코드) 느린 관제 하나 때문에 수신
        루프가 통째로 선다: 브로드캐스트가 그 관제의 소켓에서 막힌 동안
        락을 쥐고 있으므로, 같은 연결의 수신 스레드는 PING 을 받은 자리에서
        멈춘다. 그러면 뒤따르는 명령·하트비트를 읽지 못해 note_client 가
        끊기고, 로봇은 **접속이 살아 있는 링크**를 두절로 오판해 임무를
        세운다(2026-08-13 실측: 관제 어댑터만 2~4초 주기로 플랩. 라이브러리
        keepalive 를 안 쓰는 단순 클라이언트는 TEXT 경로가 락을 안 잡아
        멀쩡했다 — 그 비대칭이 이 결함의 지문이다).

        그렇다고 **버리면 안 된다.** 클라이언트(websockets)는 자기가 보낸 ping
        하나에 대한 pong 을 ping_timeout 동안 기다리다 없으면 1011 로 끊는다 —
        pong 을 거르는 것만으로 우리가 고치려던 플랩이 그대로 다시 난다
        (실측: 거르기만 했더니 link_lost 는 사라졌는데 keepalive timeout 이 남았다).

        그래서 **미뤄 둔다**: 락을 못 잡으면 들고 있다가 다음 송신에 얹어
        보낸다(_flush_pong_locked). 수신 루프는 즉시 돌아가고, pong 은 다음
        프레임 경계에서 나간다(5 Hz 텔레메트리면 0.2초 안).

        미뤄 둔 것이 이미 있으면 **최신 것만** 남긴다 — RFC 6455 §5.5.3 이
        명시적으로 허용하고(밀린 ping 여러 개에 대해 마지막 것만 답해도 된다),
        websockets 도 나중 pong 하나로 앞선 ping 들을 함께 해소한다.
        """
        if not self.alive:
            return False
        if not self.lock.acquire(timeout=PONG_LOCK_WAIT_S):
            with self._pong_lock:
                self._pending_pong = data
            self.deferred_pongs += 1
            return False
        try:
            self._flush_pong_locked()
            self._send_locked(data)
            return True
        except Exception:
            self.alive = False
            return False
        finally:
            self.lock.release()

    def ping(self):
        try:
            with self.lock:
                self._send_locked(encode_frame(b"", OP_PING))
        except Exception:
            self.alive = False

    def close(self):
        self.alive = False
        try:
            with self.lock:
                # 닫는 길에서 오래 붙들지 않는다 — 이미 못 읽는 상대다.
                self._send_locked(encode_frame(b"", OP_CLOSE), timeout=0.2)
        except Exception:
            pass
        try:
            self.sock.close()
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════════════════
# 토큰 → 역할
# ═══════════════════════════════════════════════════════════════════════════
def _token_roles(auth_token):
    """auth_token 파라미터를 ({토큰: 역할}, 경고목록, 잠금여부) 로 정규화한다.

        None / ""          → ({}, [], False)   개방 모드 (인증 없음)
        "abc..."           → ({"abc...": admin}, [], False)
        {"abc":"operator"} → 그대로 (역할 이름은 protocol 이 검증)
        {}  또는 전부 무효 → ({}, [경고], True)  **전면 거부**

    **단일 문자열이 admin 인 이유는 하위호환이다.** 지금까지 그 토큰 하나가
    전권이었고, 이미 배포된 launch 파일·스크립트·북마크 URL 이 전부 그 토큰을
    쓴다. 여기서 조용히 권한을 낮추면 현장에서 비상정지 해제가 안 되는 형태로
    터진다 — 권한 축소는 설정을 dict 로 바꾸는 **명시적 행동**이어야 한다.

    역할 이름이 이상하면 예외를 던지지 않고 observer 로 떨군다. 관제 서버가
    설정 오타 때문에 아예 안 뜨는 것보다, 뜨되 아무 명령도 안 먹는 쪽이 낫다
    (붙어서 로그를 볼 수는 있어야 원인을 찾는다). 다만 **조용히** 떨구지는
    않는다 — 경고 목록으로 돌려 준다.

    잠금(세 번째 값)이 있는 이유. 예전에는 사전의 항목이 전부 걸러져 {} 가
    되면 `not self.token_roles` 가 참이 되어 **개방 모드**로 열렸다. 즉 토큰을
    걸려고 쓴 설정이 오타 하나로 문을 활짝 여는 방향으로 실패했다. 사전을
    준 것 자체가 "인증을 걸겠다"는 의사표시이므로, 쓸 수 있는 토큰이 0개면
    개방이 아니라 전면 거부다. 설정 실수로 문이 열리는 것보다 안 열리는 편이
    낫다 — 안 열리면 즉시 알아채고 고치지만, 열린 것은 아무도 모른다.
    """
    if auth_token is None or auth_token == "":
        return {}, [], False
    if isinstance(auth_token, str):
        return {auth_token: P.ROLE_ADMIN}, [], False
    if isinstance(auth_token, dict):
        out, notes = {}, []
        for tok, role in auth_token.items():
            if not isinstance(tok, str) or not tok:
                notes.append(f"토큰 항목을 버렸다 — 열쇠가 비어 있거나 "
                             f"문자열이 아니다: {type(tok).__name__}")
                continue
            r = P.normalize_role(role)
            if not P.is_role(role):
                # 토큰 값은 절대 로그에 싣지 않는다. 역할만 말해도 고칠 수 있다.
                notes.append(f"토큰 하나의 역할 {role!r} 을 알 수 없어 "
                             f"{r} 로 떨궜다 (쓸 수 있는 이름: "
                             f"{', '.join(P.ROLES)})")
            out[tok] = r
        if not out:
            notes.append("유효한 토큰이 하나도 없다 — 모든 접속을 거부한다 "
                         "(개방 모드로 열지 않는다)")
            return {}, notes, True
        return out, notes, False
    # 예외를 던지면 노드가 아예 안 뜬다. 그러면 로그를 볼 수도 없으니
    # 원인을 못 찾는다. 뜨되 문은 닫아 두고 사유를 남긴다.
    return {}, [f"auth_token 이 문자열도 사전도 아니다 "
                f"({type(auth_token).__name__}) — 모든 접속을 거부한다"], True


# ═══════════════════════════════════════════════════════════════════════════
# 서버
# ═══════════════════════════════════════════════════════════════════════════
class ControlServer:
    """정적 대시보드(HTTP) + 텔레메트리/명령(WebSocket) 을 한 포트에서 낸다.

    on_message(conn, text) 는 관제에서 온 메시지마다 호출된다. 워커 스레드에서
    불리므로, 호출 측에서 ROS 노드 상태를 만질 때 락을 잡아야 한다.

    conn.role 에 인증 때 정해진 역할이 들어 있다(observer/operator/admin).
    이 서버는 역할을 **붙이기만** 하고 명령을 막지는 않는다 — 판정은
    protocol.authorize(conn.role, 명령), 거부는 on_message 쪽(control_agent)
    담당이다. 전송 계층이 인가 정책을 알면 MQTT 로 갈아탈 때 정책이 따라
    흩어진다.
    """

    def __init__(self, static_dir, port=8080, host="0.0.0.0", ws_path="/ws",
                 on_message=None, on_open=None, on_close=None, logger=None,
                 auth_token=None, tls_cert=None, tls_key=None):
        self.static_dir = os.path.abspath(static_dir)
        self.port, self.host, self.ws_path = port, host, ws_path
        self.on_message = on_message
        self.on_open = on_open
        self.on_close = on_close
        self.log = logger or (lambda m: None)
        # 인증: 브라우저 WebSocket API 는 커스텀 헤더를 못 붙이므로 질의문자열로
        # 받는다 (?token=...). 헤더(Authorization: Bearer)도 함께 받아 도구·스크립트
        # 접속을 지원한다. 비교는 hmac.compare_digest 로 — 타이밍 공격 방지.
        #
        # auth_token 은 두 형태를 받는다 (아래 _token_roles 참조).
        #   "abc..."                    단일 토큰 — 그 토큰이 admin (하위호환)
        #   {"abc":"admin","def":"observer"}   토큰별 역할
        # 원래 값을 그대로 들고 있는 이유는 기존 호출부·로그가 참조하기 때문이다.
        self.auth_token = auth_token or None
        # auth_locked = 인증을 걸겠다고 했는데 쓸 수 있는 토큰이 0개인 상태.
        # 개방 모드(token_roles 가 비었고 잠금도 아님)와 반드시 구분해야 한다 —
        # 둘 다 사전은 비어 있지만 하나는 전원 admin, 하나는 전원 거부다.
        self.token_roles, self.auth_notes, self.auth_locked = _token_roles(auth_token)
        self.tls_cert, self.tls_key = tls_cert, tls_key
        self.conns = []
        self.clock = threading.Lock()
        self.rejected = 0
        self._httpd = None
        self._thread = None

    @staticmethod
    def _presented(path, headers):
        """요청에서 토큰 문자열을 뽑는다. 없으면 None."""
        q = urllib.parse.urlsplit(path).query
        vals = urllib.parse.parse_qs(q).get("token")
        if vals:
            return vals[0]
        hdr = headers.get("Authorization", "")
        if hdr.startswith("Bearer "):
            return hdr[7:].strip()
        return None

    def _role_for(self, path, headers):
        """(허용 여부, 역할) 을 돌려준다. 거부면 (False, None).

        토큰이 여러 개여도 **중간에 빠져나오지 않는다.** compare_digest 가
        한 건의 비교 시간을 감춰도, 몇 번째에서 맞았는지가 응답 시간으로
        새면 토큰을 하나씩 확인해 볼 수 있게 된다.
        """
        if self.auth_locked:
            # 설정이 깨졌다. 여기서 개방 모드로 넘어가면 "토큰을 걸었다고
            # 믿는 채로 문이 열려 있는" 최악의 상태가 된다.
            return False, None
        if not self.token_roles:
            # 개방 모드(토큰 미설정). 여기서 권한을 낮추면 개발·검증 흐름이
            # 통째로 막힌다. 토큰을 안 걸었다는 것은 신뢰 경계를 이 서버 밖
            # (사내망·VPN)에 뒀다는 뜻이므로 붙은 사람을 admin 으로 본다.
            # 이 상태의 위험은 control_agent 가 기동 로그로 경고한다.
            return True, P.ROLE_ADMIN
        tok = self._presented(path, headers)
        if tok is None:
            return False, None
        # bytes 로 맞춰 비교한다 — compare_digest 는 비ASCII str 을 받으면
        # TypeError 를 던지고, 그 예외가 인증 경로를 500 으로 만든다.
        try:
            given = tok.encode("utf-8")
        except Exception:
            return False, None
        role = None
        for known, r in self.token_roles.items():
            if hmac.compare_digest(given, known.encode("utf-8")):
                role = r
        return (role is not None), role

    def _authorized(self, path, headers):
        """예전 시그니처 유지 — 불리언만 필요한 호출부용."""
        return self._role_for(path, headers)[0]

    def _roles_note(self) -> str:
        """기동 로그에 실을 역할 구성 요약.

        토큰 값은 절대 찍지 않는다(로그가 자격증명 저장소가 되면 안 된다).
        역할별 개수만 보여도 "observer 로 줄 걸 admin 으로 줬다" 같은 설정
        실수는 눈에 띈다. 개방 모드면 붙는 사람이 전부 admin 임을 못 박는다.
        """
        if self.auth_locked:
            return " (잠금 — 유효한 토큰이 0개라 모든 접속을 거부한다)"
        if not self.token_roles:
            return " (개방 모드 — 접속자 전원 admin)"
        n = {}
        for r in self.token_roles.values():
            n[r] = n.get(r, 0) + 1
        order = [r for r in P.ROLES if r in n]
        return " · 토큰 " + ", ".join(f"{r} {n[r]}개" for r in order)

    # ── 브로드캐스트 ────────────────────────────────────────────────────────
    def broadcast(self, obj):
        """살아 있는 모든 관제 화면에 보낸다. 끊긴 연결은 걷어낸다.

        직렬화 실패로 노드가 죽으면 안 된다 — 관제가 죽는 것보다 그 한 프레임을
        버리는 게 낫다 (2026-07-26: numpy int32 가 섞여 들어와 노드가 종료됨).
        """
        try:
            text = json.dumps(obj, separators=(",", ":"))
        except TypeError as e:
            self.log(f"직렬화 실패 — 프레임 폐기: {e}")
            return
        dead = []
        with self.clock:
            conns = list(self.conns)
        for c in conns:
            if not c.send_text(text):
                dead.append(c)
        if dead:
            with self.clock:
                for c in dead:
                    if c in self.conns:
                        self.conns.remove(c)
            for c in dead:
                if self.on_close:
                    self.on_close(c)

    def client_count(self):
        with self.clock:
            return sum(1 for c in self.conns if c.alive)

    # ── 기동 ────────────────────────────────────────────────────────────────
    def start(self):
        server = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, fmt, *a):
                pass                        # 기본 stderr 로깅은 시끄럽다

            def do_GET(self):
                if self.path.split("?")[0] == server.ws_path:
                    return self._upgrade()
                return self._static()

            # 대시보드(HTML)는 인증 없이 낸다 — 토큰을 입력할 화면 자체가 필요하다.
            # 실제 제어 권한은 WebSocket 업그레이드에서만 검사한다.

            # 정적 파일
            def _static(self):
                rel = self.path.split("?")[0].lstrip("/") or "index.html"
                path = os.path.normpath(os.path.join(server.static_dir, rel))
                if not path.startswith(server.static_dir) or not os.path.isfile(path):
                    self.send_error(404)
                    return
                with open(path, "rb") as f:
                    body = f.read()
                ext = os.path.splitext(path)[1].lower()
                self.send_response(200)
                self.send_header("Content-Type", MIME.get(ext, "application/octet-stream"))
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            # WebSocket 업그레이드
            def _upgrade(self):
                key = self.headers.get("Sec-WebSocket-Key")
                if not key or "websocket" not in (
                        self.headers.get("Upgrade", "").lower()):
                    # 사유(reason phrase)는 상태줄에 들어가고 latin-1 로만
                    # 인코딩된다 — 한글을 넣으면 UnicodeEncodeError 가 나서
                    # 응답이 통째로 안 나가고 연결이 그냥 끊긴다. 클라이언트는
                    # 그것을 '서버 죽음'과 구분할 수 없다. 한글은 본문(explain,
                    # UTF-8)으로 보낸다.
                    self.send_error(400, "Bad Request", "WebSocket 업그레이드 아님")
                    return
                ok, role = server._role_for(self.path, self.headers)
                if not ok:
                    with server.clock:
                        server.rejected += 1
                    server.log(f"인증 거부: {self.client_address[0]} "
                               f"(누적 {server.rejected})")
                    # 401 이 실제로 나가야 한다. 응답 없이 끊기면 화면에서
                    # '토큰이 틀렸다'와 '로봇이 죽었다'가 똑같아 보인다.
                    self.send_error(401, "Unauthorized", "인증 토큰 불일치")
                    return
                accept = base64.b64encode(
                    hashlib.sha1((key + GUID).encode()).digest()).decode()
                self.send_response(101, "Switching Protocols")
                self.send_header("Upgrade", "websocket")
                self.send_header("Connection", "Upgrade")
                self.send_header("Sec-WebSocket-Accept", accept)
                self.end_headers()
                try:
                    self.wfile.flush()
                except Exception:
                    return

                sock = self.connection
                sock.settimeout(None)
                conn = Conn(sock, self.client_address, role=role)
                with server.clock:
                    server.conns.append(conn)
                # 역할을 접속 로그에 남긴다 — 나중에 "누가 무엇을 했나"를
                # 되짚을 때 명령 이벤트만으로는 부족하다.
                server.log(f"관제 접속: {conn.addr[0]}:{conn.addr[1]} "
                           f"[{conn.role}] (총 {server.client_count()})")
                if server.on_open:
                    server.on_open(conn)
                try:
                    while conn.alive:
                        op, data = decode_frame(sock)
                        if op == OP_CLOSE:
                            break
                        if op == OP_PING:
                            # 락을 기다리며 여기 멈추면 안 된다 — Conn.pong 참조.
                            conn.pong(encode_frame(data, OP_PONG))
                            continue
                        if op == OP_PONG:
                            continue
                        if op == OP_TEXT and server.on_message:
                            try:
                                server.on_message(conn, data.decode("utf-8"))
                            except Exception as e:
                                server.log(f"메시지 처리 실패: {e}")
                except Exception:
                    pass
                finally:
                    conn.alive = False
                    with server.clock:
                        if conn in server.conns:
                            server.conns.remove(conn)
                    server.log(f"관제 해제: {conn.addr[0]}:{conn.addr[1]} "
                               f"(총 {server.client_count()})")
                    if server.on_close:
                        server.on_close(conn)
                    try:
                        sock.close()
                    except Exception:
                        pass

        class TS(ThreadingHTTPServer):
            daemon_threads = True
            allow_reuse_address = True

        self._httpd = TS((self.host, self.port), Handler)
        if self.tls_cert and self.tls_key:
            ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ctx.load_cert_chain(self.tls_cert, self.tls_key)
            ctx.minimum_version = ssl.TLSVersion.TLSv1_2
            self._httpd.socket = ctx.wrap_socket(self._httpd.socket, server_side=True)
        self._thread = threading.Thread(target=self._httpd.serve_forever,
                                        kwargs=dict(poll_interval=0.2), daemon=True)
        self._thread.start()
        ips = _local_ips()
        sch = "https" if (self.tls_cert and self.tls_key) else "http"
        self.log(f"관제 서버 시작 — {sch}://{ips[0]}:{self.port}/  "
                 f"(WebSocket {self.ws_path})")
        if len(ips) > 1:
            self.log(f"  다른 PC 에서: " + ", ".join(f"{sch}://{i}:{self.port}/"
                                                     for i in ips[1:]))
        # 설정 실수는 기동 로그에서 반드시 보여야 한다. 조용히 걸러 두면
        # "토큰을 걸었는데 왜 아무도 못 붙지" 를 현장에서 헤매게 된다.
        for note in self.auth_notes:
            self.log(f"  auth_token 경고: {note}")
        if self.auth_locked:
            auth = "잠금(설정 오류 — 전면 거부)"
        elif self.token_roles:
            auth = "토큰 필요"
        else:
            auth = "없음(개방)"
        self.log(f"  전송 {'TLS' if sch == 'https' else '평문(TLS 미설정)'} · "
                 f"인증 {auth}{self._roles_note()}")
        return self

    def stop(self):
        for c in list(self.conns):
            c.close()
        if self._httpd:
            self._httpd.shutdown()
            self._httpd.server_close()


def _local_ips():
    """접속 안내에 쓸 IP 목록. 별도 PC 에서 붙을 주소를 바로 알려주기 위한 것."""
    out = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))          # 실제로 보내지 않는다 — 경로만 조회
        out.append(s.getsockname()[0])
        s.close()
    except Exception:
        pass
    out.append("127.0.0.1")
    return out
