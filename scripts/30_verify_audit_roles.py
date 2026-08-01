#!/usr/bin/env python3
"""
감사 로그 + 거부 억제 수정 검증

    python3 scripts/30_verify_audit_roles.py --port 8080 --audit /tmp/audit.jsonl \
        --token-observer OBS --token-admin ADM

2026-07-31 적대적 검증이 잡은 잔여 결함 두 건을 실제로 막았는지 확인한다.

  [중간] 거부 이벤트 억제가 **공격자 제어 문자열**을 열쇠로 썼다.
         같은 명령 30회 → 이벤트 1건(억제 작동), 다른 명령 30회 → 30건 전부 통과.
         억제가 막으려던 것이 정확히 "거부가 이벤트 창을 뒤덮는 것" 인데
         그 공격에는 안 들었다. → 열쇠를 (주소, 역할) 로만 잡도록 고쳤다.

  [중간] 사유 길이 제한이 없어 2만 자 명령 이름이 모든 화면으로 브로드캐스트됐다.
         → 160 자로 자른다.

  [정보] control/audit.py 가 통째로 미배선이었다 (grep 0건).
         → 명령 수락/거부·안전 이벤트를 영속 기록하도록 연결했다.
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import socket
import sys
import time

sys.path.insert(0, "ros2_ws/src/orchard_sim")
from orchard_sim.link import protocol as P                       # noqa: E402
from orchard_sim.link.wsserver import (OP_TEXT, decode_frame,    # noqa: E402
                                       encode_frame)

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="127.0.0.1")
ap.add_argument("--port", type=int, default=8080)
ap.add_argument("--audit", default="/tmp/audit.jsonl")
ap.add_argument("--token-observer", default="OBS")
ap.add_argument("--token-admin", default="ADM")
a = ap.parse_args()

OK, NG = "\033[92m✔\033[0m", "\033[91m✗\033[0m"
res = []


def check(name, cond, detail=""):
    res.append(bool(cond))
    print(f"   {OK if cond else NG} {name}" + (f"  — {detail}" if detail else ""))


def ws(token):
    s = socket.create_connection((a.host, a.port), timeout=8)
    key = base64.b64encode(os.urandom(16)).decode()
    s.sendall((f"GET /ws?token={token} HTTP/1.1\r\nHost: {a.host}\r\n"
               "Upgrade: websocket\r\nConnection: Upgrade\r\n"
               f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
               ).encode())
    buf = b""
    while b"\r\n\r\n" not in buf:
        buf += s.recv(4096)
    if "101" not in buf.decode("latin1").split("\r\n")[0]:
        raise RuntimeError("업그레이드 실패: " + buf.decode("latin1")[:80])
    return s


def send(s, suffix, payload, robot="scout01"):
    m = P.envelope(f"orchard/{robot}/{suffix}", payload, int(time.time() * 1e9), 1)
    s.sendall(encode_frame(json.dumps(m).encode(), OP_TEXT, mask=True))


def drain(s, secs=2.5):
    out, end = [], time.time() + secs
    s.settimeout(0.4)
    while time.time() < end:
        try:
            op, data = decode_frame(s)
        except Exception:
            continue
        if op != OP_TEXT:
            continue
        try:
            t, pl, _, _ = P.parse(json.loads(data.decode()))
        except Exception:
            continue
        out.append((t.rsplit("/", 1)[-1], pl))
    return out


print("감사 로그 + 거부 억제 검증")
print("=" * 74)

# ── 1. 억제 우회 차단 ───────────────────────────────────────────────────────
print("\n── 1. 거부 이벤트 억제 (공격자가 사유를 바꿔도 눌리는가) ──")
obs = ws(a.token_observer)
adm = ws(a.token_admin)
drain(obs, 1.0); drain(adm, 1.0)

for i in range(30):                       # 매번 다른 명령 이름
    send(obs, "cmd", dict(cmd=f"bogus_cmd_{i}"))
    time.sleep(0.01)
ev = [p for k, p in drain(adm, 3.0) if k == "event" and p.get("kind") == "denied"]
check("서로 다른 명령 30회 → 이벤트 억제됨", len(ev) <= 3,
      f"{len(ev)}건 (예전에는 30건 전부 통과)")
check("첫 거부는 올라옴", len(ev) >= 1, f"{len(ev)}건")

# ── 2. 사유 길이 제한 ───────────────────────────────────────────────────────
print("\n── 2. 긴 명령 이름이 화면으로 증폭되는가 ──")
time.sleep(2.2)                            # 억제 창 넘기기
send(obs, "cmd", dict(cmd="X" * 20000))
ev2 = [p for k, p in drain(adm, 3.0) if k == "event" and p.get("kind") == "denied"]
longest = max((len(p.get("msg", "")) for p in ev2), default=0)
check("2만 자 명령 → 이벤트가 잘림", longest < 1000,
      f"최대 {longest}자 (예전 20,071자)")

# ── 3. 권한이 여전히 막는가 (회귀) ──────────────────────────────────────────
print("\n── 3. 권한 회귀 ──")
time.sleep(2.2)
send(obs, "cmd", dict(cmd=P.CMD_CLEAR_ESTOP_REQUEST))
send(obs, "cmd", dict(cmd=P.CMD_MISSION_START, alleys=[0]))
st = [p for k, p in drain(adm, 3.0) if k == "state"]
if st:
    check("observer 의 해제 승인·임무 무효", not st[-1].get("estop")
          and st[-1].get("mission") is None)
else:
    check("상태 수신", False, "state 없음")
obs.close(); adm.close()

# ── 4. 감사 로그 ────────────────────────────────────────────────────────────
print("\n── 4. 감사 로그 영속 기록 ──")
time.sleep(1.5)
check("파일 생성됨", os.path.exists(a.audit), a.audit)
if os.path.exists(a.audit):
    lines = [json.loads(x) for x in open(a.audit, encoding="utf-8")
             if x.strip().startswith("{")]
    check("JSON Lines 로 읽힘", len(lines) > 0, f"{len(lines)}건")
    kinds = {x.get("kind") for x in lines}
    check("명령 기록 있음", "command" in kinds, str(sorted(kinds)))
    rej = [x for x in lines if x.get("result") == "rejected"]
    check("거부가 **전부** 기록됨 (이벤트는 억제해도)", len(rej) >= 25,
          f"{len(rej)}건 — 이벤트는 {len(ev)}건만 올라갔다")
    blob = json.dumps(lines, ensure_ascii=False)
    check("토큰이 안 남음", a.token_observer not in blob
          and a.token_admin not in blob)
    longest = max((len(json.dumps(x, ensure_ascii=False)) for x in lines), default=0)
    check("기록 한 줄이 과도하게 길지 않음", longest < 4000, f"최대 {longest}자")

print("\n" + "=" * 74)
n_ok, n = sum(res), len(res)
print(f"{n_ok}/{n} 통과")
sys.exit(0 if n_ok == n else 1)
