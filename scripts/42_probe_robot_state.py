#!/usr/bin/env python3
"""로봇 상태 진단 — 왜 안 움직이는가

    python3 scripts/42_probe_robot_state.py [--secs 6]

관제 링크로 붙어 state/event 를 받아 '무엇이 막고 있는지'를 한 화면에 보여준다.
링크를 유지하기 위해 ping 을 계속 보낸다 (SafetyArbiter 의 링크두절 판정을
피하려면 관제 쪽에서 트래픽이 계속 있어야 한다 — 실제로 겪은 함정).
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
ap.add_argument("--secs", type=float, default=6.0)
ap.add_argument("--host", default="127.0.0.1")
ap.add_argument("--port", type=int, default=8080)
a = ap.parse_args()

s = socket.create_connection((a.host, a.port), timeout=8)
k = base64.b64encode(os.urandom(16)).decode()
s.sendall((f"GET /ws HTTP/1.1\r\nHost: {a.host}\r\nUpgrade: websocket\r\n"
           f"Connection: Upgrade\r\nSec-WebSocket-Key: {k}\r\n"
           "Sec-WebSocket-Version: 13\r\n\r\n").encode())
b = b""
while b"\r\n\r\n" not in b:
    b += s.recv(4096)


def send(cmd, **kw):
    m = P.envelope("orchard/scout01/cmd", dict(cmd=cmd, **kw), time.time_ns(), 1)
    s.sendall(encode_frame(json.dumps(m).encode(), OP_TEXT, mask=True))


s.settimeout(0.4)
end = time.time() + a.secs
st, hello, events, poses = None, None, [], []
while time.time() < end:
    send(P.CMD_PING)
    try:
        op, dd = decode_frame(s)
    except Exception:
        continue
    if op != OP_TEXT:
        continue
    try:
        t, pl, _, _ = P.parse(json.loads(dd.decode()))
    except Exception:
        continue
    if t.endswith("/state"):
        st = pl
        if pl.get("pose"):
            poses.append((pl["pose"]["x"], pl["pose"]["y"]))
    elif t.endswith("/hello"):
        hello = pl
    elif t.endswith("/event"):
        events.append(pl)
s.close()

if st is None:
    print("state 수신 실패 — 에이전트가 떠 있는지 확인할 것")
    sys.exit(1)

print("로봇 상태 진단")
print("=" * 74)
print(f"  모드        {st.get('mode')}")
print(f"  일시정지    {st.get('paused')}")
print(f"  비상정지    {st.get('estop')} (단계 {st.get('estop_stage')})")
print(f"  정비모드    {st.get('service_mode') or '-'}")
print(f"  속도 게이트 {st.get('gate') or '(열림)'}   ← 무엇이 막고 있는지")
p = st.get("pose") or {}
print(f"  자세        ({p.get('x')}, {p.get('y')}) · 기울기 {st.get('tilt')}°")

m = st.get("mission")
if m:
    print(f"\n  임무        통로 {m.get('alleys')} · {m.get('idx')}/{m.get('total')}")
    print(f"              현재 통로 {m.get('alley')} · 구간 {m.get('phase')}")
    print(f"              경과 {m.get('elapsed', 0):.0f}초")
    for k2 in ("wp", "target", "goal", "dist", "remaining"):
        if k2 in m:
            print(f"              {k2}: {m[k2]}")
else:
    print("\n  임무        없음")

if len(poses) >= 2:
    dx = poses[-1][0] - poses[0][0]
    dy = poses[-1][1] - poses[0][1]
    print(f"\n  {a.secs:.0f}초간 이동 {(dx*dx+dy*dy)**0.5:.3f} m")

if events:
    print("\n  최근 사건")
    for e in events[-6:]:
        print(f"    [{e.get('level','info')}] {e.get('kind')}: {e.get('msg')}")

gate = st.get("gate")
print("\n" + "=" * 74)
if gate == "link":
    print("판정: 관제 링크가 끊긴 것으로 보고 있다 — 관제가 주기적으로 보내야 한다")
elif gate == "estop":
    print("판정: 비상정지 래치 — 관제 승인 + 현장 확인(~/local_reset) 필요")
elif gate == "paused":
    print("판정: 일시정지 — mission_resume 이 필요하다")
elif gate == "maintenance":
    print("판정: 정비 모드 — 원격 구동이 차단돼 있다")
elif gate == "deadman":
    print("판정: 데드맨 — 조종 입력이 끊겼다")
elif not gate:
    print("판정: 게이트는 열려 있다. 안 움직인다면 임무 로직 쪽을 봐야 한다")
else:
    print(f"판정: 게이트 '{gate}'")
