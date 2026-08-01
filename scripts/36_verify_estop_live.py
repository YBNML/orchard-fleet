#!/usr/bin/env python3
"""2단계 비상정지 해제 — 실기(ROS 에이전트) 대상 검증

    server/.venv/bin/python scripts/36_verify_estop_live.py

35번은 안전 코어를 단독으로 검증한다(순수 파이썬). 이 스크립트는 **실제로
돌고 있는 control_agent** 에 관제 링크로 붙어서 같은 성질을 확인한다 —
가짜 로봇이 아니라 진짜 노드가 규격대로 버티는지 본다.

현장 확인은 링크로 보낼 수 없으므로(그게 이 절차의 요점이다) ROS 토픽
~/local_reset 으로 넣는다 — 실기에서는 기체의 물리 리셋 버튼 자리다.
"""
from __future__ import annotations

import base64
import json
import os
import socket
import subprocess
import sys
import time

sys.path.insert(0, "ros2_ws/src/orchard_sim")
from orchard_sim.link import protocol as P                        # noqa: E402
from orchard_sim.link.wsserver import (OP_TEXT, decode_frame,     # noqa: E402
                                       encode_frame)

HOST, PORT = "127.0.0.1", 8080
OK, NG = "\033[92m✔\033[0m", "\033[91m✗\033[0m"
res = []


def check(name, cond, detail=""):
    res.append(bool(cond))
    print(f"   {OK if cond else NG} {name}" + (f"  — {detail}" if detail else ""))


def ws():
    s = socket.create_connection((HOST, PORT), timeout=8)
    key = base64.b64encode(os.urandom(16)).decode()
    s.sendall((f"GET /ws HTTP/1.1\r\nHost: {HOST}\r\nUpgrade: websocket\r\n"
               f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
               "Sec-WebSocket-Version: 13\r\n\r\n").encode())
    buf = b""
    while b"\r\n\r\n" not in buf:
        buf += s.recv(4096)
    if "101" not in buf.decode("latin1").split("\r\n")[0]:
        raise RuntimeError("업그레이드 실패")
    return s


def send(s, suffix, payload):
    m = P.envelope(f"orchard/scout01/{suffix}", payload, time.time_ns(), 1)
    s.sendall(encode_frame(json.dumps(m).encode(), OP_TEXT, mask=True))


def state(s, secs=3.0):
    """최근 state 페이로드 하나를 받아온다."""
    end, last = time.time() + secs, None
    s.settimeout(0.5)
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
        if t.endswith("/state"):
            last = pl
    return last


def local_reset():
    """현장 확인 — 기체의 물리 리셋 버튼에 해당한다."""
    subprocess.run(
        ["bash", "-lc",
         "source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash && "
         "ros2 topic pub --once /control_agent/local_reset std_msgs/msg/Empty '{}'"],
        capture_output=True, timeout=30)


print("2단계 비상정지 해제 — 실기 검증")
print("=" * 74)

c = ws()
st = state(c)
check("에이전트 접속·상태 수신", st is not None, f"mode={st and st.get('mode')}")
if st and st.get("estop"):                 # 이전 시험 잔재 정리
    send(c, "cmd", dict(cmd=P.CMD_CLEAR_ESTOP_REQUEST))
    time.sleep(0.5); local_reset(); time.sleep(1.5); st = state(c)

print("\n── 1. 비상정지 ──")
send(c, "cmd", dict(cmd=P.CMD_ESTOP, reason="실기 검증"))
time.sleep(1.5)
st = state(c)
check("정지 래치", bool(st and st.get("estop")), f"stage={st and st.get('estop_stage')}")
check("단계 latched", st and st.get("estop_stage") == "latched")

print("\n── 2. 관제 승인만으로는 안 풀린다 ──")
for _ in range(10):
    send(c, "cmd", dict(cmd=P.CMD_CLEAR_ESTOP_REQUEST))
    time.sleep(0.05)
time.sleep(1.5)
st = state(c)
check("승인 10회 반복 후에도 정지 유지", bool(st and st.get("estop")),
      f"stage={st and st.get('estop_stage')}")
check("단계가 '현장 확인 대기'", st and st.get("estop_stage") == "awaiting_local")
check("현장 확인이 남았다고 보고", st and st.get("needs_local_ok") is True)

print("\n── 3. 링크로 온 '현장 확인'은 거부된다 ──")
send(c, "cmd", dict(cmd=P.CMD_LOCAL_RESET))
time.sleep(1.2)
st = state(c)
check("원격 local_reset 으로는 안 풀림", bool(st and st.get("estop")),
      f"stage={st and st.get('estop_stage')}")

print("\n── 4. 기체에서 누르면 풀린다 ──")
local_reset()
time.sleep(2.0)
st = state(c)
check("현장 확인 후 해제", bool(st and not st.get("estop")),
      f"stage={st and st.get('estop_stage')}")
check("해제해도 임무는 일시정지 유지", bool(st and st.get("paused")))

print("\n── 5. 정비 모드 ──")
send(c, "cmd", dict(cmd=P.CMD_SET_SERVICE_MODE, mode="maintenance"))
time.sleep(1.5)
st = state(c)
check("정비 모드 진입", st and st.get("service_mode") == "maintenance")
send(c, "cmd", dict(cmd=P.CMD_SET_SERVICE_MODE, mode=""))
time.sleep(1.2)
st = state(c)
check("정상 운용 복귀", st and not st.get("service_mode"))

c.close()
print("\n" + "=" * 74)
n_ok, n = sum(res), len(res)
print(f"{n_ok}/{n} 통과")
sys.exit(0 if n_ok == n else 1)
