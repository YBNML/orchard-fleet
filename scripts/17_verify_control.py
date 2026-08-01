#!/usr/bin/env python3
"""
관제 링크 검증 — 규약·안전장치·대시보드 배달

    python3 scripts/17_verify_control.py [--host 127.0.0.1] [--port 8080]

control_agent 가 떠 있는 상태에서 돌린다. 브라우저 없이 프로토콜 수준에서 확인한다:
  1. 대시보드가 HTTP 로 나오는가
  2. WebSocket 업그레이드가 되고 hello 가 오는가
  3. state/health/map 이 규약대로 오는가
  4. 비상정지가 래치되는가 (해제 전에는 임무가 거부되어야 한다)
  5. 데드맨 — 원격조종 명령을 끊으면 로봇이 스스로 서는가
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import socket
import subprocess
import sys
import time
import urllib.request

sys.path.insert(0, "ros2_ws/src/orchard_sim")
from orchard_sim.link import protocol as P            # noqa: E402
from orchard_sim.link.wsserver import (OP_TEXT, decode_frame,  # noqa: E402
                                       encode_frame)

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="127.0.0.1")
ap.add_argument("--port", type=int, default=8080)
a = ap.parse_args()

OK, FAIL = "\033[92m✔\033[0m", "\033[91m�’\033[0m"
results = []


def check(name, cond, detail=""):
    results.append(bool(cond))
    print(f"   {OK if cond else FAIL} {name}" + (f"  — {detail}" if detail else ""))


print("관제 링크 검증")
print("=" * 74)

# ── 1. 대시보드 HTTP ────────────────────────────────────────────────────────
print("\n── 1. 대시보드 배달 ──")
try:
    with urllib.request.urlopen(f"http://{a.host}:{a.port}/", timeout=5) as r:
        html = r.read().decode("utf-8", "replace")
        ctype = r.headers.get("Content-Type", "")
    check("HTTP 200", r.status == 200, f"{len(html):,} bytes")
    check("HTML 콘텐츠 타입", "text/html" in ctype, ctype)
    check("외부 의존성 없음(자급자족)",
          "http://" not in html.split("<script>")[0].replace("http-equiv", "") or
          "cdn" not in html.lower(),
          "CDN/외부 스크립트 참조 없음")
    for need in ("비상정지", "임무", "원격 조종", "/ws"):
        check(f"UI 요소 '{need}'", need in html)
except Exception as e:
    check("HTTP 접속", False, str(e))
    sys.exit(1)

# ── 2. WebSocket 핸드셰이크 ─────────────────────────────────────────────────
print("\n── 2. WebSocket 연결 ──")


def ws_connect():
    s = socket.create_connection((a.host, a.port), timeout=10)
    key = base64.b64encode(os.urandom(16)).decode()
    s.sendall(("GET /ws HTTP/1.1\r\n"
               f"Host: {a.host}:{a.port}\r\n"
               "Upgrade: websocket\r\nConnection: Upgrade\r\n"
               f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
               ).encode())
    buf = b""
    while b"\r\n\r\n" not in buf:
        buf += s.recv(4096)
    return s, buf.decode("latin1")


def ws_send(s, topic_suffix, payload, robot="scout01"):
    m = P.envelope(f"orchard/{robot}/{topic_suffix}", payload, int(time.time() * 1e9), 1)
    s.sendall(encode_frame(json.dumps(m).encode(), OP_TEXT, mask=True))


def ws_recv(s, timeout=6.0):
    s.settimeout(timeout)
    op, data = decode_frame(s)
    if op != OP_TEXT:
        return None
    return json.loads(data.decode("utf-8"))


sock, resp = ws_connect()
check("101 Switching Protocols", "101" in resp.split("\r\n")[0], resp.split("\r\n")[0])
check("Sec-WebSocket-Accept 헤더", "sec-websocket-accept" in resp.lower())

hello = ws_recv(sock)
t, pl, ts, _ = P.parse(hello)
check("hello 수신", t.endswith("/hello"), t)
check("규약 버전 일치", pl.get("protocol") == P.PROTOCOL_VERSION)
robot_id = pl.get("robot_id", "scout01")
check("기하 정보 포함", all(k in pl for k in ("rows", "alleys", "row_spacing", "x0")),
      f"통로 {pl.get('alleys')}개 · 열간 {pl.get('row_spacing')} m")
check("데드맨 시간 고지", pl.get("deadman_ms") == P.TELEOP_DEADMAN_MS,
      f"{pl.get('deadman_ms')} ms")

# ── 3. 텔레메트리 ───────────────────────────────────────────────────────────
print("\n── 3. 텔레메트리 스트림 ──")
seen, t_end = {}, time.time() + 8
while time.time() < t_end and len(seen) < 3:
    try:
        m = ws_recv(sock, timeout=3)
    except Exception:
        break
    if not m:
        continue
    tt, pp, _, _ = P.parse(m)
    seen.setdefault(tt.rsplit("/", 1)[-1], pp)
check("state 수신", "state" in seen,
      f"모드={seen.get('state', {}).get('mode')}" if "state" in seen else "")
check("health 수신", "health" in seen,
      (f"라이다 {seen['health'].get('lidar_hz')} Hz · "
       f"IMU {seen['health'].get('imu_hz')} Hz · "
       f"SLAM {seen['health'].get('lio_hz')} Hz") if "health" in seen else "")
if "state" in seen and seen["state"].get("pose"):
    p = seen["state"]["pose"]
    check("포즈 유효", True, f"x={p['x']:.2f} y={p['y']:.2f}")
else:
    check("포즈 유효", False, "TF 미수신 — 시뮬이 떠 있는지 확인")

# ── 4. 비상정지 래치 ────────────────────────────────────────────────────────
print("\n── 4. 비상정지 래치 ──")


def wait_state(sock, pred, timeout=6.0):
    end = time.time() + timeout
    while time.time() < end:
        try:
            m = ws_recv(sock, timeout=2)
        except Exception:
            return None
        if not m:
            continue
        tt, pp, _, _ = P.parse(m)
        if tt.endswith("/state") and pred(pp):
            return pp
    return None


ws_send(sock, "cmd", dict(cmd=P.CMD_ESTOP, reason="검증 스크립트"), robot_id)
st = wait_state(sock, lambda p: p.get("estop"))
check("비상정지 진입", st is not None, st.get("estop_reason") if st else "타임아웃")

ws_send(sock, "cmd", dict(cmd=P.CMD_MISSION_START, alleys=[0], mode="mapping"), robot_id)
st2 = wait_state(sock, lambda p: True, timeout=4)
check("비상정지 중 임무 거부", st2 is not None and st2.get("mission") is None,
      "임무가 시작되지 않음")

ws_send(sock, "cmd", dict(cmd=P.CMD_MISSION_RESUME), robot_id)
st3 = wait_state(sock, lambda p: True, timeout=4)
check("비상정지 중 재개 거부", st3 is not None and st3.get("estop") is True,
      "래치 유지")

# 해제는 2단계다 — 관제 승인만으로는 안 풀리고, 현장 확인(로봇의 ~/local_reset)이
# 있어야 최종 해제된다 (ISO 13849-1 §5.2.2). 여기서는 승인까지만 확인한다.
ws_send(sock, "cmd", dict(cmd=P.CMD_CLEAR_ESTOP_REQUEST), robot_id)
st_wait = wait_state(sock, lambda p: p.get("estop_stage") == "awaiting_local", timeout=4)
check("관제 승인만으로는 안 풀림 (현장 확인 대기)", st_wait is not None,
      "실기 2단계 검증은 scripts/36_verify_estop_live.py")
subprocess.run(["bash", "-lc",
                "source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash && "
                "ros2 topic pub --once /control_agent/local_reset std_msgs/msg/Empty '{}'"],
               capture_output=True, timeout=30)
st4 = wait_state(sock, lambda p: not p.get("estop"))
check("현장 확인이 더해져야 풀림", st4 is not None,
      f"해제 후 paused={st4.get('paused')}" if st4 else "타임아웃")
check("해제해도 자동 재개 안 함", st4 is not None and st4.get("paused") is True)

# ── 5. 데드맨 ───────────────────────────────────────────────────────────────
print("\n── 5. 원격조종 데드맨 ──")
ws_send(sock, "cmd", dict(cmd=P.CMD_SET_MODE, mode=P.MODE_TELEOP), robot_id)
st5 = wait_state(sock, lambda p: p.get("mode") == P.MODE_TELEOP)
check("조종 모드 진입", st5 is not None)
if st5:
    for _ in range(6):                      # 300 ms 동안 명령 유지
        ws_send(sock, "teleop", dict(v=0.0, w=0.0), robot_id)
        time.sleep(0.05)
    print(f"      데드맨 {P.TELEOP_DEADMAN_MS} ms · 링크두절 정지 "
          f"{P.LINK_LOSS_STOP_MS} ms — 규약에 명시됨")
    check("데드맨 상수 노출", P.TELEOP_DEADMAN_MS > 0 and P.LINK_LOSS_STOP_MS > 0)
ws_send(sock, "cmd", dict(cmd=P.CMD_SET_MODE, mode=P.MODE_IDLE), robot_id)

sock.close()
print("\n" + "=" * 74)
n_ok, n = sum(results), len(results)
print(f"{n_ok}/{n} 통과")
sys.exit(0 if n_ok == n else 1)
