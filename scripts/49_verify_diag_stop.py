#!/usr/bin/env python3
"""로컬라이저 경보 → 자동 정지 실기 검증

    python3 scripts/49_verify_diag_stop.py

무엇을 확인하나
    control_agent 의 `_on_loc_diag` 에는 **관제 링크로는 절대 도달할 수 없는**
    분기가 있다 — 로컬라이저가 critical 진단을 올렸을 때 스스로 일시정지하고
    바퀴를 세우는 곳이다. 35·36 번은 안전 코어와 2단계 해제를 덮지만 진단
    토픽을 발행하지 않아 이 줄을 밟지 않는다. 실제로 2026-08-11 재배선에서
    이 분기의 정지 호출만 옛 이름(`_write_cmd`)으로 남았고, 그대로 뒀다면
    **밟는 순간 구독 콜백에서 AttributeError 가 나 에이전트가 죽는다** —
    그것도 하필 로봇이 미끄러지고 있는 순간에, 정지도 경보도 없이.

    그래서 여기서는 `/map_localizer/diagnostics` 에 must_stop 조건의 JSON 을
    직접 넣어 그 줄을 밟게 하고, 세 가지를 본다.
        1. 에이전트가 살아 있다 (주입 뒤에도 텔레메트리가 계속 온다)
        2. 스스로 일시정지했다 (paused=True + 'paused' 이벤트)
        3. 개입 큐로 경보가 나갔다 ('assistance' 이벤트에 code 가 실린다)
    'paused' 이벤트는 문제의 정지 호출 **다음 줄**에서 나간다 — 그 이벤트가
    도착했다는 것은 정지 호출이 예외 없이 끝났다는 뜻이다.

주입하는 두 가지가 다 must_stop 인 이유 (recovery 분기로 새지 않게)
    · severity=critical + code=LOST_LONG — 후진 재시도는 code 가 정확히
      TRACTION_LOSS 일 때만 걸리므로 임무 중이든 아니든 결정적으로 정지다.
    · code=TRACTION_LOSS — 임무가 없으면(phase 없음) 재시도 자격이 없어
      역시 정지로 간다. 임무 중이면 재시도로 샐 수 있어 그때는 건너뛴다.

이 스크립트는 로봇을 **세운다.** 끝에서 mission_resume 으로 되돌리지만,
임무 주행 중에는 돌리지 말 것.
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import socket
import sys
import threading
import time

sys.path.insert(0, "ros2_ws/src/orchard_sim")
import rclpy                                                       # noqa: E402
from rclpy.node import Node                                        # noqa: E402
from std_msgs.msg import String as StringMsg                       # noqa: E402

from orchard_sim.link import protocol as P                         # noqa: E402
from orchard_sim.link.wsserver import (OP_TEXT, decode_frame,      # noqa: E402
                                       encode_frame)

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="127.0.0.1")
ap.add_argument("--port", type=int, default=8080)
ap.add_argument("--robot", default="scout01")
# 로컬라이저 진단은 로봇 네임스페이스에 딸린다 (다중 로봇, 2026-08-14).
ap.add_argument("--topic", default=None,
                help="기본값: /<robot>/map_localizer/diagnostics")
a = ap.parse_args()
if not a.topic:
    a.topic = f"/{a.robot}/map_localizer/diagnostics"

OK, NG = "\033[92m✔\033[0m", "\033[91m✗\033[0m"
res = []


def check(name, cond, detail=""):
    res.append(bool(cond))
    print(f"   {OK if cond else NG} {name}" + (f"  — {detail}" if detail else ""))


# ── 관제 링크 ───────────────────────────────────────────────────────────────
s = socket.create_connection((a.host, a.port), timeout=8)
k = base64.b64encode(os.urandom(16)).decode()
s.sendall((f"GET /ws HTTP/1.1\r\nHost: {a.host}\r\nUpgrade: websocket\r\n"
           f"Connection: Upgrade\r\nSec-WebSocket-Key: {k}\r\n"
           "Sec-WebSocket-Version: 13\r\n\r\n").encode())
b = b""
while b"\r\n\r\n" not in b:
    b += s.recv(4096)
s.settimeout(0.3)


def send(cmd, **kw):
    """명령 하나. 링크가 끊겨 있으면 조용히 넘긴다 — 죽은 것은 check 가 잡는다.

    **여기서 예외를 올리지 않는 것이 중요하다.** 이 스크립트가 잡으려는 결함은
    '에이전트가 죽는 것'이라, 죽었을 때 스크립트가 역추적을 뿜고 끝나면 무엇이
    실패했는지가 오히려 안 보인다. 판정으로 남겨야 한다.
    """
    m = P.envelope(f"orchard/{a.robot}/cmd", dict(cmd=cmd, **kw),
                   time.time_ns(), 1)
    try:
        s.sendall(encode_frame(json.dumps(m).encode(), OP_TEXT, mask=True))
        return True
    except OSError:
        return False


def pump(secs, ping=True):
    """secs 동안 받은 (topic, payload). 링크두절 정지를 피하려고 ping 을 계속 넣는다."""
    out, end = [], time.time() + secs
    while time.time() < end:
        if ping and not send(P.CMD_PING):
            time.sleep(0.2)
            continue
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
        out.append((t, pl))
    return out


def last_state(msgs):
    st = [pl for t, pl in msgs if t.endswith("/state")]
    return st[-1] if st else None


def events(msgs):
    return [pl for t, pl in msgs if t.endswith("/event")]


# ── 진단 발행자 ─────────────────────────────────────────────────────────────
rclpy.init()
node = Node(f"verify_diag_stop_{a.robot}")
pub = node.create_publisher(StringMsg, a.topic, 10)
threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()


def inject(payload):
    m = StringMsg()
    m.data = json.dumps(payload, ensure_ascii=False)
    pub.publish(m)


print("로컬라이저 경보 → 자동 정지 실기 검증")
print("=" * 74)

msgs = pump(2.0)
st0 = last_state(msgs)
if st0 is None:
    print("state 수신 실패 — 에이전트가 떠 있는지 확인할 것")
    sys.exit(1)
check("에이전트 접속·상태 수신", True, f"mode={st0.get('mode')}")
if st0.get("paused"):
    send(P.CMD_MISSION_RESUME)                 # 정지 상태에서 시작하면 판정이 무의미
    msgs = pump(1.5)
    st0 = last_state(msgs) or st0
check("시작 시 일시정지 아님", not st0.get("paused"),
      f"paused={st0.get('paused')}")
in_mission = bool(st0.get("mission"))

# ── 1. critical 진단 → 자동 정지 ────────────────────────────────────────────
print("\n── 1. critical 진단 (severity=critical) → 스스로 선다 ──")
MSG1 = "검증 주입 — 장기 위치상실"
inject(dict(kind="assistance", code="LOST_LONG", severity="critical", msg=MSG1))
msgs = pump(2.5)
ev = events(msgs)
st1 = last_state(msgs)
check("에이전트 생존 (주입 뒤에도 텔레메트리가 온다)", st1 is not None,
      f"state {len([1 for t, _ in msgs if t.endswith('/state')])}건"
      if st1 is not None else "링크가 끊겼다 — 콜백에서 죽었을 것이다"
                              " (agent 로그의 역추적을 볼 것)")
check("스스로 일시정지", bool(st1 and st1.get("paused")),
      f"paused={st1 and st1.get('paused')} · gate={st1 and st1.get('gate')}")
pev = [e for e in ev if e.get("kind") == "paused"]
check("'paused' 이벤트 (정지 호출 다음 줄에서 나간다)", bool(pev),
      pev[0].get("msg") if pev else "없음")
aev = [e for e in ev if e.get("kind") == "assistance" and e.get("msg") == MSG1]
check("'assistance' 이벤트가 개입 큐로", bool(aev),
      f"code={aev[0].get('code') if aev else '없음'}")
check("code 가 실려 있다 (관제 라우팅 열쇠)",
      bool(aev) and aev[0].get("code") == "LOST_LONG")

# ── 2. resolved 경로 ────────────────────────────────────────────────────────
print("\n── 2. 해소 진단 → resolved 이벤트 ──")
inject(dict(kind="resolved", code="LOST_LONG", msg="검증 주입 — 해소"))
ev = events(pump(2.0))
rev = [e for e in ev if e.get("kind") == "resolved"]
check("'resolved' 이벤트", bool(rev), rev[0].get("msg") if rev else "없음")

# ── 3. TRACTION_LOSS (임무 없을 때는 재시도 자격이 없어 정지로 간다) ────────
print("\n── 3. TRACTION_LOSS → 정지 ──")
if in_mission:
    check("임무 주행 중이라 건너뜀 (재시도 분기로 샐 수 있다)", True)
else:
    send(P.CMD_MISSION_RESUME)                 # 다시 풀어 놓고 같은 줄을 한 번 더 밟는다
    pump(1.5)
    MSG3 = "검증 주입 — 궤도 슬립"
    inject(dict(kind="assistance", code="TRACTION_LOSS", msg=MSG3))
    msgs = pump(2.5)
    ev, st3 = events(msgs), last_state(msgs)
    check("에이전트 생존", st3 is not None)
    check("스스로 일시정지", bool(st3 and st3.get("paused")),
          f"paused={st3 and st3.get('paused')}")
    check("'paused' 이벤트", any(e.get("kind") == "paused" for e in ev))
    check("'assistance' 이벤트 code=TRACTION_LOSS",
          any(e.get("kind") == "assistance" and e.get("code") == "TRACTION_LOSS"
              and e.get("msg") == MSG3 for e in ev))

# ── 되돌리기 ────────────────────────────────────────────────────────────────
send(P.CMD_MISSION_RESUME)
st = last_state(pump(1.5))
check("검증 뒤 일시정지 해제 (원래 상태로 되돌림)",
      bool(st and not st.get("paused")), f"paused={st and st.get('paused')}")

s.close()
node.destroy_node()
rclpy.shutdown()

print("\n" + "=" * 74)
print(f"{sum(res)}/{len(res)} 통과")
sys.exit(0 if all(res) else 1)
