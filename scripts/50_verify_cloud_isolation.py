#!/usr/bin/env python3
"""지도 점군 공급 고장이 밀착 정지 판단을 데려가지 않는지 검증

    python3 scripts/50_verify_cloud_isolation.py

무엇을 확인하나
    2026-08-11 이관(T7)에서 지도 격자 기능이 갖고 있던 점군 구독이 사라지고,
    control_agent 의 `_on_cloud` **한 콜백 안에서** 두 가지를 하게 됐다.

        1. 지도 공급  — cloud_world.feed(msg)   (관제 화면용, 안전과 무관)
        2. 밀착 판단  — sensors.feed_cloud(msg) → 코앞이 막혔으면 세운다

    예전에는 1번이 남의 콜백이라 거기서 무슨 일이 나든 2번은 제 갈 길을
    갔다. 합치면서 그 격리가 사라졌다 — 1번이 예외를 던지면 2번은 아예
    실행되지 않고, 그 예외가 구독 콜백 밖으로 나가면 **노드가 죽는다**
    (T6 의 `_write_cmd` 사고가 정확히 그 모양이었다). 하필 로봇이 무언가에
    코를 박고 있을 때 그 판단이 사라지면, 바퀴는 계속 돈다.

    점군 파싱은 밖에서 온 자료를 다룬다: 필드 이름이 없으면 KeyError,
    길이가 안 맞으면 reshape 오류가 난다. '설마'가 아니라 '언젠가'다.

어떻게 확인하나
    ROS 없이 `ControlAgent._on_cloud` 를 **함수로** 불러 그 안의 분기만
    본다(실기 기동 불필요). 지도 공급이 터지는 상태로 만들고,
        · 밀착 정지가 그대로 발동하는가 (정지 호출 + OBSTACLE_FRONT 경보)
        · 예외가 콜백 밖으로 새지 않는가 (= 노드가 살아남는가)
        · 고장이 조용하지 않은가 (3회 한도 경고 로그)
        · 정상일 때 동작은 예전 그대로인가
    를 본다. 마지막으로 어댑터(RosCloudWorld)에 실제로 결손 메시지와
    터지는 sink 를 먹여 같은 결론을 한 번 더 확인한다.
"""
from __future__ import annotations

import os
import sys
import time
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, "ros2_ws/src/orchard_sim")

from orchard_sim.adapters.ros_cloud import RosCloudWorld     # noqa: E402
from orchard_sim.control_agent import ControlAgent  # noqa: E402
from robomw.core.base import Blackboard                       # noqa: E402
from robomw.core.safety import SafetyArbiter                  # noqa: E402
from robomw.link import protocol as P                         # noqa: E402

res = []


def check(name, ok, detail=""):
    res.append(bool(ok))
    print(f"   {'✔' if ok else '✘'} {name}" + (f"  — {detail}" if detail else ""))


class Logger:
    def __init__(self):
        self.warns = []

    def warn(self, m):
        self.warns.append(m)

    def info(self, m):
        pass


class Drive:
    def __init__(self):
        self.stops = 0

    def stop(self):
        self.stops += 1


def make_agent(feed):
    """`_on_cloud` 가 쓰는 면만 갖춘 가짜 self. 코어 부품은 진짜를 쓴다."""
    a = SimpleNamespace()
    a.cloud_world = SimpleNamespace(feed=feed)
    a.sensors = SimpleNamespace(feed_cloud=lambda msg: (0.4, 0.95))  # 코앞 밀착
    a.bb = Blackboard()
    a.bb.extra["mode"] = P.MODE_MISSION
    a.safety = SafetyArbiter()
    a.drive = Drive()
    a.events = []
    a.event = lambda kind, msg, level="info", **kw: a.events.append(
        dict(kind=kind, msg=msg, level=level, **kw))
    a._blocked_since = time.monotonic() - 3.0    # 이미 2초 넘게 막혀 있었다
    a._cloud_map_errs = 0
    a._log = Logger()
    a.get_logger = lambda: a._log
    return a


def boom(msg):
    raise KeyError("x")                          # 점군에 x 필드가 없다


print("지도 점군 공급 고장 격리 검증")

print("── 1. 지도 공급이 터져도 밀착 정지는 산다 ──")
a = make_agent(boom)
try:
    ControlAgent._on_cloud(a, object())
    raised = None
except Exception as e:                            # noqa: BLE001
    raised = e
check("예외가 콜백 밖으로 새지 않는다 (노드 생존)", raised is None,
      f"샌 예외: {raised!r}" if raised else "")
check("밀착 판단이 실행됐다 (블랙보드 갱신)",
      getattr(a.bb, "near_frac", None) == 0.95
      and getattr(a.bb, "clearance", None) == 0.4,
      f"clearance={getattr(a.bb, 'clearance', None)} "
      f"near_frac={getattr(a.bb, 'near_frac', None)}")
check("스스로 일시정지했다", a.safety.paused is True)
check("바퀴를 세웠다", a.drive.stops == 1, f"stop 호출 {a.drive.stops}회")
ev = [e for e in a.events if e.get("code") == "OBSTACLE_FRONT"]
check("개입 큐로 경보가 나갔다 (OBSTACLE_FRONT/critical)",
      bool(ev) and ev[0]["level"] == "critical",
      ev[0]["msg"] if ev else "없음")
check("고장이 조용하지 않다 (경고 로그)", len(a._log.warns) == 1,
      a._log.warns[0] if a._log.warns else "없음")

print("── 2. 반복 고장은 3회까지만 로그 (초당 10프레임을 뒤덮지 않게) ──")
a2 = make_agent(boom)
for _ in range(6):
    a2._blocked_since = time.monotonic() - 3.0
    a2.safety.set_paused(False)
    ControlAgent._on_cloud(a2, object())
check("경고는 3건에서 멈춘다", len(a2._log.warns) == 3, f"{len(a2._log.warns)}건")
check("정지 판단은 6번 다 살아 있다", a2.drive.stops == 6,
      f"stop 호출 {a2.drive.stops}회")

print("── 3. 정상 경로 회귀 (지도 공급이 멀쩡할 때) ──")
seen = []
a3 = make_agent(lambda msg: seen.append(msg))
ControlAgent._on_cloud(a3, object())
check("지도 공급이 호출된다", len(seen) == 1)
check("밀착 정지 동작은 예전 그대로", a3.drive.stops == 1 and a3.safety.paused)
check("경고 로그 없음", not a3._log.warns)

print("── 4. 솎인 프레임(None)에서도 지도는 받는다 ──")
seen4 = []
a4 = make_agent(lambda msg: seen4.append(msg))
a4.sensors = SimpleNamespace(feed_cloud=lambda msg: None)   # 3프레임 솎기
ControlAgent._on_cloud(a4, object())
check("지도 공급 호출됨", len(seen4) == 1)
check("솎인 프레임은 판단을 건드리지 않는다",
      a4.drive.stops == 0 and not a4.safety.paused)

print("── 5. 어댑터에 실제 결손 메시지·터지는 sink 를 먹인다 ──")


def cloud(fields=("x", "y", "z"), n=300):
    pts = np.zeros((n, 3), dtype=np.float32)
    pts[:, 0] = np.linspace(1.0, 5.0, n)
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="livox"),
        fields=[SimpleNamespace(name=f, offset=4 * i) for i, f in enumerate(fields)],
        width=n, height=1, point_step=12, data=pts.tobytes())


# 라이다 광학 원점의 지상 높이 — model.sdf livox_frame z 와 같아야 한다.
# 2026-08-15 상부 하이브리드(스펙 ④ §3)에서 마스트 0.645 → 아치 상단 0.80.
# 하드코딩을 남기면 model.sdf 가 움직일 때마다 이 검증이 조용히 옛 기하를 재현한다.
SENSOR_Z = float(os.environ.get("LIDAR_Z", "0.80"))

tr = SimpleNamespace(transform=SimpleNamespace(
    translation=SimpleNamespace(x=0.0, y=0.0, z=SENSOR_Z),
    rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
node = SimpleNamespace(_tf_buffer=SimpleNamespace(lookup_transform=lambda *a, **k: tr),
                       get_logger=lambda: Logger())
bb = Blackboard()
bb.pose = (0.0, 0.0, 0.0)
cw = RosCloudWorld(node, bb)

got = []
bb.extra["cloud_sinks"] = [lambda pts, z: got.append((pts.shape, z))]
cw.feed(cloud())
check("정상 점군은 sink 로 흘러간다", len(got) == 1 and got[0][0][1] == 3,
      str(got[:1]))

bb.extra["cloud_sinks"] = [lambda pts, z: (_ for _ in ()).throw(ValueError("sink 고장"))]
try:
    cw.feed(cloud())
    sink_raised = None
except Exception as e:                            # noqa: BLE001
    sink_raised = e
check("sink 예외는 어댑터가 잡는다", sink_raised is None, repr(sink_raised))

# 결손 메시지(z 필드 없음)는 어댑터 안에서 터진다 — 그것을 호스트가 삼키는지가
# 이 검증의 핵심이다. 어댑터 단독으로는 던지는 것이 정상이다(조용한 오작동 금지).
bb.extra["cloud_sinks"] = [lambda pts, z: got.append(1)]
try:
    cw.feed(cloud(fields=("x", "y", "i")))
    adapter_raised = None
except Exception as e:                            # noqa: BLE001
    adapter_raised = e
check("결손 메시지는 어댑터에서 예외가 된다 (조용히 넘어가지 않는다)",
      isinstance(adapter_raised, KeyError), repr(adapter_raised))

a5 = make_agent(lambda msg: cw.feed(msg))
try:
    ControlAgent._on_cloud(a5, cloud(fields=("x", "y", "i")))
    host_raised = None
except Exception as e:                            # noqa: BLE001
    host_raised = e
check("그 예외를 호스트가 삼킨다 (노드 생존)", host_raised is None, repr(host_raised))
check("결손 메시지에도 밀착 정지는 발동한다",
      a5.drive.stops == 1 and a5.safety.paused)

print("\n" + "=" * 74)
print(f"{sum(res)}/{len(res)} 통과")
sys.exit(0 if all(res) else 1)
