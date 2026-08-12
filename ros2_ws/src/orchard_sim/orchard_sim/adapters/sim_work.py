"""작업(work) SDK 어댑터 — 시뮬레이터에는 켜고 끌 하드웨어가 없다.

정찰(scout)은 이동 자체가 작업이라 start/stop 이 조작할 실제 장치가 없다.
그래서 이 어댑터는 상태 플래그만 들고 있고, 진행률은 새로 계산하지 않는다 —
mission(robomw.profiles.orchard.mission.DriveMission)이 매 틱 갱신하는
bb.extra["mission_coverage"] 를 그대로 돌려준다. 진행률 산식을 두 곳에 두면
언젠가 갈라진다 — 그 산식은 이미 완료 보고(coverage)가 갖고 있다.

control_agent(호스트)가 bb.extra["sdk_work"] 에 이 인스턴스를 꽂는다
(bb.extra["cloud_sinks"] 전례 — 어댑터를 블랙보드로 건네 코어(robomw)가
ROS 를 몰라도 되게 한다. 방향은 반대다: cloud_sinks 는 기능이 어댑터에게
콜백을 건네지만, 여기서는 호스트가 어댑터를 기능에게 건넨다).
"""
from __future__ import annotations

from robomw.sdk.interfaces import Work
from robomw.sdk.types import WorkStatus


class SimWork(Work):
    """시뮬레이터용 Work 구현. 상태만 지키고 실행할 것이 없다."""

    def __init__(self, bb):
        self._bb = bb
        self._active = False
        self._type = ""
        self._scale = 1.0

    def start(self, type_: str, params: dict) -> None:
        self._active = True
        self._type = type_
        self._scale = float((params or {}).get("speed_scale", 1.0))

    def stop(self) -> None:
        self._active = False

    def status(self) -> WorkStatus:
        progress = float(self._bb.extra.get("mission_coverage", 0.0))
        return WorkStatus(self._active, self._type, progress, "")
