"""정찰(work) 게이트·전통로 자동·work_stop — mission 기능을 가짜 ctx 로 구동한다.

①T7(test_mission_report.py)에 비슷한 FakeSafety/FakeCtx 전례가 있지만, 이
파일은 work 관련 시나리오만 다루므로 최소한으로 자체 정의한다. mk_mission 의
스텁 세부는 mission.py 의 setup()/on_command() 가 실제로 쓰는 ctx 인터페이스
(param·bb·safety.snapshot/set_paused·event·emit_cmd_result)를 그대로 흉내낸 것
— ①T7 스모크(test_mission_report.py 의 FakeCtx)와 같은 모양이다.

rows=10 을 쓰는 이유: control_agent 의 기본 과수원 기하(rows=10)와 맞춰,
"alleys 생략 → 전통로 자동"이 만드는 값이 range(9)(통로 0~8) 가 되게 하려는
것이다(가로줄 10개 = 통로 9개).
"""
import robomw.link.protocol as P
from robomw.core.base import Blackboard
from robomw.profiles.orchard.mission import DriveMission

PARAMS = dict(rows=10, trees_per_row=41, row_spacing=3.5, tree_spacing=1.5,
              headland=6.0, speed=0.7, turn_speed=0.5, wp_tol=0.5)


class FakeWork:
    def __init__(self): self.started = None; self.stopped = 0
    def start(self, t, p): self.started = (t, p); return True
    def stop(self): self.stopped += 1
    def status(self):
        from robomw.sdk.types import WorkStatus
        return WorkStatus(self.started is not None and not self.stopped, "scout", 0.5, "")


class FakeSafety:
    def __init__(self):
        self.paused = False
        self.estop = False

    def snapshot(self):
        return {"estop": self.estop}

    def set_paused(self, v):
        self.paused = bool(v)
        return not self.estop


class FakeCtx:
    """DriveMission.setup()/on_command() 이 실제로 쓰는 면만 흉내낸다."""

    def __init__(self, work_types):
        self.bb = Blackboard()
        self.bb.pose = (0.0, 0.0, 0.0)          # 측위 미준비(BUSY) 거부를 피한다
        self.bb.extra["sdk_work"] = FakeWork()
        self.bb.extra["work_types"] = work_types
        self.safety = FakeSafety()
        self.events = []
        self.results = []

    def param(self, name, default=None):
        return PARAMS.get(name, default)

    def event(self, kind, msg, level="info", **extra):
        self.events.append(dict(kind=kind, msg=msg, level=level, **extra))

    def emit_cmd_result(self, cmd_id, cmd, status, code="OK", data=None):
        if not cmd_id:
            return None
        res = P.make_cmd_result(cmd_id, cmd, status, code, data)
        self.results.append(res)
        return res


def mk_mission(work_types=("scout",)):
    ctx = FakeCtx(work_types)
    m = DriveMission()
    m.setup(ctx)
    return m, ctx


def test_scout_defaults_all_alleys():
    m, ctx = mk_mission()
    m.on_command(P.CMD_MISSION_START, {"work": {"type": "scout"}, "cmd_id": "w1"})
    assert m.mission is not None
    assert m.mission["alleys"] == list(range(9))
    assert ctx.bb.extra["sdk_work"].started[0] == "scout"


def test_unsupported_work_type_rejected():
    m, ctx = mk_mission(work_types=("scout",))
    m.on_command(P.CMD_MISSION_START, {"work": {"type": "spray"}, "cmd_id": "w2"})
    assert m.mission is None
    last = ctx.results[-1]
    assert last["status"] == "rejected" and last["code"] == "UNSUPPORTED"


def test_bad_speed_scale_rejected():
    m, ctx = mk_mission()
    m.on_command(P.CMD_MISSION_START,
                 {"work": {"type": "scout", "params": {"speed_scale": 3.0}}, "cmd_id": "w3"})
    assert m.mission is None and ctx.results[-1]["code"] == "BAD_PARAM"


def test_work_stop_keeps_mission():
    m, ctx = mk_mission()
    m.on_command(P.CMD_MISSION_START, {"work": {"type": "scout"}, "cmd_id": "w4"})
    m.on_command(P.CMD_WORK_STOP, {"cmd_id": "w5"})
    assert m.mission is not None                      # 주행은 계속
    assert ctx.bb.extra["sdk_work"].stopped == 1
