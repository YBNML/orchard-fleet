"""임무 완료 보고·work 수용·측위 미준비 거부 (ROS 없이 기능만 돌린다)."""
import time

import robomw.link.protocol as P
from robomw.core.base import Blackboard
from robomw.profiles.orchard.mission import DriveMission

PARAMS = dict(rows=4, trees_per_row=5, row_spacing=3.5, tree_spacing=1.5,
              headland=6.0, speed=0.7, turn_speed=0.5, wp_tol=0.5)


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
    """Context 의 기능이 실제로 쓰는 면만 흉내낸다."""

    def __init__(self):
        self.bb = Blackboard()
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


def mk(pose=(0.0, 0.0, 0.0)):
    ctx = FakeCtx()
    ctx.bb.pose = pose
    f = DriveMission()
    f.setup(ctx)
    return f, ctx


def drive_to_end(f, ctx, limit=200):
    """웨이포인트마다 그 자리로 순간이동시켜 임무를 끝까지 돌린다."""
    for _ in range(limit):
        m = f.mission
        if m is None:
            return True
        wp = m["wps"][m["idx"]] if m["idx"] < len(m["wps"]) else None
        if wp is not None:
            p = ctx.bb.pose
            ctx.bb.pose = (p[0] if wp["x"] is None else wp["x"],
                           p[1] if wp.get("y") is None else wp["y"], 0.0)
        f.tick(time.monotonic())
    return False


def test_pose_none_rejects_busy():
    """측위가 서기 전 mission_start 는 BUSY 거부 — 임무를 만들지 않는다."""
    f, ctx = mk(pose=None)
    assert f.on_command(P.CMD_MISSION_START, dict(alleys=[0], cmd_id="m1")) is True
    assert f.mission is None
    assert ctx.results[-1]["status"] == "rejected"
    assert ctx.results[-1]["code"] == "BUSY"
    assert ctx.results[-1]["data"]["reason"] == "측위 미준비"
    assert ctx.events[-1]["kind"] == "rejected"          # 옛 클라이언트 화면도 그대로


def test_estop_and_bad_alleys_are_rejected_with_codes():
    f, ctx = mk()
    f.ctx.safety.estop = True
    f.on_command(P.CMD_MISSION_START, dict(alleys=[0], cmd_id="m2"))
    assert ctx.results[-1]["code"] == "ESTOPPED" and f.mission is None
    f.ctx.safety.estop = False
    f.on_command(P.CMD_MISSION_START, dict(alleys=[99], cmd_id="m3"))
    assert ctx.results[-1]["code"] == "BAD_PARAM" and f.mission is None


def test_work_schema_checked_and_stored():
    f, ctx = mk()
    bad = dict(alleys=[0], cmd_id="m4", work={"type": "laser"})
    assert f.on_command(P.CMD_MISSION_START, bad) is True
    assert ctx.results[-1]["code"] == "BAD_PARAM" and f.mission is None
    assert "work" not in ctx.bb.extra                    # 불합격은 저장하지 않는다

    good = dict(alleys=[0], cmd_id="m5",
                work={"type": "spray", "params": {"speed_scale": 0.5}})
    assert f.on_command(P.CMD_MISSION_START, good) is True
    assert f.mission is not None
    assert ctx.bb.extra["work"] == good["work"]


def test_completion_report_keys_and_values():
    f, ctx = mk()
    f.on_command(P.CMD_MISSION_START, dict(alleys=[0, 1], cmd_id="m6"))
    ctx.bb.extra["mission_interventions"] = 2            # 호스트가 센 개입
    assert drive_to_end(f, ctx)
    res = ctx.results[-1]
    assert res["status"] == "completed" and res["cmd_id"] == "m6"
    assert res["cmd"] == P.CMD_MISSION_START
    d = res["data"]
    assert set(d) == set(P.MISSION_REPORT_KEYS)          # 계약이 정한 다섯 칸
    assert d["alleys_done"] == [0, 1] and d["coverage"] == 1.0
    assert d["interventions"] == 2 and d["duration_s"] >= 0.0


def test_distance_ignores_teleport_jumps():
    """0.5 m 넘는 한 틱 변위는 재초기화·텔레포트다 — 주행거리에 안 넣는다."""
    f, ctx = mk()
    f.on_command(P.CMD_MISSION_START, dict(alleys=[0], cmd_id="m7"))
    for x in (0.0, 0.2, 0.4, 30.0, 30.2):                # 30 m 점프 하나
        ctx.bb.pose = (x, -3.0, 0.0)
        f._track_distance(f.mission, ctx.bb.pose)
    assert abs(f._report["distance_m"] - 0.6) < 1e-6


def test_cancelled_mission_makes_no_report():
    f, ctx = mk()
    f.on_command(P.CMD_MISSION_START, dict(alleys=[0], cmd_id="m8"))
    f.on_command(P.CMD_MISSION_CANCEL, {})
    assert f.mission is None and f._report is None
    assert not [r for r in ctx.results if r["status"] == "completed"]
