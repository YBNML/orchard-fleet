"""유지보수 기능(self_test) — FakeDiag 로 기능 로직만 검증한다.

mk_feature 스텁은 ②T1(test_work_mission.py 의 mk_mission)·①T7
(test_mission_report.py 의 FakeCtx) 전례를 따라 이 파일에 최소한으로 자체
정의한다 — MaintenanceFeature.setup()/on_command() 가 실제로 쓰는 ctx 면
(bb·event·emit_cmd_result)만 흉내낸다.
"""
import robomw.link.protocol as P
from robomw.core.base import Blackboard
from robomw.features.maintenance import MaintenanceFeature
from robomw.sdk.types import SelfTestItem


class FakeDiag:
    def self_test(self, items):
        return [SelfTestItem("lidar", True, "9.8 Hz"), SelfTestItem("imu", False, "42 Hz")]

    def blackbox_dump(self, window_s):
        raise NotImplementedError


class FakeCtx:
    """MaintenanceFeature.on_command 이 실제로 쓰는 면만 흉내낸다."""

    def __init__(self, diag, in_mission):
        self.bb = Blackboard()
        self.bb.extra["sdk_diag"] = diag
        # mission 존재의 bb 대리 — mission.py 가 매 틱 갱신하는
        # bb.extra["mission_status"](임무 없음 → None) 를 그대로 흉내낸다.
        if in_mission:
            self.bb.extra["mode"] = P.MODE_MISSION
            self.bb.extra["mission_status"] = dict(alleys=[0], mode="mapping")
        else:
            self.bb.extra["mode"] = P.MODE_IDLE
            self.bb.extra["mission_status"] = None
        self.events = []
        self.results = []

    def event(self, kind, msg, level="info", **extra):
        self.events.append(dict(kind=kind, msg=msg, level=level, **extra))

    def emit_cmd_result(self, cmd_id, cmd, status, code="OK", data=None):
        if not cmd_id:
            return None
        res = P.make_cmd_result(cmd_id, cmd, status, code, data)
        self.results.append(res)
        return res


def mk_feature(diag=None, in_mission=False):
    ctx = FakeCtx(diag, in_mission)
    f = MaintenanceFeature()
    f.setup(ctx)
    return f, ctx


def test_self_test_reports_items():
    f, ctx = mk_feature(diag=FakeDiag())
    f.on_command(P.CMD_SELF_TEST, {"cmd_id": "s1"})
    last = ctx.results[-1]
    assert last["status"] == "completed" and last["data"]["all_ok"] is False
    assert {i["name"] for i in last["data"]["items"]} == {"lidar", "imu"}


def test_self_test_busy_while_driving():
    f, ctx = mk_feature(diag=FakeDiag(), in_mission=True)
    f.on_command(P.CMD_SELF_TEST, {"cmd_id": "s2"})
    assert ctx.results[-1]["code"] == "BUSY"


def test_self_test_missing_diag_adapter():
    """어댑터 미배선(sdk_diag=None) — 조용히 성공한 척하지 않는다."""
    f, ctx = mk_feature(diag=None)
    f.on_command(P.CMD_SELF_TEST, {"cmd_id": "s3"})
    last = ctx.results[-1]
    assert last["status"] == "failed" and last["code"] == "INTERNAL"
