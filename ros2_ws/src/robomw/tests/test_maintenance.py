"""유지보수 기능(self_test·relocalize) — 가짜 어댑터로 기능 로직만 검증한다.

mk_feature 스텁은 ②T1(test_work_mission.py 의 mk_mission)·①T7
(test_mission_report.py 의 FakeCtx) 전례를 따라 이 파일에 최소한으로 자체
정의한다 — MaintenanceFeature.setup()/on_command() 가 실제로 쓰는 ctx 면
(bb·event·emit_cmd_result)만 흉내낸다.
"""
import math

import robomw.link.protocol as P
from robomw.core.base import Blackboard
from robomw.features.maintenance import MaintenanceFeature
from robomw.sdk.types import SelfTestItem

# 실기 기본 기하 (control_agent 파라미터 기본값과 같은 값) — hello 의
# site.geometry 와 같은 사전을 흉내낸다. x0=-15.75, col_len=60.0
GEOM = dict(rows=10, alleys=9, row_spacing=3.5, x0=-15.75,
            col_len=60.0, headland=6.0)


class FakeDiag:
    def self_test(self, items):
        return [SelfTestItem("lidar", True, "9.8 Hz"), SelfTestItem("imu", False, "42 Hz")]

    def blackbox_dump(self, window_s):
        raise NotImplementedError


class FakeLocalizer:
    """Localizer 중 relocalize 가 쓰는 두 면(reinit·diagnostics)만 흉내낸다."""

    def __init__(self, ok=True, quality=0.72):
        self.ok = ok
        self.quality = quality
        self.calls = []                 # reinit 으로 받은 Pose 들

    def reinit(self, pose):
        self.calls.append(pose)
        return self.ok

    def diagnostics(self):
        return dict(quality=self.quality, gate="", n_fix=12)


class FakeSafety:
    """SafetyArbiter 중 relocalize 가 보는 면(snapshot()["gate"])만 흉내낸다.

    gate="" 는 "마지막 중재에서 속도 요청이 통과했다"(= 지금 구동 중),
    "idle"/"paused"/"deadman" 등은 "0 을 냈다"는 뜻이다 — core/safety.py 참조.
    """

    def __init__(self, gate="idle"):
        self._gate = gate

    def snapshot(self):
        return dict(estop=False, paused=False, gate=self._gate)


class FakeCtx:
    """MaintenanceFeature.on_command 이 실제로 쓰는 면만 흉내낸다."""

    def __init__(self, diag, in_mission, loc=None, geom=GEOM,
                 mode=None, gate="idle"):
        self.bb = Blackboard()
        self.bb.extra["sdk_diag"] = diag
        self.bb.extra["sdk_loc"] = loc
        self.safety = FakeSafety(gate)
        if geom is not None:
            self.bb.extra["site_geom"] = dict(geom)
        # mission 존재의 bb 대리 — mission.py 가 매 틱 갱신하는
        # bb.extra["mission_status"](임무 없음 → None) 를 그대로 흉내낸다.
        if in_mission:
            self.bb.extra["mode"] = P.MODE_MISSION
            self.bb.extra["mission_status"] = dict(alleys=[0], mode="mapping")
        else:
            self.bb.extra["mode"] = P.MODE_IDLE
            self.bb.extra["mission_status"] = None
        if mode is not None:
            self.bb.extra["mode"] = mode
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


def mk_feature(diag=None, in_mission=False, loc=None, geom=GEOM,
               mode=None, gate="idle"):
    ctx = FakeCtx(diag, in_mission, loc, geom, mode, gate)
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


# ── relocalize ──────────────────────────────────────────────────────────────
def test_relocalize_grid_south_end():
    """{alley:3, end:"south"} → (-3.5, -31.5, +π/2).

    x = x0 + (k+0.5)·S = -15.75 + 3.5·3.5 = -3.5
    y = -(col_len/2 + 1.5) = -(30 + 1.5) = -31.5   (남단이라 부호 −)
    yaw = +π/2 — 남단에 선 로봇은 통로 안쪽(북)을 본다
    """
    loc = FakeLocalizer()
    f, ctx = mk_feature(loc=loc)
    f.on_command(P.CMD_RELOCALIZE, {"cmd_id": "r1", "alley": 3, "end": "south"})
    assert len(loc.calls) == 1
    p = loc.calls[0]
    assert (round(p.x, 6), round(p.y, 6)) == (-3.5, -31.5)
    assert math.isclose(p.yaw, math.pi / 2)
    last = ctx.results[-1]
    assert last["status"] == "completed" and last["data"]["quality"] == 0.72


def test_relocalize_grid_north_end():
    """북단은 y 부호와 요가 뒤집힌다 — (alley 0) → (-14.0, +31.5, -π/2)."""
    loc = FakeLocalizer()
    f, ctx = mk_feature(loc=loc)
    f.on_command(P.CMD_RELOCALIZE, {"cmd_id": "r2", "alley": 0, "end": "north"})
    p = loc.calls[0]
    assert (round(p.x, 6), round(p.y, 6)) == (-14.0, 31.5)
    assert math.isclose(p.yaw, -math.pi / 2)


# ── site_geom v2 (SDK 계약 개정, additive) ──────────────────────────────────
# alley_centers_x / row_span_y — 불균일 통로·열별 길이·비대칭 블록을 표현한다.
# 실사 정사영상 농장(스펙 ④) 값을 축약해 썼다: 통로 간격이 4.75~5.25 로
# 불균일하고, 남단이 y **최댓값**이며(farm.json axes_note: world +y = 지리적 남),
# 통로마다 남단 y 가 다르다.
GEOM_V2 = dict(
    rows=4, alleys=3, row_spacing=4.9903, x0=-66.9167,
    col_len=141.109, headland=2.043,
    alley_centers_x=[-64.4167, -59.5417, -54.5417],
    row_span_y=[[34.920, -89.078],       # 통로 0 (남단 y 최대)
                [37.575, -97.948],       # 통로 1
                [39.198, -97.025]],      # 통로 2
)


def test_grid_pose_prefers_nonuniform_arrays():
    """배열이 있으면 균일 격자를 **무시**한다 — 통로 2 남단.

    x = alley_centers_x[2] = -54.5417 (균일 격자였다면 -54.4409 → 다른 값)
    남단이 y 최댓값이므로 정지선은 +y 로 물러나고(39.198+1.5), 요는 안쪽(-y).
    """
    loc = FakeLocalizer()
    f, ctx = mk_feature(loc=loc, geom=GEOM_V2)
    f.on_command(P.CMD_RELOCALIZE, {"cmd_id": "v1", "alley": 2, "end": "south"})
    p = loc.calls[0]
    assert (round(p.x, 4), round(p.y, 4)) == (-54.5417, 40.698)
    assert math.isclose(p.yaw, -math.pi / 2)


def test_grid_pose_north_end_on_nonuniform():
    """같은 통로의 북단 — y 최솟값 쪽으로 물러나고 요가 뒤집힌다."""
    loc = FakeLocalizer()
    f, ctx = mk_feature(loc=loc, geom=GEOM_V2)
    f.on_command(P.CMD_RELOCALIZE, {"cmd_id": "v2", "alley": 2, "end": "north"})
    p = loc.calls[0]
    assert (round(p.x, 4), round(p.y, 4)) == (-54.5417, -98.525)
    assert math.isclose(p.yaw, math.pi / 2)


def test_grid_pose_boundary_alleys_use_own_span():
    """경계 통로 0 과 마지막 통로가 **각자의** y 구간을 쓴다(공용 col_len 아님)."""
    loc = FakeLocalizer()
    f, _ = mk_feature(loc=loc, geom=GEOM_V2)
    f.on_command(P.CMD_RELOCALIZE, {"cmd_id": "v3", "alley": 0, "end": "south"})
    f.on_command(P.CMD_RELOCALIZE, {"cmd_id": "v4", "alley": 2, "end": "south"})
    assert round(loc.calls[0].y, 4) == 36.420      # 34.920 + 1.5
    assert round(loc.calls[1].y, 4) == 40.698      # 39.198 + 1.5
    assert loc.calls[0].y != loc.calls[1].y


def test_grid_pose_shared_span_pair_applies_to_all_alleys():
    """row_span_y 를 [남,북] 한 쌍으로 주면 전 통로 공통으로 쓴다."""
    loc = FakeLocalizer()
    geom = dict(GEOM_V2, row_span_y=[50.0, -50.0])
    f, _ = mk_feature(loc=loc, geom=geom)
    f.on_command(P.CMD_RELOCALIZE, {"cmd_id": "v5", "alley": 0, "end": "south"})
    f.on_command(P.CMD_RELOCALIZE, {"cmd_id": "v6", "alley": 2, "end": "south"})
    assert round(loc.calls[0].y, 4) == round(loc.calls[1].y, 4) == 51.5


def test_grid_pose_falls_back_to_uniform_grid():
    """신규 키가 없으면 종전 균일 격자 그대로 — 계단식 경로 불변 보증."""
    loc = FakeLocalizer()
    f, _ = mk_feature(loc=loc, geom=GEOM)          # v1 기하(배열 없음)
    f.on_command(P.CMD_RELOCALIZE, {"cmd_id": "v7", "alley": 3, "end": "south"})
    p = loc.calls[0]
    assert (round(p.x, 6), round(p.y, 6)) == (-3.5, -31.5)
    assert math.isclose(p.yaw, math.pi / 2)


def test_grid_pose_rejects_wrong_length_arrays():
    """길이가 안 맞으면 조용히 폴백하지 않고 실패한다(무음 오정위 금지)."""
    loc = FakeLocalizer()
    geom = dict(GEOM_V2, alley_centers_x=[-64.4167, -59.5417])   # 3 이어야 함
    f, ctx = mk_feature(loc=loc, geom=geom)
    f.on_command(P.CMD_RELOCALIZE, {"cmd_id": "v8", "alley": 1, "end": "south"})
    assert loc.calls == []
    last = ctx.results[-1]
    assert last["status"] == "rejected" and last["code"] == "BAD_PARAM"
    assert "alley_centers_x" in last["data"]["reason"]


def test_grid_pose_alley_range_from_alleys_key():
    """통로 번호 범위는 alleys 가 정한다 — 배열이 있어도 동일."""
    loc = FakeLocalizer()
    f, ctx = mk_feature(loc=loc, geom=GEOM_V2)
    f.on_command(P.CMD_RELOCALIZE, {"cmd_id": "v9", "alley": 3, "end": "south"})
    assert loc.calls == []
    assert ctx.results[-1]["code"] == "BAD_PARAM"


def test_relocalize_busy_while_driving():
    """주행 중에는 재초기화 불가 — 어댑터를 부르지도 않는다."""
    loc = FakeLocalizer()
    f, ctx = mk_feature(loc=loc, in_mission=True)
    f.on_command(P.CMD_RELOCALIZE, {"cmd_id": "r3", "alley": 3, "end": "south"})
    assert loc.calls == []
    last = ctx.results[-1]
    assert last["status"] == "rejected" and last["code"] == "BUSY"


def test_relocalize_busy_while_teleop():
    """텔레옵도 주행이다 — 재정위 대기 중에는 데드맨도 안 돌아 기체가 계속 간다.

    (리뷰 라운드 1 [High]: mode==MODE_MISSION 만 보던 게이트가 텔레옵을
    통과시켰다. 유휴가 아니면 거부한다.)
    """
    loc = FakeLocalizer()
    f, ctx = mk_feature(loc=loc, mode=P.MODE_TELEOP)
    f.on_command(P.CMD_RELOCALIZE, {"cmd_id": "r3t", "alley": 3, "end": "south"})
    assert loc.calls == []
    last = ctx.results[-1]
    assert last["status"] == "rejected" and last["code"] == "BUSY"
    assert last["data"]["state"] == "모드 teleop"


def test_relocalize_busy_while_velocity_passing():
    """모드가 유휴여도 중재가 속도 요청을 통과시키는 중이면 거부한다.

    gate=="" 는 안전 조정자가 마지막 중재에서 아무것도 막지 않았다는 뜻 —
    모드 표기가 늦는 창(mission_start 직후 50 ms)까지 이 두 번째 겹이 잡는다.
    """
    loc = FakeLocalizer()
    f, ctx = mk_feature(loc=loc, gate="")
    f.on_command(P.CMD_RELOCALIZE, {"cmd_id": "r3v", "alley": 3, "end": "south"})
    assert loc.calls == []
    assert ctx.results[-1]["code"] == "BUSY"


def test_relocalize_allowed_when_idle_and_gated():
    """유휴 + 중재가 0 을 내는 중이면 통과한다 (게이트가 과하게 막지 않는다)."""
    loc = FakeLocalizer()
    f, ctx = mk_feature(loc=loc, gate="paused")
    f.on_command(P.CMD_RELOCALIZE, {"cmd_id": "r3i", "alley": 3, "end": "south"})
    assert len(loc.calls) == 1
    assert ctx.results[-1]["status"] == "completed"


def test_relocalize_explicit_xy_yaw():
    """{x,y,yaw} 는 격자 변환 없이 그대로 간다."""
    loc = FakeLocalizer()
    f, ctx = mk_feature(loc=loc)
    f.on_command(P.CMD_RELOCALIZE,
                 {"cmd_id": "r4", "x": 1.5, "y": -2.25, "yaw": 0.3})
    p = loc.calls[0]
    assert (p.x, p.y, p.yaw) == (1.5, -2.25, 0.3)
    assert ctx.results[-1]["status"] == "completed"


def test_relocalize_timeout_reports_diagnostics():
    """어댑터가 False → failed(TIMEOUT) + 진단 스냅샷."""
    loc = FakeLocalizer(ok=False, quality=0.05)
    f, ctx = mk_feature(loc=loc)
    f.on_command(P.CMD_RELOCALIZE, {"cmd_id": "r5", "alley": 1, "end": "north"})
    last = ctx.results[-1]
    assert last["status"] == "failed" and last["code"] == "TIMEOUT"
    assert last["data"]["quality"] == 0.05        # 진단 스냅샷이 그대로 실린다


def test_relocalize_bad_payload():
    """통로 번호가 범위 밖이거나 좌표가 없으면 BAD_PARAM — 어댑터 미호출."""
    for pl in ({"cmd_id": "r6"},                       # 아무것도 없다
               {"cmd_id": "r6", "alley": 99, "end": "north"},
               {"cmd_id": "r6", "alley": 1, "end": "east"},
               {"cmd_id": "r6", "x": "abc", "y": 0.0, "yaw": 0.0}):
        loc = FakeLocalizer()
        f, ctx = mk_feature(loc=loc)
        f.on_command(P.CMD_RELOCALIZE, pl)
        assert loc.calls == [], pl
        last = ctx.results[-1]
        assert last["status"] == "rejected" and last["code"] == "BAD_PARAM", pl


def test_relocalize_missing_localizer_adapter():
    """어댑터 미배선(sdk_loc=None) — 조용히 성공한 척하지 않는다."""
    f, ctx = mk_feature(loc=None)
    f.on_command(P.CMD_RELOCALIZE, {"cmd_id": "r7", "alley": 1, "end": "north"})
    last = ctx.results[-1]
    assert last["status"] == "failed" and last["code"] == "INTERNAL"


def test_relocalize_without_geometry():
    """호스트가 기하를 안 얹었으면 격자 변환은 불가 — 하드코딩하지 않는다."""
    loc = FakeLocalizer()
    f, ctx = mk_feature(loc=loc, geom=None)
    f.on_command(P.CMD_RELOCALIZE, {"cmd_id": "r8", "alley": 1, "end": "north"})
    assert loc.calls == []
    last = ctx.results[-1]
    assert last["status"] == "failed" and last["code"] == "INTERNAL"
