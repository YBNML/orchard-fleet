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


def test_double_start_rejected_busy():
    """임무 진행 중 mission_start 가 다시 오면 예전처럼 조용히 교체하지 않고
    BUSY 로 거부해야 한다 — 기존 임무는 그대로 유지된다(스펙 ③ §5 이월 1건)."""
    m, ctx = mk_mission()
    m.on_command(P.CMD_MISSION_START, {"alleys": [0, 1], "cmd_id": "m1"})
    first = m.mission
    assert first is not None

    m.on_command(P.CMD_MISSION_START, {"alleys": [3, 4], "cmd_id": "m2"})

    assert m.mission is first                          # 교체되지 않았다
    last = ctx.results[-1]
    assert last["cmd_id"] == "m2"
    assert last["status"] == "rejected" and last["code"] == "BUSY"
    assert last["data"]["reason"] == "임무 진행 중"


# ── 웨이포인트 기하 (SDK 계약 개정 2026-08-15 T4 — site_geom v2) ────────────
#
# 실사 정사영상 농장은 통로 간격이 불균일하고(4.75~15.25 m) 열마다 길이가
# 다르며 블록이 y=0 대칭이 아니다. 균일 격자 계산이 그대로 남아 있으면
# 웨이포인트가 옆 열의 나무 앞에 서고 통로를 수십 m 지나쳐 나간다.

def _uniform_expect(i, k, S=3.5, x0=-15.75, col_l=60.0, HL=6.0):
    cx = x0 + (k + 0.5) * S
    y_lo, y_hi = -col_l / 2 - HL * 0.25, col_l / 2 + HL * 0.25
    return cx, (y_lo, y_hi) if i % 2 == 0 else (y_hi, y_lo)


def test_waypoints_uniform_grid_unchanged_without_site_geom():
    """site_geom 미배선 — 계단식 월드의 종전 값이 한 점도 안 바뀐다."""
    m, _ = mk_mission()
    wps = m.build_waypoints([0, 1, 2])
    assert [w["kind"] for w in wps] == [
        "traverse", "exit", "cross", "enter", "traverse",
        "exit", "cross", "enter", "traverse"]
    for i, k in enumerate([0, 1, 2]):
        cx, (y_start, y_end) = _uniform_expect(i, k)
        tv = [w for w in wps if w["kind"] == "traverse" and w["alley"] == k][0]
        assert abs(tv["x"] - cx) < 1e-9 and abs(tv["y"] - y_end) < 1e-9
    # 횡단선은 ±(col_l/2 + headland*0.6667) = ±34.0
    yc = 30.0 + 6.0 * 0.6667                       # cross_y 의 종전 식 그대로
    assert abs(wps[1]["y"] - yc) < 1e-9            # i=1 은 북단(+y)에서 건넌다
    assert abs(wps[5]["y"] + yc) < 1e-9            # i=2 는 남단(−y)에서


def test_waypoints_uniform_site_geom_matches_no_site_geom():
    """호스트가 균일 격자 site_geom 을 얹어도 값이 같아야 한다(계단식 실기 경로)."""
    m, ctx = mk_mission()
    base = m.build_waypoints([0, 1, 2])
    ctx.bb.extra["site_geom"] = dict(rows=10, alleys=9, row_spacing=3.5,
                                     x0=-15.75, col_len=60.0, headland=6.0)
    assert m.build_waypoints([0, 1, 2]) == base


def test_waypoints_use_nonuniform_arrays():
    """불균일 배열이 있으면 계산이 아니라 **그 값**을 쓴다."""
    m, ctx = mk_mission()
    ctx.bb.extra["site_geom"] = dict(
        rows=10, alleys=9, row_spacing=3.5, x0=-15.75, col_len=60.0, headland=6.0,
        alley_centers_x=[-64.4167, -59.5417] + [0.0] * 7,
        # 실사 농장 규약: 남단 = y 최댓값 (계단식과 부호 반대)
        row_span_y=[[34.92, -89.08], [37.57, -97.02]] + [[1.0, -1.0]] * 7,
        alley_cross_y=[[39.6, -91.1], [44.8, -99.99]] + [[2.0, -2.0]] * 7)
    wps = m.build_waypoints([0, 1])
    tv0 = wps[0]
    assert abs(tv0["x"] + 64.4167) < 1e-6
    # i=0 은 남단(y 최대)에서 출발해 북단으로 — 부호가 배열 순서에서 나온다
    assert tv0["y"] < 0 and abs(tv0["y"] - (-89.08 - 1.5)) < 1e-6
    assert wps[1]["kind"] == "exit" and abs(wps[1]["y"] - (-99.99)) < 1e-6
    assert abs(wps[2]["x"] + 59.5417) < 1e-6
    assert abs(wps[3]["y"] - (-97.02 - 1.5)) < 1e-6     # 통로 1 북단 진입(바깥으로 물림)
    assert abs(wps[4]["y"] - (37.57 + 1.5)) < 1e-6      # 통로 1 남단으로 훑는다


def test_waypoints_cross_line_is_outer_not_span():
    """횡단선은 row_span_y 가 아니라 alley_cross_y 에서 온다.

    통로의 '안쪽 끝'(row_span_y)으로 횡단선을 잡으면 사이 열의 끝 나무를
    들이받는다 — 그 정보는 row_span_y 에 없다(양쪽 통로 모두에서 '바깥').
    """
    m, ctx = mk_mission()
    geom = dict(rows=10, alleys=9, row_spacing=3.5, x0=-15.75, col_len=60.0,
                headland=6.0, row_span_y=[[34.92, -89.08], [37.57, -97.02]] + [[1.0, -1.0]] * 7,
                alley_cross_y=[[39.6, -91.1], [44.8, -99.99]] + [[2.0, -2.0]] * 7)
    ctx.bb.extra["site_geom"] = geom
    yc = m.build_waypoints([0, 1])[1]["y"]
    assert yc == -99.99                     # 통로 0·1 합성 = min(-91.1, -99.99)
    del geom["alley_cross_y"]               # 없으면 균일 폴백(±(30+4.0002))
    assert abs(m.build_waypoints([0, 1])[1]["y"] - (30.0 + 6.0 * 0.6667)) < 1e-9


def test_mission_rejected_on_malformed_site_geom():
    """배열 길이가 안 맞으면 조용히 균일 격자로 떨어지지 않고 거부한다."""
    m, ctx = mk_mission()
    ctx.bb.extra["site_geom"] = dict(rows=10, alleys=9, row_spacing=3.5, x0=-15.75,
                                     col_len=60.0, headland=6.0,
                                     alley_centers_x=[0.0, 1.0])   # 길이 2 ≠ 9
    m.on_command(P.CMD_MISSION_START, {"alleys": [0], "cmd_id": "bad1"})
    assert m.mission is None
    assert ctx.results[-1]["status"] == "rejected"
    assert ctx.results[-1]["code"] == "BAD_PARAM"


# ── C1 (리뷰 수정 1) — 비인접 전이의 횡단선 ─────────────────────────────────
#
# alley_cross_y[k] 는 통로 k 를 낀 두 열(k·k+1)의 바깥 끝이다. 인접 전이는 사이
# 열이 하나라 그것으로 충분하지만, **비인접 전이는 사이 열 전부**를 넘어야 한다.
# 아래 리터럴은 maps/orchard_real/farm.json 에서 유도한 실제 값이고, 식재 규약
# (`y0 = ceil(canopy_y0 / tree_spacing) * tree_spacing`, gen_world.build_farm_trees)
# 으로 그 열의 **첫 나무** y 를 재구성해 실제로 피하는지 단언한다.
import math as _math

_TS = 1.5
# 열 0~4 의 북단 캐노피 끝 (farm.json row_origins.y + headland_m)
_CANOPY_N = [-89.0781, -97.9482, -97.0247, -94.2542, -91.8300]
# 통로 0~3 의 북측 횡단선 (min(캐노피끝 두 열) − headland_m)
_CROSS_N = [-99.9912, -99.9912, -99.0677, -96.2972]
# 통로 19~21 의 북측 횡단선 · 열 19~22 의 북단 캐노피 끝
_CANOPY_N2 = [-52.8123, -50.3881, -42.4229, -39.8833]      # 열 19,20,21,22
_CROSS_N2 = [-54.8553, -52.4311, -44.4659]                 # 통로 19,20,21


def _first_trunk(canopy_y0, ts=_TS):
    """gen_world.build_farm_trees 의 식재 규약 — 전 열 공통 격자에 스냅."""
    return _math.ceil(canopy_y0 / ts) * ts


def _real_geom(centers, span, cross, alleys=26):
    """실사 농장 규약(남단 = y 최댓값)의 site_geom 조각."""
    return dict(rows=alleys + 1, alleys=alleys, row_spacing=4.9903, x0=-64.8739,
                col_len=141.0, headland=2.043,
                alley_centers_x=centers, row_span_y=span, alley_cross_y=cross)


def test_cross_line_clears_rows_on_nonadjacent_transition_0_to_3():
    """통로 0→3: 도착 통로의 값만 쓰면 넘어야 할 열 1 의 첫 나무보다 안쪽이다."""
    m, ctx = mk_mission()
    n = 26
    centers = [-64.4167 + 5.0 * i for i in range(n)]
    span = [[34.9199 + 2.5 * i, -89.0781 - 2.5 * i] for i in range(n)]
    cross = [[39.6178 + 2.5 * i, _CROSS_N[i] if i < 4 else -96.0 + 2.5 * i]
             for i in range(n)]
    ctx.bb.extra["site_geom"] = _real_geom(centers, span, cross)
    yc = m.build_waypoints([0, 3])[1]["y"]        # i=1 → 북단에서 건넌다
    assert yc == min(_CROSS_N)                    # 합성 = −99.9912
    # 넘는 열은 1·2·3. 전부 첫 나무보다 바깥(더 작은 y)이어야 한다.
    for r in (1, 2, 3):
        assert yc < _first_trunk(_CANOPY_N[r]), f"열 {r} 을 못 넘는다"
    # 회귀 방어 — 도착 통로 값만 쓰면 열 1 을 1.2 m 파고든다
    assert _CROSS_N[3] > _first_trunk(_CANOPY_N[1])


def test_cross_line_clears_rows_on_nonadjacent_transition_19_to_21():
    """통로 20(농로)을 건너뛰는 전이 — 브리프가 T7 에 지시한 바로 그 모양."""
    m, ctx = mk_mission()
    n = 26
    centers = [-64.4167 + 5.0 * i for i in range(n)]
    span = [[34.9199 + 2.5 * i, -89.0781 + 2.5 * i] for i in range(n)]
    cross = [[39.6178 + 2.5 * i, -99.9912 + 2.5 * i] for i in range(n)]
    for j, v in zip((19, 20, 21), _CROSS_N2):
        cross[j][1] = v
    ctx.bb.extra["site_geom"] = _real_geom(centers, span, cross)
    yc = m.build_waypoints([19, 21])[1]["y"]
    assert yc == min(_CROSS_N2)                   # 합성 = −54.8553
    for r, c in zip((20, 21), (_CANOPY_N2[1], _CANOPY_N2[2])):
        assert yc < _first_trunk(c), f"열 {r} 을 못 넘는다"
    assert _CROSS_N2[2] > _first_trunk(_CANOPY_N2[1])   # 현행은 5.03 m 안쪽


def test_cross_line_identity_on_adjacent_transitions_uniform_grid():
    """균일 격자(계단식)에서는 합성이 항등 — 인접·비인접, 오름·내림 전부."""
    m, ctx = mk_mission()
    yc = 30.0 + 6.0 * 0.6667
    ctx.bb.extra["site_geom"] = dict(rows=10, alleys=9, row_spacing=3.5, x0=-15.75,
                                     col_len=60.0, headland=6.0)
    for a, b in ((0, 1), (1, 0), (3, 4), (4, 3), (0, 3), (8, 5)):
        cs, cn = m.cross_lines(a, b)
        assert abs(cs + yc) < 1e-9 and abs(cn - yc) < 1e-9, (a, b)
    # 계단식 웨이포인트 전체도 종전과 동일 (site_geom 유무 무관)
    ctx.bb.extra.pop("site_geom")
    base = m.build_waypoints([0, 1, 2])
    ctx.bb.extra["site_geom"] = dict(rows=10, alleys=9, row_spacing=3.5, x0=-15.75,
                                     col_len=60.0, headland=6.0)
    assert m.build_waypoints([0, 1, 2]) == base


# ── m4 (수정 라운드 2) — farm.json 직독 속성 시험: 650쌍 전수 ────────────────
#
# 위 두 시험은 리뷰어가 짚은 두 사례를 고정한다. 이것은 **모든 전이**를 본다 —
# 26개 통로의 순서쌍 650개 각각에서 합성 횡단선이 그 사이의 열을 전부 넘는지.
# farm.json 이 바뀌면(열 추출 재실행) 이 시험이 먼저 깨진다.
def _load_farm():
    import json
    import pathlib
    here = pathlib.Path(__file__).resolve()
    for up in here.parents:
        p = up / "maps" / "orchard_real" / "farm.json"
        if p.exists():
            return json.loads(p.read_text())
    return None


# 횡단선이 넘는 열의 끝 나무에서 최소한 이만큼은 떨어져야 한다 [m].
# 로봇 반폭 0.29 + 수관 반경 여유 — '넘기는 넘었다'와 '안전하게 넘었다'는 다르다.
MIN_ROW_CLEARANCE_M = 1.0


def test_cross_lines_clear_every_row_for_all_650_transitions():
    farm = _load_farm()
    if farm is None:
        import pytest
        import warnings
        warnings.warn("maps/orchard_real/farm.json 없음 — 650쌍 속성 시험이 "
                      "건너뛰어졌다(조용한 무검증 방지 경고)", stacklevel=2)
        pytest.skip("maps/orchard_real/farm.json 없음 — 실사 농장 속성 시험 건너뜀")
    hl = float(farm["headland_m"])
    ts = float(farm["tree_spacing_m"])
    xs = [float(p[0]) for p in farm["row_origins"]]
    cy0 = [float(p[1]) + hl for p in farm["row_origins"]]                       # 북단 캐노피 끝
    cy1 = [float(p[1]) + float(L) - hl
           for p, L in zip(farm["row_origins"], farm["row_lengths_m"])]         # 남단 캐노피 끝
    n = int(farm["rows"])
    a_n = n - 1
    # control.launch.py::_farm_geom 과 **같은 식**으로 site_geom 을 만든다
    geom = dict(
        rows=n, alleys=a_n, row_spacing=float(farm["row_spacing_m"]),
        x0=-((n - 1) * float(farm["row_spacing_m"])) / 2.0,
        col_len=141.0, headland=hl,
        alley_centers_x=[(xs[k] + xs[k + 1]) / 2.0 for k in range(a_n)],
        row_span_y=[[min(cy1[k], cy1[k + 1]), max(cy0[k], cy0[k + 1])]
                    for k in range(a_n)],
        alley_cross_y=[[max(cy1[k], cy1[k + 1]) + hl, min(cy0[k], cy0[k + 1]) - hl]
                       for k in range(a_n)])
    m, ctx = mk_mission()
    ctx.bb.extra["site_geom"] = geom

    def first_trunk(r):     # gen_world.build_farm_trees 의 식재 규약
        return _math.ceil(cy0[r] / ts) * ts

    def last_trunk(r):
        y0 = first_trunk(r)
        return y0 + _math.floor((cy1[r] - y0) / ts) * ts

    bad = []
    for a in range(a_n):
        for b in range(a_n):
            if a == b:
                continue
            cs, cn = m.cross_lines(a, b)
            # a→b 로 건널 때 넘는 열은 min+1 … max (통로 k 는 열 k·k+1 사이)
            for r in range(min(a, b) + 1, max(a, b) + 1):
                # 넘었는가만이 아니라 **얼마나 여유 있게** 넘었는가까지 본다
                gap_s = cs - last_trunk(r)      # 남단은 y 가 큰 쪽이 바깥
                gap_n = first_trunk(r) - cn     # 북단은 y 가 작은 쪽이 바깥
                if gap_s < MIN_ROW_CLEARANCE_M:
                    bad.append((a, b, r, "남", round(gap_s, 3)))
                if gap_n < MIN_ROW_CLEARANCE_M:
                    bad.append((a, b, r, "북", round(gap_n, 3)))
    assert not bad, (f"여유 {MIN_ROW_CLEARANCE_M} m 미만인 전이 {len(bad)}건 "
                     f"(예: {bad[:3]})")


def test_property_suite_covers_650_ordered_pairs():
    """위 시험이 실제로 650쌍을 도는지 — 커버리지 자체를 고정한다."""
    farm = _load_farm()
    if farm is None:
        import pytest
        import warnings
        warnings.warn("maps/orchard_real/farm.json 없음 — 커버리지 단언 건너뜀",
                      stacklevel=2)
        pytest.skip("maps/orchard_real/farm.json 없음")
    a_n = int(farm["rows"]) - 1
    assert a_n * (a_n - 1) == 650
