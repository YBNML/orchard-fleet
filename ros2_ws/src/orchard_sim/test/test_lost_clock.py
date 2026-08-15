"""상실 타이머의 시계 선택 — `lost_clock` (wall | sim).

    cd ros2_ws/src/orchard_sim && python3 -m pytest test/ -v
    (ROS 를 소싱해야 map_localizer 가 import 된다 — 아니면 skip)

왜 필요한가 (2026-08-15, 스펙 ④ T4 수정 라운드 2)
    `lost_critical_s` 는 '이만큼 위치를 못 잡으면 세운다'는 문턱인데, 판정
    대상('횡단 한 번')은 **시뮬 시간의 사건**이다. 문턱을 벽시계로 재면 판정이
    RTF 에 딸려 흔들린다 — 1대 실측 RTF 0.374 에 맞춘 벽시계 상수는 2대 동시
    운용에서 RTF 가 조금만 떨어져도 같은 횡단이 갑자기 문턱을 넘는다.

    그래서 시계를 고를 수 있게 했다. **기본은 wall — 계단식 월드의 기존 실측
    (150초)이 전부 벽시계 초라, 기본값이 바뀌면 그 월드의 판정이 통째로
    달라진다.** 이 파일이 그 불변을 고정한다.

`RosSensors.rates()` 가 수신율에 대해 세운 이중 시계 관례와 같은 모양이다.
"""
import pytest

ml = pytest.importorskip("orchard_sim.map_localizer",
                         reason="ROS 미소싱 — map_localizer 를 import 할 수 없다")


# ── 순수 선택 로직 ──────────────────────────────────────────────────────────
def test_wall_mode_ignores_sim_clock():
    """기본(wall)에서는 sim 시각이 무엇이든 단조시계를 쓴다 — 계단식 불변."""
    assert ml.lost_clock_now("wall", 3000.0, 400000.0) == (400000.0, "wall")
    assert ml.lost_clock_now("wall", 0.0, 12.5) == (12.5, "wall")


def test_sim_mode_uses_ros_clock():
    assert ml.lost_clock_now("sim", 3000.0, 400000.0) == (3000.0, "sim")


def test_sim_mode_falls_back_to_wall_before_clock_arrives():
    """/clock 이 아직이면(sim<=0) 벽시계로 폴백한다 — rates() 와 같은 방향."""
    assert ml.lost_clock_now("sim", 0.0, 400000.0) == (400000.0, "wall")
    assert ml.lost_clock_now("sim", -1.0, 400000.0) == (400000.0, "wall")


# ── 시계 전환 시 기준점 재설정 ──────────────────────────────────────────────
class _Clock:
    def __init__(self, sec):
        self.sec = sec

    @property
    def nanoseconds(self):
        return int(self.sec * 1e9)


class _Stub:
    """`_lost_t` 가 실제로 만지는 것만 갖춘 최소 대역."""

    def __init__(self, mode, sim_s):
        self.lost_clock = mode
        self._lost_src = None
        self._sim = sim_s
        self.last_ok_t = None
        self.last_anchor_t = None
        self.warned = []

    def get_clock(self):
        return type("C", (), {"now": lambda _s=None: _Clock(self._sim)})()

    def get_logger(self):
        stub = self

        class L:
            @staticmethod
            def warning(m):
                stub.warned.append(m)
        return L

    _lost_t = ml.MapLocalizer._lost_t


def test_lost_t_anchors_on_first_call():
    s = _Stub("sim", 3000.0)
    t = s._lost_t()
    assert t == 3000.0 and s._lost_src == "sim"
    assert s.last_ok_t == 3000.0 and s.last_anchor_t == 3000.0
    assert s.warned == []                      # 첫 호출은 전환이 아니다


def test_lost_t_resets_reference_when_clock_switches():
    """sim → wall 폴백에서 기준점을 안 옮기면 '수십만 초 상실'로 즉시 격상된다."""
    s = _Stub("sim", 3000.0)
    s._lost_t()
    s.last_ok_t = 2900.0                       # 100 시뮬초 전에 마지막 보정
    s._sim = 0.0                               # /clock 두절 → 벽시계 폴백
    t = s._lost_t()
    assert s._lost_src == "wall" and t > 1000.0
    assert s.last_ok_t == t and s.last_anchor_t == t   # 경과 0 으로 재설정
    assert len(s.warned) == 1                  # 전환은 로그로 남긴다


def test_wall_mode_never_switches():
    s = _Stub("wall", 3000.0)
    a = s._lost_t()
    s._sim = 9999.0
    s.last_ok_t = a - 100.0
    b = s._lost_t()
    assert s._lost_src == "wall" and s.warned == []
    assert s.last_ok_t == a - 100.0            # 기준점을 건드리지 않는다
    assert b >= a


def test_sim_elapsed_drives_the_judgement():
    """sim 선택 시 경과는 **시뮬 초**로 잰다 — 벽시계가 얼마나 흘렀든 무관."""
    s = _Stub("sim", 1000.0)
    s._lost_t()
    s.last_ok_t = 1000.0
    s._sim = 1000.0 + 219.0
    assert s._lost_t() - s.last_ok_t == pytest.approx(219.0)   # 220 문턱 아래
    s._sim = 1000.0 + 221.0
    assert s._lost_t() - s.last_ok_t == pytest.approx(221.0)   # 문턱 초과
