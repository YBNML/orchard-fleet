"""핑 폭주 회귀 — ping 한 건이 전 관제 브로드캐스트 한 건을 만들면 안 된다.

    cd ros2_ws/src/orchard_sim && python3 -m pytest test/ -v
    (ROS 를 소싱해야 control_agent 가 import 된다 — 아니면 skip)

2026-08-13 실기 사고. 진단 도구(scripts/42_probe_robot_state.py)가 **프레임을
받을 때마다** ping 을 보냈다. ping 한 건은 로봇이 전 관제로 뿌리는 pong
이벤트 한 건을 만들고, 그 프레임이 도구에 도착하면 또 ping 을 보내게 된다 —
고리가 닫힌다. 루프백에서는 왕복이 마이크로초라 발산했다:

    로봇 발행 pong  1,056 → 2,241 → 3,317 → 6,673 건/초 (1초 간격, 실측)

관제(fleet_server)는 그 pong 을 건건이 **동기 DB INSERT** 로 적으므로 이벤트
루프가 굶고, 1 Hz 하트비트가 최대 5초까지 밀렸다. 로봇은 그 침묵을
링크두절(LINK_LOSS_STOP_MS=1.5초)로 읽어 임무를 세웠다.

도구는 고쳤지만(1 Hz), **로봇이 아무 클라이언트에게나 이렇게 증폭당하면 안
된다.** 여기서 고정하는 것은 로봇 쪽 상한이다.
"""
import time

import pytest

ca = pytest.importorskip("orchard_sim.control_agent",
                         reason="ROS 미소싱 — control_agent 를 import 할 수 없다")
P = ca.P


class _Stub:
    """_handle_core_cmd 의 ping 분기가 실제로 만지는 것만 갖춘 최소 대역.

    다른 분기(estop·해제·local_reset)는 전부 `if c == ...` 로 갈라져 있어
    ping 을 넣으면 실행되지 않는다 — 그래서 노드를 통째로 띄우지 않아도 된다
    (띄우면 8080 을 잡아 실기와 충돌한다).
    """

    def __init__(self):
        self._last_pong_emit = 0.0
        self.emitted = []

    def _emit(self, kind, payload):
        self.emitted.append((kind, payload))

    def _core_result(self, *a, **k):
        pass


def _ping(stub):
    return ca.ControlAgent._handle_core_cmd(stub, P.CMD_PING, {}, "1.2.3.4:1", "admin")


def test_ping_flood_does_not_become_broadcast_flood():
    """할 수 있는 한 빠르게 ping 해도 브로드캐스트는 상한에 묶인다."""
    stub = _Stub()
    t0 = time.monotonic()
    n = 0
    while time.monotonic() - t0 < 1.0:
        assert _ping(stub) is True
        n += 1

    assert n > 500, f"시험 전제 미달 — 폭주를 못 만들었다 (ping {n}회)"
    # 1초 동안 0.5초 간격이면 많아야 2~3건. 넉넉히 잡아도 5건을 넘으면 안 된다.
    assert len(stub.emitted) <= 5, (
        f"ping {n}회에 pong 브로드캐스트 {len(stub.emitted)}건 — 상한이 안 걸렸다")


def test_normal_heartbeat_rate_still_emits_pong():
    """정상 운영(관제 하트비트 1 Hz)은 그대로 통과해야 한다.

    상한이 너무 빡세면 관제 화면에서 pong 이 사라진다 — 고치려던 것보다
    나쁜 회귀다. 1초 전에 냈다면 지금 것은 반드시 나가야 한다.
    """
    stub = _Stub()
    stub._last_pong_emit = time.monotonic() - 1.0
    _ping(stub)
    assert len(stub.emitted) == 1
    assert stub.emitted[0][0] == "event"
    assert stub.emitted[0][1]["kind"] == "pong"


def test_pong_gap_is_below_link_budget():
    """상한은 링크두절 판정보다 **짧아야** 한다.

    상한이 1.5초를 넘으면, 관제가 pong 을 링크 확인에 쓰는 순간 상한 자체가
    링크두절을 만든다. 두 상수의 관계를 고정해 둔다.
    """
    assert ca.PONG_EVENT_MIN_GAP_S * 1000 < P.LINK_LOSS_STOP_MS
