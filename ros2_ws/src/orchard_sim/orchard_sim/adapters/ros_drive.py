"""Drive SDK 구현 — /cmd_vel 발행. 중재(arbitrate) 결과만 이 클래스로 온다.

**퍼블리셔를 가진 곳은 여기 하나다.** 기능은 속도를 요청할 뿐이고, 요청은
SafetyArbiter.arbitrate 를 통과한 뒤에야 set_velocity 로 들어온다. 퍼블리셔가
여러 곳에 흩어지면 "지금 누가 로봇을 움직이고 있나"를 코드로 답할 수 없게
되고, 그 순간 비상정지가 보장이 아니라 희망이 된다.

한계값(v_max·w_max)은 마지막 그물이다. 중재 앞단에서 이미 기능별 상한
(teleop_max_v 등)이 걸리므로 정상 경로에서는 여기서 잘릴 일이 없다 — 잘린다면
기능이나 중재가 계약을 어긴 것이다. 그래도 자르는 이유: 계약 위반이 곧바로
바퀴 속도가 되는 것을 막는 값싼 보험이기 때문이다.
"""
from geometry_msgs.msg import Twist

from robomw.sdk.interfaces import Drive
from robomw.sdk.types import DriveLimits


class RosDrive(Drive):
    def __init__(self, node, v_max, w_max, topic="/cmd_vel"):
        self._pub = node.create_publisher(Twist, topic, 10)
        self._lim = DriveLimits(float(v_max), float(w_max))

    def set_velocity(self, v, w):
        t = Twist()
        t.linear.x = float(max(-self._lim.v_max, min(self._lim.v_max, v)))
        t.angular.z = float(max(-self._lim.w_max, min(self._lim.w_max, w)))
        self._pub.publish(t)

    def stop(self):
        self.set_velocity(0.0, 0.0)

    def limits(self):
        return self._lim
