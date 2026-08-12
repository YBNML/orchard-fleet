"""ROS 어댑터 — robomw SDK 인터페이스의 이 기체(시뮬 Scout Mini) 구현.

robomw 에는 ROS import 가 **한 건도 없다** (테스트로 강제한다 —
robomw/tests/test_no_ros_imports.py). 그래서 rclpy·메시지 타입·tf2 에 닿는
코드는 전부 이 디렉터리로 모인다. 기체나 전송을 갈아탈 때 손대는 곳이 여기
하나가 되게 하려는 것이다: 코어(안전·라우팅·계약)는 그대로 두고 어댑터만
새로 쓴다.

    Drive      → ros_drive.RosDrive       (/cmd_vel 발행)
    Localizer  → ros_sensors.RosSensors   (TF 포즈·로컬라이저 진단)
    Perception → ros_sensors.RosSensors   (점군 여유거리·밀착률)
    (점군 공급) → ros_cloud.RosCloudWorld  (PointCloud2 → map 프레임 점 배열)
    Work       → sim_work.SimWork         (하드웨어 없음 — 상태 플래그만)
"""
from orchard_sim.adapters.ros_cloud import RosCloudWorld
from orchard_sim.adapters.ros_drive import RosDrive
from orchard_sim.adapters.ros_sensors import RosSensors
from orchard_sim.adapters.sim_work import SimWork

__all__ = ["RosCloudWorld", "RosDrive", "RosSensors", "SimWork"]
