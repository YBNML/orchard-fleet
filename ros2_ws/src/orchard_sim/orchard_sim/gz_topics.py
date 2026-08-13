"""gz ↔ ROS 토픽 이름 한 장 — 로봇 인스턴스별로 만든다 (다중 로봇, 2026-08-14).

`sim/models/scout_mini_mid70/model.sdf` 에서 `<topic>`·`<odom_topic>`·`<tf_topic>`·
`<frame_id>` 를 전부 걷어냈다. gz-sim 은 그 태그가 있으면 값을 **리터럴 전역
토픽**으로 쓰고, 없을 때만 엔티티 이름으로 스코프한 기본 토픽을 만든다 — 태그를
그대로 둔 채 로봇 2대를 올리면 `/cmd_vel`·`/odom`·`/tf`·`/livox/points_raw` 가 한
토픽에 뒤섞였다(2026-08-13 실측). 지금은 gz 기본값에 맡기고, **이 모듈이 그
스코프 이름을 ROS 쪽 `/scout0N/…` 로 환원한다.**

실측한 gz 기본 이름 (gz-sim 8.11, 2로봇 스모크, 2026-08-14):

    /model/<로봇>/cmd_vel                      DiffDrive 입력
    /model/<로봇>/odometry                     DiffDrive 오도메트리
    /model/<로봇>/tf                           DiffDrive TF
    /model/<로봇>/pose                         PosePublisher 참값
    /world/<월드>/model/<로봇>/joint_state
    /world/<월드>/model/<로봇>/link/<링크>/sensor/<센서>/<접미>

TF 프레임은 gz 가 `<로봇>/odom` → `<로봇>/base_link` 로 채운다(실측). 그래서 ROS
쪽 TF 트리는 **하나**로 두고(`/tf` 전역) 프레임 이름에만 로봇 접두를 붙인다:

    map → scout01/odom → scout01/base_link → scout01/livox_frame …

왜 이 모듈이 따로 있나: 이름 규칙이 런치·노드·검증 도구 세 군데에 흩어지면 로봇을
한 대 더 붙일 때마다 세 곳이 서로 다르게 틀어진다. 여기가 유일한 출처다.

주의: 링크·센서 이름은 model.sdf 의 것을 그대로 쓴다. model.sdf 에서 이름을
바꾸면 아래 표도 같이 바꿔야 한다 (SDF 를 파싱해 자동으로 알아내지 않는 이유는,
런치가 SDF 파일 경로를 모르는 자리에서도 이 표를 써야 하기 때문이다).
"""
from __future__ import annotations

from collections import namedtuple

#: model.sdf 의 (링크, 센서) 이름과 gz 가 붙이는 접미사
LIDAR_LINK, LIDAR_SENSOR = "livox_frame", "livox_mid70"
IMU_LINK, IMU_SENSOR = "imu_link", "imu"
NAVSAT_LINK, NAVSAT_SENSOR = "navsat_link", "navsat"
CAM_LINKS = {            # ROS 쪽 짧은 이름 → model.sdf 의 (링크, 센서)
    "left": ("cam_canopy_left", "cam_canopy_left"),
    "right": ("cam_canopy_right", "cam_canopy_right"),
    "forward": ("cam_forward", "cam_forward"),
}

GZ_TO_ROS, ROS_TO_GZ = "[", "]"

#: 브리지 한 줄. gz 토픽 하나를 ROS 이름 하나로 잇는다.
Bridge = namedtuple("Bridge", "gz ros ros_type gz_type direction")


def _arg(b: Bridge) -> str:
    """ros_gz_bridge parameter_bridge 의 인자 문법으로 옮긴다.

    `<토픽>@<ROS타입>[<gz타입>` 이면 gz→ROS, `]` 면 ROS→gz. 이 문법은 ROS 쪽
    토픽 이름을 따로 못 준다 — 그래서 gz 이름으로 노드를 띄우고 remap 으로
    원하는 ROS 이름을 씌운다(`remaps()`). 기존 gt_bridge 가 쓰던 방식과 같다.
    """
    return f"{b.gz}@{b.ros_type}{b.direction}{b.gz_type}"


def args(bridges) -> list:
    return [_arg(b) for b in bridges]


def remaps(bridges) -> list:
    return [(b.gz, b.ros) for b in bridges]


def model_topic(robot: str, leaf: str) -> str:
    """`/model/<로봇>/<leaf>` — 월드 이름에 안 매인 플러그인 기본 토픽."""
    return f"/model/{robot}/{leaf}"


def sensor_topic(world: str, robot: str, link: str, sensor: str, leaf: str) -> str:
    """`/world/<월드>/model/<로봇>/link/<링크>/sensor/<센서>/<leaf>`."""
    return f"/world/{world}/model/{robot}/link/{link}/sensor/{sensor}/{leaf}"


def ns_topic(ns: str, leaf: str) -> str:
    """ROS 쪽 이름. ns 가 비면 전역(`/leaf`) — 단일 로봇 옛 배치용 탈출구."""
    ns = (ns or "").strip("/")
    return f"/{ns}/{leaf}" if ns else f"/{leaf}"


def frame(robot: str, name: str) -> str:
    """TF 프레임 이름. gz DiffDrive 가 붙이는 접두와 **같은 규칙**이어야 한다."""
    robot = (robot or "").strip("/")
    return f"{robot}/{name}" if robot else name


def cloud_bridges(world: str, robot: str, ns: str) -> list:
    """점군만. **반드시 제 프로세스에 혼자 둔다.**

    parameter_bridge 는 단일 스레드 실행기라 한 프로세스 안의 모든 토픽이 한
    콜백 줄을 나눠 쓴다. 여기 흐르는 한 프레임은 19,200점 × 32 B ≈ 600 KB 이고,
    같은 프로세스에 200 Hz IMU·50 Hz TF·수백 Hz `/clock` 이 끼면 점군이 굶는다.
    실측(2026-08-14, 같은 월드·같은 RTF 0.36):

        /clock·IMU·TF·점군 한 프로세스   점군 3.6 Hz(sim)   ← 옛 bridge_core.yaml
        /clock 만 분리                    점군 7.5 Hz(sim)
        점군까지 분리                     아래 게이트 수치 참조

    사양은 10 Hz 다. 굶주림은 RTF 저하와 증상이 같아서(둘 다 "라이다가 느리다")
    오래 RTF 탓으로 오해받았다 — 분리해 두지 않으면 다시 그렇게 된다.
    """
    return [
        Bridge(sensor_topic(world, robot, LIDAR_LINK, LIDAR_SENSOR, "scan/points"),
               ns_topic(ns, "livox/points_raw/points"),
               "sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked", GZ_TO_ROS),
    ]


def core_bridges(world: str, robot: str, ns: str) -> list:
    """주행·매핑에 실제로 쓰는 것 중 점군을 뺀 나머지 (전부 작은 메시지).

    브리지를 켤수록 RTF 가 떨어진다(ros_gz #368) — 실제로 소비하는 것만 켤 것.

    `/tf` 는 **전역 그대로** 둔다 — 프레임 이름에 이미 로봇 접두가 붙어 있어
    한 트리에 모여도 섞이지 않고, 오히려 로봇 간 상대 위치를 그 한 트리에서
    바로 물어볼 수 있다.
    """
    return [
        Bridge(sensor_topic(world, robot, IMU_LINK, IMU_SENSOR, "imu"),
               ns_topic(ns, "imu"),
               "sensor_msgs/msg/Imu", "gz.msgs.IMU", GZ_TO_ROS),
        Bridge(model_topic(robot, "odometry"), ns_topic(ns, "odom"),
               "nav_msgs/msg/Odometry", "gz.msgs.Odometry", GZ_TO_ROS),
        Bridge(model_topic(robot, "cmd_vel"), ns_topic(ns, "cmd_vel"),
               "geometry_msgs/msg/Twist", "gz.msgs.Twist", ROS_TO_GZ),
        Bridge(sensor_topic(world, robot, NAVSAT_LINK, NAVSAT_SENSOR, "navsat"),
               ns_topic(ns, "navsat"),
               "sensor_msgs/msg/NavSatFix", "gz.msgs.NavSat", GZ_TO_ROS),
        Bridge(f"/world/{world}/model/{robot}/joint_state",
               ns_topic(ns, "joint_states"),
               "sensor_msgs/msg/JointState", "gz.msgs.Model", GZ_TO_ROS),
        # DiffDrive 의 <로봇>/odom → <로봇>/base_link
        Bridge(model_topic(robot, "tf"), "/tf",
               "tf2_msgs/msg/TFMessage", "gz.msgs.Pose_V", GZ_TO_ROS),
    ]


def camera_bridges(world: str, robot: str, ns: str) -> list:
    """확인·데이터셋 생성 전용. 1920×1080 RGB 2대를 10 Hz 로 흘리면 초당 약
    62 MB 가 ROS 로 들어오고 RTF 가 약 22% 떨어진다(2026-07-25 실측)."""
    out = []
    for short, (link, sensor) in CAM_LINKS.items():
        out.append(Bridge(sensor_topic(world, robot, link, sensor, "image"),
                          ns_topic(ns, f"cam/{short}/image"),
                          "sensor_msgs/msg/Image", "gz.msgs.Image", GZ_TO_ROS))
        if short != "forward":
            out.append(Bridge(sensor_topic(world, robot, link, sensor, "camera_info"),
                              ns_topic(ns, f"cam/{short}/camera_info"),
                              "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo",
                              GZ_TO_ROS))
    return out


def ground_truth_bridge(robot: str, ns: str) -> Bridge:
    """PosePublisher 참값 → `/<ns>/gz_ground_truth`.

    월드의 `pose/info` 는 5,478 엔트리라 브리지하면 안 되고, `dynamic_pose/info`
    는 가볍지만 ros_gz 변환기가 frame_id 를 안 채운다(2026-07-25 실측). 모델의
    PosePublisher 는 header 를 제대로 채워 주므로 이쪽을 쓴다 — child_frame_id
    가 곧 로봇 인스턴스 이름이라 수신측이 자기 것을 골라낼 수 있다.
    """
    return Bridge(model_topic(robot, "pose"), ns_topic(ns, "gz_ground_truth"),
                  "tf2_msgs/msg/TFMessage", "gz.msgs.Pose_V", GZ_TO_ROS)


def clock_bridge() -> Bridge:
    """`/clock` 은 전역 하나뿐이다 — gz 인스턴스가 하나이므로 로봇 수와 무관하다.
    로봇 2번째 스택은 `clock:=false` 로 띄워 중복 발행을 피한다."""
    return Bridge("/clock", "/clock",
                  "rosgraph_msgs/msg/Clock", "gz.msgs.Clock", GZ_TO_ROS)
