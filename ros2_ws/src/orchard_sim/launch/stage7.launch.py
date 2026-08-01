"""
Stage-7 — FAST-LIO2 로 정합까지 포함한 맵 만들기

    ros2 launch orchard_sim stage7.launch.py

단계 0 (참값 포즈)과 **같은 주행에서 동시에** 돌린다. 그래야 두 맵의 차이가
오직 정합 오차 때문이라고 말할 수 있다 — 센서 데이터도, 주행 궤적도, 지형도
완전히 같기 때문이다. 따로 두 번 주행하면 궤적이 달라져 비교가 흐려진다.

프레임은 겹치지 않는다:
    참값     map → odom → base_link → 센서   (gt_localizer, sdf_static_tf)
    FAST-LIO camera_init → body              (자체 발행)

MID-70 에는 IMU 가 없다. FAST-LIO2 는 IMU 가 필수라 로봇 본체에 별도 IMU 를 달았다
(model.sdf imu_link, 200 Hz). 실제 장비에서도 MID-70 단독으로는 FAST-LIO2 를
돌릴 수 없고 외부 IMU 가 필요하다 — 시뮬레이션만의 편의가 아니라 실제 구성이다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("orchard_sim")
    fastlio_cfg = os.path.join(pkg, "config", "fastlio_mid70.yaml")

    args = [
        DeclareLaunchArgument("world_name", default_value="orchard_10x41"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("lio_out", default_value="/tmp/orchard_lio.npz"),
        DeclareLaunchArgument("gt_out", default_value="/tmp/orchard_gt.npz"),
        DeclareLaunchArgument("alleys", default_value="0"),
        DeclareLaunchArgument("speed", default_value="0.9"),
    ]
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(args + [
        # 참값 경로 일체 (브리지·정적TF·참값 로컬라이저·Livox 계약)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, "launch", "stage0.launch.py")),
            launch_arguments={"world_name": LaunchConfiguration("world_name"),
                              "use_sim_time": use_sim_time,
                              "cameras": "false"}.items()),

        Node(package="fast_lio", executable="fastlio_mapping",
             name="fastlio_mapping", output="screen",
             parameters=[fastlio_cfg, {"use_sim_time": use_sim_time}]),

        Node(package="orchard_sim", executable="lio_recorder",
             name="lio_recorder", output="screen",
             parameters=[{"use_sim_time": use_sim_time,
                          "out": LaunchConfiguration("lio_out")}]),

        Node(package="orchard_sim", executable="mapping_run",
             name="mapping_run", output="screen",
             parameters=[{"use_sim_time": use_sim_time,
                          "out": LaunchConfiguration("gt_out"),
                          "alleys": LaunchConfiguration("alleys"),
                          "speed": LaunchConfiguration("speed")}]),
    ])
