"""
gz → ROS 2 브리지 + Livox 계약 변환을 함께 띄운다.

    ros2 launch orchard_sim livox_bridge.launch.py

    # 원형 FOV 마스크를 끄고 비교하고 싶을 때
    ros2 launch orchard_sim livox_bridge.launch.py apply_fov_mask:=false

use_sim_time 은 모든 노드에 반드시 켠다 — 부분 적용은 Nav2 시뮬 실패의 최다 원인이고,
시계 오류가 아니라 불규칙한 컨트롤러 거동으로 나타나 진단이 어렵다 (설계서 §12-10).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("orchard_sim")
    bridge_cfg = os.path.join(pkg, "config", "livox_bridge.yaml")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("apply_fov_mask", default_value="true",
                              description="MID-70 원형 FOV 마스크 (정사각 격자의 모서리 21% 제거)"),
        DeclareLaunchArgument("publish_custom_msg", default_value="true",
                              description="FAST-LIO2 용 livox_ros_driver2/CustomMsg 발행"),
        DeclareLaunchArgument("publish_freq_hz", default_value="10.0"),
    ]
    use_sim_time = LaunchConfiguration("use_sim_time")

    gz_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="gz_bridge", output="screen",
        parameters=[{"config_file": bridge_cfg, "use_sim_time": use_sim_time}],
    )

    livox = Node(
        package="orchard_sim", executable="livox_sim_bridge",
        name="livox_sim_bridge", output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "apply_fov_mask": LaunchConfiguration("apply_fov_mask"),
            "publish_custom_msg": LaunchConfiguration("publish_custom_msg"),
            "publish_freq_hz": LaunchConfiguration("publish_freq_hz"),
        }],
    )

    return LaunchDescription(args + [gz_bridge, livox])
