"""
Stage-0 로컬라이제이션 스캐폴드 (설계서 §7.5)

    ros2 launch orchard_sim stage0.launch.py world_name:=orchard_10x41

띄우는 것:
    gz_bridge        센서·오도메트리·/tf
    gt_bridge        gz dynamic_pose/info → /gz_ground_truth   (참값, 13 엔트리로 가볍다)
    sdf_static_tf    base_link → 센서 프레임 (model.sdf 에서 직접 읽음)
    gt_localizer     완벽한 map→odom
    livox_sim_bridge Livox 계약 변환

TF 트리:  map →(gt_localizer)→ odom →(DiffDrive)→ base_link →(sdf_static_tf)→ 센서

use_sim_time 은 전 노드에 켠다 — 부분 적용은 Nav2 시뮬 실패의 최다 원인이고
시계 오류가 아니라 불규칙한 컨트롤러 거동으로 나타난다 (설계서 §12-10).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("orchard_sim")
    bridge_cfg = os.path.join(pkg, "config", "livox_bridge.yaml")

    args = [
        DeclareLaunchArgument("world_name", default_value="orchard_10x41"),
        DeclareLaunchArgument("robot_model_name", default_value="scout_mini_mid70"),
        DeclareLaunchArgument("model_sdf",
                              default_value="/home/myhome/YBNML/sim/models/scout_mini_mid70/model.sdf"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
    ]
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world_name")

    # 로봇 모델의 PosePublisher 가 내는 참값. 월드의 dynamic_pose/info 를 쓰면
    # ros_gz 변환기가 frame_id 를 비워두므로(2026-07-25 실측) 이쪽을 쓴다.
    gt_topic = PythonExpression(
        ["'/model/' + '", LaunchConfiguration("robot_model_name"), "' + '/pose'"])

    return LaunchDescription(args + [
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="gz_bridge", output="screen",
             parameters=[{"config_file": bridge_cfg, "use_sim_time": use_sim_time}]),

        # 참값 포즈 — 로봇 모델의 PosePublisher (frame_id/child_frame_id 가 채워진다).
        # 월드의 pose/info 는 5,478 엔트리라 브리지하면 안 되고,
        # dynamic_pose/info 는 가볍지만 ros_gz 변환기가 프레임 이름을 안 채운다.
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="gt_bridge", output="screen",
             arguments=[PythonExpression(
                 ["'", gt_topic, "' + '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'"])],
             remappings=[(gt_topic, "/gz_ground_truth")],
             parameters=[{"use_sim_time": use_sim_time}]),

        Node(package="orchard_sim", executable="sdf_static_tf",
             name="sdf_static_tf", output="screen",
             parameters=[{"use_sim_time": use_sim_time,
                          "model_sdf": LaunchConfiguration("model_sdf")}]),

        Node(package="orchard_sim", executable="gt_localizer",
             name="gt_localizer", output="screen",
             parameters=[{"use_sim_time": use_sim_time,
                          "robot_model_name": LaunchConfiguration("robot_model_name")}]),

        Node(package="orchard_sim", executable="livox_sim_bridge",
             name="livox_sim_bridge", output="screen",
             parameters=[{"use_sim_time": use_sim_time}]),
    ])
