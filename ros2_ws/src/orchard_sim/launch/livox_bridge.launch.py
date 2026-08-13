"""
gz → ROS 2 라이다 브리지 + Livox 계약 변환만 따로 띄운다 (센서 확인용).

    ros2 launch orchard_sim livox_bridge.launch.py
    ros2 launch orchard_sim livox_bridge.launch.py robot_id:=scout02

    # 원형 FOV 마스크를 끄고 비교하고 싶을 때
    ros2 launch orchard_sim livox_bridge.launch.py apply_fov_mask:=false

주행 스택 전체는 stage0/control.launch.py 를 쓴다 — 이 런치는 라이다 한 줄만
보고 싶을 때의 최소 구성이다. 토픽 이름은 stage0 과 **같은 출처**
(orchard_sim/gz_topics.py)에서 만든다: 여기만 옛 이름으로 남으면 확인 도구가
주행 때와 다른 토픽을 보게 된다.

use_sim_time 은 모든 노드에 반드시 켠다 — 부분 적용은 Nav2 시뮬 실패의 최다 원인이고,
시계 오류가 아니라 불규칙한 컨트롤러 거동으로 나타나 진단이 어렵다 (설계서 §12-10).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from orchard_sim import gz_topics as gzt

ARGS = [
    DeclareLaunchArgument("world_name", default_value="orchard_10x41"),
    DeclareLaunchArgument("robot_id", default_value="scout01"),
    DeclareLaunchArgument("ns", default_value="",
                          description="ROS 네임스페이스. 비우면 robot_id 를 쓴다"),
    DeclareLaunchArgument("use_sim_time", default_value="true"),
    DeclareLaunchArgument("clock", default_value="true"),
    DeclareLaunchArgument("apply_fov_mask", default_value="true",
                          description="MID-70 원형 FOV 마스크 (정사각 격자의 모서리 21% 제거)"),
    DeclareLaunchArgument("publish_custom_msg", default_value="true",
                          description="FAST-LIO2 용 livox_ros_driver2/CustomMsg 발행"),
    DeclareLaunchArgument("publish_freq_hz", default_value="10.0"),
]


def _setup(context, *_a, **_k):
    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        # 런치 인자는 문자열이다 — bool 파라미터에 그대로 실으면 노드가 죽는다
        # (stage0.launch.py 의 같은 주석 참조).
        return cfg(name).lower() in ("1", "true", "yes", "on")

    world, robot = cfg("world_name"), cfg("robot_id")
    ns = cfg("ns") or robot
    common = {"use_sim_time": flag("use_sim_time")}

    lidar = gzt.cloud_bridges(world, robot, ns)
    nodes = []
    if flag("clock"):
        clk = [gzt.clock_bridge()]
        nodes.append(Node(package="ros_gz_bridge", executable="parameter_bridge",
                          name="clock_bridge", output="screen",
                          arguments=gzt.args(clk), remappings=gzt.remaps(clk),
                          parameters=[common]))
    nodes.append(Node(package="ros_gz_bridge", executable="parameter_bridge",
                      name="gz_bridge_cloud", namespace=ns, output="screen",
                      arguments=gzt.args(lidar), remappings=gzt.remaps(lidar),
                      parameters=[common]))
    nodes.append(Node(
        package="orchard_sim", executable="livox_sim_bridge",
        name="livox_sim_bridge", namespace=ns, output="screen",
        parameters=[dict(common,
                         robot_id=robot,
                         apply_fov_mask=flag("apply_fov_mask"),
                         publish_custom_msg=flag("publish_custom_msg"),
                         publish_freq_hz=float(cfg("publish_freq_hz")))]))
    return nodes


def generate_launch_description():
    return LaunchDescription(ARGS + [OpaqueFunction(function=_setup)])
