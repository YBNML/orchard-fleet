"""
Stage-0 로컬라이제이션 스캐폴드 (설계서 §7.5) — 로봇 **한 대분**

    ros2 launch orchard_sim stage0.launch.py world_name:=orchard_10x41
    ros2 launch orchard_sim stage0.launch.py robot_id:=scout02 clock:=false

띄우는 것:
    gz_bridge_cloud  점군 전용 브리지 (600 KB/프레임 — 다른 토픽과 섞으면 굶는다)
    gz_bridge        나머지 센서·오도메트리·/tf   (gz 인스턴스 토픽 → /<ns>/*)
    gt_bridge        /model/<로봇>/pose    → /<ns>/gz_ground_truth
    sdf_static_tf    <ns>/base_link → 센서 프레임 (model.sdf 에서 직접 읽음)
    gt_localizer     완벽한 map→<ns>/odom
    livox_sim_bridge Livox 계약 변환

TF 트리:  map →(gt_localizer)→ <로봇>/odom →(DiffDrive)→ <로봇>/base_link
                                                      →(sdf_static_tf)→ 센서

**다중 로봇**: 이 런치를 로봇마다 한 번씩 띄운다. gz 쪽 토픽은 model.sdf 가
`<topic>` 을 안 쓰는 덕에 인스턴스별로 갈라져 있고(orchard_sim/gz_topics.py 머리말),
ROS 쪽은 노드 네임스페이스 `ns` 와 `/<ns>/*` 토픽으로 가른다. `/tf` 와 `/clock`
만 전역이다 — TF 는 프레임 이름에 로봇 접두가 붙어 한 트리로 모아도 안전하고
(오히려 로봇 간 상대 위치를 바로 물어볼 수 있다), 시계는 gz 인스턴스가 하나라
둘 이상 붙일 필요가 없다(두 번째 로봇은 `clock:=false`).

use_sim_time 은 전 노드에 켠다 — 부분 적용은 Nav2 시뮬 실패의 최다 원인이고
시계 오류가 아니라 불규칙한 컨트롤러 거동으로 나타난다 (설계서 §12-10).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from orchard_sim import gz_topics as gzt

ARGS = [
    DeclareLaunchArgument("world_name", default_value="orchard_10x41"),
    DeclareLaunchArgument("robot_id", default_value="scout01",
                          description="gz 월드의 로봇 인스턴스 이름 "
                                      "(=<include><name>). 토픽·TF 접두가 여기서 나온다"),
    DeclareLaunchArgument("ns", default_value="",
                          description="ROS 네임스페이스. 비우면 robot_id 를 쓴다"),
    DeclareLaunchArgument("model_sdf",
                          default_value=os.path.join(
                              os.environ.get("ORCHARD_ROOT", "/home/myhome/YBNML"),
                              "sim/models/scout_mini_mid70/model.sdf")),
    DeclareLaunchArgument("use_sim_time", default_value="true"),
    DeclareLaunchArgument("cameras", default_value="true",
                          description="카메라 브리지. 매핑·주행 중에는 false 로 두면 RTF 가 약 22% 올라간다"),
    DeclareLaunchArgument("clock", default_value="true",
                          description="/clock 브리지. gz 인스턴스당 하나면 충분하다 — "
                                      "두 번째 로봇 스택은 false"),
    DeclareLaunchArgument("gt_localizer", default_value="true",
                          description="참값 로컬라이저. map→<ns>/odom 을 내는 노드는 "
                                      "하나뿐이어야 하므로, map_localizer 를 쓸 때는 "
                                      "false (control.launch.py slam:=maplocalizer)"),
]


def _setup(context, *_a, **_k):
    """LaunchConfiguration 을 문자열로 풀고 나서 이름표를 만든다.

    PythonExpression 으로 문자열을 이어 붙이면 토픽 하나에 대여섯 줄이 든다.
    OpaqueFunction 안에서는 그냥 파이썬이라 gz_topics 를 직접 부를 수 있다.
    """
    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        """런치 인자는 전부 문자열로 풀린다. bool 파라미터에 문자열을 그대로
        실으면 노드가 `InvalidParameterTypeException` 으로 죽는다 — 치환
        (LaunchConfiguration)을 그대로 넘길 때는 launch_ros 가 알아서 캐스팅해
        주지만, OpaqueFunction 안에서는 우리가 해야 한다."""
        return cfg(name).lower() in ("1", "true", "yes", "on")

    world = cfg("world_name")
    robot = cfg("robot_id")
    ns = cfg("ns") or robot
    want_cameras = flag("cameras")
    want_clock = flag("clock")
    common = {"use_sim_time": flag("use_sim_time")}

    nodes = []

    if want_clock:
        clk = [gzt.clock_bridge()]
        nodes.append(Node(package="ros_gz_bridge", executable="parameter_bridge",
                          name="clock_bridge", output="screen",
                          arguments=gzt.args(clk), remappings=gzt.remaps(clk),
                          parameters=[common]))

    # 점군은 제 프로세스에 혼자 둔다 — gz_topics.cloud_bridges 머리말의 실측 참조.
    cloud = gzt.cloud_bridges(world, robot, ns)
    nodes.append(Node(package="ros_gz_bridge", executable="parameter_bridge",
                      name="gz_bridge_cloud", namespace=ns, output="screen",
                      arguments=gzt.args(cloud), remappings=gzt.remaps(cloud),
                      parameters=[common]))

    core = gzt.core_bridges(world, robot, ns)
    nodes.append(Node(package="ros_gz_bridge", executable="parameter_bridge",
                      name="gz_bridge", namespace=ns, output="screen",
                      arguments=gzt.args(core), remappings=gzt.remaps(core),
                      parameters=[common]))

    if want_cameras:
        cams = gzt.camera_bridges(world, robot, ns)
        nodes.append(Node(package="ros_gz_bridge", executable="parameter_bridge",
                          name="gz_bridge_cameras", namespace=ns, output="screen",
                          arguments=gzt.args(cams), remappings=gzt.remaps(cams),
                          parameters=[common]))

    # 참값 포즈는 브리지를 따로 둔다 — Pose_V→TFMessage 라는 특이한 짝이라,
    # 한 노드에 묶었다가 이 짝 하나가 틀어지면 센서 브리지까지 같이 죽는다.
    gt = [gzt.ground_truth_bridge(robot, ns)]
    nodes.append(Node(package="ros_gz_bridge", executable="parameter_bridge",
                      name="gt_bridge", namespace=ns, output="screen",
                      arguments=gzt.args(gt), remappings=gzt.remaps(gt),
                      parameters=[common]))

    nodes.append(Node(package="orchard_sim", executable="sdf_static_tf",
                      name="sdf_static_tf", namespace=ns, output="screen",
                      parameters=[dict(common, robot_id=robot,
                                       model_sdf=cfg("model_sdf"))]))

    if flag("gt_localizer"):
        nodes.append(Node(package="orchard_sim", executable="gt_localizer",
                          name="gt_localizer", namespace=ns, output="screen",
                          parameters=[dict(common, robot_id=robot)]))

    nodes.append(Node(package="orchard_sim", executable="livox_sim_bridge",
                      name="livox_sim_bridge", namespace=ns, output="screen",
                      parameters=[dict(common, robot_id=robot)]))

    return nodes


def generate_launch_description():
    return LaunchDescription(ARGS + [OpaqueFunction(function=_setup)])
