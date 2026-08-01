"""
통합관제 — 로봇측 스택 일체

    ros2 launch orchard_sim control.launch.py
    ros2 launch orchard_sim control.launch.py slam:=fastlio     # 참값 대신 FAST-LIO2

띄우는 것
    stage0        브리지 · 정적TF · 참값 로컬라이저 · Livox 계약
    fastlio       (선택) FAST-LIO2 — slam:=fastlio 일 때만
    control_agent 텔레메트리 + 웹 대시보드 + 명령 처리

**이 launch 는 로봇 PC 에서 돈다.** 관제 PC 에는 아무것도 깔지 않는다 — 브라우저로
http://<로봇IP>:8080/ 을 열면 된다. ROS 2 도, 파이썬 패키지도 필요 없다.
근거와 통신 선택 이유는 docs/findings/2026-07-30-fleet-stack-decision.md 참조.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("orchard_sim")
    fastlio_cfg = os.path.join(pkg, "config", "fastlio_mid70.yaml")

    args = [
        DeclareLaunchArgument("world_name", default_value="orchard_10x41"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("bind", default_value="0.0.0.0",
                              description="0.0.0.0 이어야 다른 PC 에서 붙는다"),
        DeclareLaunchArgument("robot_id", default_value="scout01"),
        DeclareLaunchArgument("slam", default_value="groundtruth",
                              description="groundtruth | fastlio"),
        DeclareLaunchArgument("speed", default_value="0.7"),
        DeclareLaunchArgument("cameras", default_value="false"),
        # 보안 — 비우면 개방 모드로 뜨고 기동 로그에 경고가 남는다.
        # scripts/gen_cert.sh 로 인증서·토큰을 만들어 넘긴다.
        # TLS 없이 토큰만 켜면 토큰이 평문으로 흐르므로 둘은 같이 켜야 한다.
        #
        # 역할을 나눠 주려면 JSON 사전을 그대로 넘긴다. 관측용 토큰(화면만
        # 보는 사람)을 admin 으로 뿌리지 않으려면 이 형태를 써야 한다.
        # 셸이 중괄호와 따옴표를 먹지 않게 작은따옴표로 통째로 감싼다:
        #
        #   ros2 launch orchard_sim control.launch.py \
        #     'auth_token:={"운전자토큰":"operator","관리자토큰":"admin"}'
        DeclareLaunchArgument(
            "auth_token", default_value="",
            description=(
                "빈 값이면 개방 모드(접속자 전원 admin). "
                "일반 문자열이면 그 토큰 하나가 admin. "
                '\'{"토큰":"역할", ...}\' JSON 이면 토큰마다 역할을 준다 '
                "(역할: observer | operator | admin). "
                "'{' 로 시작하는데 JSON 이 아니면 설정 오류로 보고 "
                "모든 접속을 거부한다 — 통째로 토큰 하나로 삼키지 않는다.")),
        DeclareLaunchArgument("tls_cert", default_value=""),
        DeclareLaunchArgument("tls_key", default_value=""),
    ]
    ust = LaunchConfiguration("use_sim_time")
    use_fastlio = IfCondition(PythonExpression(
        ["'", LaunchConfiguration("slam"), "' == 'fastlio'"]))

    return LaunchDescription(args + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, "launch", "stage0.launch.py")),
            launch_arguments={"world_name": LaunchConfiguration("world_name"),
                              "use_sim_time": ust,
                              "cameras": LaunchConfiguration("cameras")}.items()),

        Node(package="fast_lio", executable="fastlio_mapping",
             name="fastlio_mapping", output="screen", condition=use_fastlio,
             parameters=[fastlio_cfg, {"use_sim_time": ust}]),

        Node(package="orchard_sim", executable="control_agent",
             name="control_agent", output="screen",
             parameters=[{"use_sim_time": ust,
                          "robot_id": LaunchConfiguration("robot_id"),
                          "port": LaunchConfiguration("port"),
                          "bind": LaunchConfiguration("bind"),
                          "speed": LaunchConfiguration("speed"),
                          # value_type=str 이 없으면 launch_ros 가 값의 생김새로
                          # 타입을 추론한다. 역할 사전을 넘기면 '{"a":"admin"}'
                          # 이 dict 로 추론돼 "Allowed value types are ..." 로
                          # 기동 자체가 실패했다 — 토큰별 역할을 줄 방법이
                          # 없었던 진짜 이유가 여기다. 숫자만으로 된 토큰이
                          # int 로 추론되는 문제도 같이 막는다.
                          "auth_token": ParameterValue(
                              LaunchConfiguration("auth_token"), value_type=str),
                          "tls_cert": LaunchConfiguration("tls_cert"),
                          "tls_key": LaunchConfiguration("tls_key")}]),
    ])
