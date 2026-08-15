"""
통합관제 — 로봇측 스택 일체

    ros2 launch orchard_sim control.launch.py
    ros2 launch orchard_sim control.launch.py slam:=fastlio     # 참값 대신 FAST-LIO2

    # 2호기 — 로봇당 한 세트씩 띄운다 (포트·네임스페이스가 겹치지 않게)
    ros2 launch orchard_sim control.launch.py \
        robot_id:=scout02 ns:=scout02 port:=8081 clock:=false

    # 실사 월드 (sim/worlds/orchard_real.sdf) — 기하를 farm.json 에서 받는다
    ros2 launch orchard_sim control.launch.py world:=real

띄우는 것
    stage0        브리지 · 정적TF · 참값 로컬라이저 · Livox 계약
    fastlio       (선택) FAST-LIO2 — slam:=fastlio 일 때만
    control_agent 텔레메트리 + 웹 대시보드 + 명령 처리

토픽·TF 표준 (다중 로봇):
    /<ns>/{cmd_vel, odom, imu, livox/lidar, gz_ground_truth, …}
    map → <robot_id>/odom → <robot_id>/base_link → <robot_id>/{livox_frame, …}
`/tf`·`/tf_static`·`/clock` 만 전역이다. 근거는 orchard_sim/gz_topics.py 머리말.

**이 launch 는 로봇 PC 에서 돈다.** 관제 PC 에는 아무것도 깔지 않는다 — 브라우저로
http://<로봇IP>:8080/ 을 열면 된다. ROS 2 도, 파이썬 패키지도 필요 없다.
근거와 통신 선택 이유는 docs/findings/2026-07-30-fleet-stack-decision.md 참조.

월드 선택 (`world:=terraced|real`, **기본 terraced**):
    terraced  sim/worlds/orchard_nav.sdf   · gz 월드 이름 orchard_10x41
              기하는 control_agent 의 기본 파라미터(rows=10, row_spacing=3.5 …)
    real      sim/worlds/orchard_real.sdf  · gz 월드 이름 orchard_real
              기하(rows·row_spacing·tree_spacing·headland)를 `farm:=`(기본
              maps/orchard_real/farm.json)에서 읽어 control_agent 파라미터로 넘긴다.
              로봇 코드는 무수정 — 기하는 데이터로만 들어간다(스펙 ④).
**기본값을 real 로 바꾸는 것은 스펙 ④ T7 게이트의 몫이다.** 여기서는 경로만 낸다.
이 런치는 gz 를 띄우지 않는다(월드 SDF 기동은 scripts/run_control.sh 또는 gz sim).
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# world:=real 일 때 gz 월드 이름 (gen_world --farm 의 --world-name 기본값과 같아야 한다)
REAL_WORLD_NAME = "orchard_real"
TERRACED_WORLD_NAME = "orchard_10x41"
DEFAULT_FARM = "maps/orchard_real/farm.json"
DEFAULT_BUNDLE = {"real": "maps/orchard_real", "terraced": "maps/orchard_v1"}

# 측위 상실 격상(자동 정지) 문턱 — world 별 기본값. **단위가 world 마다 다르다**:
# 어느 시계로 재는지를 `lost_clock` 이 정하고, 문턱은 그 시계의 초다.
#
# ── terraced: 150초, 벽시계 (기존 실측 그대로 불변) ─────────────────────────
#
# ── real: 220 **시뮬초**, sim 시계 ─────────────────────────────────────────
# ① 왜 문턱이 유일한 방어선인가 — 이 월드에는 겉보기 벽이 하나도 없다
#    (47 캘리브레이션 52/52 미검출). `map_localizer._tick` 의 격상 조건은
#    `(now-last_ok) > T AND (now-last_anchor) > T` 인데, 앵커가 영원히 안 잡히면
#    `last_anchor_t` 는 기동 시각에 멈춘 채 **영구 노화**한다 — AND 가 사실상
#    `last_ok` 단독으로 환원된다.
# ② 왜 sim 시계인가 — 판정 대상('횡단 한 번')은 시뮬 시간의 사건인데 문턱을
#    벽시계로 두면 판정이 **RTF 에 딸려 흔들린다**. 1대 실측 RTF 0.374 에
#    맞춰 놓은 벽시계 상수는 2대 동시 운용에서 RTF 가 조금만 떨어져도 같은
#    횡단이 갑자기 문턱을 넘는다(T7 게이트가 반드시 실패한다).
# ③ 220 의 산정 — 구조점 사막(통로 종단 → 횡단선 → 다음 통로 진입) 왕복
#    거리를 farm.json 기하로 전수 계산하고, 실측 실효 속도로 나눈다.
#      실효 속도: 게이트 실측 통로 0→1 북측 왕복 17.7 m 를 무보정 160.8초
#                (벽) = 60.1 시뮬초에 통과 → 0.295 m/s(피벗·구조 소실 선행분 포함)
#      최악 인접 전이      31.7 m → 107 시뮬초 (통로 20↔21 북측)
#      최악 한 칸 건너 전이 39.1 m → 132 시뮬초 (통로 20→22 북측)
#      → 132 × 여유 1.5 ≈ 198, 리뷰어 독립 산정(145 시뮬초 × 1.5 ≈ 220) 중
#        **큰 쪽을 취해 220**. 두 통로 이상 건너뛰는 전이는 이 예산 밖이다
#        (예: 0→25 는 733 시뮬초 = 예산의 3.3배 — 리포트 §8-8).
LOST_CRITICAL_S = {"real": "220.0", "terraced": "150.0"}
LOST_CLOCK = {"real": "sim", "terraced": "wall"}


def _farm_geom(farm_path):
    """farm.json → 에이전트 기하 파라미터.

    로봇 계약은 무수정이다(스펙 ④ Global Constraints) — 기하는 코드가 아니라
    **파라미터**로 흘러간다. 여기가 그 유일한 통로다. 파일이 없으면 조용히
    기본값으로 떨어지지 않고 죽는다(스펙 ④ §6: 무음 기본값 금지).
    """
    if not os.path.exists(farm_path):
        raise RuntimeError(
            f"world:=real 인데 농장 매니페스트가 없습니다: {farm_path}\n"
            f"    farm:=<경로> 로 지정하거나 world:=terraced 로 띄우세요.")
    with open(farm_path) as f:
        farm = json.load(f)
    # trees_per_row 는 실사 농장에서 열마다 다르다(row_lengths_m). 에이전트는
    # col_len 계산에만 쓰므로 **중앙값 열 길이**로 환산해 넘긴다.
    tps = float(farm["tree_spacing_m"])
    hl = float(farm["headland_m"])
    n = int(farm["rows"])

    # ── site_geom v2 배열 (robomw SDK 계약 개정, additive) ──────────────────
    # 균일 격자로는 이 농장을 표현할 수 없다(통로 간격 4.75~5.25 m, 열별 길이
    # 128~141 m, y 비대칭). 그래서 통로별 중심 x 와 [남단, 북단] y 를 직접 준다.
    #
    # **부호 규약**: farm.json axes_note — world +y 는 이미지 +y 와 나란하고
    # 그 이미지의 맨 윗행이 최대 northing 이다. 즉 **world +y = 지리적 남**이라
    # 이 현장에서 '남단' 은 y 가 **큰** 쪽이다(계단식 월드와 부호가 반대).
    #
    # y 구간은 캐노피 구간(row_origins ± headland_m)의 **더 안쪽**을 쓴다 —
    # 이웃 두 열 중 짧은 쪽이 통로의 실질 끝이다(gen_world.farm_alley_spawn 과
    # 같은 규약. 긴 쪽으로 잡으면 통로 중앙에서 정사영상 프레임을 넘어간다).
    def canopy(r):
        x, y0 = farm["row_origins"][r]
        return float(x), float(y0) + hl, float(y0) + float(farm["row_lengths_m"][r]) - hl

    centers, south, north = [], [], []
    cross_s, cross_n = [], []
    for k in range(n - 1):
        xa, ya0, ya1 = canopy(k)
        xb, yb0, yb1 = canopy(k + 1)
        centers.append((xa + xb) / 2.0)
        south.append(min(ya1, yb1))      # y 최대 쪽 = 지리적 남
        north.append(max(ya0, yb0))      # y 최소 쪽 = 지리적 북
        # 횡단선은 **바깥 끝** 기준이다. 통로 사이를 가로지르려면 그 사이의
        # 열을 넘어가야 하는데, 그 열의 끝 나무는 통로의 '안쪽 끝'(위 south/
        # north, 짧은 쪽 규약)보다 최대 2.7 m 바깥에 있다 — 안쪽 기준으로
        # 횡단선을 잡으면 열 끝 나무를 정면으로 들이받는다. 여유는 헤드랜드
        # 한 폭(수관 끝 ~ 이미지 프레임 실측 거리)으로 준다.
        cross_s.append(max(ya1, yb1) + hl)
        cross_n.append(min(ya0, yb0) - hl)
    return {
        "rows": n,
        "row_spacing": float(farm["row_spacing_m"]),
        "tree_spacing": tps,
        "trees_per_row": int(round(float(farm["row_length_m"]) / tps)) + 1,
        "headland": hl,
        "alley_centers_x": centers,
        "alley_south_y": south,
        "alley_north_y": north,
        "alley_cross_south_y": cross_s,
        "alley_cross_north_y": cross_n,
    }


def generate_launch_description():
    args = [
        DeclareLaunchArgument("world", default_value="terraced",
                              description="terraced | real. 기본은 terraced — "
                                          "실사 월드로의 기본 전환은 스펙 ④ T7 게이트 이후다"),
        DeclareLaunchArgument("farm", default_value=DEFAULT_FARM,
                              description="world:=real 일 때 읽는 농장 기하 매니페스트"),
        DeclareLaunchArgument("world_name", default_value="",
                              description="gz 월드 이름(토픽 스코프). 비우면 world 인자에서 정한다"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("bind", default_value="0.0.0.0",
                              description="0.0.0.0 이어야 다른 PC 에서 붙는다"),
        DeclareLaunchArgument("robot_id", default_value="scout01",
                              description="gz 월드의 로봇 인스턴스 이름. "
                                          "토픽·TF 접두가 전부 여기서 나온다"),
        DeclareLaunchArgument("ns", default_value="",
                              description="ROS 네임스페이스. 비우면 robot_id 를 쓴다"),
        DeclareLaunchArgument("clock", default_value="true",
                              description="/clock 브리지. 2호기부터는 false"),
        DeclareLaunchArgument("slam", default_value="groundtruth",
                              description="groundtruth | fastlio | maplocalizer. "
                                          "maplocalizer 는 참값 로컬라이저 대신 "
                                          "사전 맵 번들 위에서 위치를 잡는다"),
        DeclareLaunchArgument("bundle", default_value="",
                              description="slam:=maplocalizer 의 맵 번들. 비우면 "
                                          "world 에서 정한다(real → maps/orchard_real)"),
        DeclareLaunchArgument("init_x", default_value="0.0"),
        DeclareLaunchArgument("init_y", default_value="0.0"),
        DeclareLaunchArgument("init_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "lost_critical_s", default_value="",
            description="측위 상실 격상(자동 정지) 문턱 [s, 단위는 lost_clock 의 "
                        "시계]. 비우면 world 에서 정한다 — terraced 150(벽) / "
                        "real 220(시뮬)"),
        DeclareLaunchArgument(
            "lost_clock", default_value="",
            description="상실 타이머의 시계: wall | sim. 비우면 world 에서 "
                        "정한다 — terraced wall(불변) / real sim"),
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
    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])


def _launch_setup(context, *a, **kw):
    pkg = get_package_share_directory("orchard_sim")
    fastlio_cfg = os.path.join(pkg, "config", "fastlio_mid70.yaml")

    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    world = cfg("world")
    if world not in ("terraced", "real"):
        raise RuntimeError(f"world 는 terraced | real 여야 합니다: {world!r}")
    world_name = cfg("world_name") or (
        REAL_WORLD_NAME if world == "real" else TERRACED_WORLD_NAME)
    geom = _farm_geom(cfg("farm")) if world == "real" else {}

    ust = LaunchConfiguration("use_sim_time")
    robot_id = LaunchConfiguration("robot_id")
    # ns 를 비워 두면 robot_id 를 쓴다. 런치 인자에는 "비었으면 다른 값" 이라는
    # 기본값 문법이 없어서 파이썬 표현식으로 고른다.
    #
    # 주의 — ns 는 robot_id 와 같게 두거나 비워라(비우면 robot_id 를 그대로
    # 쓴다). 다르게 주면 갈라진다: 아래로 넘어가는 stage0/livox_bridge 는 이
    # ns 로 노드 네임스페이스가 잡혀 /<ns>/... 토픽·프레임을 쓰는데,
    # control_agent 의 센서 토픽·TF 프레임은 여기 ns 가 아니라 robot_id
    # 파라미터에서 직접 파생된다(control_agent.py 의 robot_id 파생 절 참조).
    # 즉 브리지는 ns 파생, 노드의 절대 토픽은 robot_id 파생 — 둘을 다르게
    # 주면 브리지는 /<ns>/... 로 뜨고 control_agent 는 /<robot_id>/... 를
    # 구독/발행해 서로 어긋난다. 에러도 경고도 없이 그냥 데이터가 안
    # 온다(무증상 무데이터) — 다중 로봇으로 늘릴 때 가장 조용히 새는 지점.
    ns = cfg("ns") or cfg("robot_id")
    slam = cfg("slam")
    if slam not in ("groundtruth", "fastlio", "maplocalizer"):
        raise RuntimeError(
            f"slam 은 groundtruth | fastlio | maplocalizer 여야 합니다: {slam!r}")
    # map→<ns>/odom 을 내는 노드는 하나뿐이어야 한다 — map_localizer 를 쓰면
    # stage0 의 참값 로컬라이저를 끈다(둘이 같이 돌면 TF 가 번갈아 덮인다).
    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, "launch", "stage0.launch.py")),
            launch_arguments={"world_name": world_name,
                              "use_sim_time": ust,
                              "robot_id": robot_id,
                              "ns": ns,
                              "clock": LaunchConfiguration("clock"),
                              "gt_localizer": ("false" if slam == "maplocalizer"
                                               else "true"),
                              "cameras": LaunchConfiguration("cameras")}.items()),
    ]
    # FAST-LIO2 는 토픽을 절대 이름(/Odometry 등)으로 발행한다 — 네임스페이스를
    # 씌워도 갈라지지 않는다. 다중 로봇 SLAM 은 별건이라 여기서는 손대지 않고,
    # 참값 경로(slam:=groundtruth)만 다중화 대상이다.
    if slam == "fastlio":
        nodes.append(Node(package="fast_lio", executable="fastlio_mapping",
                          name="fastlio_mapping", namespace=ns, output="screen",
                          parameters=[fastlio_cfg, {"use_sim_time": ust}]))
    if slam == "maplocalizer":
        # 게이트 재현 경로 — 번들·격상 문턱의 기본값이 여기 있어야 저장소만으로
        # 같은 조건을 다시 세울 수 있다(문턱 산정 근거는 LOST_CRITICAL_S 주석).
        bundle = cfg("bundle") or DEFAULT_BUNDLE[world]
        if not os.path.exists(bundle):
            raise RuntimeError(
                f"slam:=maplocalizer 인데 맵 번들이 없습니다: {bundle}\n"
                f"    python3 scripts/37_build_map_bundle.py"
                + (f" --farm {cfg('farm')} --terrain sim/models/orchard_terrain_real"
                   f" --out {bundle}" if world == "real" else f" --out {bundle}"))
        nodes.append(Node(
            package="orchard_sim", executable="map_localizer",
            name="map_localizer", namespace=ns, output="screen",
            parameters=[{"use_sim_time": ust, "robot_id": robot_id,
                         "bundle": bundle,
                         "init_x": LaunchConfiguration("init_x"),
                         "init_y": LaunchConfiguration("init_y"),
                         "init_yaw": LaunchConfiguration("init_yaw"),
                         "lost_critical_s": float(
                             cfg("lost_critical_s") or LOST_CRITICAL_S[world]),
                         "lost_clock": cfg("lost_clock") or LOST_CLOCK[world]}]))

    nodes.append(
        Node(package="orchard_sim", executable="control_agent",
             name="control_agent", namespace=ns, output="screen",
             parameters=[dict(geom, **{
                          "use_sim_time": ust,
                          "robot_id": robot_id,
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
                          "tls_key": LaunchConfiguration("tls_key")})]))
    return nodes
