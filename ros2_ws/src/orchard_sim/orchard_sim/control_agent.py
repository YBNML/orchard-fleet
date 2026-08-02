#!/usr/bin/env python3
"""
control_agent — 로봇에 올라가는 관제 에이전트 (코어 호스트)

    ros2 run orchard_sim control_agent --ros-args -p port:=8080
    ros2 launch orchard_sim control.launch.py

하는 일은 넷뿐이다. 나머지는 전부 기능(플러그인)이 한다.
    1. ROS 토픽에서 공통 상태를 모아 블랙보드에 올린다 (포즈·자세·센서 주기)
    2. 웹 대시보드를 내고 관제와 WebSocket 으로 주고받는다
    3. 안전 조정자를 돌린다 — 비상정지 래치·데드맨·링크두절·전복
    4. 기능들을 적재하고 명령을 라우팅하고 속도 요청을 조정해 /cmd_vel 로 낸다

**DDS 는 이 노드 밖으로 나가지 않는다.** 관제 PC 에 ROS 를 깔 필요가 없고,
열어야 할 포트도 하나다. 규약은 link/protocol.py, 선택 근거는
docs/findings/2026-07-30-fleet-stack-decision.md 참조.

기능 늘리고 줄이기
    파라미터 features 목록으로 정한다. 기본값은 지금 쓰는 다섯 개다.
        ros2 run orchard_sim control_agent --ros-args \
            -p "features:=['telemetry_state','telemetry_health','drive_teleop']"
    기능을 새로 만들려면 control/features/ 에 모듈 하나 넣고 목록에 이름을
    추가한다 — 이 파일은 고치지 않는다. 계약은 control/base.py 참조.

안전은 기능이 아니다
    비상정지·데드맨·링크두절·전복은 control/safety.py 의 코어에 있고, 기능은
    이를 우회할 수단이 없다. 기능은 속도를 **요청**할 뿐이고 최종 출력은 항상
    조정자를 통과한다. /cmd_vel 을 쓰는 곳도 이 파일 한 군데다.
"""
from __future__ import annotations

import json
import math
import os
import queue
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Empty
from std_msgs.msg import String as StringMsg
from tf2_ros import Buffer, TransformListener

from orchard_sim import transforms as tfu
from orchard_sim.control.audit import (R_ACCEPT, R_BLOCKED, R_REJECT,
                                        AuditLog)
from orchard_sim.control.base import Blackboard, Context
from orchard_sim.control.registry import Registry
from orchard_sim.control.safety import SafetyArbiter
from orchard_sim.link import protocol as P
from orchard_sim.link.wsserver import ControlServer

DEFAULT_FEATURES = ["telemetry_state", "telemetry_health", "telemetry_map",
                    "drive_mission", "drive_teleop"]

# 같은 곳에서 같은 사유로 거부가 반복되면 이 간격으로만 이벤트를 올린다.
# 조종은 데드맨(400 ms) 때문에 클라이언트가 초당 10회 남짓 보낸다 — 관측자가
# 조종간을 잡고 있으면 거부 이벤트가 그 속도로 쏟아져 정작 봐야 할 경고를
# 이벤트 창 밖으로 밀어낸다. **첫 거부는 언제나 올린다.**
DENY_REPEAT_S = 2.0
DENY_PRUNE_S = 30.0             # 오래된 거부 기록은 버린다 (사전이 무한정 크지 않게)
DENY_KEYS_MAX = 256             # 나이 가지치기만으로는 안 준다 — 개수 상한도 둔다
DENY_WHY_MAX = 160              # 사유 문자열 상한. 클라이언트가 보낸 이름이 섞인다


def _clip(s, n=DENY_WHY_MAX):
    """클라이언트가 준 문자열이 섞인 사유를 화면에 올리기 전에 자른다.

    거부 사유에는 보낸 명령 이름이 그대로 들어간다. 길이 제한이 없으면
    2만 자짜리 명령 이름 하나가 접속한 모든 화면으로 그대로 퍼진다.
    """
    s = str(s)
    return s if len(s) <= n else s[:n] + "…"


class RateMeter:
    """토픽 수신 주기 추정. 센서가 죽은 걸 관제에서 바로 보이게 하기 위한 것."""

    def __init__(self, window=30):
        self.t = []
        self.window = window
        self.last = 0.0

    def tick(self, now):
        self.t.append(now)
        if len(self.t) > self.window:
            self.t.pop(0)
        self.last = now

    def hz(self, now):
        if len(self.t) < 2 or now - self.last > 2.0:
            return 0.0
        span = self.t[-1] - self.t[0]
        return (len(self.t) - 1) / span if span > 1e-6 else 0.0


class ControlAgent(Node):

    def __init__(self):
        super().__init__("control_agent")
        d = self.declare_parameter
        d("robot_id", "scout01")
        d("port", 8080)
        d("bind", "0.0.0.0")
        d("static_dir", "")
        d("features", DEFAULT_FEATURES)
        # 보안 — 비우면 개방 모드로 뜨고 경고를 남긴다 (scripts/gen_cert.sh)
        d("auth_token", "")
        d("tls_cert", "")
        d("tls_key", "")
        # 감사 로그 — 사람을 다치게 할 수 있는 기계다. 누가 언제 무슨 명령을
        # 내렸는지 재시작해도 남아야 한다. 빈 값이면 끈다(기록 없음을 로그로 알림).
        d("audit_path", "")
        d("audit_max_bytes", 8 * 1024 * 1024)
        d("audit_keep", 5)
        # 과수원 기하 (기능이 param 으로 읽는다 — gen_world 와 같아야 한다)
        d("rows", 10); d("trees_per_row", 41)
        d("row_spacing", 3.5); d("tree_spacing", 1.5); d("headland", 6.0)
        # 주행
        d("speed", 0.7); d("turn_speed", 0.5)
        d("y_slow_in", 25.0); d("slow_factor", 0.40); d("decel_dist", 3.0)
        d("wp_tol", 0.5); d("teleop_max_v", 0.8); d("teleop_max_w", 1.2)
        # 텔레메트리
        d("state_hz", 5.0); d("health_hz", 1.0); d("map_period", 3.0)
        d("map_cell", 0.25); d("map_max_points", 6000)
        d("min_lidar_hz", 5.0); d("min_imu_hz", 100.0); d("min_lio_hz", 5.0)
        d("cloud_topic", "/livox/lidar")
        d("lio_odom_topic", "/Odometry")
        d("tilt_limit_deg", 35.0)
        # 대기/비상정지에서 0 속도를 계속 낼지. True 가 안전하지만, 다른 주행
        # 노드와 같이 띄워 관찰만 할 때는 False 로 둔다.
        d("idle_publish", True)

        g = lambda k, dflt=None: (self.get_parameter(k).value  # noqa: E731
                                  if self.has_parameter(k) else dflt)
        self.robot_id = str(g("robot_id"))
        self.idle_publish = bool(g("idle_publish"))
        self.seq = 0
        # 큐에는 payload 만이 아니라 (역할, 주소, payload) 를 싣는다. 예전에는
        # payload 만 넣어서 소비 시점(_handle_cmd)에는 누가 보냈는지가 사라졌다 —
        # 판정도 감사 기록도 불가능했다.
        self.cmdq = queue.Queue()
        self.events = []
        self._deny_seen = {}            # (주소, 명령) → 마지막 거부 이벤트 시각
        self._lock = threading.RLock()

        # ── 공통 상태 ───────────────────────────────────────────────────────
        self.bb = Blackboard()
        self.bb.extra["mode"] = P.MODE_IDLE
        self.rate = dict(lidar=RateMeter(), imu=RateMeter(), lio=RateMeter())

        self.buf = Buffer()
        self._tf_buffer = self.buf          # 기능이 쓸 수 있게 노출
        self.tfl = TransformListener(self.buf, self)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        sqos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=5)
        self._cloud_n = 0
        self._blocked_since = None
        self._recover_until = 0.0       # 슬립 자율 복구 (후진) 종료 시각
        self._recover_tries = {}        # 웨이포인트 idx → 재시도 횟수
        self.create_subscription(PointCloud2, str(g("cloud_topic")),
                                 self._on_cloud, sqos)
        self._imu_R = None              # 기체 자세 (월드←바디) — 점군 수평화용
        self.create_subscription(Imu, "/imu", self._on_imu, sqos)
        self.create_subscription(Odometry, str(g("lio_odom_topic")), self._on_lio, 10)

        # ── 안전 (코어) ─────────────────────────────────────────────────────
        self.safety = SafetyArbiter(
            tilt_limit_deg=float(g("tilt_limit_deg")),
            on_event=self.event,
            on_estop=lambda _r: self._write_cmd(0.0, 0.0))

        # 현장 확인(로컬 리셋) — 실기에서는 기체의 물리 리셋 버튼이 이 자리에 온다.
        # **링크를 타고 오지 않는다**: 관제가 아무리 승인해도 위험구역을 눈으로
        # 확인한 사람이 여기를 누르기 전에는 래치가 풀리지 않는다
        # (ISO 13849-1 §5.2.2 의 '시야 확보된 위치에서의 리셋').
        self.create_subscription(Empty, "~/local_reset",
                                 lambda _m: self._on_local_reset(), 10)

        # 로컬라이저 진단 → 관제 개입 큐. 슬립(TRACTION_LOSS)이면 즉시 멈춘다 —
        # 박힌 채 바퀴를 계속 돌리는 것은 기체·나무 양쪽을 갉아먹는 짓이다.
        self.create_subscription(StringMsg, "/map_localizer/diagnostics",
                                 self._on_loc_diag, 10)

        # ── 감사 로그 (코어) ────────────────────────────────────────────────
        ap_ = str(g("audit_path") or "")
        self.audit = AuditLog(ap_, max_bytes=int(g("audit_max_bytes")),
                              keep=int(g("audit_keep"))) if ap_ else None
        if self.audit is None:
            self.get_logger().warn(
                "audit_path 가 비어 있다 — 명령 기록이 재시작하면 사라진다. "
                "현장 전개 시에는 반드시 지정할 것.")
        elif not self.audit.stats().get("enabled", False):
            self.get_logger().warn(
                f"감사 로그를 열지 못했다 ({self.audit.stats().get('reason')}) — "
                "기록 없이 계속 진행한다")
        else:
            self.get_logger().info(f"감사 로그 → {ap_}")

        # ── 웹 ─────────────────────────────────────────────────────────────
        static = str(g("static_dir")) or self._default_static()
        tok = self._auth_config(str(g("auth_token") or ""))
        cert, key = str(g("tls_cert")) or None, str(g("tls_key")) or None
        if not (cert and key):
            self.get_logger().warn(
                "TLS 미설정 — 명령과 텔레메트리가 평문으로 나간다. "
                "무선 회선에서는 반드시 붙일 것 (scripts/gen_cert.sh).")
        self.server = ControlServer(
            static, port=int(g("port")), host=str(g("bind")),
            on_message=self._on_ws_message, on_open=self._on_ws_open,
            logger=lambda m: self.get_logger().info(m),
            auth_token=tok, tls_cert=cert, tls_key=key).start()

        # ── 기능 적재 ───────────────────────────────────────────────────────
        ctx = Context(self, self.bb, self._emit, self.event,
                      lambda n, dflt=None: g(n, dflt), self.safety)
        names = list(g("features") or DEFAULT_FEATURES)
        self.registry = Registry(ctx, on_event=self.event).load(names)
        if self.registry.failed:
            self.get_logger().warn(f"적재 실패 기능: {self.registry.failed}")
        # 기능이 setup 에서 선언한 명령 역할 중 삼킨 것이 있으면 여기서 드러낸다.
        # protocol 은 로거를 모르므로 모아 뒀다가 적재가 끝난 지금 비운다.
        for w in P.take_role_warnings():
            self.get_logger().warn(f"명령 역할 등록: {w}")

        self.create_timer(0.05, self.control_tick)      # 20 Hz — 코어 루프
        self.create_timer(0.05, self.telemetry_tick)
        self.get_logger().info(
            f"control_agent 시작 — robot_id={self.robot_id} · "
            f"기능 {len(self.registry.features)}개")
        # 무엇이 실제로 막히는지 기동 로그에 남긴다. 표만 만들고 배선을 안 한
        # 채로 "토큰 observer 2개" 를 찍던 것이 이번 결함의 뿌리였다 — 로그가
        # 걸리지도 않은 제한을 걸린 것처럼 보이게 했다.
        self.get_logger().info(
            f"권한 판정 활성 — 명령·조종 모두 역할 검사를 거친다 "
            f"(조종 {P.required_role(P.ACT_TELEOP)}, "
            f"비상정지 {P.required_role(P.CMD_ESTOP)}, "
            f"해제 승인 {P.required_role(P.CMD_CLEAR_ESTOP_REQUEST)} 이상) "
            f"— 해제는 2단계다: 관제 승인 + 현장 확인(~/local_reset)")

    def _auth_config(self, raw):
        """auth_token 파라미터를 ControlServer 가 받는 형태로 바꾼다.

        ROS 파라미터는 문자열이라 **토큰별 역할을 줄 방법이 없었다.** 그래서
        wsserver 의 사전 분기가 도달 불가였고, 결과적으로 모든 토큰이 admin
        이었다 — 역할 표를 만들어 두고도 역할을 나눌 수 없는 상태였다. JSON 을
        문자열에 담아 넘기는 길을 연다. **양쪽 다 문자열로 넘어가게 하는 것이
        핵심이다** (둘 다 실측으로 확인한 형태다):

            # launch — control.launch.py 가 ParameterValue(value_type=str) 로 받는다
            ros2 launch orchard_sim control.launch.py \
                'auth_token:={"tok-a":"admin","tok-b":"observer"}'

            # run — rcl 이 YAML 로 읽으므로 작은따옴표로 한 번 더 감싼다.
            #       안 감싸면 '{' 를 YAML 매핑으로 보고 파싱 오류로 죽는다.
            ros2 run orchard_sim control_agent --ros-args \
                -p "auth_token:='{\\"tok-a\\":\\"admin\\"}'"

        '{' 로 시작하면 JSON 으로 본다. 실제 토큰이 '{' 로 시작할 일은 없고,
        길이 하나뿐인 이 판단을 헤더나 별도 파라미터로 나누면 설정할 곳이
        늘어나 오히려 틀리기 쉽다.

        **파싱에 실패하면 통째로 토큰 하나로 취급하지 않는다.** 그러면 아무도
        모르는 긴 문자열 하나만 통과하는 상태가 되는데, 로그에는 "토큰 admin
        1개"로 정상처럼 찍힌다. 아무도 못 붙는 이유를 찾느라 헤매다 결국
        토큰을 지우고(=개방 모드로) 쓰게 된다. 그래서 빈 사전을 넘겨 서버가
        **전면 거부 + 사유 출력** 상태로 뜨게 한다.
        """
        raw = raw.strip()
        if not raw:
            self.get_logger().warn(
                "auth_token 이 비어 있다 — 접속한 누구나 로봇을 조종할 수 있다"
                "(전원 admin). 현장 전개 전에 반드시 설정할 것.")
            return None
        # '[' 도 구조를 적으려던 흔적으로 본다. 이걸 단일 토큰으로 삼키면
        # 아무도 제시하지 않을 '[1,2]' 같은 문자열이 유일한 admin 토큰이 되고,
        # 로그에는 "토큰 admin 1개"로 멀쩡하게 찍힌다 — 아무도 못 붙는 이유를
        # 찾다가 결국 토큰을 지우게 되는, 여기서 막으려는 바로 그 실패다.
        if raw[0] not in "{[":
            return raw                  # 단일 토큰 = admin (하위호환)
        try:
            obj = json.loads(raw)
        except Exception as e:
            self.get_logger().error(
                f"auth_token 이 '{raw[0]}' 로 시작하는데 JSON 이 아니다 ({e}) — "
                f"모든 접속을 거부한다. 형식: "
                f'{{"토큰":"observer|operator|admin", ...}}')
            return {}                   # 빈 사전 = 전면 거부 (개방 모드가 아니다)
        if not isinstance(obj, dict):
            self.get_logger().error(
                f"auth_token JSON 이 사전이 아니다 ({type(obj).__name__}) — "
                f"모든 접속을 거부한다. 형식: "
                f'{{"토큰":"observer|operator|admin", ...}}')
            return {}
        return obj

    def _default_static(self):
        try:
            from ament_index_python.packages import get_package_share_directory
            return os.path.join(get_package_share_directory("orchard_sim"), "web")
        except Exception:
            return os.path.join(os.path.dirname(__file__), "..", "web")

    # ═══════════════════════════════════════════════════════════════════════
    # 공통 상태 수집
    # ═══════════════════════════════════════════════════════════════════════
    def _on_lio(self, msg):
        self.rate["lio"].tick(time.monotonic())
        p = msg.pose.pose.position
        self.bb.set(lio_pose=(p.x, p.y, p.z))

    def _on_local_reset(self):
        """현장 확인 — 기체 곁의 사람이 위험구역을 보고 누른 것으로 간주한다."""
        ok, why = self.safety.local_reset("현장")
        if ok and not self.safety.estop:              # 관제 승인이 이미 있었다
            self.bb.extra["mode"] = P.MODE_IDLE
        self.get_logger().warn(f"현장 리셋 입력 — {why}")
        if self.audit:
            self.audit.command("local_reset", role="현장", addr="robot",
                               result=("수락" if ok else "거부"))

    def _on_imu(self, msg):
        self.rate["imu"].tick(time.monotonic())
        q = msg.orientation
        if q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 0.5:
            self._imu_R = tfu.quat_to_matrix(q.x, q.y, q.z, q.w)

    def _level_points(self, p):
        """점군을 기울기 보정한다 (요 제외 — 롤·피치만 편다).

        경사에서 코가 숙으면 라이다가 지면을 '벽'으로 읽는다 — 실측: 남단
        내리막(피치 −16°)에서 여유거리 0.25 m 로 읽혀 출구 도착이 3.5 m
        조기 발동, 로봇이 열 끝 모서리에 쐐기로 박혔다(08-02). z 필터와
        여유거리는 수평화된 점에서 재야 한다.
        """
        R = self._imu_R
        if R is None:
            return p
        yaw = math.atan2(R[1, 0], R[0, 0])
        c, s = math.cos(-yaw), math.sin(-yaw)
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        return p @ (Rz @ R).T           # (요 제거한 기울기)로 점을 편다

    def _on_cloud(self, msg):
        """점군에서 두 가지를 읽는다 — 전방 여유거리와 코앞 밀착률.

        여유거리(중앙 ±8° 원뿔의 하위 10% 거리)는 헤드랜드에서 둑까지의
        거리다 — 임무 기능이 '벽 앞 도착' 판정에 쓴다. 밀착률(0.8 m 안
        점 비율)이 높으면 코가 무언가에 박힌 것이다: 라이다가 흙만 보므로
        로컬리제이션도 슬립 감지도 눈이 먼다(08-02 실측). 여기서만 잡을
        수 있으니 여기서 세운다.
        """
        self.rate["lidar"].tick(time.monotonic())
        self._cloud_n += 1
        if self._cloud_n % 3:           # 10 Hz 입력을 3.3 Hz 로 솎는다
            return
        from orchard_sim.map_localizer import read_xyz
        p = read_xyz(msg)
        if len(p) < 200:
            return
        p = self._level_points(p)
        r = np.hypot(p[:, 0], p[:, 1])
        near_frac = float((r < 0.8).mean())
        ang = np.abs(np.arctan2(p[:, 1], p[:, 0]))
        cone = (ang < math.radians(8.0)) & (r > 0.25) & (p[:, 2] > -0.35)
        clearance = (float(np.percentile(r[cone], 10))
                     if int(cone.sum()) >= 30 else float("inf"))
        self.bb.set(clearance=clearance, near_frac=near_frac)

        # 밀착 정지 — 임무 중 2초 넘게 코앞이 막혀 있으면 박힌 것이다
        now = time.monotonic()
        in_mission = self.bb.extra.get("mode") == P.MODE_MISSION
        if near_frac > 0.6 and in_mission and not self.safety.paused:
            if self._blocked_since is None:
                self._blocked_since = now
            elif now - self._blocked_since > 2.0:
                self._blocked_since = None
                self.safety.set_paused(True)
                self._write_cmd(0.0, 0.0)
                self.event("assistance",
                           f"전방 밀착 {near_frac:.0%} — 장애물 접촉으로 정지",
                           level="critical", code="OBSTACLE_FRONT")
        else:
            self._blocked_since = None

    def _on_loc_diag(self, msg):
        """로컬라이저 진단을 관제 이벤트로 올린다 (code 가 개입 큐 라우팅 키).

        TRACTION_LOSS 는 즉시 일시정지다. 슬립 상태에서 명령을 계속 내리면
        오도메트리만 쌓이고 기체는 제자리에서 갈린다. 재개는 사람이 한다 —
        박힌 원인(나무·진흙)을 치우지 않으면 재개해도 똑같이 박힌다.
        """
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        kind, code = d.get("kind"), d.get("code", "")
        text = d.get("msg", "")
        if kind == "assistance":
            # 슬립, 또는 장기 위치상실 격상(critical) — 계속 달리면 추정은
            # 오도메트리 환상이 된다. 세우고 사람을 부른다. 단 헤드랜드
            # 구간의 슬립은 먼저 스스로 물러났다 재시도한다 (최대 2회).
            must_stop = (code == "TRACTION_LOSS"
                         or d.get("severity") == "critical")
            ms = self.bb.extra.get("mission_status") or {}
            phase, widx = ms.get("phase"), ms.get("idx")
            if (code == "TRACTION_LOSS" and phase in ("exit", "cross", "enter")
                    and self._recover_tries.get(widx, 0) < 2
                    and not self.safety.paused):
                self._recover_tries[widx] = self._recover_tries.get(widx, 0) + 1
                self._recover_until = time.monotonic() + 4.0
                self.event("assistance",
                           f"클라임 슬립 — 후진 재시도 "
                           f"{self._recover_tries[widx]}/2 ({text})",
                           level="warn", code=code)
                return
            if must_stop and not self.safety.paused:
                self.safety.set_paused(True)
                self._write_cmd(0.0, 0.0)
                self.event("paused", f"로컬라이저 경보로 자동 정지 — {text}",
                           level="warn")
            self.event("assistance", text, level="warn", code=code)
        elif kind == "resolved":
            self.event("resolved", text, level="info", code=code)

    def _read_pose(self):
        try:
            tr = self.buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None, 0.0
        t, q = tr.transform.translation, tr.transform.rotation
        R = tfu.quat_to_matrix(q.x, q.y, q.z, q.w)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))
        return (t.x, t.y, math.atan2(R[1, 0], R[0, 0])), tilt

    # ═══════════════════════════════════════════════════════════════════════
    # 관제 링크
    # ═══════════════════════════════════════════════════════════════════════
    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def next_seq(self):
        with self._lock:
            self.seq += 1
            return self.seq

    def _emit(self, kind, payload):
        self.server.broadcast(P.envelope(
            f"orchard/{self.robot_id}/{kind}", payload, self.now_ns(), self.next_seq()))

    def event(self, kind, msg, level="info", **extra):
        """이벤트 브로드캐스트 + 감사 기록.

        비상정지·전복·링크두절은 사후 조사에서 가장 먼저 찾는 것이라 반드시
        영속 기록에 남겨야 한다. extra 로 정지 사유 코드(code=...) 등을 실으면
        관제 서버가 개입 큐로 바로 라우팅한다.
        """
        e = dict(kind=kind, msg=msg, level=level, t=time.time(), **extra)
        with self._lock:
            self.events.append(e)
            self.events = self.events[-50:]
        self._emit("event", e)
        # SafetyArbiter 가 event 를 콜백으로 물고 있어 audit 생성 전에도 불릴 수 있다
        if getattr(self, "audit", None) and kind not in ("denied",):   # denied 는 _deny 가 이미 남긴다
            self.audit.event(kind, _clip(msg, 400), level)
        if level in ("warn", "critical"):
            self.get_logger().warn(f"{kind}: {msg}")

    def _on_ws_open(self, conn):
        pr = lambda k, d=None: (self.get_parameter(k).value  # noqa: E731
                                if self.has_parameter(k) else d)
        R = int(pr("rows", 10))
        conn.send_json(P.envelope(
            f"orchard/{self.robot_id}/hello",
            dict(robot_id=self.robot_id, protocol=P.PROTOCOL_VERSION,
                 rows=R, alleys=R - 1,
                 row_spacing=float(pr("row_spacing", 3.5)),
                 x0=-((R - 1) * float(pr("row_spacing", 3.5))) / 2.0,
                 col_len=(int(pr("trees_per_row", 41)) - 1) * float(pr("tree_spacing", 1.5)),
                 headland=float(pr("headland", 6.0)),
                 limits=dict(speed=float(pr("speed", 0.7)),
                             teleop_v=float(pr("teleop_max_v", 0.8)),
                             teleop_w=float(pr("teleop_max_w", 1.2))),
                 deadman_ms=P.TELEOP_DEADMAN_MS,
                 link_loss_ms=P.LINK_LOSS_STOP_MS,
                 # 대시보드가 이걸 보고 패널을 켜고 끈다 — 기능을 빼면 화면도 준다
                 features=self.registry.describe()),
            self.now_ns(), self.next_seq()))
        self.safety.note_client()

    def _deny(self, why, who, role, action=None):
        """권한 거부를 이벤트로 올린다. **조용히 버리지 않는다.**

        거부를 소리 없이 버리면 화면에서는 '명령이 안 먹는다'로만 보이고,
        그것이 권한 때문인지 링크가 끊긴 것인지 구분되지 않는다. 그러면
        운전자가 로봇이 멈춘 줄 알고 걸어서 접근한다 — 실제로는 임무를
        수행 중일 수 있다. 사유를 화면까지 올려야 하는 이유다.

        같은 곳에서 같은 사유가 연달아 오는 것만 눌러 준다 (DENY_REPEAT_S).
        첫 거부는 언제나 올라간다.

        **억제 열쇠에 사유 문자열을 쓰지 않는다.** 사유에는 클라이언트가 보낸
        명령 이름이 그대로 들어가서, 매번 다른 이름을 보내면 억제가 통째로
        우회된다 (2026-07-31 검증에서 실측: 같은 명령 30회 → 이벤트 1건,
        다른 명령 30회 → 30건 전부 통과). 억제가 막으려던 것은 "거부가 이벤트
        창을 뒤덮어 정작 봐야 할 경고를 밀어내는 것" 인데, 그 공격이 바로
        그것을 한다. 그래서 열쇠는 **(주소, 역할)** 로만 잡는다 — 한 접속이
        무엇을 보내든 2초에 한 번만 올라간다.

        사유 길이도 자른다. 20,000자짜리 명령 이름을 보내면 그만한 이벤트가
        접속한 모든 화면으로 브로드캐스트된다 (프레임 한도 8 MB).
        """
        now = time.monotonic()
        with self._lock:
            last = self._deny_seen.get((who, role), 0.0)
            if now - last >= DENY_REPEAT_S:
                self._deny_seen[(who, role)] = now
                if len(self._deny_seen) > DENY_KEYS_MAX:
                    # 나이로만 거르면 전부 새 항목일 때 하나도 안 지워진다.
                    # 개수 상한을 함께 두고 오래된 순으로 잘라낸다.
                    keep = [(k, v) for k, v in self._deny_seen.items()
                            if now - v < DENY_PRUNE_S]
                    keep.sort(key=lambda kv: kv[1], reverse=True)
                    self._deny_seen = dict(keep[:DENY_KEYS_MAX])
                fresh = True
            else:
                fresh = False
        if fresh:
            self.event("denied", f"{_clip(why)} · {who}", "warn")
        # 이벤트는 억제해도 **감사는 전부 남긴다.** 화면을 안 뒤덮는 것과
        # 기록을 빠뜨리는 것은 다른 문제다.
        if getattr(self, "audit", None):
            self.audit.command(_clip(action or "?"), role=role, addr=who,
                               result=R_REJECT, why=_clip(why))

    def _on_ws_message(self, conn, text):
        """워커 스레드에서 불린다. **권한 판정은 여기서 한다.**

        명령은 큐를 거쳐 control_tick 에서 소비되지만 조종은 지연에 민감해
        큐를 건너뛰고 즉시 dispatch 된다. 소비 시점에서만 막으면 조종은 그대로
        통과한다 — 하필 가장 막아야 할 것이다. 그래서 두 경로가 갈라지기
        전, 들어오는 길목 한 곳에서 판정한다.
        """
        try:
            t, payload, _, _ = P.parse(json.loads(text))
        except Exception as e:
            conn.send_json(dict(v=P.PROTOCOL_VERSION, topic="error",
                                payload=dict(msg=str(e))))
            return
        # 링크 갱신은 판정보다 먼저 한다. 거부당한 메시지도 '관제가 살아
        # 있다'는 증거다. 여기서 안 세면 observer 만 붙어 있을 때 링크두절로
        # 오인해 임무가 멈춘다 — 권한이 안전 정지를 유발하면 안 된다.
        self.safety.note_client()

        kind = P.cmd_name(t)
        if kind == "teleop":
            action = P.ACT_TELEOP
        elif kind == "cmd":
            action = payload.get("cmd")
        else:
            return                      # 규약에 없는 토픽은 무시

        role = getattr(conn, "role", P.ROLE_FALLBACK)
        ok, why = P.authorize(role, action)
        if not ok:
            addr = getattr(conn, "addr", None)
            who = f"{addr[0]}:{addr[1]}" if addr else "?"
            self._deny(why, who, role, action)
            return

        if kind == "teleop":
            # 조종은 데드맨 때문에 초당 10회 넘게 온다 — 개별 기록은 무의미하고
            # 로그만 뒤덮는다. 모드 진입/이탈만 남기면 충분하다.
            self.registry.dispatch("teleop", payload)       # 지연 민감 — 즉시
        else:
            addr = getattr(conn, "addr", None)
            who = f"{addr[0]}:{addr[1]}" if addr else "?"
            if self.audit:
                self.audit.command(_clip(str(action)), payload=payload,
                                   role=role, addr=who, result=R_ACCEPT)
            # 소비 시점에서도 '누가 시켰나'를 알 수 있어야 한다 (되받이 검사·감사).
            self.cmdq.put((role, conn.addr, payload))

    def _handle_cmd(self, role, addr, payload):
        c = payload.get("cmd")
        who = f"{addr[0]}:{addr[1]}" if addr else "큐"
        # 되받이 검사. 정상 경로는 _on_ws_message 에서 이미 걸렀지만, 큐에
        # 넣는 경로가 나중에 하나 더 생겨도 여기가 마지막 문이 된다. 판정이
        # 한 군데뿐이면 그 한 줄이 사라졌을 때 아무 흔적 없이 문이 열린다 —
        # 이번에 고친 결함이 정확히 그 모양이었다.
        ok, why = P.authorize(role, c)
        if not ok:
            self._deny(why, who, role, c)     # 예전엔 미정의 이름(action)이라 NameError 였다
            return
        # 비상정지 계열은 기능에 맡기지 않는다 — 코어가 직접 처리한다
        if c == P.CMD_ESTOP:
            self.safety.trigger(payload.get("reason", "관제 지시"))
            return
        if c == P.CMD_CLEAR_ESTOP_REQUEST:
            ok, why = self.safety.request_clear(who or "관제")
            if ok and not self.safety.estop:          # 현장 확인이 이미 끝나 있었다
                self.bb.extra["mode"] = P.MODE_IDLE
            self._emit("event", dict(kind="estop_clear", msg=why,
                                     level="warn", t=time.time()))
            return
        if c == P.CMD_CLEAR_ESTOP_CANCEL:
            self.safety.cancel_clear(who or "관제")
            return
        if c == P.CMD_SET_SERVICE_MODE:
            self.safety.set_service_mode(str(payload.get("mode", "")), who or "관제")
            return
        if c == P.CMD_LOCAL_RESET:
            # 링크로 온 '현장 확인'은 현장 확인이 아니다 — 절차의 존재 이유가
            # 시야를 가진 사람이므로, 원격에서 두 단계를 다 눌러 버리면 무의미하다.
            self._deny("현장 확인은 로봇에서만 가능합니다 (~/local_reset)",
                       who, role, c)
            return
        if c == P.CMD_PING:
            self._emit("event", dict(kind="pong", msg="pong", level="info",
                                     t=time.time()))
            return
        if not self.registry.dispatch(c, payload):
            self.event("rejected", f"처리할 기능이 없는 명령: {c}", "warn")

    # ═══════════════════════════════════════════════════════════════════════
    # 코어 루프
    # ═══════════════════════════════════════════════════════════════════════
    def _write_cmd(self, v, w):
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self.pub_cmd.publish(m)

    def control_tick(self):
        while True:
            try:
                role, addr, payload = self.cmdq.get_nowait()
            except queue.Empty:
                break
            try:
                self._handle_cmd(role, addr, payload)
            except Exception as e:
                self.get_logger().warn(f"명령 처리 실패: {e}")

        now = time.monotonic()
        pose, tilt = self._read_pose()
        self.bb.set(pose=pose, tilt_deg=tilt,
                    rates={k: m.hz(now) for k, m in self.rate.items()})
        self.bb.extra["clients"] = self.server.client_count()

        self.safety.check_attitude(tilt if pose is not None else None)
        self.safety.update_link(self.server.client_count(), now)

        # 슬립 자율 복구 — 사람이 하듯 조금 물러났다 다시 돌진한다.
        # 클라임 슬립비(0.3~0.5)가 감시 문턱(0.35)에 걸쳐 있어, 즉시 정지만
        # 하면 횡단 시도의 절반이 사람 손을 기다린다 (실측: 남단 4회 정지).
        if self._recover_until > now:
            self._write_cmd(-0.3, 0.0)
            return
        requests = self.registry.tick(now)
        v, w, _why = self.safety.arbitrate(requests, now)
        if v == 0.0 and w == 0.0 and not self.idle_publish:
            return                       # 다른 주행 노드와 공존 관찰 모드
        self._write_cmd(v, w)

    def telemetry_tick(self):
        if self.server.client_count() == 0:
            return
        now = time.monotonic()
        for kind, payload in self.registry.telemetry(now):
            self._emit(kind, payload)

    def destroy_node(self):
        try:
            self.registry.teardown()
            self._write_cmd(0.0, 0.0)
            self.server.stop()
        except Exception:
            pass
        try:
            if self.audit:
                self.audit.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControlAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
