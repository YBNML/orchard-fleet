"""Localizer·Perception SDK 구현 — TF 포즈·점군 원뿔·로컬라이저 진단.

control_agent 에 흩어져 있던 **센서 해석**을 한 곳에 모은 것이다. 해석만
한다 — 멈출지 말지(밀착 정지·슬립 후진 재시도)는 코어의 판단이라 여기 두지
않았다. 그래야 기체를 바꿀 때 이 파일만 갈아끼우면 되고, 안전 정책은 코어에
그대로 남는다.

**여기 있는 숫자는 실측으로 정해진 값이다.** 옮기면서 하나라도 바뀌면 주행이
달라진다 — 원뿔 반각 ±8°, 자기반사 제외 r>1.6, 수관·지면 제외 z∈(−0.35, 0.9)
(MID-360), 하위 10% 백분위, 점 200개 하한, 3프레임 솎기, 밀착 반경 0.8 m 는
control_agent 에 있던 식과 주석을 그대로 옮겼다.

이 클래스는 콜백을 직접 구독하지 않는다. 구독은 노드(control_agent)가 하고
메시지만 feed_* 로 흘려준다 — 토픽 이름·QoS 는 파라미터로 정해지는 노드의
몫이고, 어댑터가 그것까지 알면 재사용이 안 된다.
"""
from __future__ import annotations

import json
import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from orchard_sim import transforms as tfu
from robomw.sdk.interfaces import Localizer, Perception
from robomw.sdk.types import Pose

# 수신율 창(초) — self_test(ScoutDiag)의 lidar/imu 판정용. control_agent 의
# RateMeter(표시용, 표본 개수 창)와 다르게 **시간 창**을 쓴다: 자가진단은
# "지금 죽었나"를 즉시 답해야 하는데, 표본 개수 창은 저빈도 구간에서 창이
# 다 차기까지 느리다. 시간 창은 창 밖으로 나간 표본이 바로 빠져 반응이 빠르다.
RATE_WINDOW_S = 3.0

# 재정위 확인 — 요청을 낸 뒤 이 시간 안에 로컬라이저가 이 품질 이상을 보고해야
# 성공으로 답한다. 문턱 0.3 은 ScoutDiag 의 localizer 자가진단과 같은 값이다
# (같은 "측위가 살아 있다"는 판정을 두 곳이 다른 잣대로 재면 안 된다).
REINIT_TIMEOUT_S = 2.0
REINIT_MIN_QUALITY = 0.3
LOC_DIAG_TOPIC = "/map_localizer/diagnostics"
LOC_REINIT_TOPIC = "/map_localizer/reinit"


class RosSensors(Localizer, Perception):

    def __init__(self, node, reinit_topic=LOC_REINIT_TOPIC,
                 diag_topic=LOC_DIAG_TOPIC):
        self._node = node
        self.buf = Buffer()
        self._tfl = TransformListener(self.buf, node)
        self._imu_R = None              # 기체 자세 (월드←바디) — 점군 수평화용
        self._cloud_n = 0
        self._clearance = float("inf")
        self._near_frac = 0.0
        self._diag = {}                 # 마지막 로컬라이저 진단 payload
        self._lio = None                # 마지막 LIO 위치 (x, y, z)
        # 수신 시각 카운터 — feed_cloud/feed_imu 가 호출될 때마다(솎기 전) 시각을
        # 남긴다. rates() 가 3초 창으로 Hz 를 낸다.
        self._rx_times = {"lidar": deque(), "imu": deque()}

        # ── 재정위(reinit) 배선 ────────────────────────────────────────────
        # 요청은 노드의 발행자로 나가지만, **응답은 이 클래스가 직접 받는다.**
        # 다른 입력(feed_*)과 달리 reinit 은 동기 계약(bool 반환)이라 호출자가
        # 답을 기다리는 동안 호스트 노드의 콜백은 돌지 못한다 — 그 노드의
        # 실행기가 지금 이 함수 안에 갇혀 있기 때문이다. 그래서 진단만 받는
        # 작은 전용 노드를 따로 두고 그 노드만 제 실행기로 돌린다. 호스트의
        # 구독(_on_loc_diag)은 그대로고 이쪽은 순전히 대기용 귀다.
        self._reinit_pub = node.create_publisher(String, reinit_topic, 10)
        self._ack_node = Node(f"{node.get_name()}_relocalize_ack")
        self._ack_node.create_subscription(String, diag_topic, self._on_ack, 10)
        self._ack_exec = SingleThreadedExecutor()
        self._ack_exec.add_node(self._ack_node)
        self._ack_quality = None        # 마지막으로 받은 재정위 확인 품질

    def _note_rx(self, key, now=None):
        now = now if now is not None else time.monotonic()
        dq = self._rx_times[key]
        dq.append(now)
        cutoff = now - RATE_WINDOW_S
        while dq and dq[0] < cutoff:
            dq.popleft()

    def rates(self):
        """수신율(Hz) — self_test(ScoutDiag) 가 읽는다.

        3초 창 안 도착 시각의 간격으로 잰다. 창 안에 표본이 2개 미만이면(막
        시작했거나 완전히 끊긴 것) 0.0 — "모른다"가 아니라 "지금은 없다"로
        답해야 자가진단이 센서 죽음을 놓치지 않는다.
        """
        now = time.monotonic()
        out = {}
        for key, dq in self._rx_times.items():
            cutoff = now - RATE_WINDOW_S
            while dq and dq[0] < cutoff:
                dq.popleft()
            if len(dq) >= 2 and dq[-1] > dq[0]:
                out[key] = (len(dq) - 1) / (dq[-1] - dq[0])
            else:
                out[key] = 0.0
        return out

    # ── 입력 (노드 콜백이 흘려준다) ─────────────────────────────────────────
    def feed_imu(self, msg):
        """IMU 자세를 기억한다. 점군 수평화(_level_points)에만 쓴다."""
        self._note_rx("imu")
        q = msg.orientation
        if q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 0.5:
            self._imu_R = tfu.quat_to_matrix(q.x, q.y, q.z, q.w)

    def feed_lio(self, msg):
        """LIO 오도메트리 위치를 기억하고 (x, y, z) 로 돌려준다."""
        p = msg.pose.pose.position
        self._lio = (p.x, p.y, p.z)
        return self._lio

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

    def feed_cloud(self, msg):
        """점군에서 두 가지를 읽는다 — 전방 여유거리와 코앞 밀착률.

        여유거리(중앙 ±8° 원뿔의 하위 10% 거리)는 헤드랜드에서 둑까지의
        거리다 — 임무 기능이 '벽 앞 도착' 판정에 쓴다. 밀착률(0.8 m 안
        점 비율)이 높으면 코가 무언가에 박힌 것이다: 라이다가 흙만 보므로
        로컬리제이션도 슬립 감지도 눈이 먼다(08-02 실측).

        (여유거리, 밀착률) 을 돌려준다. 솎였거나 점이 너무 적어 이번 프레임을
        건너뛰었으면 None — 호출부는 **직전 값을 그대로 유지**해야 한다
        (0 이나 inf 로 덮으면 임무가 없는 벽을 보거나 있는 벽을 못 본다).
        """
        # 수신율은 솎기 전에 잰다 — self_test 의 lidar 판정은 "브리지가 살아
        # 있는가"를 보는 것이지 "처리에 쓰는 프레임 수"가 아니다. 원본은
        # ~10 Hz, 처리는 3.3 Hz 로 솎는데 판정 문턱(8 Hz)은 원본 기준이다.
        self._note_rx("lidar")
        self._cloud_n += 1
        if self._cloud_n % 3:           # 10 Hz 입력을 3.3 Hz 로 솎는다
            return None
        from orchard_sim.map_localizer import read_xyz
        p = read_xyz(msg)
        if len(p) < 200:
            return None
        p = self._level_points(p)
        r = np.hypot(p[:, 0], p[:, 1])
        near_frac = float((r < 0.8).mean())
        ang = np.abs(np.arctan2(p[:, 1], p[:, 0]))
        cone = (ang < math.radians(8.0)) & (r > 1.6) \
            & (p[:, 2] > -0.35) & (p[:, 2] < 0.9)   # 수관·자기반사 제외 (MID-360)
        clearance = (float(np.percentile(r[cone], 10))
                     if int(cone.sum()) >= 30 else float("inf"))
        self._clearance, self._near_frac = clearance, near_frac
        return clearance, near_frac

    def feed_diag(self, msg):
        """로컬라이저 진단(JSON 문자열)을 사전으로 푼다. 형식이 아니면 None.

        무엇을 할지는 코어가 정한다 — 여기서는 파싱과 보관만 한다.
        """
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            return None
        if isinstance(d, dict):
            self._diag = d
        return d

    # ── Localizer ───────────────────────────────────────────────────────────
    def pose_tilt(self):
        """(x, y, yaw) 와 기울기(도)를 함께 돌려준다. 못 얻으면 (None, 0.0).

        코어는 포즈와 기울기를 같은 시점의 한 쌍으로 써야 한다(전복 판정이
        포즈 유무에 딸려 있다). TF 를 두 번 뒤지면 그 사이가 벌어지므로 한
        번에 돌려준다 — SDK 의 pose() 는 이 결과를 감싼 것이다.
        """
        try:
            tr = self.buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None, 0.0
        t, q = tr.transform.translation, tr.transform.rotation
        R = tfu.quat_to_matrix(q.x, q.y, q.z, q.w)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))
        return (t.x, t.y, math.atan2(R[1, 0], R[0, 0])), tilt

    def pose(self):
        """SDK 규약의 포즈. 측위가 아직 없으면 None.

        quality 는 1.0 고정이다 — 이 배치의 map→base_link 는 로컬라이저가
        내는 단일 해라 신뢰도 척도가 따로 없다. 융합 추정으로 바꿀 때 여기에
        실으면 된다(임의의 숫자를 지금 지어내지 않는다).
        """
        xyt, _tilt = self.pose_tilt()
        return None if xyt is None else Pose(xyt[0], xyt[1], xyt[2])

    def _on_ack(self, msg):
        """전용 노드의 진단 콜백 — 재정위 확인(quality)이 실린 것만 챙긴다.

        받은 payload 는 diagnostics() 스냅샷에도 그대로 반영한다. 실패를 보고할
        때 관제가 보는 것이 바로 이 스냅샷이라, 재정위 직후의 최신 값이어야
        "왜 못 잡았나"를 답할 수 있다.
        """
        d = self.feed_diag(msg)
        if isinstance(d, dict) and "quality" in d:
            try:
                self._ack_quality = float(d["quality"])
            except (TypeError, ValueError):
                pass

    def reinit(self, pose):
        """포즈 재초기화(relocalize) — 요청을 보내고 **잡혔는지 확인해서** 답한다.

        로컬라이저에게 새 자세를 주는 것(발행)은 즉시 끝나지만, 그것이 옳은
        자세였는지는 다음 보정이 나와야 안다. 여기서 그 확인까지 기다리는
        이유: 이 함수의 반환값이 관제 화면의 completed/failed 가 된다. 발행만
        하고 True 를 돌려주면 "재정위 완료"라는 초록불 아래에서 로봇은 여전히
        엉뚱한 곳을 자기 자리로 믿는다.

        기다리는 동안 호스트 노드의 콜백은 멈춘다(그 실행기가 이 함수를 부른
        장본인이다). 그래서 **정지 상태에서만 이 명령이 온다**는 전제가
        중요하다 — 기능(maintenance)이 유휴가 아니면 BUSY 로 먼저 막는다.
        전제가 깨지면 대기하는 동안 속도 명령·데드맨·비상정지 처리가 함께
        멈추고, 구동부는 마지막 속도를 그대로 유지한다.

        **True 가 증명하는 것의 한계.** 잣대인 품질은 스캔 구조의 위상
        집중도다 — 요 오차에는 민감하지만 위치 오지정에는 둔감하다(과수원은
        주기 구조라 한 통로 옆에서도 같은 값이 나온다). True 는 "재초기화 뒤
        측위가 건강하다"이지 "준 자세가 옳다"가 아니다.

        Returns:
            bool: 한도(2초) 안에 품질 0.3 이상을 확인했으면 True.
        """
        # 밀려 있던 옛 진단부터 비운다 — 재정위 **이전**의 품질을 답으로
        # 오인하면 엉뚱한 자세를 completed 로 보고한다.
        for _ in range(50):
            self._ack_exec.spin_once(timeout_sec=0.0)
        self._ack_quality = None
        self._reinit_pub.publish(String(data=json.dumps(
            dict(x=float(pose.x), y=float(pose.y), yaw=float(pose.yaw)))))
        deadline = time.monotonic() + REINIT_TIMEOUT_S
        while time.monotonic() < deadline:
            self._ack_exec.spin_once(timeout_sec=0.05)
            q = self._ack_quality
            if q is not None and q >= REINIT_MIN_QUALITY:
                return True
        return False

    def diagnostics(self):
        """마지막 로컬라이저 진단 payload 사본. 아직 없으면 빈 사전."""
        return dict(self._diag)

    # ── Perception ──────────────────────────────────────────────────────────
    def clearance(self):
        """전방 여유거리(m). 원뿔에 점이 없으면 float("inf") — 개활."""
        return self._clearance

    def near_frac(self):
        """코앞 0.8 m 안 점 비율 (0.0~1.0)."""
        return self._near_frac
