#!/usr/bin/env python3
"""map_localizer — 사전 맵 위에서 위치를 잡는 노드 (gt_localizer 의 대체재)

    ros2 run orchard_sim map_localizer --ros-args \
        -p bundle:=maps/orchard_v1 -p init_x:=-14.0 -p init_y:=-28.0 -p init_yaw:=1.5708

무엇을 내는가
    map → odom 변환 하나. 로봇의 휠 오도메트리가 odom → base_link 를 내므로,
    둘을 이으면 map → base_link 가 완성된다. gt_localizer 와 **같은 인터페이스**라
    둘을 바꿔 끼우며 비교할 수 있다.

어떻게 잡는가
    휠 오도메트리가 자세를 밀고 가고(고빈도), 라이다 스캔이 사전 맵의 나무 열에
    대해 그 자세를 되돌려 놓는다(저빈도). 되돌리는 계산은 rowlocalize 에 있다.

    핵심은 **방향마다 다르게 다룬다**는 것이다. 과수원은 주기 구조라
        횡·요  나무 열이 절대 기준을 준다 → 매번 보정 (누적되지 않음)
        종     1.5 m 마다 같은 그림 → 위상으로 묶어만 두고 절대 기준은 열 끝에서
    이 비대칭을 무시하면 종방향이 한 칸 미끄러진 해로 조용히 수렴한다.

무엇을 안 하는가
    보정을 못 믿을 때는 **하지 않는다**. 구조가 부족하거나(선회 구역) 마지막
    보정 이후 표류가 열 간격 절반을 넘었으면 그대로 오도메트리로 간다.
    틀린 보정은 보정을 안 하느니만 못하다. 그 상태가 오래 가면 정지 사유
    LOCALIZATION_LOST 로 개입을 요청한다 (관제의 개입 큐로 올라간다).
"""
from __future__ import annotations

import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from orchard_sim import mapbundle
from orchard_sim import rowlocalize as rl
from orchard_sim import transforms as tfu


def read_xyz(msg: PointCloud2) -> np.ndarray:
    off = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3))
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)

    def f32(o):
        return raw[:, o:o + 4].copy().view(np.float32).ravel()

    return np.stack([f32(off["x"]), f32(off["y"]), f32(off["z"])], axis=1)


# ── 2D 자세 대수 (x, y, yaw) ────────────────────────────────────────────────
def compose(a, b):
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (a[0] + ca * b[0] - sa * b[1],
            a[1] + sa * b[0] + ca * b[1],
            a[2] + b[2])


def inverse(a):
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (-ca * a[0] - sa * a[1], sa * a[0] - ca * a[1], -a[2])


def wrap(t):
    return (t + math.pi) % (2 * math.pi) - math.pi


class MapLocalizer(Node):

    def __init__(self):
        super().__init__("map_localizer")
        d = self.declare_parameter
        d("bundle", "maps/orchard_v1")
        d("map_frame", "map")
        d("odom_frame", "odom")
        d("base_frame", "base_link")
        d("cloud_topic", "/livox/lidar")
        d("odom_topic", "/odom")
        d("publish_rate_hz", 30.0)
        d("fix_period_s", 0.5)          # 보정 주기 (2 Hz)
        d("init_x", 0.0); d("init_y", 0.0); d("init_yaw", 0.0)
        d("lost_timeout_s", 8.0)        # 이 시간 넘게 못 잡으면 개입 요청
        d("slip_check_m", 0.5)          # 오도메트리가 이만큼 갈 때마다 스캔과 대조
        d("slip_ratio", 0.35)           # 스캔변위/오도변위 가 이 밑이면 슬립
        d("reacquire_after_s", 5.0)     # 이만큼 못 잡으면 요 탐색을 넓혀 재획득
        d("reacquire_yaw_deg", 30.0)
        d("anchor_max_range_m", 12.0)   # 둑 앵커 — 이 거리 안의 벽만 믿는다
        d("anchor_gain", 0.5)
        d("anchor_wall_offset_m", 0.7)  # 주행불가 경계(둑 발치)와 라이다가 보는
                                        # 면(0.3 m 높이) 사이 법면 후퇴량 (08-02 실측)
        d("sensor_fwd_m", 0.275)        # 라이다 광학 원점의 base_link 전방 오프셋
        d("lost_critical_s", 90.0)      # 이만큼 못 잡으면 격상 — 로봇을 세워야 한다
        d("imu_topic", "/imu")          # 요는 자이로 적분 — 바퀴는 회전을 속인다
        g = lambda k: self.get_parameter(k).value                     # noqa: E731

        self.bundle = mapbundle.Bundle(str(g("bundle")))
        self.geom = self.bundle.meta["geom"]
        self.get_logger().info(
            f"맵 번들 적재 — 해시 {self.bundle.hash} · 통로 {self.bundle.alley_count()}개"
            f" · 무결성 {'OK' if self.bundle.verify() else '불일치!'}")

        self.map_frame = str(g("map_frame"))
        self.odom_frame = str(g("odom_frame"))
        self.fix_period = float(g("fix_period_s"))
        self.lost_timeout = float(g("lost_timeout_s"))
        self.slip_check = float(g("slip_check_m"))
        self.slip_ratio = float(g("slip_ratio"))
        self.reacquire_after = float(g("reacquire_after_s"))
        self.reacquire_yaw = float(g("reacquire_yaw_deg"))
        self.anchor_max_range = float(g("anchor_max_range_m"))
        self.anchor_gain = float(g("anchor_gain"))
        self.anchor_wall_off = float(g("anchor_wall_offset_m"))
        self.sensor_fwd = float(g("sensor_fwd_m"))
        self.lost_critical = float(g("lost_critical_s"))
        self._lost_critical_reported = False

        # 자이로 요 적분 — 램프에서 궤도가 헛돌면 바퀴는 "회전했다"고 속이지만
        # (실측: 선회 명령 두 번에 실제 회전 3°) 자이로는 몸체의 실제 회전을
        # 잰다. 요만 자이로에서 받고, 병진은 휠 오도메트리 변위 크기를 그 요
        # 방향으로 다시 적분한다 (표준 추측항법 재구성).
        self._imu_yaw = 0.0
        self._imu_t = None
        self._odom_prev = None          # 직전 odom 원자세 (x, y, yaw)
        self._odom_raw = (0.0, 0.0, 0.0)  # 바퀴 오도메트리 원값 — TF 발행 보정용

        # map → odom. 초기값은 '초기 자세를 안다'는 전제에서 만든다
        # (설계: 대시보드에서 지정하거나 지정 주차 지점에서 기동)
        # init_* 는 **로봇의 초기 자세**지 map→odom 이 아니다. 오도메트리는 이미
        # 얼마간 누적돼 있을 수 있으므로, 첫 오도메트리를 받은 순간
        #     map→odom = 초기자세 ∘ (odom→base)⁻¹
        # 로 잡아야 한다. 이 한 줄을 빼먹으면 시작부터 그만큼 어긋난 채로 간다.
        self.init_pose = (float(g("init_x")), float(g("init_y")), float(g("init_yaw")))
        self.T_mo = self.init_pose
        self.T_ob = (0.0, 0.0, 0.0)     # 휠 오도메트리가 채운다
        self._have_odom = False

        self.last_cloud = None
        self.last_fix_t = 0.0
        self.last_ok_t = time.monotonic()
        self.drift_ref = None           # 마지막 채택 보정 시점의 odom 자세
        self.stat = dict(n_fix=0, n_reject=0, quality=0.0, n_struct=0, gate="")
        self._lost_reported = False

        # 슬립 감시 — 오도메트리가 간다는데 스캔이 안 간다면 바퀴가 헛도는 것.
        # 나무에 박힌 채 오도메트리만 35 m 달아난 사고(08-02)의 재발 방지다.
        self._slip_ref = None           # (구조점, 그때 odom, 그때 map 자세)
        self._slip_count = 0
        self._slip_anchor = None        # 슬립 시작 직전의 map 자세 — 여기 묶는다
        self.slip_active = False

        sqos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=2)
        self.create_subscription(PointCloud2, str(g("cloud_topic")),
                                 self._on_cloud, sqos)
        self.create_subscription(Odometry, str(g("odom_topic")), self._on_odom, 20)
        self.create_subscription(Imu, str(g("imu_topic")), self._on_imu, sqos)
        self.tfb = TransformBroadcaster(self)
        self.diag = self.create_publisher(String, "~/diagnostics", 10)
        self.create_timer(1.0 / max(float(g("publish_rate_hz")), 1.0), self._tick)
        self.create_timer(3.0, self._log_stat)      # 채택/거부가 눈에 보여야 한다
        self.get_logger().info(
            f"초기 자세 map→odom = ({self.T_mo[0]:.2f}, {self.T_mo[1]:.2f}, "
            f"{math.degrees(self.T_mo[2]):.1f}°)")

    # ── 입력 ────────────────────────────────────────────────────────────────
    def _on_imu(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._imu_t is not None:
            dt = t - self._imu_t
            if 0.0 < dt < 0.5:
                self._imu_yaw += msg.angular_velocity.z * dt
        self._imu_t = t

    def _on_odom(self, msg: Odometry):
        """휠 오도메트리 + 자이로 요를 자체 추측항법 프레임으로 재적분한다.

        휠 오도메트리의 요를 그대로 쓰면 안 된다 — 램프에서 궤도가 헛돌면
        바퀴는 "회전했다"고 보고하는데 몸은 안 돌았다(08-02 실측: 선회 두 번
        명령에 실제 회전 ~3°). 그 순간 추정 방위가 열린 루프로 달아나고,
        앵커·슬립 감시는 방향 기준을 잃는다. 자이로는 몸의 실제 회전을 재므로
        요는 자이로 적분에서, 병진은 휠 변위 크기를 그 요 방향으로 적분한다.
        """
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        R = tfu.quat_to_matrix(q.x, q.y, q.z, q.w)
        raw = (p.x, p.y, math.atan2(R[1, 0], R[0, 0]))
        self._odom_raw = raw
        if not self._have_odom:
            self._odom_prev = raw
            self._imu_yaw0 = self._imu_yaw
            self.T_ob = (0.0, 0.0, 0.0)          # 자체 프레임의 원점
            self.T_mo = self.init_pose
            self._have_odom = True
            self.get_logger().info("추측항법 기준점 설정 (자이로 요 융합)")
            return
        dx, dy = raw[0] - self._odom_prev[0], raw[1] - self._odom_prev[1]
        dd = math.hypot(dx, dy)
        if dd > 0.0:
            fwd = math.cos(self._odom_prev[2]) * dx + math.sin(self._odom_prev[2]) * dy
            if fwd < 0.0:
                dd = -dd                          # 후진
        self._odom_prev = raw
        yaw = wrap(self._imu_yaw - self._imu_yaw0)
        X, Y, _ = self.T_ob
        self.T_ob = (X + dd * math.cos(yaw), Y + dd * math.sin(yaw), yaw)
        if self.slip_active:
            # 헛도는 오도메트리는 자세에 반영하지 않는다 — 앵커에 묶는다
            self.T_mo = compose(self._slip_anchor, inverse(self.T_ob))

    def _on_cloud(self, msg: PointCloud2):
        self.last_cloud = read_xyz(msg)

    # ── 현재 추정 ───────────────────────────────────────────────────────────
    def pose(self):
        return compose(self.T_mo, self.T_ob)

    def _drift_since_fix(self) -> float:
        if self.drift_ref is None:
            return 0.0
        dx = self.T_ob[0] - self.drift_ref[0]
        dy = self.T_ob[1] - self.drift_ref[1]
        return math.hypot(dx, dy)

    # ── 슬립 감시 ───────────────────────────────────────────────────────────
    def _slip_points(self, sp, pts):
        """슬립 대조에 쓸 점을 고른다 — 나무가 없으면 지형 기복으로.

        줄기 구조만 쓰면 헤드랜드(나무 없는 곳)에서 감시가 꺼진다 — 로봇이
        박히는 곳이 정확히 거기다(08-02 실측: 둑에 코 박힌 채 구조점 0).
        생구름 폴백은 단, 세로 기복이 있는 장면(둑·계단 법면)에서만 믿는다.
        평평한 맨땅의 히스토그램은 센서를 따라다녀서(장면이 아니라 스캔 패턴)
        이동 중에도 '안 움직였다'고 답하기 때문이다 — 오탐으로 로봇을 세운다.
        """
        if len(sp) >= 150:
            return sp
        r = np.hypot(pts[:, 0], pts[:, 1])
        near = pts[(r > 0.8) & (r <= 20.0)]
        if len(near) < 600:
            return None                 # 코앞이 막혔거나 장면이 없다
        z = near[:, 2]
        if float(np.percentile(z, 95) - np.percentile(z, 5)) < 0.5:
            return None                 # 평평한 맨땅 — 판단 불가
        return near

    def _check_slip(self, sp, pts):
        """오도메트리 변위와 스캔 변위를 대조한다.

        오도메트리가 slip_check 만큼 갔다고 할 때마다 기준 스캔과 상관을
        구한다. 스캔이 "안 갔다"고 두 번 연속 답하면 슬립 확정 — 자세를
        직전 앵커에 묶고(오도메트리 무시) 개입을 요청한다. 회전 중에는
        1차원 상관이 성립하지 않으므로 기준만 갱신하고 판단하지 않는다.
        """
        sp = self._slip_points(sp, pts)
        if sp is None:
            self._slip_ref = None       # 판단 불가 — 기준도 버린다
            return
        if self._slip_ref is None:
            self._slip_ref = (sp, self.T_ob, self.pose())
            return
        ref_sp, ref_ob, ref_pose = self._slip_ref
        odo_d = math.hypot(self.T_ob[0] - ref_ob[0], self.T_ob[1] - ref_ob[1])
        if abs(wrap(self.T_ob[2] - ref_ob[2])) > math.radians(6.0):
            self._slip_ref = (sp, self.T_ob, self.pose())
            return
        if odo_d < self.slip_check:
            return
        travel, conf = rl.scan_travel(ref_sp, sp)
        self._slip_ref = (sp, self.T_ob, self.pose())
        if conf < 0.3:
            return                      # 상관이 흐리면 판단 유보
        if abs(travel) / odo_d < self.slip_ratio:
            if self._slip_count == 0:
                self._slip_anchor = ref_pose    # 헛돌기 시작 전 자세
            self._slip_count += 1
        else:
            self._slip_count = 0
            if self.slip_active:
                self.slip_active = False
                self._emit("resolved", "TRACTION_LOSS", "구동이 회복되었습니다")
                self.get_logger().info("슬립 해제 — 스캔 변위가 오도메트리와 일치")
        if self._slip_count >= 2 and not self.slip_active:
            self.slip_active = True
            self.T_mo = compose(self._slip_anchor, inverse(self.T_ob))
            self._emit("assistance", "TRACTION_LOSS",
                       f"오도메트리는 {odo_d:.2f} m 전진을 보고하는데 "
                       f"스캔 변위는 {travel:.2f} m — 바퀴 헛돎")
            self.get_logger().warning(
                f"슬립 감지 — 오도 {odo_d:.2f} m vs 스캔 {travel:.2f} m "
                f"(상관 {conf:.2f}) · 자세를 앵커에 고정")

    # ── 열 끝 앵커 ──────────────────────────────────────────────────────────
    def _try_anchor(self, pts):
        """헤드랜드 접근 중 전방 둑까지의 거리로 종방향을 절대 보정한다.

        통로 안 종방향은 오도메트리뿐이다(설계 §2 — 위상은 주기 모호, 절대
        기준은 열 끝에서). 램프에서 궤도가 미끄러지면 오도메트리가 실제보다
        덜 세고, 제어기는 '아직 못 왔다'며 둑까지 밀어붙인다 — 실제로 출구
        웨이포인트를 4.3 m 지나쳐 박혔다(08-02). 전방의 둑은 맵에 있는 절대
        기준이다: 잰 거리와 맵이 기대하는 거리의 차가 곧 종방향 오차다.
        """
        est = self.pose()
        half = float(self.geom["col_len"]) / 2.0
        hx, hy = math.cos(est[2]), math.sin(est[2])
        # 가까운 끝의 둑을 **향해** 달릴 때만 잰다. 남단에서 북향 출발처럼
        # 둑을 등지고 있으면 전방에 벽이 없다 — 그때 잰 것은 벽이 아니라
        # 딴것이고, 실제로 기운 레이가 나무 열을 '벽'으로 오인해 추정을
        # 2 m 끌어내린 오발이 있었다 (08-02).
        inward = ((est[1] > half - 2.0 and hy > 0.7)
                  or (est[1] < -(half - 2.0) and hy < -0.7))
        if not inward:
            return
        r = np.hypot(pts[:, 0], pts[:, 1])
        ang = np.abs(np.arctan2(pts[:, 1], pts[:, 0]))
        cone = (ang < math.radians(8.0)) & (r > 0.3) & (pts[:, 2] > -0.35)
        if cone.sum() < 40:
            return
        measured = float(np.percentile(r[cone], 10)) + self.sensor_fwd
        if measured > self.anchor_max_range:
            return
        # 맵이 기대하는 둑까지 거리 — 주행가능 격자를 전방으로 긁는다.
        # 격자 경계는 둑 '발치'고 라이다 원뿔(0.3 m 높이)이 보는 것은 법면
        # 위쪽 면이라, 법면 후퇴량만큼 더 멀다 (보정 없이는 그만큼 편향된
        # 고정점에 수렴한다 — 08-02 실측 0.7 m). 첫 비주행 셀만 보면 기운
        # 레이가 스치는 나무 열(얇은 띠)도 잡힌다 — 1.2 m 두께가 이어져야
        # 둑이다.
        expected = None
        for s in np.arange(0.3, self.anchor_max_range + 3.0, 0.1):
            if all(not self.bundle.is_drivable(est[0] + hx * (s + q),
                                               est[1] + hy * (s + q))
                   for q in (0.0, 0.6, 1.2)):
                expected = float(s) + self.anchor_wall_off
                break
        if expected is None:
            return
        err = expected - measured       # >0: 실제가 추정보다 둑에 가깝다
        if abs(err) > 4.0:
            return                      # 상식 밖 — 벽이 아닌 것을 봤다
        corr = err * self.anchor_gain
        new_pose = (est[0] + hx * corr, est[1] + hy * corr, est[2])
        self.T_mo = compose(new_pose, inverse(self.T_ob))
        if self.slip_active:
            self._slip_anchor = new_pose
        self.drift_ref = self.T_ob
        self.stat["n_anchor"] = self.stat.get("n_anchor", 0) + 1
        self.stat["anchor_err"] = round(err, 2)

    # ── 보정 ────────────────────────────────────────────────────────────────
    def _try_fix(self):
        pts = self.last_cloud
        if pts is None or len(pts) == 0:
            return
        self._check_slip(rl.structure_points(pts), pts)
        self._try_anchor(pts)
        est = self.pose()
        # 오래 못 잡았으면 요 탐색을 넓혀 재획득 — 선회 직후에는 오도메트리
        # 요 오차가 ±12° 를 넘을 수 있다 (실측 19.5°)
        lost_for = time.monotonic() - self.last_ok_t
        if lost_for > self.reacquire_after:
            fix = rl.estimate(pts, est, self.geom,
                              yaw_range_deg=self.reacquire_yaw,
                              coarse=121, fine=25)
        else:
            fix = rl.estimate(pts, est, self.geom)
        # 슬립 중 오도메트리 변위는 허깨비다 — 표류로 세지 않는다
        drift = 0.0 if self.slip_active else self._drift_since_fix()
        ok, why = rl.gate(fix, drift, self.geom)
        self.stat.update(quality=round(fix.quality, 3), n_struct=fix.n_struct,
                         gate="" if ok else why,
                         dx=round(fix.dx, 3), dy=round(fix.dy, 3),
                         dyaw_deg=round(math.degrees(fix.dyaw), 2),
                         lon_ok=fix.longitudinal_ok, at_end=fix.at_row_end)
        if not ok:
            self.stat["n_reject"] += 1
            return

        # 보정을 로봇 자세에 적용하고, 그만큼 map→odom 을 옮긴다.
        #   새 자세 = (로봇 위치를 중심으로 dyaw 회전) + (dx, dy 평행이동)
        # 종방향(dy)은 위상 잠금이라 신뢰도가 낮다 — 절반만 반영해 흔들림을 줄인다.
        px, py, pyaw = est
        dyaw = fix.dyaw
        dy_apply = fix.dy * 0.5 if fix.longitudinal_ok else 0.0
        new_pose = (px + fix.dx, py + dy_apply, wrap(pyaw + dyaw))
        self.T_mo = compose(new_pose, inverse(self.T_ob))
        self.drift_ref = self.T_ob
        if self.slip_active:
            self._slip_anchor = new_pose    # 동결 중에도 횡·요는 계속 다듬는다
        self.last_ok_t = time.monotonic()
        self.stat["n_fix"] += 1
        if self._lost_reported:
            self._lost_reported = False
            self._lost_critical_reported = False
            self._emit("resolved", "LOCALIZATION_LOST", "위치를 다시 잡았습니다")

    def _log_stat(self):
        p = self.pose()
        n = 0 if self.last_cloud is None else len(self.last_cloud)
        self.get_logger().info(
            f"자세 ({p[0]:+.2f}, {p[1]:+.2f}, {math.degrees(p[2]):+.1f}°) · "
            f"보정 {self.stat['n_fix']}채택/{self.stat['n_reject']}거부 · "
            f"구조점 {self.stat['n_struct']} · 집중도 {self.stat['quality']:.2f} · "
            f"점군 {n} · dx {self.stat.get('dx')} dy {self.stat.get('dy')} "
            f"dyaw {self.stat.get('dyaw_deg')}° 종보정 {self.stat.get('lon_ok')}"
            + (f" · 앵커 {self.stat['n_anchor']}회(잔차 {self.stat.get('anchor_err')})"
               if self.stat.get("n_anchor") else "")
            + (" · 슬립 동결 중" if self.slip_active else "")
            + (f" · 거부사유: {self.stat['gate']}" if self.stat["gate"] else ""))

    def _emit(self, kind, code, msg, severity="warn"):
        self.diag.publish(String(data=json.dumps(
            dict(kind=kind, code=code, msg=msg, severity=severity,
                 t=time.time()), ensure_ascii=False)))

    # ── 주기 ────────────────────────────────────────────────────────────────
    def _tick(self):
        if not self._have_odom:
            return
        now = time.monotonic()
        if now - self.last_fix_t >= self.fix_period:
            self.last_fix_t = now
            self._try_fix()

        # 오래 못 잡으면 개입 요청 — 관제의 개입 큐로 올라간다
        if (now - self.last_ok_t) > self.lost_timeout and not self._lost_reported:
            self._lost_reported = True
            self._emit("assistance", "LOCALIZATION_LOST",
                       f"{now - self.last_ok_t:.0f}초째 위치 보정 실패 "
                       f"({self.stat.get('gate') or '구조 부족'})")

        # 아주 오래 못 잡으면 격상 — 이 상태로 임무를 계속하면 추정은 순수
        # 오도메트리 환상이 된다 (실측: 5분간 환상 속에서 통로 하나를 '완주',
        # 실제 로봇은 둑에 박혀 정지). 선회의 정상 무보정 구간(30~45초)보다
        # 훨씬 길게 잡아, 진짜 상실만 세운다.
        if ((now - self.last_ok_t) > self.lost_critical
                and not self._lost_critical_reported):
            self._lost_critical_reported = True
            self._emit("assistance", "LOCALIZATION_LOST",
                       f"{now - self.last_ok_t:.0f}초째 위치 상실 — 정지 필요",
                       severity="critical")

        # TF 소비자는 map→base 를 (여기서 낸 map→odom) ∘ (바퀴 odom→base) 로
        # 합성한다. 내부 추측항법은 자이로 요를 쓰므로 T_mo 를 그대로 내면
        # 바퀴 요가 자이로와 갈라진 만큼 합성 결과가 틀어진다 — 실제로 제어기가
        # 허구의 자세로 조향해 열을 넘어갔다(08-02). 발행값은 반드시
        # 융합자세 ∘ (바퀴 원값)⁻¹ 로 계산해야 합성이 융합 추정과 일치한다.
        pub = compose(self.pose(), inverse(self._odom_raw))
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = float(pub[0])
        t.transform.translation.y = float(pub[1])
        t.transform.rotation.z = math.sin(pub[2] / 2.0)
        t.transform.rotation.w = math.cos(pub[2] / 2.0)
        self.tfb.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MapLocalizer()
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
