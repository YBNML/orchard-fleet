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
from sensor_msgs.msg import PointCloud2
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
        self.tfb = TransformBroadcaster(self)
        self.diag = self.create_publisher(String, "~/diagnostics", 10)
        self.create_timer(1.0 / max(float(g("publish_rate_hz")), 1.0), self._tick)
        self.create_timer(3.0, self._log_stat)      # 채택/거부가 눈에 보여야 한다
        self.get_logger().info(
            f"초기 자세 map→odom = ({self.T_mo[0]:.2f}, {self.T_mo[1]:.2f}, "
            f"{math.degrees(self.T_mo[2]):.1f}°)")

    # ── 입력 ────────────────────────────────────────────────────────────────
    def _on_odom(self, msg: Odometry):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        R = tfu.quat_to_matrix(q.x, q.y, q.z, q.w)
        self.T_ob = (p.x, p.y, math.atan2(R[1, 0], R[0, 0]))
        if not self._have_odom:
            self.T_mo = compose(self.init_pose, inverse(self.T_ob))
            self.get_logger().info(
                f"오도메트리 기준점 정렬 — odom→base ({self.T_ob[0]:+.2f}, "
                f"{self.T_ob[1]:+.2f}) 를 상쇄")
        self._have_odom = True
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
    def _check_slip(self, sp):
        """오도메트리 변위와 스캔 변위를 대조한다.

        오도메트리가 slip_check 만큼 갔다고 할 때마다 기준 스캔과 상관을
        구한다. 스캔이 "안 갔다"고 두 번 연속 답하면 슬립 확정 — 자세를
        직전 앵커에 묶고(오도메트리 무시) 개입을 요청한다. 회전 중에는
        1차원 상관이 성립하지 않으므로 기준만 갱신하고 판단하지 않는다.
        """
        if len(sp) < 150:
            self._slip_ref = None       # 구조가 없으면 판단 불가
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

    # ── 보정 ────────────────────────────────────────────────────────────────
    def _try_fix(self):
        pts = self.last_cloud
        if pts is None or len(pts) == 0:
            return
        self._check_slip(rl.structure_points(pts))
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
            + (" · 슬립 동결 중" if self.slip_active else "")
            + (f" · 거부사유: {self.stat['gate']}" if self.stat["gate"] else ""))

    def _emit(self, kind, code, msg):
        self.diag.publish(String(data=json.dumps(
            dict(kind=kind, code=code, msg=msg, severity="warn",
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

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = float(self.T_mo[0])
        t.transform.translation.y = float(self.T_mo[1])
        t.transform.rotation.z = math.sin(self.T_mo[2] / 2.0)
        t.transform.rotation.w = math.cos(self.T_mo[2] / 2.0)
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
