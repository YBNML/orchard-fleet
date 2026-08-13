#!/usr/bin/env python3
"""
단계 5 검증 — Stage-0 로컬라이제이션 스캐폴드 (설계서 §7.5)

    # stage0.launch.py 가 떠 있는 상태에서
    python3 scripts/06_verify_stage0.py

확인:
  1. TF 트리가 map → odom → base_link → 센서 로 완성되는가
  2. map→base_link 가 gz 참값과 일치하는가  (이게 "완벽한 로컬라이제이션"의 정의)
  3. 로봇을 주행시켜도 참값 추종이 유지되는가 (휠 드리프트가 map→odom 에 흡수되는가)
  4. 센서 프레임 외부파라미터가 SDF 와 일치하는가
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, "ros2_ws/src/orchard_sim")
from orchard_sim import transforms as tfu  # noqa: E402
from orchard_sim.sdf_static_tf import base_relative, parse_link_poses  # noqa: E402

import argparse

# 로봇 인스턴스 이름 = 토픽·TF 접두 (다중 로봇, 2026-08-14).
# 기본 1호기라 옛 호출 형태(인자 없음)가 그대로 돈다.
_ap = argparse.ArgumentParser()
_ap.add_argument("--robot", default="scout01")
ROBOT = _ap.parse_known_args()[0].robot
BASE = f"{ROBOT}/base_link"

# model.sdf 는 여전히 한 벌이다 — 인스턴스 이름과 무관하다
SDF = "sim/models/scout_mini_mid70/model.sdf"
SENSOR_FRAMES = ["livox_frame", "imu_link", "navsat_link",
                 "cam_canopy_left", "cam_canopy_right", "cam_forward"]


class V(Node):
    def __init__(self):
        super().__init__(f"stage0_verifier_{ROBOT}")
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.gt = None
        self.gt_stamp = None
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(TFMessage, f"/{ROBOT}/gz_ground_truth",
                                 self._on_gt, q)
        self.cmd = self.create_publisher(Twist, f"/{ROBOT}/cmd_vel", 10)

    def _on_gt(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == ROBOT:
                p, r = t.transform.translation, t.transform.rotation
                self.gt = tfu.tf_from_pos_quat((p.x, p.y, p.z), (r.x, r.y, r.z, r.w))
                self.gt_stamp = t.header.stamp
                return

    def lookup(self, target, source, stamp=None):
        t = (rclpy.time.Time(seconds=stamp.sec, nanoseconds=stamp.nanosec)
             if stamp is not None else rclpy.time.Time())
        return self.buf.lookup_transform(target, source, t)

    def lookup_at_gt(self, target, source):
        """참값과 같은 시각으로 조회한다 — 시각을 섞으면 회전 중 오차가 실린다."""
        try:
            return self.lookup(target, source, self.gt_stamp)
        except Exception:
            return self.lookup(target, source)


def tf_to_matrix(tr):
    p, q = tr.transform.translation, tr.transform.rotation
    return tfu.tf_from_pos_quat((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def pose_error(A, B):
    """두 4×4 사이의 (평면거리, z차, yaw차[deg])."""
    d = A[:3, 3] - B[:3, 3]
    dyaw = tfu.yaw_of(A) - tfu.yaw_of(B)
    dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi
    return float(math.hypot(d[0], d[1])), float(d[2]), float(math.degrees(dyaw))


def main():
    rclpy.init()
    n = V()
    fails = []

    print("TF·참값 수신 대기...\n")
    deadline = time.monotonic() + 40
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.2)
        try:
            n.lookup("map", BASE)
            for f in SENSOR_FRAMES:          # static TF 전파까지 기다린다
                n.lookup(BASE, f"{ROBOT}/{f}")
            if n.gt is not None:
                break
        except Exception:
            pass
        if time.monotonic() > deadline:
            print(f"✗ 40초 안에 준비 안 됨 (참값 {'수신' if n.gt is not None else '미수신'})")
            return 2

    # ── 1. TF 트리 ──────────────────────────────────────────────────
    print("── 1. TF 트리 ──")
    chain = [("map", "odom"), ("odom", "base_link")] + \
            [("base_link", f) for f in SENSOR_FRAMES]
    for parent, child in chain:
        try:
            n.lookup(parent, child)
            print(f"  ✔ {parent} → {child}")
        except Exception as e:
            print(f"  ✘ {parent} → {child}  ({type(e).__name__})")
            fails.append(f"TF 없음: {parent}→{child}")

    # ── 2. 정지 상태 참값 일치 ──────────────────────────────────────
    print("\n── 2. map→base_link 가 gz 참값과 일치하는가 ──")
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.1)
    T_tf = tf_to_matrix(n.lookup_at_gt("map", BASE))
    exy, ez, eyaw = pose_error(T_tf, n.gt)
    print(f"  참값  x={n.gt[0,3]:+.3f} y={n.gt[1,3]:+.3f} z={n.gt[2,3]:+.3f} "
          f"yaw={math.degrees(tfu.yaw_of(n.gt)):+.2f}°")
    print(f"  TF    x={T_tf[0,3]:+.3f} y={T_tf[1,3]:+.3f} z={T_tf[2,3]:+.3f} "
          f"yaw={math.degrees(tfu.yaw_of(T_tf)):+.2f}°")
    print(f"  오차  평면 {exy*1000:.1f} mm, z {ez*1000:+.1f} mm, yaw {eyaw:+.3f}°")
    if exy > 0.02 or abs(ez) > 0.02 or abs(eyaw) > 1.0:
        fails.append(f"정지 상태 참값 오차 과다 (평면 {exy*1000:.0f} mm, yaw {eyaw:.2f}°)")

    # ── 3. 주행 중 추종 ─────────────────────────────────────────────
    print("\n── 3. 주행 중에도 참값을 추종하는가 (휠 드리프트 흡수) ──")
    errs = []
    t_end = time.monotonic() + 8.0
    tw = Twist()
    tw.linear.x = 0.5
    tw.angular.z = 0.15          # 곡선 주행 — 스키드스티어 슬립을 유발한다
    while time.monotonic() < t_end:
        n.cmd.publish(tw)
        rclpy.spin_once(n, timeout_sec=0.05)
        try:
            T = tf_to_matrix(n.lookup_at_gt("map", BASE))
            if n.gt is not None:
                errs.append(pose_error(T, n.gt))
        except Exception:
            pass
    n.cmd.publish(Twist())
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.05)

    if len(errs) < 10:
        fails.append(f"주행 중 표본 부족 ({len(errs)})")
    else:
        a = np.array(errs)
        print(f"  표본 {len(errs)}개")
        print(f"  평면 오차  평균 {a[:,0].mean()*1000:.1f} mm, 최대 {a[:,0].max()*1000:.1f} mm")
        print(f"  yaw 오차   평균 {abs(a[:,2]).mean():.3f}°, 최대 {abs(a[:,2]).max():.3f}°")
        if a[:, 0].max() > 0.10:
            fails.append(f"주행 중 평면 오차 최대 {a[:,0].max()*1000:.0f} mm (한계 100 mm)")
        if abs(a[:, 2]).max() > 3.0:
            fails.append(f"주행 중 yaw 오차 최대 {abs(a[:,2]).max():.2f}° (한계 3°)")

    # ── 4. 오도메트리 드리프트가 실제로 쌓였는가 ────────────────────
    print("\n── 4. map→odom 이 휠 드리프트를 흡수하는가 ──")
    T_mo = tf_to_matrix(n.lookup("map", "odom"))
    dxy = math.hypot(T_mo[0, 3], T_mo[1, 3])
    print(f"  map→odom 이동량  평면 {dxy:.3f} m, yaw {math.degrees(tfu.yaw_of(T_mo)):+.2f}°")
    print("  ※ 0 이 아니어야 정상 — 휠 오도메트리가 참값에서 벌어진 만큼이 여기 담긴다")
    if dxy < 1e-6:
        fails.append("map→odom 이 정확히 0 — 참값이 반영되지 않았을 수 있다")

    # ── 5. 센서 외부파라미터 vs SDF ─────────────────────────────────
    print("\n── 5. 센서 외부파라미터가 SDF 와 일치하는가 ──")
    rel = base_relative(parse_link_poses(SDF))
    worst = 0.0
    for f in SENSOR_FRAMES:
        if f not in rel:
            continue
        try:
            T = tf_to_matrix(n.lookup("base_link", f))
        except Exception:
            fails.append(f"센서 TF 없음: {f}")
            continue
        d = float(np.linalg.norm(T[:3, 3] - rel[f][:3, 3]))
        worst = max(worst, d)
        print(f"  {f:<18} 차이 {d*1000:.3f} mm  {'✔' if d < 1e-4 else '✘'}")
        if d >= 1e-4:
            fails.append(f"{f} 외부파라미터가 SDF 와 {d*1000:.2f} mm 어긋남")

    n.destroy_node()
    rclpy.shutdown()

    print()
    if fails:
        print("✗ 실패 항목:")
        for f in fails:
            print(f"   · {f}")
        return 1
    print("✔ Stage-0 검증 통과 — TF 트리·참값 일치·주행 추종·외부파라미터 모두 정상")
    return 0


if __name__ == "__main__":
    sys.exit(main())
