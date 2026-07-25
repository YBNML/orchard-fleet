#!/usr/bin/env python3
"""
로봇 모델 검증 (설계서 §5.3 게이트) — 실제 Scout Mini 메시 버전

    # robot_test.sdf + /cmd_vel·/odom 브리지가 떠 있는 상태에서
    python3 scripts/03_verify_robot.py

셸 백그라운드 잡으로 cmd_vel 을 쏘던 이전 방식은 잡 번호가 어긋나면 명령이
계속 나가 결과를 오염시킨다(2026-07-25에 실제로 겪음). 여기서는 한 노드가
명령·계측·정지를 모두 소유해 그런 오염이 원천적으로 생기지 않는다.

확인:
  1. 센서 토픽이 전부 나오는가
  2. 직진 — odom x 증가, y 드리프트 ~0  (윤거·휠반경 정확)
  3. 제자리회전 — x·y 불변, yaw 변화  (스키드스티어)
  4. 왕복 복귀 — 전진 후 후진하면 출발점으로 돌아오는가 (부호·스케일 대칭)
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

EXPECT_TOPICS = ["/odom", "/livox/points_raw/points", "/imu",
                 "/cam/left/image", "/cam/forward/image"]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class Driver(Node):
    def __init__(self):
        super().__init__("robot_verifier")
        self.odom = None
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Odometry, "/odom",
                                 lambda m: setattr(self, "odom", m), q)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)

    def pose(self):
        p = self.odom.pose.pose
        return np.array([p.position.x, p.position.y]), yaw_of(p.orientation)

    def drive(self, vx, wz, seconds):
        """명령을 유지하며 주행하고, 끝나면 반드시 정지시킨다."""
        tw = Twist()
        tw.linear.x = float(vx)
        tw.angular.z = float(wz)
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            self.cmd.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.stop()

    def stop(self, settle=1.2):
        z = Twist()
        t_end = time.monotonic() + settle
        while time.monotonic() < t_end:
            self.cmd.publish(z)
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_odom(self, timeout=30):
        t_end = time.monotonic() + timeout
        while rclpy.ok() and self.odom is None:
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.monotonic() > t_end:
                return False
        return True


def main():
    rclpy.init()
    n = Driver()
    fails = []

    print("odom 대기...")
    if not n.wait_odom():
        print("✗ /odom 미수신 — 월드와 브리지가 떠 있는지 확인하세요")
        return 2

    # ── 1. 센서 토픽 ────────────────────────────────────────────────
    print("\n── 1. 센서 토픽 ──")
    have = dict(n.get_topic_names_and_types())
    for t in EXPECT_TOPICS:
        ok = t in have
        print(f"  {'✔' if ok else '✘'} {t}")
        if not ok:
            fails.append(f"토픽 없음: {t}")

    n.stop(0.5)

    # ── 2. 직진 ─────────────────────────────────────────────────────
    print("\n── 2. 직진 1.0 m/s × 3 s ──")
    p0, y0 = n.pose()
    n.drive(1.0, 0.0, 3.0)
    p1, y1 = n.pose()
    d = p1 - p0
    fwd = d[0] * math.cos(y0) + d[1] * math.sin(y0)
    lat = -d[0] * math.sin(y0) + d[1] * math.cos(y0)
    dyaw = math.degrees((y1 - y0 + math.pi) % (2 * math.pi) - math.pi)
    print(f"  전진 {fwd:+.3f} m,  횡방향 드리프트 {lat * 1000:+.3f} mm,  yaw 변화 {dyaw:+.3f}°")
    if fwd < 2.0:
        fails.append(f"전진량 {fwd:.2f} m 가 너무 적다 (3초 × 1 m/s 기대)")
    if abs(lat) > 0.05:
        fails.append(f"직진 중 횡드리프트 {lat*1000:.0f} mm — 윤거·마찰 확인")
    if abs(dyaw) > 2.0:
        fails.append(f"직진 중 yaw 변화 {dyaw:.2f}° — 좌우 휠 불균형")

    # ── 3. 제자리회전 ───────────────────────────────────────────────
    print("\n── 3. 제자리회전 0.5 rad/s × 3 s ──")
    p2, y2 = n.pose()
    n.drive(0.0, 0.5, 3.0)
    p3, y3 = n.pose()
    move = float(np.linalg.norm(p3 - p2))
    dyaw2 = math.degrees((y3 - y2 + math.pi) % (2 * math.pi) - math.pi)
    print(f"  위치 이동 {move * 1000:.1f} mm,  yaw 변화 {dyaw2:+.2f}° (기대 ≈ +86°)")
    if move > 0.20:
        fails.append(f"제자리회전인데 {move*1000:.0f} mm 이동 — 스키드스티어 이상")
    if abs(dyaw2) < 40:
        fails.append(f"회전량 {dyaw2:.1f}° 가 너무 적다 (3초 × 0.5 rad/s ≈ 86°)")

    # ── 4. 왕복 복귀 ────────────────────────────────────────────────
    print("\n── 4. 전진 2 s → 후진 2 s (출발점 복귀) ──")
    p4, y4 = n.pose()
    n.drive(0.6, 0.0, 2.0)
    n.drive(-0.6, 0.0, 2.0)
    p5, y5 = n.pose()
    back = float(np.linalg.norm(p5 - p4))
    print(f"  출발점과의 거리 {back * 1000:.0f} mm")
    if back > 0.30:
        fails.append(f"왕복 후 {back*1000:.0f} mm 벗어남 — 전후진 스케일 비대칭")

    n.stop()
    n.destroy_node()
    rclpy.shutdown()

    print()
    if fails:
        print("✗ 실패 항목:")
        for f in fails:
            print(f"   · {f}")
        return 1
    print("✔ 로봇 모델 검증 통과 — 센서·직진·제자리회전·왕복 모두 정상")
    return 0


if __name__ == "__main__":
    sys.exit(main())
