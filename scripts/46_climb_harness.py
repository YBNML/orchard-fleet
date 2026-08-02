#!/usr/bin/env python3
"""클라임 하네스 — 테라스 사이 동진 횡단의 정책별 통과율을 직접 잰다

    python3 scripts/46_climb_harness.py [--n 10] [--policies P0,P2,P3,P5]

임무 반복(25분/회)으로는 횡단 성공이 주사위라는 것밖에 못 배운다(실측:
런당 0~3회 널뛰기). 여기서는 로봇을 램프 앞에 놓고 같은 횡단을 정책만
바꿔 연속 시도한다 — 통과율이 곧 답이다.

정책 (전부 로봇 탑재 가능한 것만 — 참값은 채점에만 쓴다):
    P0  기본: 0.7 m/s 직진
    P1  저속: 0.3 m/s 직진
    P2  도움닫기: 2.5 m 뒤에서 0.9 m/s
    P3  재돌진: 0.7 진행, 6초마다 1.5 m 후진 후 0.9 재돌진 (4회)
    P5  톱질: 0.6 전진 + ±0.25 rad/s 좌우 톱질

주의: control_agent 를 먼저 내려야 한다 (유휴 0 명령이 cmd_vel 을 덮는다).
"""
from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

sys.path.insert(0, "sim") if False else None

import json
M = json.load(open("sim/models/orchard_terrain/heightmap_meta.json"))
H = np.load("sim/models/orchard_terrain/heightmap.npy")
N, HALF, E = H.shape[0], M["half"], M["size_x"]


def gz_at(x, y):
    fc = np.clip((x + HALF) / E * (N - 1), 0, N - 1)
    fr = np.clip((y + HALF) / E * (N - 1), 0, N - 1)
    c0, r0 = int(fc), int(fr)
    c1, r1 = min(c0 + 1, N - 1), min(r0 + 1, N - 1)
    dc, dr = fc - c0, fr - r0
    return float((H[r0, c0] * (1 - dc) + H[r0, c1] * dc) * (1 - dr)
                 + (H[r1, c0] * (1 - dc) + H[r1, c1] * dc) * dr)


class Harness(Node):
    def __init__(self):
        super().__init__("climb_harness")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.p = None
        self.create_subscription(TFMessage, "/gz_ground_truth", self._cb, 20)

    def _cb(self, m):
        t = m.transforms[0]
        tr, q = t.transform.translation, t.transform.rotation
        self.p = (tr.x, tr.y,
                  math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z)))

    def spin(self, sec):
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)

    def cmd(self, v, w=0.0):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.pub.publish(t)

    def teleport(self, x, y, yaw):
        z = gz_at(x, y) + 0.30
        req = (f'name: "scout_mini_mid70", position: {{x: {x}, y: {y}, z: {z:.2f}}}, '
               f'orientation: {{x: 0, y: 0, z: {math.sin(yaw/2):.6f}, '
               f'w: {math.cos(yaw/2):.6f}}}')
        subprocess.run(["gz", "service", "-s", "/world/orchard_10x41/set_pose",
                        "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                        "--timeout", "2000", "--req", req], capture_output=True)
        self.cmd(0.0)
        self.spin(3.0)


def drive(h: Harness, policy: str, goal_x: float, t_max=45.0):
    """정책대로 몰며 참값으로 채점. 반환 (통과, 소요초, 최대정체초)."""
    t0 = time.monotonic()
    last_x, last_adv_t = h.p[0], t0
    stall_max = 0.0
    charge_t0, backing_until, charges = t0, 0.0, 0
    while True:
        now = time.monotonic()
        el = now - t0
        if h.p[0] >= goal_x:
            h.cmd(0.0)
            return True, el, stall_max
        if el > t_max:
            h.cmd(0.0)
            return False, el, stall_max
        # 정체 추적 (채점용)
        if h.p[0] > last_x + 0.05:
            last_x, last_adv_t = h.p[0], now
        stall_max = max(stall_max, now - last_adv_t)

        if policy == "P0":
            h.cmd(0.7)
        elif policy == "P1":
            h.cmd(0.3)
        elif policy == "P2":
            h.cmd(0.9)
        elif policy == "P3":
            if now < backing_until:
                h.cmd(-0.4)
            elif now - charge_t0 > 6.0 and charges < 4:
                charges += 1
                backing_until = now + 3.5
                charge_t0 = now + 3.5
                h.cmd(-0.4)
            else:
                h.cmd(0.9)
        elif policy == "P5":
            h.cmd(0.6, 0.25 * (1 if int(el * 2) % 2 == 0 else -1))
        elif policy == "P7":            # 지속 조향 등판 — 임무형 wz 재현
            h.cmd(0.7, 0.20)
        elif policy == "P8":
            h.cmd(0.7, 0.35)
        elif policy == "P9":            # 임무형 폐루프: 목표점 조준 (참값 사용)
            gx2, gy2 = goal_x + 0.5, 34.0
            err = (math.atan2(gy2 - h.p[1], gx2 - h.p[0]) - h.p[2]
                   + math.pi) % (2 * math.pi) - math.pi
            h.cmd(0.7, max(-0.6, min(0.6, 1.4 * err)))
        h.spin(0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=10)
    ap.add_argument("--policies", default="P0,P1,P2,P3,P5")
    ap.add_argument("--from-x", type=float, default=-10.5,   # 통로 1 → 2
                    dest="fx")
    ap.add_argument("--y", type=float, default=34.0)
    a = ap.parse_args()

    rclpy.init()
    h = Harness()
    h.spin(1.0)
    goal_x = a.fx + 3.5 - 0.5           # 다음 통로 중심 못 미쳐 0.5 m
    print(f"클라임 하네스 — ({a.fx}, {a.y}) → x≥{goal_x:.1f} · 시도 {a.n}회/정책")
    print("=" * 72)
    results = {}
    for pol in a.policies.split(","):
        start_x = a.fx - (2.5 if pol == "P2" else 0.0)
        ok_n, times, stalls = 0, [], []
        for i in range(a.n):
            h.teleport(start_x, a.y, 0.0)
            ok, el, st = drive(h, pol, goal_x)
            ok_n += ok
            times.append(el)
            stalls.append(st)
            print(f"  {pol} #{i+1:2d}: {'통과' if ok else '실패'} "
                  f"{el:5.1f}초 · 최대정체 {st:4.1f}초")
        results[pol] = (ok_n, np.mean(times), np.max(stalls))
        print(f"  {pol} 합계: {ok_n}/{a.n} 통과 · 평균 {np.mean(times):.1f}초")
        print("-" * 72)
    print("\n정책     통과율   평균시간  최악정체")
    for pol, (k, tm, st) in results.items():
        print(f"  {pol}   {k:2d}/{a.n}   {tm:6.1f}초  {st:5.1f}초")


if __name__ == "__main__":
    main()
