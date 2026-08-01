#!/usr/bin/env python3
"""실험 — 어느 높이 대역이 '열 위상'을 가장 잘 드러내는가

    python3 scripts/40_probe_agl_band.py --out /tmp/agl_band.npz

배경: 종방향(열을 따라가는) 위상이 참값 자세에서도 -0.309 m 어긋났다.
줄기 반경은 0.035 m 뿐이라 '앞면만 보인다'로는 설명이 안 된다. 남는 가설은
**높이 대역에 수관이 섞였다**는 것 — 수관은 열 방향으로 연속이라 1.5 m
주기가 흐려진다.

이 스크립트는 참값 자세로 점군을 펴서, 높이 대역을 바꿔가며
    횡 위상(3.5 m 주기) · 종 위상(1.5 m 주기) · 각 집중도
를 잰다. 위상이 0 에 가깝고 집중도가 높은 대역이 좋은 대역이다.
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

sys.path.insert(0, "ros2_ws/src/orchard_sim")
from orchard_sim import rowlocalize as rl          # noqa: E402
from orchard_sim import transforms as tfu          # noqa: E402
from orchard_sim.map_localizer import read_xyz     # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--scans", type=int, default=8)
ap.add_argument("--out", default="/tmp/agl_band.npz")
ap.add_argument("--model", default="scout_mini_mid70")
a = ap.parse_args()

BANDS = [(0.10, 0.35), (0.15, 0.45), (0.20, 0.55), (0.25, 0.65),
         (0.30, 0.75), (0.35, 0.90), (0.35, 1.30), (0.50, 1.60), (0.80, 2.00)]
S, T, X0 = 3.5, 1.5, -15.75

rclpy.init()
node = Node("agl_probe")
clouds, gt = [], {}
q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
               history=HistoryPolicy.KEEP_LAST, depth=2)
node.create_subscription(PointCloud2, "/livox/lidar",
                         lambda m: clouds.append(read_xyz(m)), q)


def on_gt(m):
    for t in m.transforms:
        if t.child_frame_id == a.model:
            p, qq = t.transform.translation, t.transform.rotation
            R = tfu.quat_to_matrix(qq.x, qq.y, qq.z, qq.w)
            gt["p"] = (p.x, p.y, math.atan2(R[1, 0], R[0, 0]))


node.create_subscription(TFMessage, "/gz_ground_truth", on_gt, 20)
t0 = time.time()
while time.time() - t0 < 25 and (len(clouds) < a.scans or "p" not in gt):
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()

if "p" not in gt or not clouds:
    print("수신 실패 — 시뮬레이터와 라이다가 도는지 확인할 것")
    sys.exit(1)

G = gt["p"]
print(f"참값 자세 ({G[0]:.3f}, {G[1]:.3f}, {math.degrees(G[2]):.2f}°) · 스캔 {len(clouds)}장")
print()
hdr = f"{'AGL 대역':<14}{'구조점':>7}{'횡 위상':>10}{'횡 집중':>9}{'종 위상':>10}{'종 집중':>9}"
print(hdr)
print("─" * len(hdr))

rows = []
c, s = math.cos(G[2]), math.sin(G[2])
for lo, hi in BANDS:
    ox, cx, oy, cy, ns = [], [], [], [], []
    for P in clouds:
        sp = rl.structure_points(P, agl=(lo, hi))
        if len(sp) < 60:
            continue
        wx = G[0] + c * sp[:, 0] - s * sp[:, 1]
        wy = G[1] + s * sp[:, 0] + c * sp[:, 1]
        o1, q1 = rl._phase(wx - X0, S)
        o2, q2 = rl._phase(wy, T)
        ox.append(o1); cx.append(q1); oy.append(o2); cy.append(q2); ns.append(len(sp))
    if not ns:
        print(f"{lo:.2f}~{hi:.2f} m{'점 부족':>28}")
        continue
    r = (lo, hi, np.mean(ns), np.mean(ox), np.mean(cx), np.mean(oy), np.mean(cy))
    rows.append(r)
    print(f"{lo:.2f}~{hi:.2f} m{int(r[2]):>7}{r[3]:>+10.3f}{r[4]:>9.2f}"
          f"{r[5]:>+10.3f}{r[6]:>9.2f}")

d = np.asarray(rows)
np.savez_compressed(a.out, lo=d[:, 0], hi=d[:, 1], n=d[:, 2],
                    ox=d[:, 3], cx=d[:, 4], oy=d[:, 5], cy=d[:, 6],
                    truth=np.asarray(G))
print(f"\n기록 → {a.out}")
best = min(rows, key=lambda r: abs(r[5]) + 0.3 * (1 - r[6]))
print(f"종방향이 가장 정직한 대역: {best[0]:.2f}~{best[1]:.2f} m "
      f"(위상 {best[5]:+.3f} m · 집중도 {best[6]:.2f})")
