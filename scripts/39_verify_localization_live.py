#!/usr/bin/env python3
"""사전 맵 로컬리제이션 — 시뮬레이터 실측 검증

    ros2 run orchard_sim map_localizer --ros-args -r __ns:=/scout01 -p bundle:=maps/orchard_v1 ...
    python3 scripts/39_verify_localization_live.py --secs 120 --out /tmp/loc_run.npz
    python3 scripts/39_verify_localization_live.py --robot scout02 ...

돌고 있는 map_localizer 의 추정 자세를 참값과 비교해 기록한다.
    추정 = TF map→<robot>/odom(map_localizer) ∘ <robot>/odom→<robot>/base_link
    참값 = /<robot>/gz_ground_truth (Gazebo 모델 포즈)

가설 검증의 핵심은 '오차가 커지는가(누적)' 이지 '오차가 있는가' 가 아니다.
그래서 시간에 따른 오차 궤적을 통째로 남긴다 — 나중에 차트로 본다.
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, "ros2_ws/src/orchard_sim")
from orchard_sim import transforms as tfu        # noqa: E402


class Recorder(Node):

    def __init__(self, robot="scout01"):
        super().__init__(f"loc_recorder_{robot}")
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        # gz 인스턴스 이름 = 참값 TFMessage 의 child_frame_id = TF 프레임 접두.
        # 셋이 같은 값이라는 것이 다중 로봇 이름 규약의 요지다.
        self.model = robot
        self.base_frame = f"{robot}/base_link"
        self.gt = None
        self.rows = []           # (t, ex, ey, eyaw, gx, gy, px, py)
        self.create_subscription(TFMessage, f"/{robot}/gz_ground_truth",
                                 self._on_gt, 20)
        self.create_timer(0.1, self._tick)   # 10 Hz 기록
        self.t0 = time.time()

    def _on_gt(self, msg: TFMessage):
        for t in msg.transforms:
            if t.child_frame_id != self.model:
                continue
            p, q = t.transform.translation, t.transform.rotation
            R = tfu.quat_to_matrix(q.x, q.y, q.z, q.w)
            self.gt = (p.x, p.y, math.atan2(R[1, 0], R[0, 0]))

    def _tick(self):
        if self.gt is None:
            return
        try:
            tr = self.buf.lookup_transform("map", self.base_frame, rclpy.time.Time())
        except Exception:
            return
        p, q = tr.transform.translation, tr.transform.rotation
        R = tfu.quat_to_matrix(q.x, q.y, q.z, q.w)
        est = (p.x, p.y, math.atan2(R[1, 0], R[0, 0]))
        gx, gy, gyaw = self.gt
        eyaw = (est[2] - gyaw + math.pi) % (2 * math.pi) - math.pi
        self.rows.append((time.time() - self.t0, est[0] - gx, est[1] - gy, eyaw,
                          gx, gy, est[0], est[1]))


ap = argparse.ArgumentParser()
ap.add_argument("--secs", type=float, default=120.0)
ap.add_argument("--out", default="/tmp/loc_run.npz")
ap.add_argument("--robot", default="scout01")
a = ap.parse_args()

rclpy.init()
node = Recorder(a.robot)
print(f"기록 시작 — {a.robot} · {a.secs:.0f}초")
end = time.time() + a.secs
try:
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
except KeyboardInterrupt:
    pass

d = np.asarray(node.rows, dtype=float)
node.destroy_node()
rclpy.shutdown()

if len(d) < 10:
    print(f"✗ 표본이 부족하다 ({len(d)}개) — 노드가 떠 있는지, "
          f"/{a.robot}/gz_ground_truth 가 나오는지 확인할 것")
    sys.exit(1)

np.savez_compressed(a.out, t=d[:, 0], ex=d[:, 1], ey=d[:, 2], eyaw=d[:, 3],
                    gx=d[:, 4], gy=d[:, 5], px=d[:, 6], py=d[:, 7])
err = np.hypot(d[:, 1], d[:, 2])
half = len(err) // 2
print(f"\n표본 {len(d)}개 · {d[-1,0]:.0f}초 · 이동 "
      f"{np.abs(np.diff(d[:,5])).sum():.1f} m")
print(f"위치 오차   RMS {np.sqrt((err**2).mean()):.3f} m · "
      f"평균 {err.mean():.3f} · 최대 {err.max():.3f}")
print(f"  횡(x)     RMS {np.sqrt((d[:,1]**2).mean()):.3f} m")
print(f"  종(y)     RMS {np.sqrt((d[:,2]**2).mean()):.3f} m")
print(f"방위 오차   RMS {np.degrees(np.sqrt((d[:,3]**2).mean())):.2f}°")
print(f"\n누적 여부 — 전반 RMS {np.sqrt((err[:half]**2).mean()):.3f} m → "
      f"후반 {np.sqrt((err[half:]**2).mean()):.3f} m")
print(f"기록 → {a.out}")
ok = np.sqrt((err ** 2).mean()) < 0.30
print(f"\n{'✔' if ok else '✗'} M3 기준 위치 오차 RMS < 0.30 m")
sys.exit(0 if ok else 1)
