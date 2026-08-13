#!/usr/bin/env python3
"""
mapping_run — 과수원을 보스트로피돈(왕복) 경로로 훑으며 점군을 누적한다.

    ros2 run orchard_sim mapping_run --ros-args -p out:=/tmp/orchard_map.npz

FAST-LIO2 로 전체 맵을 얻는 것 자체는 어렵지 않다. 어려운 것은 그 맵에서
**주행 가능한 통로를 찾아내는 것**이다(사용자의 실제 과수원 경험). 이 노드는 그 문제를
다루기 위한 입력, 즉 맵을 만든다.

Stage-0 참값 포즈를 쓰므로 여기서 나오는 맵은 **정합 오차가 0 인 이상적인 맵**이다.
그래서 뒤이은 OGM·중심선 추출이 실패하면 그것은 정합 문제가 아니라 순수하게
"맵에서 통로를 찾는 문제"가 어렵다는 뜻이 된다 — 문제를 분리해서 본다.
단계 7 에서 FAST-LIO2 로 갈아끼우면 정합 오차가 더해진 조건에서 다시 볼 수 있다.

출력 (npz):
    points   (N,3) float32   map 프레임 누적 점군
    origins  (N,3) float32   각 점을 관측한 센서 위치 (지면 분할·가시성 추론에 쓴다)
    path     (M,3) float32   주행 궤적
"""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

from orchard_sim import gz_topics as gzt
from orchard_sim import transforms as tfu


def read_xyz(msg: PointCloud2):
    off = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)

    def f32(o):
        return raw[:, o:o + 4].copy().view(np.float32).ravel()

    return np.stack([f32(off["x"]), f32(off["y"]), f32(off["z"])], axis=1)


class MappingRun(Node):

    def __init__(self):
        super().__init__("mapping_run")

        # 과수원 격자 — gen_world 와 같은 값이어야 한다
        self.declare_parameter("rows", 10)
        self.declare_parameter("trees_per_row", 41)
        self.declare_parameter("row_spacing", 3.5)
        self.declare_parameter("tree_spacing", 1.5)
        self.declare_parameter("headland", 6.0)
        # 주행
        self.declare_parameter("speed", 0.9)            # 매핑 속도 (촬영 0.6 보다 빠르게)
        self.declare_parameter("turn_speed", 0.6)
        # 통로 끝·선회 구간 감속 (단계 7 실측 대응 — speed_limit 주석 참조)
        self.declare_parameter("y_slow_in", 25.0)      # |y| 이 값 이상이면 감속
        self.declare_parameter("slow_factor", 0.40)    # 감속 배율
        self.declare_parameter("decel_dist", 3.0)      # 목표 접근 감속 거리 [m]
        self.declare_parameter("alleys", 0)             # 0 = 전부
        self.declare_parameter("voxel", 0.05)           # 누적 시 다운샘플 [m]
        self.declare_parameter("max_range", 30.0)       # 먼 점은 밀도가 낮아 잡음이 된다
        self.declare_parameter("out", "/tmp/orchard_map.npz")
        # 토픽·프레임 기본값은 robot_id 파생이다 (다중 로봇).
        self.declare_parameter("robot_id", "scout01")
        _rid = str(self.get_parameter("robot_id").value)
        self.declare_parameter("cloud_topic", gzt.ns_topic(_rid, "livox/lidar"))
        self.declare_parameter("cmd_vel_topic", gzt.ns_topic(_rid, "cmd_vel"))
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", gzt.frame(_rid, "base_link"))

        g = lambda k: self.get_parameter(k).value
        self.R = int(g("rows")); self.T = int(g("trees_per_row"))
        self.S = float(g("row_spacing")); self.ts = float(g("tree_spacing"))
        self.HL = float(g("headland"))
        self.speed = float(g("speed")); self.turn_speed = float(g("turn_speed"))
        self.y_slow_in = float(g("y_slow_in")); self.slow_factor = float(g("slow_factor"))
        self.decel_dist = float(g("decel_dist"))
        self.voxel = float(g("voxel")); self.max_range = float(g("max_range"))
        self.out = g("out")
        self.map_frame = str(g("map_frame")); self.base_frame = str(g("base_frame"))

        self.x0 = -((self.R - 1) * self.S) / 2.0
        self.col_l = (self.T - 1) * self.ts
        self.y0 = -self.col_l / 2.0

        n_alleys = int(g("alleys")) or (self.R - 1)
        self.alleys = list(range(min(n_alleys, self.R - 1)))

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.cmd = self.create_publisher(Twist, str(g("cmd_vel_topic")), 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(PointCloud2, g("cloud_topic"), self.on_cloud, qos)

        # 복셀 해시로 누적 — 전체를 메모리에 들고 dedup 하지 않는다
        self._vox = {}
        self.path = []
        self.frames = 0
        self.collecting = False

        self.get_logger().info(
            f"매핑 주행 — 통로 {len(self.alleys)}개, 속도 {self.speed} m/s, "
            f"복셀 {self.voxel} m → {self.out}")

    # ── 점군 누적 ───────────────────────────────────────────────────────
    def on_cloud(self, msg: PointCloud2):
        if not self.collecting:
            return
        try:
            tr = self.buf.lookup_transform(self.map_frame, msg.header.frame_id,
                                           rclpy.time.Time())
        except Exception:
            return
        p, q = tr.transform.translation, tr.transform.rotation
        T = tfu.tf_from_pos_quat((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

        pts = read_xyz(msg)
        d = np.linalg.norm(pts, axis=1)
        pts = pts[(d > 0.2) & (d < self.max_range)]
        if pts.shape[0] == 0:
            return

        w = (T[:3, :3] @ pts.T).T + T[:3, 3]
        org = T[:3, 3].astype(np.float32)

        keys = np.floor(w / self.voxel).astype(np.int64)
        for k, pt in zip(map(tuple, keys), w):
            if k not in self._vox:
                self._vox[k] = (pt.astype(np.float32), org)
        self.frames += 1

    def robot_xy(self):
        """map 프레임에서의 (x, y) 와 yaw. TF 가 아직 없으면 (None, None)."""
        try:
            tr = self.buf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return None, None
        t, q = tr.transform.translation, tr.transform.rotation
        R = tfu.quat_to_matrix(q.x, q.y, q.z, q.w)
        return np.array([t.x, t.y]), math.atan2(R[1, 0], R[0, 0])

    def check_upright(self, max_tilt_deg=35.0):
        """전복 감지. 뒤집힌 채로 계속 돌면 쓰레기 데이터만 쌓인다 —
        2026-07-25 에 실제로 통로 8개 분량을 그렇게 날렸다."""
        try:
            tr = self.buf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return True
        q = tr.transform.rotation
        R = tfu.quat_to_matrix(q.x, q.y, q.z, q.w)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))
        if tilt > max_tilt_deg:
            self.get_logger().error(f"기울기 {tilt:.1f}° — 전복/전도 상태")
            return False
        return True

    # ── 주행 원시동작 ───────────────────────────────────────────────────
    def spin_once(self, dt=0.02):
        rclpy.spin_once(self, timeout_sec=dt)

    def publish(self, vx, wz):
        tw = Twist(); tw.linear.x = float(vx); tw.angular.z = float(wz)
        self.cmd.publish(tw)

    def stop(self, secs=0.8):
        t_end = time.monotonic() + secs
        while time.monotonic() < t_end:
            self.publish(0.0, 0.0); self.spin_once()

    def speed_limit(self, xy, dist):
        """구간별 속도 상한.

        단계 7 실측(2026-07-26): FAST-LIO2 의 10 m 구간 거리오차가 통로 안에서는
        0.61% 인데 **통로 끝(|y| 25~30 m)에서 38.3%** 로 튄다. 통로를 빠져나가는
        순간 양옆 나무벽이 사라지고 70.4° 전방 FOV 에 빈 헤드랜드만 남아 기하가
        퇴화하기 때문이다.

        기하가 퇴화하는 구간에서는 **거리당 스캔 수를 늘리는 것** 말고 할 수 있는
        게 없다. 그래서 통로 끝과 선회 구간에서 감속한다. 시간은 더 걸리지만
        정합이 무너지면 맵 전체를 버려야 한다.
        """
        v = self.speed
        ay = abs(float(xy[1]))
        if ay >= self.y_slow_in:                  # 통로 끝 ~ 선회 구간
            v = min(v, self.speed * self.slow_factor)
        if dist < self.decel_dist:                # 목표 접근 감속
            v = min(v, max(self.speed * 0.25, self.speed * dist / self.decel_dist))
        return max(v, self.speed * 0.20)

    def goto(self, tx, ty, tol=0.35, timeout=180.0):
        """단순 비례 추종 — Nav2 이전 단계이므로 최소한의 항법만 쓴다."""
        t_end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < t_end:
            xy, yaw = self.robot_xy()
            if xy is None:
                self.spin_once(); continue
            d = np.array([tx, ty]) - xy
            dist = float(np.linalg.norm(d))
            if dist < tol:
                return True
            hdg = math.atan2(d[1], d[0])
            err = (hdg - yaw + math.pi) % (2 * math.pi) - math.pi
            if abs(err) > 0.6:                       # 크게 틀어지면 제자리에서 정렬
                self.publish(0.0, self.turn_speed * (1 if err > 0 else -1))
            else:
                v = self.speed_limit(xy, dist)
                self.publish(v * max(0.25, 1.0 - abs(err)), 1.4 * err)
            self.path.append(np.array([xy[0], xy[1], 0.0], np.float32))
            self.spin_once()
        return False

    # ── 보스트로피돈 경로 ───────────────────────────────────────────────
    #
    # 계단식 과수원에서는 통로 사이를 **직선으로 가로지를 수 없다**.
    # 통로 경계에 26~50 cm 단차 둑이 있고 경사가 최대 60% 라, 목표를 향해
    # 곧장 몰면 둑을 타넘으려다 전복한다 (2026-07-25 실측: roll -175°).
    # 통로 간 이동은 반드시 선회 구역의 램프 대역(|y| >= cross_y)에서만 한다.
    #
    #   통로 k 끝 ──(같은 x 로 램프 대역까지 전진)── 램프에서 횡이동 ──
    #     ── 통로 k+1 의 x 로 ──(통로 안으로 진입)──
    def cross_y(self, sign):
        """통로 간 횡이동을 허용하는 y 좌표. 램프 대역 한가운데를 쓴다."""
        # 램프는 |y| >= L/2 + ramp_len 부터 순수해진다. 여유를 두고 선회 구역 중앙.
        return sign * (self.col_l / 2.0 + self.HL * 0.72)

    def run(self):
        # TF 준비 대기
        t_end = time.monotonic() + 60
        while rclpy.ok():
            xy, _ = self.robot_xy()
            if xy is not None:
                break
            self.spin_once(0.2)
            if time.monotonic() > t_end:
                self.get_logger().error("map→base_link TF 미수신 — Stage-0 스택 확인")
                return False

        # 통로 진입/이탈 지점 — 나무 구역 바로 바깥
        y_enter_lo = self.y0 - self.HL * 0.25
        y_enter_hi = self.y0 + self.col_l + self.HL * 0.25
        self.collecting = True
        t0 = time.monotonic()

        for i, k in enumerate(self.alleys):
            cx = self.x0 + (k + 0.5) * self.S
            up = (i % 2 == 0)
            y_start, y_end = (y_enter_lo, y_enter_hi) if up else (y_enter_hi, y_enter_lo)
            sign_start = -1.0 if up else 1.0

            self.get_logger().info(
                f"통로 {k} (x={cx:+.2f}) {'상행' if up else '하행'} — "
                f"누적 {len(self._vox):,} 복셀")

            # 1) 램프 대역으로 나가서 2) 옆 통로 x 로 횡이동 3) 통로 입구로 복귀
            #    (첫 통로는 이미 그 자리에 있으므로 건너뛴다)
            if i > 0:
                yc = self.cross_y(sign_start)
                xy, _ = self.robot_xy()
                if xy is not None:
                    if not self.goto(float(xy[0]), yc, tol=0.4, timeout=90):
                        self.get_logger().warn("램프 대역 진입 실패")
                if not self.goto(cx, yc, tol=0.4, timeout=90):
                    self.get_logger().warn(f"통로 {k} 로의 횡이동 실패")
                if not self.goto(cx, y_start, tol=0.4, timeout=90):
                    self.get_logger().warn(f"통로 {k} 입구 진입 실패")

            # 4) 통로 통과 — 여기서만 점군이 크게 쌓인다
            if not self.goto(cx, y_end, tol=0.5, timeout=240):
                self.get_logger().warn(f"통로 {k} 통과 실패")

            if not self.check_upright():
                self.get_logger().error("로봇이 기울었다 — 매핑 중단")
                break

        self.stop()
        self.collecting = False

        pts = np.array([v[0] for v in self._vox.values()], np.float32)
        orgs = np.array([v[1] for v in self._vox.values()], np.float32)
        path = np.array(self.path, np.float32) if self.path else np.zeros((0, 3), np.float32)
        np.savez_compressed(self.out, points=pts, origins=orgs, path=path)

        el = time.monotonic() - t0
        self.get_logger().info(
            f"매핑 완료 — {el/60:.1f}분, 프레임 {self.frames:,}, "
            f"점 {pts.shape[0]:,} (복셀 {self.voxel} m)")
        self.get_logger().info(
            f"  x {pts[:,0].min():.1f}~{pts[:,0].max():.1f}  "
            f"y {pts[:,1].min():.1f}~{pts[:,1].max():.1f}  "
            f"z {pts[:,2].min():.2f}~{pts[:,2].max():.2f} m")
        self.get_logger().info(f"  → {self.out}")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = MappingRun()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    main()
