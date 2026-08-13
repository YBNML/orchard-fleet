#!/usr/bin/env python3
"""
lio_recorder — FAST-LIO2 의 맵과 궤적을, 참값 궤적과 함께 기록한다

    ros2 run orchard_sim lio_recorder --ros-args -p out:=/tmp/orchard_lio.npz

단계 0 은 참값 포즈를 써서 정합 오차가 0 인 맵을 만들었다. 여기서는 FAST-LIO2 가
스스로 추정한 포즈로 만든 맵을 받는다. 두 맵에 같은 통로 추출 코드를 돌리면
**정합 오차가 통로 추출에 얼마나 영향을 주는지** 만 분리해서 볼 수 있다.

FAST-LIO2 의 camera_init 프레임은 '시작 시점의 IMU 자세'라서 map 과 다르다.
여기서는 변환하지 않고 두 궤적을 그대로 저장한다 — 사후에 Umeyama 정합으로
맞추면 정렬 오차와 표류(drift)를 분리해서 볼 수 있기 때문이다.

출력 (npz):
    points      (N,3)  FAST-LIO2 누적 맵 (camera_init 프레임)
    lio_t/lio_p (M,)/(M,3)  LIO 추정 궤적
    gt_t/gt_p   (M,)/(M,3)  같은 시각의 참값 궤적 (map→imu_link)
"""
from __future__ import annotations

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

from orchard_sim import gz_topics as gzt


def read_xyz(msg: PointCloud2) -> np.ndarray:
    off = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)

    def col(name):
        o = off[name]
        return raw[:, o:o + 4].copy().view(np.float32).ravel()
    return np.stack([col("x"), col("y"), col("z")], axis=1)


class LioRecorder(Node):
    def __init__(self):
        super().__init__("lio_recorder")
        self.declare_parameter("out", "/tmp/orchard_lio.npz")
        self.declare_parameter("cloud_topic", "/cloud_registered")
        self.declare_parameter("odom_topic", "/Odometry")
        # gt_frame 은 참값 TF 조회용 — 다중 로봇에서는 로봇 접두가 붙는다.
        self.declare_parameter("robot_id", "scout01")
        self.declare_parameter(
            "gt_frame",
            gzt.frame(str(self.get_parameter("robot_id").value), "imu_link"))
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("voxel", 0.05)
        self.declare_parameter("compact_every", 40)
        self.declare_parameter("log_every", 100)
        g = lambda k: self.get_parameter(k).value  # noqa: E731

        self.out = g("out")
        self.voxel = float(g("voxel"))
        self.gt_frame = g("gt_frame")
        self.map_frame = str(g("map_frame"))
        self.compact_every = int(g("compact_every"))
        self.log_every = int(g("log_every"))

        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, g("cloud_topic"), self.on_cloud, qos)
        self.create_subscription(Odometry, g("odom_topic"), self.on_odom, 20)

        self.chunks, self.n_cloud, self.n_pts = [], 0, 0
        self.lio_t, self.lio_p, self.gt_t, self.gt_p = [], [], [], []
        # 자세도 남긴다. 위치만으로는 융합을 못 한다 — 두 소스의 증분을 각자의
        # 자세로 body 프레임에 옮겨야 방향이 어긋나지 않기 때문이다
        # (2026-07-30: 전역 프레임에서 상수 회전으로 섞었다가 LIO 의 yaw 표류
        #  때문에 융합이 두 소스 각각보다 나빠졌다).
        self.lio_q, self.gt_q = [], []
        self.n_gt_miss = 0
        self.get_logger().info(
            f"lio_recorder 시작 — {g('cloud_topic')} / {g('odom_topic')} → {self.out}")

    # ── 점군 ────────────────────────────────────────────────────────────────
    def on_cloud(self, msg):
        P = read_xyz(msg)
        fin = np.isfinite(P).all(axis=1)
        P = P[fin]
        if P.size == 0:
            return
        self.chunks.append(P.astype(np.float32))
        self.n_cloud += 1
        self.n_pts += P.shape[0]
        if self.n_cloud % self.compact_every == 0:
            self.compact()
        if self.n_cloud % self.log_every == 0:
            self.get_logger().info(
                f"{self.n_cloud} 스캔 · 누적 {self.n_pts/1e6:.2f}M 점 · "
                f"궤적 {len(self.lio_p)} (참값 미스 {self.n_gt_miss})")

    def compact(self):
        if not self.chunks:
            return
        P = np.concatenate(self.chunks)
        key = np.floor(P / self.voxel).astype(np.int64)
        _, idx = np.unique(key, axis=0, return_index=True)
        P = P[idx]
        self.chunks = [P]
        self.n_pts = P.shape[0]

    # ── 궤적 ────────────────────────────────────────────────────────────────
    def on_odom(self, msg):
        """LIO 궤적과 참값 궤적을 **각자의 시각으로** 따로 쌓는다.

        예전에는 오도메트리 스탬프로 TF 를 조회해 둘을 한 번에 기록했는데,
        FAST-LIO2 의 스탬프가 TF 버퍼 구간과 어긋나 조회가 100% 실패했고
        그 바람에 LIO 궤적까지 통째로 버려졌다 (2026-07-26 실측).
        시각 정합은 사후 보간으로 해도 충분하다 — 둘 다 10 Hz 이상이다.
        """
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        self.lio_t.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self.lio_p.append([p.x, p.y, p.z])
        self.lio_q.append([o.x, o.y, o.z, o.w])
        try:
            tr = self.buf.lookup_transform(self.map_frame, self.gt_frame,
                                           rclpy.time.Time())
        except Exception:
            self.n_gt_miss += 1
            return
        t2, q2 = tr.transform.translation, tr.transform.rotation
        st = tr.header.stamp
        self.gt_t.append(st.sec + st.nanosec * 1e-9)
        self.gt_p.append([t2.x, t2.y, t2.z])
        self.gt_q.append([q2.x, q2.y, q2.z, q2.w])

    # ── 저장 ────────────────────────────────────────────────────────────────
    def save(self):
        self.compact()
        P = self.chunks[0] if self.chunks else np.zeros((0, 3), np.float32)
        np.savez_compressed(
            self.out, points=P,
            lio_t=np.array(self.lio_t),
            lio_p=np.array(self.lio_p, np.float64).reshape(-1, 3),
            lio_q=np.array(self.lio_q, np.float64).reshape(-1, 4),
            gt_t=np.array(self.gt_t),
            gt_p=np.array(self.gt_p, np.float64).reshape(-1, 3),
            gt_q=np.array(self.gt_q, np.float64).reshape(-1, 4),
            n_scans=self.n_cloud, n_gt_miss=self.n_gt_miss)
        self.get_logger().info(
            f"저장: {self.out} — {P.shape[0]:,} 점 · {self.n_cloud} 스캔 · "
            f"궤적 {len(self.lio_p)}")


def main():
    rclpy.init()
    node = LioRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
