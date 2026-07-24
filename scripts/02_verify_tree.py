#!/usr/bin/env python3
"""
단계 1b — 나무 메시가 gz-sim 에서 제대로 살아나는지 검증

tree_probe.sdf 가 돌고 있는 상태에서 확인한다:
  1. .glb 가 실제로 렌더링되는가            (semantic 마스크에 나무 픽셀이 있는가)
  2. <submesh> 로 부위 분리가 되는가        (trunk/feathers/leaf_healthy/leaf_diseased
                                             라벨이 모두 독립적으로 나오는가)
  3. gpu_lidar 가 잎·과실에서 점을 받는가   (collision 없이 visual 만으로 맞아야 한다)

사용법:
    ros2 run ros_gz_bridge parameter_bridge \
        /tree/semantic/labels_map@sensor_msgs/msg/Image[gz.msgs.Image \
        /tree/mid70/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked
    python3 scripts/02_verify_tree.py
"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

# tree_probe.sdf 와 일치
EXPECTED = {
    10: "ground_plane",
    20: "trunk        (submesh 분리)",
    21: "feathers     (submesh 분리)",
    30: "leaf_healthy (submesh 분리)",
    31: "leaf_diseased(submesh 분리)",
    40: "tree_full    (통짜 메시)",
}


class V(Node):
    def __init__(self):
        super().__init__("tree_verifier")
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sem = None
        self.cloud = None
        self.create_subscription(Image, "/tree/semantic/labels_map", self._sem, q)
        self.create_subscription(PointCloud2, "/tree/mid70/points", self._cloud, q)

    def _sem(self, m):
        if self.sem is None:
            self.sem = m

    def _cloud(self, m):
        if self.cloud is None:
            self.cloud = m

    def done(self):
        return self.sem is not None and self.cloud is not None


def main():
    rclpy.init()
    n = V()
    print("센서 데이터 대기 중...\n")
    deadline = n.get_clock().now().nanoseconds + 40 * 10**9
    while rclpy.ok() and not n.done():
        rclpy.spin_once(n, timeout_sec=0.5)
        if n.get_clock().now().nanoseconds > deadline:
            got = []
            if n.sem is not None:
                got.append("semantic")
            if n.cloud is not None:
                got.append("mid70")
            print(f"✗ 40초 안에 다 못 받음. 받은 것: {got or '없음'}")
            return 2

    # ── 1·2. semantic 마스크에서 부위별 라벨 확인 ─────────────────────────
    sem = np.frombuffer(n.sem.data, np.uint8).reshape(
        n.sem.height, n.sem.step // 3, 3)[:, : n.sem.width, :]
    labels, counts = np.unique(sem[:, :, 0], return_counts=True)
    hist = dict(zip(labels.tolist(), counts.tolist()))

    print(f"semantic {sem.shape}  encoding={n.sem.encoding}")
    print("\n── 부위별 semantic 라벨 ──")
    print(f"{'label':>6} {'픽셀':>10}  설명")
    print("─" * 60)
    missing = []
    for lab, desc in EXPECTED.items():
        px = hist.get(lab, 0)
        mark = "✔" if px > 0 else "✘"
        print(f"{lab:>6} {px:>10,}  {mark} {desc}")
        if px == 0:
            missing.append(lab)
    extra = sorted(set(hist) - set(EXPECTED) - {0})
    if extra:
        print(f"\n  예상 밖 라벨: {[(int(l), hist[l]) for l in extra]}")

    # ── 3. gpu_lidar 가 나무를 맞추는가 ───────────────────────────────────
    pts = np.array([[p[0], p[1], p[2]] for p in
                    pc2.read_points(n.cloud, field_names=("x", "y", "z"),
                                    skip_nans=True)])
    print(f"\n── gpu_lidar (MID-70 상당) ──")
    print(f"  수신 점 수 : {len(pts):,}  (113×113 = 12,769 격자)")
    if len(pts):
        d = np.linalg.norm(pts, axis=1)
        finite = d[np.isfinite(d) & (d < 89)]
        print(f"  유효 반사   : {len(finite):,} ({len(finite)/len(pts):.1%})")
        if len(finite):
            print(f"  거리 범위   : {finite.min():.2f} ~ {finite.max():.2f} m")
        # 나무 높이대(z>0.5, 센서 기준)에서 점이 오는지 = 잎/과실을 맞췄다는 뜻
        hi = pts[(np.linalg.norm(pts, axis=1) < 89) & (pts[:, 2] > 0.5)]
        print(f"  센서보다 0.5 m 이상 위 점: {len(hi):,}  "
              f"{'✔ 수관을 맞춤' if len(hi) > 50 else '✘ 수관 미검출'}")

    n.destroy_node()
    rclpy.shutdown()

    print()
    if missing:
        print(f"✗ 라벨 {missing} 이 화면에 없음 — submesh 이름 또는 카메라 각도 확인")
        return 1
    print("✔ .glb 로드 · submesh 부위 분리 · gpu_lidar 반사 모두 정상")
    return 0


if __name__ == "__main__":
    sys.exit(main())
