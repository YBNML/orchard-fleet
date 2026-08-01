"""누적 장애물 격자 텔레메트리 (map 토픽).

관제 화면에는 '로봇을 막는 것'만 보이면 되므로 지면 기준 높이(AGL) 대역만
격자에 쌓는다. 점군을 그대로 보내지 않는 이유는 회선 예산이다 — 셀 인덱스만
보내면 프레임당 수 KB 로 끝난다.
"""
from __future__ import annotations

import numpy as np

from orchard_sim.control.base import Feature
from orchard_sim import transforms as tfu


def _read_xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)

    def f32(o):
        return raw[:, o:o + 4].copy().view(np.float32).ravel()
    return np.stack([f32(off["x"]), f32(off["y"]), f32(off["z"])], axis=1)


class TelemetryMap(Feature):
    name = "telemetry_map"
    version = "1.0"
    summary = "누적 장애물 격자"
    topics = ("map",)

    def setup(self, ctx):
        super().setup(ctx)
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2

        self.cell = float(ctx.param("map_cell", 0.25))
        self.max_points = int(ctx.param("map_max_points", 6000))
        self.period = float(ctx.param("map_period", 3.0))
        self.agl_lo = float(ctx.param("map_agl_min", 0.30))
        self.agl_hi = float(ctx.param("map_agl_max", 1.60))
        self.sensor_h = float(ctx.param("sensor_height", 0.645))
        self.rmax = float(ctx.param("map_range_max", 25.0))
        self.cells = {}
        self._next = 0.0
        self._dirty = True

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self._sub = ctx.node.create_subscription(
            PointCloud2, str(ctx.param("cloud_topic", "/livox/lidar")),
            self._on_cloud, qos)

    def _on_cloud(self, msg):
        import rclpy
        bb = self.ctx.bb
        if bb.pose is None:
            return
        try:
            tr = self.ctx.node._tf_buffer.lookup_transform(
                "map", msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return
        t, q = tr.transform.translation, tr.transform.rotation
        T = tfu.tf_from_pos_quat((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        pts = _read_xyz(msg)
        d = np.linalg.norm(pts, axis=1)
        pts = pts[(d > 0.3) & (d < self.rmax)]
        if pts.shape[0] == 0:
            return
        w = (T[:3, :3] @ pts.T).T + T[:3, 3]
        agl = w[:, 2] - t.z + self.sensor_h
        w = w[(agl > self.agl_lo) & (agl < self.agl_hi)]
        if w.shape[0] == 0:
            return
        # numpy 정수는 JSON 으로 못 나간다 — 여기서 파이썬 int 로 바꿔둔다
        ij = np.floor(w[:, :2] / self.cell).astype(np.int32).tolist()
        with self.ctx.bb.lock:
            for k in map(tuple, ij):
                self.cells[k] = self.cells.get(k, 0) + 1
            self.ctx.bb.extra["map_cells"] = len(self.cells)
        self._dirty = True

    def telemetry(self, now):
        if now < self._next or not self._dirty:
            return ()
        self._next = now + self.period
        with self.ctx.bb.lock:
            top = sorted(self.cells.items(), key=lambda kv: -kv[1])[:self.max_points]
        self._dirty = False
        if not top:
            return ()
        flat = []
        for (ix, iy), _n in top:
            flat.append(ix)
            flat.append(iy)
        return (("map", dict(cell=self.cell, n=len(top), ij=flat)),)

    def teardown(self):
        try:
            self.ctx.node.destroy_subscription(self._sub)
        except Exception:
            pass
