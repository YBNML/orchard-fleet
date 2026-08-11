"""점군 → map 프레임 점 배열. ROS 를 모르는 기능에게 점군을 넘기는 다리.

지도 격자 기능(robomw.features.telemetry_map)은 예전에 자기가 PointCloud2 를
구독하고 TF 를 찾았다. 그 코드가 robomw 로 넘어가면 코어에 ROS 가 들어온다 —
격리 규약 위반이다(robomw/tests/test_no_ros_imports.py). 그래서 **ROS 에
닿는 부분만** 여기로 떼어냈다: 구독은 노드가 하고, 이 어댑터는 메시지를
받아 map 프레임 좌표로 풀어 sink 에게 넘긴다.

무엇을 지도에 올릴지(AGL 대역·격자 크기)는 여기서 정하지 않는다 — 그건
기능의 판단이다. 여기서 자르는 것은 **센서의 사정**뿐이다: 자기반사(30 cm
이내)와 신뢰 못 할 원거리(range_max).

sink 는 `bb.extra["cloud_sinks"]` 에 걸린 콜백들이다. 하나도 없으면 변환도
하지 않는다 — 기능을 빼면 비용도 함께 사라져야 한다.
"""
from __future__ import annotations

import numpy as np
import rclpy

from orchard_sim import transforms as tfu


def _read_xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)

    def f32(o):
        return raw[:, o:o + 4].copy().view(np.float32).ravel()
    return np.stack([f32(off["x"]), f32(off["y"]), f32(off["z"])], axis=1)


class RosCloudWorld:
    """PointCloud2 → (map 프레임 점 (N,3), 센서 z) 를 sink 들에게 넘긴다."""

    def __init__(self, node, bb, range_max=25.0):
        self._node = node
        self._bb = bb
        self.range_max = float(range_max)
        self._errs = 0

    def feed(self, msg):
        sinks = list(self._bb.extra.get("cloud_sinks") or ())
        if not sinks:
            return
        # 측위가 서기 전에는 map 프레임이 뜻이 없다 — 그때 쌓은 격자는
        # 엉뚱한 자리에 찍힌 채 지워지지 않는다 (누적 지도라서).
        if self._bb.pose is None:
            return
        try:
            tr = self._node._tf_buffer.lookup_transform(
                "map", msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return
        t, q = tr.transform.translation, tr.transform.rotation
        T = tfu.tf_from_pos_quat((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        pts = _read_xyz(msg)
        d = np.linalg.norm(pts, axis=1)
        pts = pts[(d > 0.3) & (d < self.range_max)]
        if pts.shape[0] == 0:
            return
        w = (T[:3, :3] @ pts.T).T + T[:3, 3]
        for sink in sinks:
            try:
                sink(w, float(t.z))
            except Exception as e:
                # 여기서 새면 구독 콜백이 통째로 죽어 노드가 내려간다. 조용히
                # 삼키지도 않는다 — 처음 세 번은 로그로 남긴다(초당 10 프레임이라
                # 계속 찍으면 그 자체가 장애다).
                self._errs += 1
                if self._errs <= 3:
                    self._node.get_logger().warn(
                        f"점군 sink 오류 ({self._errs}/3): {e}")
