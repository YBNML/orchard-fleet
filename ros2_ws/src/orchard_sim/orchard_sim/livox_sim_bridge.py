#!/usr/bin/env python3
"""
livox_sim_bridge — gz gpu_lidar 점군을 Livox MID-70 계약으로 변환 (설계서 §6)

    ros2 run orchard_sim livox_sim_bridge

입력  /livox/points_raw/points   sensor_msgs/PointCloud2
        gz gpu_lidar → ros_gz_bridge. 113×113 정사각 격자, x/y/z/intensity + ring,
        point_step 32, is_dense=False (무반사는 inf/NaN)
출력  /livox/lidar               sensor_msgs/PointCloud2  (PointXYZRTLT)
      /livox/lidar_custom        livox_ros_driver2/CustomMsg  (FAST-LIO2 용, 선택)

이 노드가 존재하는 이유: 하류 소비자(SLAM·인지·데이터셋)를 시뮬레이터 구현과
분리하기 위해서다. 나중에 커스텀 비반복 스캔 플러그인으로 바꾸든 실장비로 바꾸든
이 계약만 지키면 하류는 손댈 필요가 없다.
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from orchard_sim import livox_contract as lc

try:
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
    HAVE_CUSTOM = True
except ImportError:          # 메시지 패키지 미빌드 — PointCloud2 경로만 동작
    HAVE_CUSTOM = False


def _read_xyzi(msg: PointCloud2):
    """PointCloud2 에서 x/y/z/intensity 를 offset 기반으로 뽑는다.

    필드 순서·offset 은 발행자에 따라 다르므로 이름으로 찾는다.
    (gz 는 x0 y4 z8 intensity16 ring24, point_step 32)
    """
    off = {f.name: f.offset for f in msg.fields}
    for need in ("x", "y", "z"):
        if need not in off:
            raise KeyError(f"입력 점군에 '{need}' 필드가 없습니다: {list(off)}")
    n = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)

    def f32(o):
        return raw[:, o:o + 4].copy().view(np.float32).ravel()

    x, y, z = f32(off["x"]), f32(off["y"]), f32(off["z"])
    inten = f32(off["intensity"]) if "intensity" in off else np.zeros(n, np.float32)
    return x, y, z, inten


class LivoxSimBridge(Node):

    def __init__(self):
        super().__init__("livox_sim_bridge")

        self.declare_parameter("input_topic", "/livox/points_raw/points")
        self.declare_parameter("output_topic", "/livox/lidar")
        self.declare_parameter("custom_topic", "/livox/lidar_custom")
        self.declare_parameter("frame_id", "livox_frame")
        self.declare_parameter("publish_custom_msg", True)
        self.declare_parameter("apply_fov_mask", True)
        self.declare_parameter("publish_freq_hz", 10.0)
        # gz intensity 스케일. 0 이면 프레임별 자동 정규화
        self.declare_parameter("intensity_in_max", 0.0)
        self.declare_parameter("log_every_n", 100)
        # gz gpu_lidar 는 intensity 를 모델링하지 않아 항상 0 을 준다 (2026-07-25 실측).
        #   passthrough : 그대로 0 — 기본값. 없는 정보를 지어내지 않는다
        #   range       : 거리 기반 합성. 시각화·지면분할 휴리스틱용이며 **반사율이 아니다**.
        #                 재질 단서로 쓰는 알고리즘을 오도할 수 있으니 의도적으로 켤 때만 쓴다
        self.declare_parameter("intensity_mode", "passthrough")

        g = lambda k: self.get_parameter(k).value
        self.frame_id = g("frame_id")
        self.apply_mask = bool(g("apply_fov_mask"))
        self.period = 1.0 / max(float(g("publish_freq_hz")), 1e-3)
        imax = float(g("intensity_in_max"))
        self.intensity_in_max = imax if imax > 0 else None
        self.want_custom = bool(g("publish_custom_msg"))
        self.log_every_n = int(g("log_every_n"))
        self.intensity_mode = str(g("intensity_mode"))

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=5)

        self.pub = self.create_publisher(PointCloud2, g("output_topic"), sensor_qos)
        self.pub_custom = None
        if self.want_custom:
            if HAVE_CUSTOM:
                self.pub_custom = self.create_publisher(CustomMsg, g("custom_topic"), sensor_qos)
            else:
                self.get_logger().warn(
                    "livox_ros_driver2 메시지를 임포트할 수 없어 CustomMsg 발행을 끕니다. "
                    "colcon build --packages-select livox_ros_driver2 후 소싱하세요.")
                self.want_custom = False

        self.sub = self.create_subscription(PointCloud2, g("input_topic"),
                                            self.on_cloud, sensor_qos)
        self._n = 0
        self.get_logger().info(
            f"livox_sim_bridge 시작 — {g('input_topic')} → {g('output_topic')}"
            + (f" (+ {g('custom_topic')})" if self.want_custom else "")
            + f", 원형 FOV 마스크 {'켬' if self.apply_mask else '끔'}"
            f" (반각 {np.degrees(lc.FOV_HALF_ANGLE_RAD):.1f}°)"
            f", intensity 모드 {self.intensity_mode}")
        if self.intensity_mode == "passthrough":
            self.get_logger().info(
                "gz gpu_lidar 는 반사강도를 모델링하지 않아 intensity 가 전부 0 으로 나옵니다. "
                "합성이 필요하면 -p intensity_mode:=range (거리 기반, 반사율 아님)")

    # ── 콜백 ────────────────────────────────────────────────────────────
    def on_cloud(self, msg: PointCloud2):
        try:
            x, y, z, inten = _read_xyzi(msg)
        except KeyError as e:
            self.get_logger().error(str(e))
            return

        t0 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        r = lc.process_frame(x, y, z, inten, t0, self.period,
                             intensity_in_max=self.intensity_in_max,
                             apply_fov_mask=self.apply_mask,
                             intensity_mode=self.intensity_mode)

        hdr = Header(stamp=msg.header.stamp, frame_id=self.frame_id)
        self.pub.publish(self._make_pc2(hdr, r))
        # CustomMsg 는 점별 파이썬 루프라 비싸다. 구독자가 있을 때만 만든다.
        if (self.want_custom and self.pub_custom is not None
                and self.pub_custom.get_subscription_count() > 0):
            self.pub_custom.publish(self._make_custom(hdr, r, t0))

        self._n += 1
        if self.log_every_n and self._n % self.log_every_n == 0:
            self.get_logger().info(
                f"{self._n} 프레임 — 입력 {r['total']:,} → 출력 {r['kept']:,} 점 "
                f"({r['kept'] / max(r['total'], 1):.1%}), "
                f"{r['kept'] / self.period / 1000:.0f} kpts/s")

    # ── 메시지 조립 ─────────────────────────────────────────────────────
    def _make_pc2(self, hdr, r) -> PointCloud2:
        n = r["kept"]
        m = PointCloud2()
        m.header = hdr
        m.height = 1
        m.width = n
        m.is_bigendian = False
        m.point_step = lc.PXYZRTLT_POINT_STEP
        m.row_step = m.point_step * n
        m.is_dense = True                    # 무효점은 이미 걸렀다
        m.fields = [PointField(name=nm, offset=o, datatype=dt, count=c)
                    for nm, o, dt, c in lc.PXYZRTLT_FIELDS]
        m.data = lc.pack_pointxyzrtlt(r["x"], r["y"], r["z"],
                                      r["intensity"], r["timestamp"])
        return m

    def _make_custom(self, hdr, r, timebase_s):
        n = r["kept"]
        cm = CustomMsg()
        cm.header = hdr
        cm.timebase = int(timebase_s * 1e9)
        cm.point_num = n
        cm.lidar_id = 0
        cm.rsvd = [0, 0, 0]
        # 필드 불일치 함정: reflectivity 는 uint8, intensity 는 float32 0~255
        refl = lc.intensity_to_reflectivity(r["intensity"])
        offs = lc.offset_time_ns(r["timestamp"], timebase_s)
        xs, ys, zs = r["x"], r["y"], r["z"]
        pts = []
        for i in range(n):
            p = CustomPoint()
            p.offset_time = int(offs[i])
            p.x = float(xs[i]); p.y = float(ys[i]); p.z = float(zs[i])
            p.reflectivity = int(refl[i])
            p.tag = 0
            p.line = lc.LINE_ID
            pts.append(p)
        cm.points = pts
        return cm


def main(args=None):
    rclpy.init(args=args)
    node = LivoxSimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
