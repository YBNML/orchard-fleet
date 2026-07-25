#!/usr/bin/env python3
"""
gt_localizer — Stage-0 스캐폴드 (설계서 §7.5)

    ros2 run orchard_sim gt_localizer

gz 의 모델 참값 포즈를 읽어 **완벽한 map→odom** 을 발행한다.

왜 필요한가 (설계서 §7.5, "선택이 아니라 필수"):
    완벽한 위치추정 위에서 MPPI·인플레이션·경로그래프·행 기하를 먼저 튜닝해야,
    주행이 이상할 때 어느 층이 깨졌는지 알 수 있다. 위치추정과 항법을 동시에
    세우면 원인 분리가 불가능하다. 단계 7 에서 FAST-LIO2 로 갈아끼운다.

수식 (AMCL 이 하는 것과 동일):
    T_map_odom = T_map_base · (T_odom_base)⁻¹
        T_map_base : gz 참값 (map 프레임 = gz 월드 프레임으로 정의)
        T_odom_base: DiffDrive 가 내는 휠 오도메트리

입력  /gz_ground_truth   tf2_msgs/TFMessage  (gz dynamic_pose/info 브리지)
      /tf                DiffDrive 의 odom→base_link
출력  /tf                map→odom
      ~/localization_error  진단용 (참값 대비 오도메트리 드리프트)
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from orchard_sim import transforms as tfu


class GtLocalizer(Node):

    def __init__(self):
        super().__init__("gt_localizer")

        self.declare_parameter("ground_truth_topic", "/gz_ground_truth")
        self.declare_parameter("robot_model_name", "scout_mini_mid70")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("log_every_n", 150)

        g = lambda k: self.get_parameter(k).value
        self.model_name = g("robot_model_name")
        self.map_frame = g("map_frame")
        self.odom_frame = g("odom_frame")
        self.base_frame = g("base_frame")
        self.log_every_n = int(g("log_every_n"))

        self.T_map_base = None          # gz 참값
        self.gt_stamp = None            # 그 참값의 시각 — 시간 정합에 반드시 필요
        self._n = 0

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.br = TransformBroadcaster(self)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(TFMessage, g("ground_truth_topic"), self.on_gt, qos)
        self.err_pub = self.create_publisher(
            Float32MultiArray, "~/localization_error",
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=5))

        self.timer = self.create_timer(1.0 / max(float(g("publish_rate_hz")), 1.0),
                                       self.tick)
        self.get_logger().info(
            f"Stage-0 참값 로컬라이저 시작 — 모델 '{self.model_name}' 의 gz 참값으로 "
            f"{self.map_frame}→{self.odom_frame} 발행")
        self.get_logger().warn(
            "이것은 스캐폴드다. Nav2 튜닝이 끝나면 단계 7 에서 FAST-LIO2 로 교체한다.")

    # ── gz 참값 수신 ────────────────────────────────────────────────────
    def on_gt(self, msg: TFMessage):
        """dynamic_pose/info 브리지 결과에서 로봇 모델의 월드 포즈를 찾는다.

        gz 는 모델 포즈를 월드 기준으로, 링크 포즈를 모델 기준으로 낸다.
        우리가 원하는 것은 모델 엔트리(=월드→모델) 하나뿐이다.
        """
        for t in msg.transforms:
            if t.child_frame_id != self.model_name:
                continue
            p = t.transform.translation
            q = t.transform.rotation
            self.T_map_base = tfu.tf_from_pos_quat((p.x, p.y, p.z),
                                                   (q.x, q.y, q.z, q.w))
            self.gt_stamp = t.header.stamp
            return

    # ── map→odom 발행 ───────────────────────────────────────────────────
    def tick(self):
        if self.T_map_base is None:
            return
        # 시간 정합이 핵심이다. 참값(t1)과 오도메트리(t2)를 섞으면 회전 중에
        # 그 시차만큼 yaw 오차가 실린다 — 2026-07-25 실측에서 최대 3.37° 를 봤다.
        # 참값의 시각으로 오도메트리를 조회하고, 같은 시각으로 발행한다.
        stamp = self.gt_stamp
        try:
            tr = self.buf.lookup_transform(
                self.odom_frame, self.base_frame,
                rclpy.time.Time(seconds=stamp.sec, nanoseconds=stamp.nanosec))
        except Exception:
            try:                      # 그 시각이 버퍼에 없으면 최신으로 폴백
                tr = self.buf.lookup_transform(self.odom_frame, self.base_frame,
                                               rclpy.time.Time())
                stamp = tr.header.stamp
            except Exception:
                return                # DiffDrive TF 아직 없음 — 조용히 기다린다

        p, q = tr.transform.translation, tr.transform.rotation
        T_odom_base = tfu.tf_from_pos_quat((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
        T_map_odom = self.T_map_base @ tfu.invert(T_odom_base)

        t, quat = tfu.decompose(T_map_odom)
        m = TransformStamped()
        m.header.stamp = stamp          # 참값과 같은 시각 (시간 정합)
        m.header.frame_id = self.map_frame
        m.child_frame_id = self.odom_frame
        m.transform.translation.x = float(t[0])
        m.transform.translation.y = float(t[1])
        m.transform.translation.z = float(t[2])
        m.transform.rotation.x = float(quat[0])
        m.transform.rotation.y = float(quat[1])
        m.transform.rotation.z = float(quat[2])
        m.transform.rotation.w = float(quat[3])
        self.br.sendTransform(m)

        # 진단: map→odom 이 원점에서 벗어난 정도 = 휠 오도메트리 누적 드리프트
        dxy = float(math.hypot(t[0], t[1]))
        dyaw = float(tfu.yaw_of(T_map_odom))
        self.err_pub.publish(Float32MultiArray(
            data=[dxy, float(t[2]), dyaw]))

        self._n += 1
        if self.log_every_n and self._n % self.log_every_n == 0:
            self.get_logger().info(
                f"오도메트리 드리프트 — 평면 {dxy:.3f} m, z {t[2]:+.3f} m, "
                f"yaw {math.degrees(dyaw):+.2f}°")


def main(args=None):
    rclpy.init(args=args)
    node = GtLocalizer()
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
