#!/usr/bin/env python3
"""
sdf_static_tf — 로봇 SDF 를 읽어 센서 외부파라미터를 static TF 로 발행한다.

    ros2 run orchard_sim sdf_static_tf --ros-args -p model_sdf:=<경로>

왜 SDF 를 직접 읽는가: 센서 포즈를 YAML 에 다시 적어두면 model.sdf 와 조용히
어긋난다. 그러면 캘리브레이션·융합 결과가 전부 무의미해지는데 증상이 안 보인다.
같은 파일 하나를 진실의 근원으로 삼아 드리프트를 원천 차단한다.

발행: base_link → {livox_frame, imu_link, navsat_link, cam_*}  (static)
      링크 포즈는 SDF 에서 모델 원점 기준이므로 base_link 기준으로 변환한다.
"""
from __future__ import annotations

import os
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from orchard_sim import transforms as tfu

DEFAULT_SDF = "/home/myhome/YBNML/sim/models/scout_mini_mid70/model.sdf"


def parse_link_poses(sdf_path):
    """SDF 의 <link> 별 모델원점 기준 포즈를 4×4 로 읽는다."""
    root = ET.parse(sdf_path).getroot()
    model = root.find("model")
    if model is None:
        raise RuntimeError(f"{sdf_path} 에 <model> 이 없습니다")
    out = {}
    for link in model.findall("link"):
        name = link.get("name")
        pose_el = link.find("pose")
        pose = pose_el.text.strip() if pose_el is not None and pose_el.text else "0 0 0 0 0 0"
        out[name] = tfu.tf_from_pose_str(pose)
    return out


def base_relative(link_poses, base="base_link"):
    """모델원점 기준 포즈들을 base_link 기준으로 바꾼다."""
    if base not in link_poses:
        raise RuntimeError(f"SDF 에 '{base}' 링크가 없습니다: {list(link_poses)}")
    inv_base = tfu.invert(link_poses[base])
    return {n: inv_base @ T for n, T in link_poses.items() if n != base}


class SdfStaticTf(Node):

    def __init__(self):
        super().__init__("sdf_static_tf")
        self.declare_parameter("model_sdf", DEFAULT_SDF)
        self.declare_parameter("base_frame", "base_link")
        # 바퀴는 회전하므로 static 이 아니다. TF 트리에 넣지 않는다.
        self.declare_parameter("exclude", ["front_left_wheel", "front_right_wheel",
                                           "rear_left_wheel", "rear_right_wheel"])

        path = self.get_parameter("model_sdf").value
        base = self.get_parameter("base_frame").value
        exclude = set(self.get_parameter("exclude").value or [])

        if not os.path.exists(path):
            self.get_logger().error(f"SDF 를 찾을 수 없습니다: {path}")
            raise SystemExit(2)

        rel = base_relative(parse_link_poses(path), base)
        self.br = StaticTransformBroadcaster(self)

        stamp = self.get_clock().now().to_msg()
        msgs = []
        for name, T in sorted(rel.items()):
            if name in exclude:
                continue
            t, q = tfu.decompose(T)
            m = TransformStamped()
            m.header.stamp = stamp
            m.header.frame_id = base
            m.child_frame_id = name
            m.transform.translation.x = float(t[0])
            m.transform.translation.y = float(t[1])
            m.transform.translation.z = float(t[2])
            m.transform.rotation.x = float(q[0])
            m.transform.rotation.y = float(q[1])
            m.transform.rotation.z = float(q[2])
            m.transform.rotation.w = float(q[3])
            msgs.append(m)
            self.get_logger().info(
                f"  {base} → {name:<18} t=({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f})")

        if not msgs:
            self.get_logger().warn("발행할 static TF 가 없습니다")
        else:
            self.br.sendTransform(msgs)
            self.get_logger().info(f"static TF {len(msgs)}개 발행 (출처: {path})")


def main(args=None):
    rclpy.init(args=args)
    node = SdfStaticTf()
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
