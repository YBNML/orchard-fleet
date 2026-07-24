#!/usr/bin/env python3
"""
단계 1 — 라벨 인스턴스 분리 검증 분석기

label_probe.sdf 가 돌고 있는 상태에서 panoptic / semantic labels_map 을 한 장씩 받아
"과실을 어떤 SDF 구조로 배치해야 개별 인스턴스로 분리되는가"를 실측 판정한다.

panoptic labels_map 인코딩 — 2026-07-25 이 머신에서 실측으로 역공학함.
리서치 브리프가 주장한 "ch0=label, ch1..2=instance" 는 **틀렸다**. 실제는 반대다:

    channel 0 = 인스턴스 하위 바이트
    channel 1 = 인스턴스 상위 바이트      →  instance = ch1 * 256 + ch0
    channel 2 = semantic label (클래스)

(gz-sim 8.11.0 / gz-sensors 8.2.2, ros_gz_bridge 가 rgb8 로 전달)

사용법:
    # 터미널 1
    gz sim -s -r sim/worlds/label_probe.sdf
    # 터미널 2
    ros2 run ros_gz_bridge parameter_bridge \
        /probe/panoptic/labels_map@sensor_msgs/msg/Image[gz.msgs.Image \
        /probe/semantic/labels_map@sensor_msgs/msg/Image[gz.msgs.Image
    # 터미널 3
    python3 scripts/01_analyze_labels.py
"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

# label_probe.sdf 와 일치해야 한다
CONFIGS = {
    10: ("ground", "지면 (visual 1개)", 1),
    40: ("cfgA", "모델1 / 링크1 / visual 3개, 각 visual 에 Label", 3),
    41: ("cfgB", "모델1 / 링크3개, 각 링크의 visual 에 Label", 3),
    42: ("cfgC", "최상위 <model> 3개, 각 모델에 Label", 3),
    43: ("cfgD", "최상위 <include> 3개, 각 include 에 Label", 3),
    44: ("cfgE", "부모 모델 안에 중첩 <include> 3개 (gz-sim #1579)", 3),
}


def to_array(msg: Image) -> np.ndarray:
    """sensor_msgs/Image -> HxWxC uint8 배열."""
    chans = {"rgb8": 3, "bgr8": 3, "mono8": 1, "rgba8": 4, "bgra8": 4}.get(msg.encoding)
    if chans is None:
        raise RuntimeError(f"예상 못한 인코딩: {msg.encoding!r}")
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    return buf.reshape(msg.height, msg.step // chans, chans)[:, : msg.width, :]


class Analyzer(Node):
    def __init__(self):
        super().__init__("label_analyzer")
        # 센서 이미지는 best-effort 로 나온다
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.panoptic = None
        self.semantic = None
        self.create_subscription(
            Image, "/probe/panoptic/labels_map", self._on_panoptic, qos)
        self.create_subscription(
            Image, "/probe/semantic/labels_map", self._on_semantic, qos)

    def _on_panoptic(self, msg):
        if self.panoptic is None:
            self.panoptic = msg

    def _on_semantic(self, msg):
        if self.semantic is None:
            self.semantic = msg

    def done(self):
        return self.panoptic is not None and self.semantic is not None


def report(panoptic_msg, semantic_msg) -> int:
    pan = to_array(panoptic_msg)
    sem = to_array(semantic_msg)

    print(f"panoptic : {pan.shape}  encoding={panoptic_msg.encoding}")
    print(f"semantic : {sem.shape}  encoding={semantic_msg.encoding}")
    print()

    # --- semantic: 어떤 클래스 라벨이 실제로 화면에 있는가 -------------------
    sem_labels = np.unique(sem[:, :, 0])
    print("── semantic labels_map 에 존재하는 클래스 ──")
    for lab in sem_labels:
        px = int((sem[:, :, 0] == lab).sum())
        name = CONFIGS.get(int(lab), ("", "(미선언)", None))[0] or "background/기타"
        print(f"   label {int(lab):3d}  {px:8,d} px   {name}")
    print()

    # --- panoptic: 클래스별 distinct 인스턴스 개수 ---------------------------
    # 실측 인코딩: ch2 = 라벨, ch0 = 인스턴스 하위, ch1 = 인스턴스 상위
    cls = pan[:, :, 2].astype(np.uint16)
    inst = (pan[:, :, 1].astype(np.uint16) << 8) | pan[:, :, 0].astype(np.uint16)

    print("── panoptic 인스턴스 분리 판정 ──")
    print(f"{'label':>6} {'구성':<6} {'기대':>4} {'실측':>4}  {'판정':<6} 설명")
    print("─" * 92)

    failures = 0
    for lab in sorted(CONFIGS):
        name, desc, expected = CONFIGS[lab]
        mask = cls == lab
        if not mask.any():
            print(f"{lab:>6} {name:<6} {expected:>4} {'—':>4}  {'미검출':<6} 화면에 없음 (카메라 각도 확인)")
            failures += 1
            continue
        ids = np.unique(inst[mask])
        got = len(ids)
        verdict = "성공" if got == expected else "실패"
        if got != expected:
            failures += 1
        idlist = ", ".join(str(int(i)) for i in ids[:8])
        print(f"{lab:>6} {name:<6} {expected:>4} {got:>4}  {verdict:<6} {desc}")
        print(f"{'':>6} {'':<6} {'':>4} {'':>4}  {'':<6}   인스턴스 ID: [{idlist}]")

    print()
    return failures


def main():
    rclpy.init()
    node = Analyzer()
    print("labels_map 수신 대기 중...\n")

    deadline = node.get_clock().now().nanoseconds + 30 * 10**9
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.get_clock().now().nanoseconds > deadline:
            print("✗ 30초 안에 labels_map 을 받지 못했습니다.")
            print("  · gz sim 이 돌고 있는지")
            print("  · parameter_bridge 가 두 토픽을 브리지하고 있는지 확인하세요.")
            node.destroy_node()
            rclpy.shutdown()
            return 2

    failures = report(node.panoptic, node.semantic)
    node.destroy_node()
    rclpy.shutdown()

    if failures == 0:
        print("모든 구성이 기대대로 분리되었습니다.")
    else:
        print(f"{failures}개 구성이 기대와 다릅니다 — 위 표가 월드 생성기 아키텍처의 근거입니다.")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
