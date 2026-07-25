#!/usr/bin/env python3
"""
단계 4 통합 검증 — livox_sim_bridge 가 실제 gz 점군을 계약대로 변환하는가

    # 월드 + gz 브리지 + livox_sim_bridge 가 떠 있는 상태에서
    python3 scripts/05_verify_livox_bridge.py

단위테스트가 잡지 못하는 것을 본다:
  · 실제 gz 점군이 계약 형식으로 나오는가 (필드 배치·frame_id·is_dense)
  · 점 예산이 MID-70 사양 100 kpts/s 에 맞는가
  · PointCloud2 와 CustomMsg 두 경로의 값이 서로 일치하는가 (필드 불일치 함정)
  · 발행 주기가 10 Hz 인가
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, "ros2_ws/src/orchard_sim")
from orchard_sim import livox_contract as lc  # noqa: E402

try:
    from livox_ros_driver2.msg import CustomMsg
    HAVE_CUSTOM = True
except ImportError:
    HAVE_CUSTOM = False

TYPES = {1: "INT8", 2: "UINT8", 3: "INT16", 4: "UINT16",
         5: "INT32", 6: "UINT32", 7: "FLOAT32", 8: "FLOAT64"}


class V(Node):
    def __init__(self):
        super().__init__("livox_bridge_verifier")
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=10)
        self.raw = None
        self.out = None
        self.custom = None
        self.out_stamps = []
        self.raw_stamps = []
        self.create_subscription(PointCloud2, "/livox/points_raw/points", self._on_raw, q)
        self.create_subscription(PointCloud2, "/livox/lidar", self._on_out, q)
        if HAVE_CUSTOM:
            self.create_subscription(CustomMsg, "/livox/lidar_custom",
                                     lambda m: setattr(self, "custom", self.custom or m), q)

    def _on_raw(self, m):
        if self.raw is None:
            self.raw = m
        self.raw_stamps.append(time.monotonic())

    def _on_out(self, m):
        if self.out is None:
            self.out = m
        self.out_stamps.append(time.monotonic())

    def ready(self):
        return (self.raw is not None and self.out is not None
                and (self.custom is not None or not HAVE_CUSTOM)
                and len(self.out_stamps) >= 12)


def main():
    rclpy.init()
    n = V()
    print("브리지 출력 대기 중...\n")
    deadline = time.monotonic() + 45
    while rclpy.ok() and not n.ready():
        rclpy.spin_once(n, timeout_sec=0.2)
        if time.monotonic() > deadline:
            got = [k for k in ("raw", "out", "custom") if getattr(n, k) is not None]
            print(f"✗ 45초 안에 다 못 받음. 받은 것: {got}, 출력 프레임 {len(n.out_stamps)}")
            return 2

    fails = []
    raw, out = n.raw, n.out

    # ── 1. 계약 형식 ────────────────────────────────────────────────
    print("── 1. 출력 계약 형식 ──")
    print(f"  frame_id   : {out.header.frame_id!r}")
    print(f"  width      : {out.width:,} (height {out.height})")
    print(f"  point_step : {out.point_step}   is_dense: {out.is_dense}")
    print(f"\n  {'필드':>12} {'offset':>7} {'타입':>9}   기대")
    print("  " + "─" * 46)
    got = [(f.name, f.offset, f.datatype) for f in out.fields]
    exp = [(nm, o, dt) for nm, o, dt, _ in lc.PXYZRTLT_FIELDS]
    for (gn, go, gd), (en, eo, ed) in zip(got, exp):
        ok = (gn, go, gd) == (en, eo, ed)
        print(f"  {gn:>12} {go:>7} {TYPES.get(gd, gd):>9}   {'✔' if ok else f'✘ {en}@{eo}'}")
    if got != exp:
        fails.append("PointXYZRTLT 필드 배치가 계약과 다르다")
    if out.header.frame_id != "livox_frame":
        fails.append(f"frame_id 가 'livox_frame' 이 아니다: {out.header.frame_id!r}")
    if not out.is_dense:
        fails.append("무효점을 걸렀으면 is_dense 가 true 여야 한다")
    if out.point_step != lc.PXYZRTLT_POINT_STEP:
        fails.append(f"point_step {out.point_step} != {lc.PXYZRTLT_POINT_STEP}")

    # ── 2. 점 예산 ──────────────────────────────────────────────────
    # MID-70 사양 100 kpts/s 는 **발사율**이지 수신율이 아니다. 하늘로 쏜 광선은
    # 무반사이므로 실제 씬의 수신 점수는 이보다 적은 것이 정상이다.
    n_raw = raw.width * raw.height
    n_out = out.width
    period = 0.1
    r_grid = (raw.width - 1) / 2.0
    emit_expected = math.pi * r_grid * r_grid          # 113 격자의 원 내부 격자점 수
    print(f"\n── 2. 점 예산 ──")
    print(f"  격자 발사 {n_raw:,} → 원형 FOV 통과 기대 {emit_expected:,.0f}"
          f"  (= {emit_expected / period / 1000:.0f} kpts/s 발사 예산)")
    print(f"  실제 수신 {n_out:,}  (격자 대비 {n_out / max(n_raw,1):.1%},"
          f" 발사 대비 {n_out / emit_expected:.1%})")
    print(f"  수신률 {n_out / period / 1000:.1f} kpts/s"
          f"   ※ 무반사(하늘·90 m 초과)는 정상적으로 빠진다")
    if not (85_000 <= emit_expected / period <= 115_000):
        fails.append(f"발사 예산 {emit_expected/period:,.0f} pts/s 가 사양에서 벗어남")
    if n_out > emit_expected * 1.02:
        fails.append("수신 점수가 발사 예산을 초과 — FOV 마스크가 동작하지 않는다")
    if n_out < emit_expected * 0.3:
        fails.append(f"수신률이 발사 대비 {n_out/emit_expected:.0%} 로 비정상적으로 낮다")

    # ── 3. 값 정합 ──────────────────────────────────────────────────
    d = lc.unpack_pointxyzrtlt(out.data, n_out)
    dist = np.sqrt(d["x"] ** 2 + d["y"] ** 2 + d["z"] ** 2)
    az = np.arctan2(d["y"], d["x"])
    el = np.arctan2(d["z"], np.hypot(d["x"], d["y"]))
    off_axis = np.hypot(az, el)
    print(f"\n── 3. 출력 값 정합 ──")
    print(f"  거리      : {dist.min():.2f} ~ {dist.max():.2f} m")
    print(f"  축이탈각  : 최대 {np.degrees(off_axis.max()):.2f}° (한계 35.2°)")
    print(f"  intensity : {d['intensity'].min():.1f} ~ {d['intensity'].max():.1f}")
    print(f"  line      : 고유값 {sorted(set(d['line'].tolist()))}")
    print(f"  timestamp : 단조증가 {'✔' if np.all(np.diff(d['timestamp']) > 0) else '✘'}")
    if off_axis.max() > lc.FOV_HALF_ANGLE_RAD + 1e-4:
        fails.append(f"FOV 밖 점이 남아있다 ({np.degrees(off_axis.max()):.2f}°)")
    if not np.all(np.isfinite(dist)):
        fails.append("출력에 비유한 값이 있다")
    if set(d["line"].tolist()) != {lc.LINE_ID}:
        fails.append("MID-70 은 단일 레이저라 line 이 전부 0 이어야 한다")
    if d["intensity"].max() > 255.0 or d["intensity"].min() < 0.0:
        fails.append("intensity 가 0~255 범위를 벗어남")
    if d["intensity"].max() == 0.0:
        print("  ※ intensity 전부 0 — gz gpu_lidar 가 반사강도를 모델링하지 않는다.")
        print("     브리지 오류가 아니며, 필요시 -p intensity_mode:=range (합성, 반사율 아님)")

    # ── 4. 두 경로 일치 (필드 불일치 함정) ──────────────────────────
    if HAVE_CUSTOM and n.custom is not None:
        cm = n.custom
        print(f"\n── 4. CustomMsg 경로 (FAST-LIO2 용) ──")
        print(f"  point_num {cm.point_num:,} · lidar_id {cm.lidar_id} · timebase {cm.timebase}")
        cx = np.array([p.x for p in cm.points[:200]])
        crefl = np.array([p.reflectivity for p in cm.points[:200]])
        cline = {p.line for p in cm.points[:200]}
        coff = np.array([p.offset_time for p in cm.points[:200]])
        print(f"  reflectivity: {crefl.min()} ~ {crefl.max()}  (uint8)")
        print(f"  line 고유값 : {sorted(cline)}")
        print(f"  offset_time : {coff.min()} ~ {coff.max()} ns, 단조 "
              f"{'✔' if np.all(np.diff(coff) >= 0) else '✘'}")
        if cm.point_num != len(cm.points):
            fails.append("point_num 과 실제 점 개수가 다르다")
        if cline != {lc.LINE_ID}:
            fails.append("CustomMsg 의 line 이 0 이 아니다")
        if crefl.max() > 255:
            fails.append("reflectivity 가 uint8 범위를 넘음")
        if not np.all(np.diff(coff) >= 0):
            fails.append("offset_time 이 단조증가하지 않는다 (디스큐잉 깨짐)")
        # 두 경로가 같은 프레임이면 점 개수가 같아야 한다
        if abs(cm.point_num - n_out) > max(0.1 * n_out, 50):
            fails.append(f"두 경로 점 개수 불일치: PC2 {n_out} vs CustomMsg {cm.point_num}")
    else:
        print("\n── 4. CustomMsg — 건너뜀 (livox_ros_driver2 미빌드) ──")

    # ── 5. 발행 주기 ────────────────────────────────────────────────
    # 벽시계 주기는 RTF 에 비례해 느려진다 (시뮬 10 Hz × RTF).
    # 그러므로 절대 Hz 가 아니라 **입력 대비 1:1 추종**을 본다.
    hz_out = 1.0 / np.diff(np.array(n.out_stamps)).mean() if len(n.out_stamps) > 1 else 0.0
    hz_in = 1.0 / np.diff(np.array(n.raw_stamps)).mean() if len(n.raw_stamps) > 1 else 0.0
    track = hz_out / hz_in if hz_in > 0 else 0.0
    print(f"\n── 5. 발행 주기 (벽시계) ──")
    print(f"  입력 {hz_in:.2f} Hz → 출력 {hz_out:.2f} Hz  (추종률 {track:.1%})")
    print(f"  ※ 시뮬은 10 Hz(시뮬시간) 로 쏘지만 벽시계 주기는 RTF 만큼 느려진다.")
    print(f"     브리지는 입력을 1:1 로 따라가야 한다")
    if not (0.85 <= track <= 1.15):
        fails.append(f"브리지가 입력을 1:1 로 따라가지 못함 (추종률 {track:.0%})"
                     " — 프레임 드롭 또는 처리 지연")

    n.destroy_node()
    rclpy.shutdown()

    print()
    if fails:
        print("✗ 실패 항목:")
        for f in fails:
            print(f"   · {f}")
        return 1
    print("✔ livox_sim_bridge 통합 검증 통과 — 계약 형식·점 예산·FOV·두 경로·주기 모두 정상")
    return 0


if __name__ == "__main__":
    sys.exit(main())
