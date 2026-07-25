#!/usr/bin/env python3
"""
과수원 계단식(단구식) 지형 생성기 — 설계서 §4.2

    python3 scripts/gen_heightmap.py --rows 10 --terrace-step 0.30

한국의 구릉지 과수원은 비탈을 그대로 쓰지 않고 **계단식(단구식)** 으로 조성한다.
그 구조를 그대로 재현한다:

    ┌── 나무 구역 (|y| <= L/2) ──────────────────────────────┐
    │  통로마다 평지 테라스, 통로 사이는 단차(법면)          │
    │                                                        │
    │   통로0 ────┐                                          │
    │   (평지)    └─╲ 법면 (수관하부 청경 1.2 m 폭)          │
    │               ╲── 통로1 ────┐                          │
    │                   (평지)    └─╲                        │
    │                               ╲── 통로2 ───            │
    └────────────────────────────────────────────────────────┘
    ┌── 선회 구역 (|y| > L/2) ───────────────────────────────┐
    │  단차가 연속 경사로로 풀린다 → 로봇이 옆 통로로 이동    │
    └────────────────────────────────────────────────────────┘

즉 지형은 x(통로 가로지름)에 대해 계단, y(주행 방향)에 대해 평탄이며,
선회 구역에서만 계단이 매끄러운 램프로 블렌딩된다.

핵심 성질:
  · 로봇이 주행하는 통로 안은 평지 (노이즈 수 cm 만)
  · 인접 통로와 terrace-step 만큼 지면 높이 차
  · 법면(단차면)은 수관하부 청경 폭(1.2 m)에 놓여 로봇이 밟지 않는다
  · 선회 구역에서 램프로 연결되어 옆 통로 진입 가능

Gazebo 하이트맵 하드 요구사항: 정사각형, 한 변 2^n+1, 8-bit grayscale, 알파 없음.

출력:
  <out>/materials/textures/orchard_heightmap.png
  <out>/heightmap.npy         높이 필드 H (미터) — gen_world 가 나무를 앉힐 때 사용
  <out>/heightmap_meta.json   매핑·격자 파라미터 (gen_world 가 정합성 검사)
  <out>/model.sdf, model.config
"""
import argparse
import json
import math
import os
import struct
import zlib
import numpy as np


# ── 노이즈 ──────────────────────────────────────────────────────────────────
def value_noise(shape, cells, seed):
    rng = np.random.default_rng(seed)
    grid = rng.random((cells + 1, cells + 1))
    h, w = shape
    ys = np.linspace(0, cells, h, endpoint=False)
    xs = np.linspace(0, cells, w, endpoint=False)
    gy, gx = np.meshgrid(ys, xs, indexing="ij")
    y0, x0 = np.floor(gy).astype(int), np.floor(gx).astype(int)
    fy, fx = gy - y0, gx - x0
    wy = (1 - np.cos(fy * math.pi)) * 0.5
    wx = (1 - np.cos(fx * math.pi)) * 0.5
    v00 = grid[y0, x0]; v01 = grid[y0, x0 + 1]
    v10 = grid[y0 + 1, x0]; v11 = grid[y0 + 1, x0 + 1]
    top = v00 * (1 - wx) + v01 * wx
    bot = v10 * (1 - wx) + v11 * wx
    return top * (1 - wy) + bot * wy


def fractal(shape, seed, octaves=4, base=3, persistence=0.5):
    out = np.zeros(shape); amp = 1.0; total = 0.0
    for o in range(octaves):
        out += amp * value_noise(shape, base * (2 ** o), seed + o * 101)
        total += amp; amp *= persistence
    return out / total


def smoothstep(s):
    s = np.clip(s, 0.0, 1.0)
    return s * s * (3.0 - 2.0 * s)


# ── 계단식 높이 함수 ────────────────────────────────────────────────────────
K_MIN, K_MAX = -60, 60      # 테라스 인덱스 범위 (120 m 지형을 넉넉히 덮는다)


def build_terrace_levels(step_min, step_max, seed):
    """테라스별 누적 높이표를 만든다.

    실제 계단식 과수원은 단차가 일정하지 않다 — 지형·조성 시기·복토량에 따라
    통로마다 다르다. 그래서 각 단차를 [step_min, step_max] 에서 무작위로 뽑아
    누적한다. levels[i] 는 테라스 k = K_MIN + i 의 지면 높이.
    """
    rng = np.random.default_rng(seed)
    n = K_MAX - K_MIN + 1
    steps = rng.uniform(step_min, step_max, size=n - 1)
    levels = np.concatenate([[0.0], np.cumsum(steps)])
    return levels, steps


def _lvl(levels, k):
    return levels[np.clip(k - K_MIN, 0, len(levels) - 1)]


def terrace_profile(x, x0, spacing, levels, face_width):
    """통로마다 평지, 수목열 선에서 단차가 나는 계단 프로파일.

    · 테라스(통로) k 는 x ∈ [x0 + k*spacing, x0 + (k+1)*spacing] 을 차지한다
      (수목열 선 사이). 그 안은 평지.
    · 단차는 수목열 선(x0 + j*spacing) 을 중심으로 face_width 폭에 걸쳐 진다.
      face_width 는 수관하부 청경 폭(1.2 m)과 같게 두어 로봇 주행면을 침범하지 않는다.
    · 테라스 높이는 levels 표에서 온다 (통로마다 다른 단차).
    """
    phase = (x - x0) / spacing
    k = np.floor(phase).astype(int)
    f = phase - k                                  # 0(수목열) ~ 1(다음 수목열)
    hw = (face_width / 2.0) / spacing              # 정규화 반폭

    lvl_km1 = _lvl(levels, k - 1)
    lvl_k = _lvl(levels, k)
    lvl_kp1 = _lvl(levels, k + 1)

    # f < hw  : 앞 수목열 선의 단차 후반부 (k-1 → k)
    s_lo = 0.5 + f / (2.0 * hw)
    z_lo = lvl_km1 + (lvl_k - lvl_km1) * smoothstep(s_lo)
    # f > 1-hw : 다음 수목열 선의 단차 전반부 (k → k+1)
    s_hi = (f - (1.0 - hw)) / (2.0 * hw)
    z_hi = lvl_k + (lvl_kp1 - lvl_k) * smoothstep(s_hi)

    return np.where(f < hw, z_lo, np.where(f > 1.0 - hw, z_hi, lvl_k))


def ramp_profile(x, x0, spacing, levels):
    """단차를 푼 연속 경사로 (선회 구역용).

    테라스 중심들을 직선으로 잇는다 → 계단 프로파일과 테라스 중심에서 정확히 일치하고,
    로봇이 옆 통로로 넘어갈 때 끊김 없이 오르내릴 수 있다.
    """
    centers = x0 + (np.arange(K_MIN, K_MAX + 1) + 0.5) * spacing
    return np.interp(x, centers, levels)


TERRAIN_SDF = """<?xml version="1.0" ?>
<!-- 자동 생성 (gen_heightmap.py)
     계단식 과수원 지형: 통로 {rows}개 테라스, 통로간 단차 {step:.2f} m,
     법면 폭 {face:.2f} m, 선회 구역에서 연속 램프로 전환 -->
<sdf version="1.9">
  <model name="orchard_terrain">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><heightmap>
          <uri>model://orchard_terrain/materials/textures/orchard_heightmap.png</uri>
          <size>{sx} {sy} {sz:.4f}</size><pos>0 0 0</pos>
        </heightmap></geometry>
      </collision>
      <visual name="visual">
        <geometry><heightmap>
          <uri>model://orchard_terrain/materials/textures/orchard_heightmap.png</uri>
          <size>{sx} {sy} {sz:.4f}</size><pos>0 0 0</pos>
          <texture><size>8</size>
            <diffuse>model://orchard_terrain/materials/textures/grass.png</diffuse>
            <normal>model://orchard_terrain/materials/textures/flat_normal.png</normal>
          </texture>
        </heightmap></geometry>
        <material><ambient>0.42 0.46 0.32 1</ambient><diffuse>0.42 0.46 0.32 1</diffuse></material>
        <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>10</label></plugin>
      </visual>
    </link>
  </model>
</sdf>
"""

TERRAIN_CONFIG = """<?xml version="1.0"?>
<model>
  <name>orchard_terrain</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>과수원 계단식 지형 (자동 생성, gen_heightmap.py)</description>
</model>
"""


def write_gray_png(path, arr):
    h, w = arr.shape
    raw = bytearray()
    for y in range(h):
        raw.append(0)
        raw.extend(arr[y].tobytes())

    def chunk(tag, data):
        c = tag + data
        return struct.pack(">I", len(data)) + c + struct.pack(">I", zlib.crc32(c) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(bytes(raw), 9)))
        f.write(chunk(b"IEND", b""))


def main():
    ap = argparse.ArgumentParser()
    # 격자 — gen_world 와 반드시 일치해야 한다
    ap.add_argument("--rows", type=int, default=10, help="수목열 수 (gen_world 와 동일)")
    ap.add_argument("--trees-per-row", type=int, default=41)
    ap.add_argument("--row-spacing", type=float, default=3.50, help="열간 = 테라스 폭")
    ap.add_argument("--tree-spacing", type=float, default=1.50)
    ap.add_argument("--headland", type=float, default=6.0)
    # 계단 형상 — 실제 계단식 과수원처럼 단차를 통로마다 무작위로 준다
    ap.add_argument("--step-min", type=float, default=0.25,
                    help="인접 통로 간 지면 높이차 최솟값 m")
    ap.add_argument("--step-max", type=float, default=0.50,
                    help="인접 통로 간 지면 높이차 최댓값 m")
    ap.add_argument("--face-width", type=float, default=1.20,
                    help="단차면(법면) 폭 m. 수관하부 청경 폭과 같게 두는 것이 기본")
    ap.add_argument("--ramp-frac", type=float, default=0.40,
                    help="선회 구역 중 램프 전환에 쓰는 비율 (0~1). "
                         "블렌드 구간은 계단과 램프가 섞여 횡단 경사가 20~40%%로 남으므로 "
                         "짧을수록 통로 간 이동 가능 대역이 넓어진다. "
                         "0.75 였을 때 |y|<34 m 가 전부 횡단 불가였고 실제로 로봇이 전복했다.")
    ap.add_argument("--terrace-margin", type=int, default=2,
                    help="과수원 바깥으로 몇 단 더 계단을 두고 그 밖은 평탄하게 할지. "
                         "8-bit 하이트맵 양자화를 통로 평탄성보다 곱게 유지하는 데 필요")
    # 지형 판
    ap.add_argument("--size-px", type=int, default=513)
    ap.add_argument("--extent", type=float, default=120.0)
    ap.add_argument("--noise-amp", type=float, default=0.025,
                    help="통로 평탄성을 해치지 않을 만큼만. 기본 2.5 cm")
    ap.add_argument("--seed", type=int, default=2026)
    ap.add_argument("--out", default="sim/models/orchard_terrain")
    args = ap.parse_args()

    n = args.size_px
    if not (n >= 3 and ((n - 1) & (n - 2)) == 0):
        raise SystemExit(f"--size-px 는 2^k+1 이어야 합니다. 받은 값: {n}")

    E = args.extent
    half = E / 2.0
    S = args.row_spacing
    R = args.rows
    if args.step_min > args.step_max:
        raise SystemExit("--step-min 이 --step-max 보다 큽니다.")
    levels, all_steps = build_terrace_levels(args.step_min, args.step_max, args.seed)

    # 과수원 블록 바깥은 평탄하게 눌러 size_z 를 줄인다.
    # 8-bit 하이트맵은 size_z 를 256 단계로 쪼개므로, size_z 가 크면 양자화가
    # 통로 노이즈(±2.5 cm)보다 굵어져 "평지" 통로에 계단 아티팩트가 생긴다.
    _k_lo = -args.terrace_margin
    _k_hi = (R - 1) + args.terrace_margin
    _i_lo = np.clip(_k_lo - K_MIN, 0, len(levels) - 1)
    _i_hi = np.clip(_k_hi - K_MIN, 0, len(levels) - 1)
    levels[:_i_lo] = levels[_i_lo]
    levels[_i_hi + 1:] = levels[_i_hi]

    # gen_world 와 동일한 격자 원점
    x0 = -((R - 1) * S) / 2.0
    L = (args.trees_per_row - 1) * args.tree_spacing      # 나무 구역 y 길이

    coords = np.linspace(-half, half, n)
    wx = np.tile(coords, (n, 1))
    wy = np.tile(coords.reshape(-1, 1), (1, n))

    # 계단 ↔ 연속 램프 블렌딩 가중치: 나무 구역 0, 선회 구역 1
    y_edge = L / 2.0
    ramp_len = max(args.headland * args.ramp_frac, 0.5)
    w = smoothstep((np.abs(wy) - y_edge) / ramp_len)

    z_terr = terrace_profile(wx, x0, S, levels, args.face_width)
    z_ramp = ramp_profile(wx, x0, S, levels)
    H = (1.0 - w) * z_terr + w * z_ramp

    # 롤링 노이즈 — 통로 평탄성을 위해 소진폭
    H = H + (fractal((n, n), args.seed, octaves=4, base=3) - 0.5) * 2 * args.noise_amp

    H -= H.min()
    size_z = float(H.max())

    tex_dir = os.path.join(args.out, "materials", "textures")
    os.makedirs(tex_dir, exist_ok=True)
    png = (H / max(size_z, 1e-9) * 255).astype(np.uint8)
    write_gray_png(os.path.join(tex_dir, "orchard_heightmap.png"), png)

    np.save(os.path.join(args.out, "heightmap.npy"), H.astype(np.float32))
    # 과수원이 실제로 덮는 테라스 구간의 단차만 추려 기록한다
    k_lo = int(math.floor((x0 - x0) / S))                    # = 0
    k_hi = int(math.floor(((R - 1) * S + x0 - x0) / S))      # = R-1
    used_steps = all_steps[np.clip(np.arange(k_lo, k_hi) - K_MIN, 0, len(all_steps) - 1)]
    meta = dict(size_x=E, size_y=E, size_z=size_z, half=half, size_px=n,
                rows=R, trees_per_row=args.trees_per_row,
                row_spacing=S, tree_spacing=args.tree_spacing,
                headland=args.headland, x0=x0, row_length=L,
                step_min=args.step_min, step_max=args.step_max,
                face_width=args.face_width,
                ramp_len=ramp_len, noise_amp=args.noise_amp,
                terrace_steps=[round(float(s), 4) for s in used_steps],
                profile="terraced_random")
    with open(os.path.join(args.out, "heightmap_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)

    mean_step = float(np.mean(used_steps)) if len(used_steps) else 0.0
    with open(os.path.join(args.out, "model.sdf"), "w") as f:
        f.write(TERRAIN_SDF.format(sx=E, sy=E, sz=size_z, rows=R - 1,
                                   step=mean_step, face=args.face_width))
    with open(os.path.join(args.out, "model.config"), "w") as f:
        f.write(TERRAIN_CONFIG)

    # ── 요약 ────────────────────────────────────────────────────────────
    total_rise = float(used_steps.sum()) if len(used_steps) else 0.0
    face_grade_max = args.step_max / args.face_width
    ramp_grade_max = args.step_max / S
    steps_cm = " ".join(f"{s * 100:.0f}" for s in used_steps)
    print(f"[heightmap] {tex_dir}/orchard_heightmap.png  ({n}x{n}, 8-bit gray)")
    print(f"[heightmap]   계단식 — 테라스 {R - 1}단, 단차 무작위"
          f" {args.step_min * 100:.0f}~{args.step_max * 100:.0f} cm")
    print(f"[heightmap]   실제 단차(cm): {steps_cm}   평균 {mean_step * 100:.1f}")
    print(f"[heightmap]   법면 폭 {args.face_width:.2f} m → 법면 경사 최대 {face_grade_max:.0%}"
          f" (로봇 주행면 아님)")
    print(f"[heightmap]   선회 구역 램프 경사 최대 {ramp_grade_max:.1%}"
          f" ({math.degrees(math.atan(ramp_grade_max)):.1f}°) · 램프 길이 {ramp_len:.1f} m")
    print(f"[heightmap]   과수원 전체 표고차 {total_rise:.2f} m / 폭 {(R - 1) * S:.1f} m")
    print(f"[heightmap]   size_z {size_z:.3f} m · 통로 노이즈 ±{args.noise_amp * 100:.1f} cm")


if __name__ == "__main__":
    main()
