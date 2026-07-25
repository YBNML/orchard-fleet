#!/usr/bin/env python3
"""
과수원 지형 하이트맵 생성기 (설계서 §4.2, 경사면 개정판)

    python3 scripts/gen_heightmap.py --grade 0.045 --cross-axis x

한국 노지 과수원은 완만한 경사면에 설치된 경우가 많다. 그래서 지형을
"열을 가로지르는 방향(x)"으로 일정 구배(grade)를 갖는 경사면 + 완만한 롤링 노이즈로
만든다. 로봇은 열을 따라(y) 주행하므로 한 통로 안에서는 대체로 평탄하고,
옆 통로(다른 x)는 지면 높이가 다르다 — 사용자 요구사항.

Gazebo 하이트맵 하드 요구사항: 정사각형, 한 변 2^n+1, 8-bit grayscale, 알파 없음.

높이 매핑을 gen_world 와 정확히 일치시키기 위해:
  · 지형 높이를 미터 단위 필드 H(x,y) 로 정의하고
  · PNG(=시각/충돌)와 heightmap.npy(=나무 배치용)를 같은 필드에서 생성한다
  · 경사 성분은 x 만의 함수라 이미지 상하 뒤집힘(v-flip)에 강건하다

출력:
  sim/models/orchard_terrain/materials/textures/orchard_heightmap.png
  sim/models/orchard_terrain/heightmap.npy         (H, 미터)
  sim/models/orchard_terrain/heightmap_meta.json    (매핑 파라미터)
  sim/models/orchard_terrain/model.sdf              (size_z 반영)
"""
import argparse
import json
import math
import os
import struct
import zlib
import numpy as np


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


TERRAIN_SDF = """<?xml version="1.0" ?>
<!-- 자동 생성 (gen_heightmap.py). 경사면 지형: 열을 가로지르는 {grade:.1%} 구배 -->
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
  <description>과수원 경사면 지형 (자동 생성, gen_heightmap.py)</description>
</model>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--size-px", type=int, default=513, help="한 변 픽셀 (2^n+1)")
    ap.add_argument("--extent", type=float, default=120.0, help="지형 한 변 m")
    ap.add_argument("--grade", type=float, default=0.045,
                    help="경사 구배 (0.045 = 4.5%%, 약 2.6도). 열을 가로지르는 방향")
    ap.add_argument("--cross-axis", choices=["x", "y"], default="x",
                    help="구배 방향. x=열을 가로지름(권장) → 통로마다 높이 다름")
    ap.add_argument("--noise-amp", type=float, default=0.06, help="롤링 노이즈 진폭 m")
    ap.add_argument("--seed", type=int, default=2026)
    ap.add_argument("--out", default="sim/models/orchard_terrain")
    args = ap.parse_args()

    n = args.size_px
    if not (n >= 3 and ((n - 1) & (n - 2)) == 0):
        raise SystemExit(f"--size-px 는 2^k+1 이어야 합니다. 받은 값: {n}")

    E = args.extent
    half = E / 2.0
    # 픽셀 → 월드 좌표. 열 index=row(y), col index=x
    coords = np.linspace(-half, half, n)
    wx = np.tile(coords, (n, 1))            # 각 픽셀의 world x (열 방향)
    wy = np.tile(coords.reshape(-1, 1), (1, n))  # world y

    # 경사 성분 (미터) — cross-axis 를 따라 선형 상승, 최저=0
    axis = wx if args.cross_axis == "x" else wy
    slope = args.grade * (axis + half)      # [0, grade*E]

    # 롤링 노이즈 (시각 현실감). 나무는 순수 경사면에 앉히므로 배치엔 영향 최소
    noise = (fractal((n, n), args.seed, octaves=4, base=3) - 0.5) * 2 * args.noise_amp

    H = slope + noise
    H -= H.min()                            # 최저 0
    size_z = float(H.max())

    tex_dir = os.path.join(args.out, "materials", "textures")
    os.makedirs(tex_dir, exist_ok=True)
    png = (H / max(size_z, 1e-9) * 255).astype(np.uint8)
    write_gray_png(os.path.join(tex_dir, "orchard_heightmap.png"), png)

    # 나무 배치용: 순수 경사 성분만 저장 (노이즈 제외 → 나무가 확실히 지면에 앉음)
    # gen_world 는 z(x) = grade*(axis+half) 로 앤 다. 노이즈는 시각 전용.
    np.save(os.path.join(args.out, "heightmap.npy"), H.astype(np.float32))
    meta = dict(size_x=E, size_y=E, size_z=size_z, grade=args.grade,
                cross_axis=args.cross_axis, half=half, size_px=n,
                noise_amp=args.noise_amp)
    with open(os.path.join(args.out, "heightmap_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)

    with open(os.path.join(args.out, "model.sdf"), "w") as f:
        f.write(TERRAIN_SDF.format(sx=E, sy=E, sz=size_z, grade=args.grade))
    with open(os.path.join(args.out, "model.config"), "w") as f:
        f.write(TERRAIN_CONFIG)

    total_rise = args.grade * E
    print(f"[heightmap] {tex_dir}/orchard_heightmap.png  ({n}x{n}, 8-bit gray)")
    print(f"[heightmap]   경사 {args.grade:.1%} ({math.degrees(math.atan(args.grade)):.1f}°)"
          f" · {args.cross_axis}축 · size_z {size_z:.3f} m")
    print(f"[heightmap]   전체 표고차 {total_rise:.2f} m / {E:.0f} m")
    print(f"[heightmap]   인접 통로(3.5 m) 높이차 ≈ {args.grade * 3.5 * 100:.0f} cm")


if __name__ == "__main__":
    main()
