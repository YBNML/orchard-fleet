#!/usr/bin/env python3
"""
과수원 지형 하이트맵 생성기 (설계서 §4.2)

    python3 scripts/gen_heightmap.py --out sim/models/orchard_terrain/materials/textures

Gazebo 하이트맵의 하드 요구사항:
  · 정사각형, 한 변이 2^n + 1  (513 = 2^9 + 1)
  · 8-bit 그레이스케일, 알파 없음
  · 512x512 이나 RGBA 는 실패한다

평면을 쓰지 않는 이유: 완전한 평면은 오도메트리·IMU 거동과 LiDAR 지면분할을
실제보다 쉽게 만들어 결과를 오도한다. 1~2% 완경사를 준다.
"""
import argparse
import math
import os
import numpy as np


def value_noise_2d(shape, cells, seed):
    """격자 값 노이즈 + 코사인 보간. Perlin 근사이지만 외부 의존성이 없다."""
    rng = np.random.default_rng(seed)
    gh, gw = cells + 1, cells + 1
    grid = rng.random((gh, gw))

    h, w = shape
    ys = np.linspace(0, cells, h, endpoint=False)
    xs = np.linspace(0, cells, w, endpoint=False)
    gy, gx = np.meshgrid(ys, xs, indexing="ij")

    y0, x0 = np.floor(gy).astype(int), np.floor(gx).astype(int)
    y1, x1 = y0 + 1, x0 + 1
    fy, fx = gy - y0, gx - x0

    # 코사인 이징
    wy = (1 - np.cos(fy * math.pi)) * 0.5
    wx = (1 - np.cos(fx * math.pi)) * 0.5

    v00 = grid[y0, x0]; v01 = grid[y0, x1]
    v10 = grid[y1, x0]; v11 = grid[y1, x1]
    top = v00 * (1 - wx) + v01 * wx
    bot = v10 * (1 - wx) + v11 * wx
    return top * (1 - wy) + bot * wy


def fractal_noise(shape, seed, octaves=4, base_cells=4, persistence=0.5):
    out = np.zeros(shape, dtype=np.float64)
    amp, total = 1.0, 0.0
    for o in range(octaves):
        out += amp * value_noise_2d(shape, base_cells * (2 ** o), seed + o * 101)
        total += amp
        amp *= persistence
    out /= total
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", type=int, default=513, help="한 변 픽셀 (2^n+1)")
    ap.add_argument("--seed", type=int, default=2026)
    ap.add_argument("--octaves", type=int, default=4)
    ap.add_argument("--out", default="sim/models/orchard_terrain/materials/textures")
    ap.add_argument("--name", default="orchard_heightmap.png")
    args = ap.parse_args()

    n = args.size
    # 2^k + 1 검증
    if not (n >= 2 and ((n - 1) & (n - 2)) == 0):
        raise SystemExit(f"size 는 2^k+1 이어야 합니다 (예: 129/257/513/1025). 받은 값: {n}")

    field = fractal_noise((n, n), args.seed, octaves=args.octaves)

    # 가장자리를 살짝 낮춰 로봇이 월드 밖으로 나가도 벽처럼 튀지 않게 한다
    yy, xx = np.mgrid[0:n, 0:n] / (n - 1)
    edge = np.minimum.reduce([xx, 1 - xx, yy, 1 - yy])           # 0(가장자리)~0.5(중앙)
    field = field * (0.6 + 0.8 * np.clip(edge / 0.15, 0, 1))

    # 0~255 정규화
    field -= field.min()
    field /= max(field.max(), 1e-9)
    img = (field * 255).astype(np.uint8)

    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, args.name)

    # PNG 를 직접 쓴다 (8-bit 그레이스케일, 알파 없음) — PIL 없이 표준 라이브러리로
    _write_gray_png(path, img)

    relief_note = "  <size>120 120 1.5</size> 와 함께 쓰면 총기복 1.5 m ≈ 1~2% 완경사"
    print(f"[heightmap] {path}")
    print(f"[heightmap]   {n}x{n}, 8-bit grayscale, 알파 없음")
    print(f"[heightmap]   높이 히스토그램: min={img.min()} max={img.max()} mean={img.mean():.1f}")
    print(f"[heightmap] {relief_note}")


def _write_gray_png(path, arr):
    """8-bit 그레이스케일 PNG 를 표준 라이브러리(zlib)만으로 작성."""
    import struct
    import zlib

    h, w = arr.shape
    raw = bytearray()
    for y in range(h):
        raw.append(0)                       # 필터 타입 0 (None)
        raw.extend(arr[y].tobytes())

    def chunk(tag, data):
        c = tag + data
        return struct.pack(">I", len(data)) + c + struct.pack(">I", zlib.crc32(c) & 0xFFFFFFFF)

    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)   # bit depth 8, color type 0 (gray)
    idat = zlib.compress(bytes(raw), 9)
    with open(path, "wb") as f:
        f.write(sig)
        f.write(chunk(b"IHDR", ihdr))
        f.write(chunk(b"IDAT", idat))
        f.write(chunk(b"IEND", b""))


if __name__ == "__main__":
    main()
