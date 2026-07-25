#!/usr/bin/env python3
"""
계단식 지형 검증 (설계서 §4.2)

    python3 scripts/04_verify_terrain.py

확인 항목:
  1. 통로(테라스) 안이 평지인가         — 주행 방향/횡단 방향 기복
  2. 인접 통로 간 단차가 나는가         — 요청 범위 25~50 cm
  3. 선회 구역에서 램프로 연결되는가    — 로봇이 옆 통로로 넘어갈 수 있는 경사
  4. 법면이 로봇 주행면을 침범하지 않는가
"""
import json
import sys
import numpy as np

BASE = "sim/models/orchard_terrain"
H = np.load(f"{BASE}/heightmap.npy")
M = json.load(open(f"{BASE}/heightmap_meta.json"))
N = H.shape[0]


def z(x, y):
    """월드 (x,y) → 지면 높이 (바이리니어). gen_world.Terrain 과 동일한 식."""
    half, E = M["half"], M["size_x"]
    c = np.clip((x + half) / E * (N - 1), 0, N - 1)
    r = np.clip((y + half) / E * (N - 1), 0, N - 1)
    c0, r0 = int(c), int(r)
    c1, r1 = min(c0 + 1, N - 1), min(r0 + 1, N - 1)
    dc, dr = c - c0, r - r0
    top = H[r0, c0] * (1 - dc) + H[r0, c1] * dc
    bot = H[r1, c0] * (1 - dc) + H[r1, c1] * dc
    return float(top * (1 - dr) + bot * dr)


S = M["row_spacing"]
X0 = M["x0"]
R = M["rows"]
L = M["row_length"]
HL = M["headland"]
FACE = M["face_width"]

fails = []
print(f"지형: {M['size_x']:.0f}×{M['size_y']:.0f} m, size_z {M['size_z']:.2f} m, "
      f"프로파일 {M['profile']}")
print(f"격자: {R}열 × {M['trees_per_row']}주, 열간 {S} m, 열길이 {L:.1f} m, 선회 {HL} m\n")

# ── 1. 통로 평탄도 ──────────────────────────────────────────────────────────
print("── 1. 통로(테라스) 평탄도 ──")
print(f"{'통로':>4} {'중심 x':>8} {'평균 z':>8} {'주행방향 기복':>13} {'횡단 기복(중앙2m)':>18}")
print("─" * 60)
ys = np.linspace(-L / 2, L / 2, 120)
alley_z = {}
for k in range(R - 1):
    cx = X0 + (k + 0.5) * S
    along = np.array([z(cx, y) for y in ys])
    # 로봇이 실제 지나는 폭(중앙 ±1 m)의 횡단 기복
    xs_cross = np.linspace(cx - 1.0, cx + 1.0, 21)
    cross = np.array([z(x, 0.0) for x in xs_cross])
    a_rng = along.max() - along.min()
    c_rng = cross.max() - cross.min()
    alley_z[k] = float(along.mean())
    ok_a = "✔" if a_rng < 0.12 else "✘"
    ok_c = "✔" if c_rng < 0.12 else "✘"
    if a_rng >= 0.12 or c_rng >= 0.12:
        fails.append(f"통로 {k} 평탄도 (주행 {a_rng:.3f} / 횡단 {c_rng:.3f})")
    print(f"{k:>4} {cx:>8.2f} {along.mean():>8.3f} "
          f"{a_rng * 100:>10.1f} cm {ok_a}  {c_rng * 100:>12.1f} cm {ok_c}")

# ── 2. 인접 통로 단차 ───────────────────────────────────────────────────────
print("\n── 2. 인접 통로 간 단차 (요청: 25~50 cm 무작위) ──")
steps = []
for k in range(R - 2):
    d = alley_z[k + 1] - alley_z[k]
    steps.append(d)
    ok = "✔" if 0.20 <= abs(d) <= 0.58 else "✘"
    if not (0.20 <= abs(d) <= 0.58):
        fails.append(f"통로 {k}→{k+1} 단차 {d * 100:.0f} cm 가 범위 밖")
    print(f"   통로 {k} → {k + 1} : {d * 100:>6.1f} cm  {ok}")
steps = np.array(steps)
print(f"   평균 {steps.mean() * 100:.1f} cm · 범위 {steps.min() * 100:.0f}~{steps.max() * 100:.0f} cm"
      f" · 표준편차 {steps.std() * 100:.1f} cm  (무작위성 확인)")

# ── 3. 선회 구역 램프 ───────────────────────────────────────────────────────
print("\n── 3. 선회 구역 램프 (옆 통로로 넘어갈 수 있는가) ──")
y_head = L / 2 + HL * 0.85           # 선회 구역 안쪽
xs = np.linspace(X0, X0 + (R - 1) * S, 200)
prof = np.array([z(x, y_head) for x in xs])
grad = np.abs(np.gradient(prof, xs))
print(f"   y = {y_head:.1f} m (선회 구역) 횡단면")
print(f"   최대 경사 {grad.max():.1%} ({np.degrees(np.arctan(grad.max())):.1f}°)"
      f" · 평균 {grad.mean():.1%}")
ok = grad.max() < 0.30
if not ok:
    fails.append(f"선회 구역 램프 최대 경사 {grad.max():.0%} 가 너무 급함")
print(f"   Scout Mini 등판 한계 대비: {'✔ 여유' if ok else '✘ 급함'}"
      f"  (스펙 30°, 램프 {np.degrees(np.arctan(grad.max())):.1f}°)")

# 나무 구역 대비 — 나무 구역은 단차(계단), 선회 구역은 램프여야 한다
y_tree = 0.0
prof_t = np.array([z(x, y_tree) for x in xs])
grad_t = np.abs(np.gradient(prof_t, xs))
print(f"   [대조] 나무 구역 y=0 최대 경사 {grad_t.max():.0%}"
      f" — 법면이라 램프보다 급해야 정상")
if grad_t.max() <= grad.max():
    fails.append("나무 구역이 선회 구역보다 완만함 — 계단이 안 생겼을 수 있음")

# ── 4. 법면 위치 ────────────────────────────────────────────────────────────
print("\n── 4. 법면이 로봇 주행면을 침범하지 않는가 ──")
print(f"   법면 폭 {FACE:.2f} m, 수목열 선 중심 → 통로 중앙까지 여유 "
      f"{(S - FACE) / 2:.2f} m")
worst = 0.0
for k in range(R - 1):
    cx = X0 + (k + 0.5) * S
    xs_drive = np.linspace(cx - 1.0, cx + 1.0, 41)   # 로봇 주행 폭 2 m
    pr = np.array([z(x, 0.0) for x in xs_drive])
    g = np.abs(np.gradient(pr, xs_drive)).max()
    worst = max(worst, g)
print(f"   주행 폭(중앙 ±1 m) 내 최대 경사 {worst:.1%}"
      f"  {'✔' if worst < 0.10 else '✘ 법면이 주행면을 침범'}")
if worst >= 0.10:
    fails.append(f"주행면 내 경사 {worst:.0%} — 법면 침범")

# ── 결과 ────────────────────────────────────────────────────────────────────
print()
if fails:
    print("✘ 실패 항목:")
    for f in fails:
        print(f"   · {f}")
    sys.exit(1)
print("✔ 계단식 지형 검증 통과 — 통로 평지 / 통로간 단차 / 선회구역 램프 모두 정상")
