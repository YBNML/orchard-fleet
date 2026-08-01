#!/usr/bin/env python3
"""슬립 감지 + 선회 후 재획득 검증 (합성)

    python3 scripts/43_verify_slip_reacquire.py

무엇을 확인하나
    1. scan_travel — 실제로 전진하면 그만큼 나온다
    2. scan_travel — 제자리(슬립)면 0 이 나온다
    3. 슬립 판정 산술 — 오도 0.5 m vs 스캔 0 → 비율이 문턱 아래
    4. 재획득 — 요 오차 20° 는 ±12° 로는 못 잡지만 ±30° 로는 잡는다
    5. 주기 함정 — 탐색 폭이 반주기(0.75 m) 아래라 1.5 m 이웃 해에 안 붙는다
"""
from __future__ import annotations

import json
import math
import sys

import numpy as np

sys.path.insert(0, "ros2_ws/src/orchard_sim")
from orchard_sim import rowlocalize as rl                          # noqa: E402

rng = np.random.default_rng(7)
OK, NG = "✅", "❌"
res = []


def check(name, cond, detail=""):
    res.append(bool(cond))
    print(f"   {OK if cond else NG} {name}" + (f"  — {detail}" if detail else ""))


M = json.load(open("sim/models/orchard_terrain/heightmap_meta.json"))
GEOM = dict(x0=M["x0"], row_spacing=M["row_spacing"],
            tree_spacing=M.get("tree_spacing", 1.5), col_len=M["row_length"])
ROWS, NT = M["rows"], M.get("trees_per_row", 41)
TRUNK_X = np.array([GEOM["x0"] + k * GEOM["row_spacing"] for k in range(ROWS)])
TRUNK_Y = (np.arange(NT) - (NT - 1) / 2) * GEOM["tree_spacing"]

FOV_HALF = math.radians(35.2)
MAX_R = 25.0


def scan(pose_true, noise=0.03, n_per_trunk=26):
    """참 자세에서 MID-70 이 볼 법한 점군 (로봇 기준). 38번과 같은 생성기."""
    x, y, yaw = pose_true
    pts = []
    for tx in TRUNK_X:
        for ty in TRUNK_Y:
            dx, dy = tx - x, ty - y
            r = math.hypot(dx, dy)
            if r > MAX_R or r < 0.5:
                continue
            b = math.atan2(dy, dx) - yaw
            b = (b + math.pi) % (2 * math.pi) - math.pi
            if abs(b) > FOV_HALF:
                continue
            for z in np.linspace(0.35, 1.85, n_per_trunk):
                jx = tx + rng.normal(0, noise)
                jy = ty + rng.normal(0, noise)
                px = math.cos(-yaw) * (jx - x) - math.sin(-yaw) * (jy - y)
                py = math.sin(-yaw) * (jx - x) + math.cos(-yaw) * (jy - y)
                pts.append((px, py, z))
    for _ in range(len(pts)):
        rr = rng.uniform(1.0, MAX_R)
        bb = rng.uniform(-FOV_HALF, FOV_HALF)
        pts.append((rr * math.cos(bb), rr * math.sin(bb), rng.normal(0.0, 0.05)))
    return np.asarray(pts, dtype=float) if pts else np.zeros((0, 3))


ALLEY_X = GEOM["x0"] + 1.5 * GEOM["row_spacing"]      # 통로 1 중앙
HEAD = math.pi / 2                                     # 열 방향(+y)을 본다

print("슬립 감지 + 재획득 검증 (합성 MID-70)")
print("=" * 78)

# ── 1. scan_travel: 실제 전진 ───────────────────────────────────────────────
print("\n── 1. scan_travel — 전진 0.5 m ──")
sp0 = rl.structure_points(scan((ALLEY_X, -5.0, HEAD)))
sp1 = rl.structure_points(scan((ALLEY_X, -4.5, HEAD)))
tv, conf = rl.scan_travel(sp0, sp1)
check("변위가 참값(0.5 m)에 붙는다", abs(tv - 0.5) <= 0.12,
      f"측정 {tv:.2f} m · 상관 {conf:.2f}")
check("상관이 판단 문턱(0.3)을 넘는다", conf >= 0.3, f"{conf:.2f}")

# ── 2. scan_travel: 제자리 (슬립) ──────────────────────────────────────────
print("\n── 2. scan_travel — 제자리 (바퀴만 헛돎) ──")
spA = rl.structure_points(scan((ALLEY_X, -5.0, HEAD)))
spB = rl.structure_points(scan((ALLEY_X, -5.0, HEAD)))   # 노이즈만 다르다
tv0, conf0 = rl.scan_travel(spA, spB)
check("변위가 0 에 붙는다", abs(tv0) <= 0.10, f"측정 {tv0:.2f} m · 상관 {conf0:.2f}")

# ── 3. 슬립 판정 산술 ───────────────────────────────────────────────────────
print("\n── 3. 슬립 판정 — 오도 0.5 m vs 스캔 측정 ──")
ratio_stuck = abs(tv0) / 0.5
ratio_move = abs(tv) / 0.5
check("제자리면 비율이 문턱(0.35) 아래", ratio_stuck < 0.35, f"{ratio_stuck:.2f}")
check("전진이면 비율이 문턱 위", ratio_move >= 0.35, f"{ratio_move:.2f}")

# ── 4. 재획득 — 요 오차 20° ────────────────────────────────────────────────
print("\n── 4. 재획득 — 선회 직후 요 오차 20° ──")
true = (ALLEY_X, -3.0, HEAD)
prior = (ALLEY_X + 0.3, -3.0, HEAD + math.radians(20.0))
p = scan(true)
f12 = rl.estimate(p, prior, GEOM)                      # 기본 ±12°
ok12, why12 = rl.gate(f12, 0.0, GEOM)
check("±12° 로는 채택 불가 (경계 포화로 거부)", not ok12, why12)
f30 = rl.estimate(p, prior, GEOM, yaw_range_deg=30.0, coarse=121, fine=25)
ok30, _ = rl.gate(f30, 0.0, GEOM)
err_yaw = abs(math.degrees(f30.dyaw) + 20.0)
check("±30° 로는 잡는다", ok30 and err_yaw <= 1.5,
      f"dyaw {math.degrees(f30.dyaw):+.1f}° (참 −20°) · 채택 {ok30}")
err_x = abs((prior[0] + f30.dx) - true[0])
check("보정 후 횡 잔차 < 0.1 m", err_x <= 0.10, f"{err_x*1000:.0f} mm")

# ── 5. 주기 함정 — 이웃 해에 안 붙는가 ─────────────────────────────────────
print("\n── 5. 주기 함정 — 나무 간격 1.5 m 의 이웃 봉우리 ──")
spC = rl.structure_points(scan((ALLEY_X, -5.0, HEAD)))
spD = rl.structure_points(scan((ALLEY_X, -4.9, HEAD)))   # 실제 0.1 m 전진
tv5, conf5 = rl.scan_travel(spC, spD)
check("0.1 m 전진이 1.5 m 이웃 해로 튀지 않는다", abs(tv5 - 0.1) <= 0.10,
      f"측정 {tv5:.2f} m")

print("\n" + "=" * 78)
n_ok = sum(res)
print(f"결과: {n_ok}/{len(res)} 통과")
sys.exit(0 if n_ok == len(res) else 1)
