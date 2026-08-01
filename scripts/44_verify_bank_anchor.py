#!/usr/bin/env python3
"""둑 앵커 + 생구름 슬립 폴백 검증 (오프라인)

    python3 scripts/44_verify_bank_anchor.py

1. 맵 레이캐스트 — 통로에서 북/남 둑까지 기대 거리가 상식과 맞는가
2. scan_travel 이 둑 벽면 점군에서도 전진량을 재는가 (생구름 폴백의 근거)
3. 기복 판별 — 평평한 맨땅은 거부, 둑 장면은 수용
"""
from __future__ import annotations

import math
import sys

import numpy as np

sys.path.insert(0, "ros2_ws/src/orchard_sim")
from orchard_sim import rowlocalize as rl                          # noqa: E402
from orchard_sim.mapbundle import Bundle                           # noqa: E402

rng = np.random.default_rng(11)
OK, NG = "✅", "❌"
res = []


def check(name, cond, detail=""):
    res.append(bool(cond))
    print(f"   {OK if cond else NG} {name}" + (f"  — {detail}" if detail else ""))


b = Bundle("maps/orchard_v1")


def raycast(x, y, yaw, max_s=15.0):
    hx, hy = math.cos(yaw), math.sin(yaw)
    for s in np.arange(0.3, max_s, 0.1):
        if not b.is_drivable(x + hx * s, y + hy * s):
            return float(s)
    return None


print("둑 앵커 + 생구름 폴백 검증")
print("=" * 78)

print("\n── 1. 맵 레이캐스트 ──")
e_n = raycast(-10.5, 30.0, math.pi / 2)
check("북단: (통로1, y=30)에서 북쪽 둑까지 8~10 m", e_n and 8.0 <= e_n <= 10.0,
      f"{e_n} m")
e_s = raycast(-10.5, -30.0, -math.pi / 2)
check("남단: 대칭으로 8~10 m", e_s and 8.0 <= e_s <= 10.0, f"{e_s} m")
e_mid = raycast(-10.5, 0.0, math.pi / 2, max_s=15.0)
check("통로 한복판에서는 15 m 안에 둑이 없다", e_mid is None, f"{e_mid}")

print("\n── 2. 둑 벽면 점군에서의 scan_travel ──")


def bank_scan(dist, n=2500, noise=0.02):
    """전방 dist 에 서 있는 높이 1.8 m 둑 벽면 + 앞쪽 지면 (센서 기준)."""
    pts = []
    for _ in range(n):
        yy = rng.uniform(-3.0, 3.0)
        zz = rng.uniform(-0.6, 1.2)
        pts.append((dist + rng.normal(0, noise), yy, zz))
    for _ in range(n // 2):                       # 지면
        rr = rng.uniform(1.0, dist - 0.3)
        yy = rng.uniform(-2.0, 2.0)
        pts.append((rr, yy, -0.61 + rng.normal(0, 0.02)))
    return np.asarray(pts)


sA, sB = bank_scan(8.0), bank_scan(7.5)           # 0.5 m 전진
tv, conf = rl.scan_travel(sA, sB)
check("둑으로 0.5 m 접근 → 변위 0.5", abs(tv - 0.5) <= 0.1,
      f"측정 {tv:.2f} · 상관 {conf:.2f}")
sC, sD = bank_scan(8.0), bank_scan(8.0)           # 제자리
tv0, conf0 = rl.scan_travel(sC, sD)
check("제자리 → 변위 0", abs(tv0) <= 0.1, f"측정 {tv0:.2f} · 상관 {conf0:.2f}")

print("\n── 3. 기복 판별 (폴백 수용/거부 기준) ──")
bank = bank_scan(8.0)
z = bank[:, 2]
relief_bank = float(np.percentile(z, 95) - np.percentile(z, 5))
check("둑 장면 기복 > 0.5 m", relief_bank > 0.5, f"{relief_bank:.2f} m")
flat = np.column_stack([rng.uniform(1, 18, 3000), rng.uniform(-4, 4, 3000),
                        -0.61 + rng.normal(0, 0.03, 3000)])
relief_flat = float(np.percentile(flat[:, 2], 95) - np.percentile(flat[:, 2], 5))
check("맨땅 장면 기복 < 0.5 m (판단 유보)", relief_flat < 0.5, f"{relief_flat:.2f} m")

print("\n" + "=" * 78)
print(f"결과: {sum(res)}/{len(res)} 통과")
sys.exit(0 if sum(res) == len(res) else 1)
