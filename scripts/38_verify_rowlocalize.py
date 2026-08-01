#!/usr/bin/env python3
"""열 상대 로컬리제이션 검증 — 설계 가설을 합성 데이터로 시험한다.

    python3 scripts/38_verify_rowlocalize.py

가설: 사전 맵이 있으면 횡·요 오차는 **누적되지 않는다**. 열이 보이는 한
매 스캔 절대 기준에 다시 붙기 때문이다. (원래 실패했던 LIO 는 선회부에서
이 제약이 통째로 사라져 오차가 쌓였다.)

MID-70 을 흉내낸다 — 전방 원뿔 ±35.2°, 최대 사거리 25 m. 이 좁은 시야가
문제의 근원이므로 시야를 넉넉히 주고 시험하면 의미가 없다.
"""
from __future__ import annotations

import json
import math
import sys

import numpy as np

sys.path.insert(0, "ros2_ws/src/orchard_sim")
from orchard_sim import rowlocalize as rl        # noqa: E402

OK, NG = "\033[92m✔\033[0m", "\033[91m✗\033[0m"
res = []
rng = np.random.default_rng(7)


def check(name, cond, detail=""):
    res.append(bool(cond))
    print(f"   {OK if cond else NG} {name}" + (f"  — {detail}" if detail else ""))


M = json.load(open("sim/models/orchard_terrain/heightmap_meta.json"))
GEOM = dict(x0=M["x0"], row_spacing=M["row_spacing"],
            tree_spacing=M.get("tree_spacing", 1.5), col_len=M["row_length"])
ROWS, NT = M["rows"], M.get("trees_per_row", 41)
TRUNK_X = np.array([GEOM["x0"] + k * GEOM["row_spacing"] for k in range(ROWS)])
TRUNK_Y = (np.arange(NT) - (NT - 1) / 2) * GEOM["tree_spacing"]

FOV_HALF = math.radians(35.2)            # MID-70 원형 시야 반각
MAX_R = 25.0


def scan(pose_true, noise=0.03, n_per_trunk=26):
    """참 자세에서 MID-70 이 볼 법한 점군을 만든다 (로봇 기준 좌표)."""
    x, y, yaw = pose_true
    pts = []
    for tx in TRUNK_X:
        for ty in TRUNK_Y:
            dx, dy = tx - x, ty - y
            r = math.hypot(dx, dy)
            if r > MAX_R or r < 0.5:
                continue
            b = math.atan2(dy, dx) - yaw            # 로봇 기준 방위
            b = (b + math.pi) % (2 * math.pi) - math.pi
            if abs(b) > FOV_HALF:                    # 전방 원뿔 밖
                continue
            # 줄기 표면 점 (수직으로 세운 기둥)
            for z in np.linspace(0.35, 1.85, n_per_trunk):
                jx = tx + rng.normal(0, noise)
                jy = ty + rng.normal(0, noise)
                px = math.cos(-yaw) * (jx - x) - math.sin(-yaw) * (jy - y)
                py = math.sin(-yaw) * (jx - x) + math.cos(-yaw) * (jy - y)
                pts.append((px, py, z))
    # 지면 점도 섞는다 — 실제 스캔의 55~69% 는 지면이다
    for _ in range(len(pts)):
        rr = rng.uniform(1.0, MAX_R)
        bb = rng.uniform(-FOV_HALF, FOV_HALF)
        pts.append((rr * math.cos(bb), rr * math.sin(bb), rng.normal(0.0, 0.05)))
    return np.asarray(pts, dtype=float) if pts else np.zeros((0, 3))


print("열 상대 로컬리제이션 검증")
print("=" * 78)

# ── 1. 지면 제거 ────────────────────────────────────────────────────────────
print("\n── 1. 구조점 추출 ──")
p = scan((GEOM["x0"] + 1.5 * GEOM["row_spacing"], 0.0, math.pi / 2))
sp = rl.structure_points(p)
check("지면점이 걸러짐", len(sp) < len(p) * 0.6, f"{len(p)} → {len(sp)}")
check("구조점이 남음", len(sp) > 200, f"{len(sp)}점")

# ── 2. 단발 보정 정확도 ─────────────────────────────────────────────────────
print("\n── 2. 자세를 일부러 틀어놓고 되찾는가 ──")
alley_x = GEOM["x0"] + 1.5 * GEOM["row_spacing"]     # 통로 1 중앙
errs_x, errs_yaw = [], []
for dx_true in (-1.2, -0.6, -0.2, 0.2, 0.6, 1.2):
    for dyaw_true in (math.radians(-6), 0.0, math.radians(6)):
        true = (alley_x + dx_true, -5.0, math.pi / 2 + dyaw_true)
        prior = (alley_x, -5.0, math.pi / 2)         # 틀린 사전 자세
        f = rl.estimate(scan(true), prior, GEOM)
        # 보정을 적용했을 때 남는 오차
        errs_x.append(abs((prior[0] + f.dx) - true[0]))
        errs_yaw.append(abs((prior[2] + f.dyaw) - true[2]))
ex, ey = np.array(errs_x), np.degrees(errs_yaw)
check("횡 보정 후 잔차 < 0.10 m", ex.max() < 0.10,
      f"평균 {ex.mean()*1000:.0f} mm · 최대 {ex.max()*1000:.0f} mm")
check("요 보정 후 잔차 < 2°", ey.max() < 2.0,
      f"평균 {ey.mean():.2f}° · 최대 {ey.max():.2f}°")

# ── 3. 누적되지 않는가 (설계 가설의 핵심) ───────────────────────────────────
print("\n── 3. 통로를 왕복해도 횡 오차가 누적되지 않는가 ──")
# 오도메트리가 매 걸음 편의(bias)를 갖고 밀린다고 가정하고, 보정을 걸며 주행
step, drift_per_step = 0.5, 0.02          # 걸음당 2 cm 씩 한쪽으로 밀리는 오도메트리
pose = np.array([alley_x, -M["row_length"] / 2 + 2.0, math.pi / 2])
true = pose.copy()
hist_corr, hist_raw = [], []
raw = pose.copy()
for i in range(int((M["row_length"] - 4.0) / step)):
    true[1] += step
    pose[1] += step; pose[0] += drift_per_step      # 보정 있는 쪽
    raw[1] += step;  raw[0] += drift_per_step       # 보정 없는 쪽 (비교군)
    f = rl.estimate(scan(tuple(true)), tuple(pose), GEOM)
    okc, _ = rl.gate(f, abs(drift_per_step), GEOM)
    if okc:
        pose[0] += f.dx
        pose[2] += f.dyaw
    hist_corr.append(abs(pose[0] - true[0]))
    hist_raw.append(abs(raw[0] - true[0]))
hc, hr = np.array(hist_corr), np.array(hist_raw)
check("보정 없으면 오차가 누적된다 (비교군)", hr[-1] > 0.8,
      f"종단 {hr[-1]:.2f} m")
check("보정하면 누적되지 않는다", hc.max() < 0.15,
      f"평균 {hc.mean()*1000:.0f} mm · 최대 {hc.max()*1000:.0f} mm")
# 통로 끝으로 갈수록 앞쪽 나무가 줄어 보정이 성겨진다 — 오차가 조금 커지는 것은
# 물리적으로 당연하다. 중요한 건 '발산하지 않고 예산 안에 머무는가' 다.
q = len(hc) // 4
head, tail = hc[:q].mean(), hc[-q:].mean()
check("구간별로 나눠 봐도 발산하지 않는다", tail < 0.20 and tail < head + 0.15,
      f"앞 1/4 {head*1000:.0f} mm → 뒤 1/4 {tail*1000:.0f} mm "
      f"(M3 예산 300 mm)")

# ── 4. 퇴화 감지 ────────────────────────────────────────────────────────────
print("\n── 4. 열이 안 보이면 스스로 물러나는가 ──")
empty = np.zeros((0, 3))
f = rl.estimate(empty, (alley_x, 0.0, math.pi / 2), GEOM)
ok_e, why = rl.gate(f, 0.0, GEOM)
check("빈 스캔에서 보정 거부", not ok_e, why)

# 선회 구역 — 밭 밖으로 나가 열이 시야에서 사라진 상황
far = (alley_x, M["row_length"] / 2 + M["headland"] - 1.0, 0.0)   # 옆을 봄
f2 = rl.estimate(scan(far), far, GEOM)
ok_f, why2 = rl.gate(f2, 0.0, GEOM)
print(f"      선회 구역: 구조점 {f2.n_struct} · 집중도 {f2.quality:.2f} → "
      f"{'채택' if ok_f else '거부 (' + why2 + ')'}")

# 표류가 한계를 넘으면 거부해야 한다 (엉뚱한 열에 붙는 것보다 낫다)
f3 = rl.estimate(scan((alley_x, 0.0, math.pi / 2)), (alley_x, 0.0, math.pi / 2), GEOM)
ok3, why3 = rl.gate(f3, GEOM["row_spacing"], GEOM)
check("표류가 열 간격 절반을 넘으면 거부", not ok3, why3)

# ── 5. 잘못된 열에 붙지 않는가 ──────────────────────────────────────────────
print("\n── 5. 위상 접힘 한계 ──")
# 사전 자세가 한 열(3.5 m) 통째로 틀리면 위상만으로는 구분할 수 없다 —
# 그래서 gate 가 표류를 감시한다. 여기서는 그 한계를 문서화한다.
true5 = (alley_x, 0.0, math.pi / 2)
f5 = rl.estimate(scan(true5), (alley_x + GEOM["row_spacing"], 0.0, math.pi / 2), GEOM)
resid = abs((alley_x + GEOM["row_spacing"] + f5.dx) - true5[0])
check("한 열 어긋난 사전값은 위상으로 못 고친다 (한계 확인)",
      resid > GEOM["row_spacing"] * 0.8,
      f"잔차 {resid:.2f} m — gate 의 표류 감시가 이 상황을 막는 장치")

print("\n" + "=" * 78)
n_ok, n = sum(res), len(res)
print(f"{n_ok}/{n} 통과")
sys.exit(0 if n_ok == n else 1)
