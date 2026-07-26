#!/usr/bin/env python3
"""
통로 추출 파이프라인 시각화

    python3 scripts/08_plot_corridors.py [--out docs/figures/corridors.png]

각 단계가 무엇을 하는지 눈으로 볼 수 있게 6개 패널로 그린다.
수치만으로는 "왜 순진한 방법이 실패하는지"가 안 보이므로, 비교 패널을 함께 넣는다.
"""
import argparse
import json
import os
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt              # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402

# 한글 라벨이 두부(□)로 깨지지 않게 CJK 폰트를 지정한다.
# 이 머신에는 Noto Sans CJK JP 만 있고, 한글 글리프를 포함한다.
matplotlib.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

sys.path.insert(0, "ros2_ws/src/orchard_sim")
from orchard_sim import corridors as co       # noqa: E402
from orchard_sim import traversability as tv  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--out", default="docs/figures/corridors.png")
ap.add_argument("--cell", type=float, default=0.10)
args = ap.parse_args()

BASE = "sim/models/orchard_terrain"
M = json.load(open(f"{BASE}/heightmap_meta.json"))
S, X0, R = M["row_spacing"], M["x0"], M["rows"]
L, HL = M["row_length"], M["headland"]
PAD = 3.0
BOUNDS = (X0 - PAD, -(L / 2 + HL + PAD), X0 + (R - 1) * S + PAD, L / 2 + HL + PAD)

print("파이프라인 실행 중...")
g, ground = tv.dem_from_terrain(f"{BASE}/heightmap.npy", f"{BASE}/heightmap_meta.json",
                                cell=args.cell, bounds=BOUNDS)
trav, feat = tv.traversability(ground, cell=args.cell, **{
    k: tv.DEFAULTS[k] for k in ("step_win", "step_max", "slope_max", "rough_max")})
free = tv.drivable_mask(trav, cell=args.cell, inflate=tv.DEFAULTS["inflate"])
dist = co.distance_field(free, args.cell)
sk = co.skeleton(free, min_branch_px=tv.DEFAULTS["min_corridor_px"])
segs = co.trace_segments(sk)
graph = co.RouteGraph(g, segs, dist)
alleys = co.alley_polylines(graph, min_length=10.0)
inblock = [a for a in alleys
           if 0 <= int(round((a["x_mean"] - X0) / S - 0.5)) <= R - 2]

# 비교용 — 순진한 방법: 전역 z 임계로 장애물 판정
z_thresh = float(np.nanpercentile(ground, 50)) + 0.20
naive = ground < z_thresh

ext = [g.xmin, g.xmin + g.shape[1] * g.cell, g.ymin, g.ymin + g.shape[0] * g.cell]
kw = dict(origin="lower", extent=ext, aspect="equal", interpolation="nearest")

fig, axes = plt.subplots(2, 3, figsize=(19, 15), constrained_layout=True)
fig.suptitle("계단식 과수원 — 고도격자에서 주행가능 통로·중심선 추출 (참값 지형)",
             fontsize=15, fontweight="bold")

# ── 1. 고도격자 ─────────────────────────────────────────────────────────────
ax = axes[0, 0]
im = ax.imshow(ground, cmap="terrain", **kw)
plt.colorbar(im, ax=ax, shrink=0.75, label="지면 높이 [m]")
ax.set_title(f"1. 2.5D 고도격자\n통로마다 평지, 통로 사이 26~50 cm 단차", fontsize=11)

# ── 2. 창 내 단차 ───────────────────────────────────────────────────────────
ax = axes[0, 1]
im = ax.imshow(np.clip(feat["step"], 0, 0.5), cmap="inferno", **kw)
plt.colorbar(im, ax=ax, shrink=0.75, label="0.7 m 창 내 최대 고저차 [m]")
ax.set_title(f"2. 단차 (판정 임계 {tv.DEFAULTS['step_max']} m)\n"
             f"둑 0.22~0.41 m  vs  통로 0.00~0.02 m", fontsize=11)

# ── 3. 순진한 방법 (실패 예시) ──────────────────────────────────────────────
ax = axes[0, 2]
ax.imshow(naive, cmap=ListedColormap(["#b23a2e", "#cfd8bd"]), **kw)
ax.set_title(f"3. [비교] 전역 z 임계 방법\n"
             f"경사지에서 통로가 통째로 장애물/자유공간이 된다", fontsize=11)

# ── 4. 주행가능 판정 ────────────────────────────────────────────────────────
ax = axes[1, 0]
ax.imshow(trav, cmap=ListedColormap(["#8c2f22", "#7fa86b"]), **kw)
for k in range(R):
    ax.axvline(X0 + k * S, color="#1b1b1b", lw=0.5, alpha=0.35, ls=":")
ax.set_title(f"4. 주행가능 판정 (지형 계산만, 규칙 없음)\n"
             f"통로 중앙 9/9 가능 · 단차 둑 8/8 불가", fontsize=11)

# ── 5. 자유공간 + 거리장 ────────────────────────────────────────────────────
ax = axes[1, 1]
d_show = np.where(free, dist, np.nan)
im = ax.imshow(d_show, cmap="viridis", **kw)
plt.colorbar(im, ax=ax, shrink=0.75, label="가장 가까운 장애물까지 [m]")
ax.set_title(f"5. 자유공간 + 거리장 (로봇 반경 {tv.DEFAULTS['inflate']} m 팽창)\n"
             f"통로가 서로 격리됐다 — 사다리 위상", fontsize=11)

# ── 6. 중심선 + 경로 그래프 ─────────────────────────────────────────────────
ax = axes[1, 2]
ax.imshow(np.where(free, 1, 0), cmap=ListedColormap(["#20241c", "#e8ecdf"]), **kw)
for e in graph.edges:
    p = e["pts"]
    ax.plot(p[:, 0], p[:, 1], lw=1.0,
            color=("#2d7ef7" if e["kind"] == "alley" else "#f0a020"),
            alpha=0.85, zorder=3)
for a in inblock:
    k = int(round((a["x_mean"] - X0) / S - 0.5))
    ax.plot([X0 + (k + 0.5) * S] * 2, [-L / 2, L / 2],
            color="#e03050", lw=0.8, ls="--", alpha=0.8, zorder=2)

# 통로 0 → 5 최단경로 강조
if len(inblock) >= 2:
    a0, a1 = inblock[0], inblock[min(5, len(inblock) - 1)]
    n0 = graph.node_nearest(a0["pts"][0, 0], a0["pts"][0, 1])
    n1 = graph.node_nearest(a1["pts"][0, 0], a1["pts"][0, 1])
    pe, plen = graph.shortest_path(n0, n1)
    if pe:
        pp = np.vstack([graph.edges[e]["pts"] for e in pe])
        ax.plot(pp[:, 0], pp[:, 1], lw=3.2, color="#00d0b0", alpha=0.95,
                zorder=5, label=f"통로 0→5 최단경로 ({plen:.1f} m)")
        ax.scatter(pp[[0, -1], 0], pp[[0, -1], 1], s=55, c="#00d0b0",
                   ec="k", lw=0.8, zorder=6)
ax.axhline(L / 2, color="#ffffff", lw=0.8, ls="-.", alpha=0.6)
ax.axhline(-L / 2, color="#ffffff", lw=0.8, ls="-.", alpha=0.6)
ax.text(ext[0] + 0.5, L / 2 + 0.6, "나무 구역 끝", color="w", fontsize=8)
ax.legend(loc="lower left", fontsize=8, framealpha=0.85)
ax.set_title("6. 중심선(파랑) + 선회 연결(주황) + 참값(빨강 점선)\n"
             "경유점 없이 경로가 선회 구간을 돈다", fontsize=11)

for ax in axes.ravel():
    ax.set_xlabel("x [m]", fontsize=9)
    ax.set_ylabel("y [m]", fontsize=9)
    ax.tick_params(labelsize=8)

os.makedirs(os.path.dirname(args.out), exist_ok=True)
fig.savefig(args.out, dpi=110, facecolor="white")
print(f"→ {args.out}")

# 수치 요약
errs = []
for a in inblock:
    k = int(round((a["x_mean"] - X0) / S - 0.5))
    m = np.abs(a["pts"][:, 1]) <= L / 2
    if m.sum() >= 5:
        errs.append(a["pts"][m, 0] - (X0 + (k + 0.5) * S))
if errs:
    E = np.concatenate(errs)
    print(f"   통로 {len(inblock)}개 · 횡오차 RMSE {np.sqrt((E**2).mean())*1000:.1f} mm"
          f" · 최대 {np.abs(E).max()*1000:.0f} mm")
