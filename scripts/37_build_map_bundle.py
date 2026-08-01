#!/usr/bin/env python3
"""맵 번들 생성 — 검증된 통로 추출 파이프라인을 오프라인 산출물로 승격한다.

    python3 scripts/37_build_map_bundle.py --out maps/orchard_v1
    python3 scripts/37_build_map_bundle.py --cloud /tmp/sweep_cloud.npz --out maps/orchard_sensed

입력을 두 가지로 받는다.
    참값 지형(기본)  sim/models/orchard_terrain 의 하이트맵. 맵핑 세션 품질과
                     무관하게 항법·경로 로직을 먼저 검증하려는 용도다.
    센서 점군        --cloud 로 준다. 실제 맵핑 세션 산출물에 해당한다.

두 경로 모두 같은 모듈(traversability → row_structure → corridors)을 쓴다.
14번 스크립트가 증명한 파이프라인을 그대로 쓰되, 그림 대신 번들을 낸다.
"""
from __future__ import annotations

import argparse
import json
import sys

import numpy as np

sys.path.insert(0, "ros2_ws/src/orchard_sim")

from orchard_sim import corridors as co          # noqa: E402
from orchard_sim import mapbundle                # noqa: E402
from orchard_sim import row_structure as rowmod  # noqa: E402
from orchard_sim import traversability as tv     # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--cloud", default="", help="센서 점군 npz (없으면 참값 지형)")
ap.add_argument("--out", default="maps/orchard_v1")
ap.add_argument("--cell", type=float, default=0.10)
ap.add_argument("--terrain", default="sim/models/orchard_terrain")
a = ap.parse_args()
CELL = a.cell
INFLATE = tv.DEFAULTS["inflate"]

M = json.load(open(f"{a.terrain}/heightmap_meta.json"))
S, X0, R = M["row_spacing"], M["x0"], M["rows"]
L, HL = M["row_length"], M["headland"]
PAD = 3.0
BOUNDS = (X0 - PAD, -(L / 2 + HL + PAD), X0 + (R - 1) * S + PAD, L / 2 + HL + PAD)

print("맵 번들 생성")
print("=" * 78)

# ── 1. 지면 격자 ────────────────────────────────────────────────────────────
if a.cloud:
    d = np.load(a.cloud)
    P = d["points"]
    g, ground, _relief, _cnt = tv.dem_from_points(P, cell=CELL, bounds=BOUNDS)
    src = f"센서 점군 {a.cloud} ({P.shape[0]:,} 점)"
    wall, _clutter = tv.obstacle_layers(P, g, ground)
    cloud_pts = P
else:
    g, ground = tv.dem_from_terrain(f"{a.terrain}/heightmap.npy",
                                    f"{a.terrain}/heightmap_meta.json",
                                    cell=CELL, bounds=BOUNDS)
    src = f"참값 지형 {a.terrain}"
    # 참값 경로에는 점군이 없다 — 나무를 기하로 세워 정합 기준 점군을 만든다.
    # (실제 맵핑 세션에서는 이 자리에 관측 점군이 온다)
    wall, cloud_pts = None, None
print(f"   입력: {src}")
print(f"   격자 {ground.shape} · 셀 {CELL} m · 관측 {np.isfinite(ground).mean():.1%}")

# ── 2. 주행가능 판정 ────────────────────────────────────────────────────────
trav, feat = tv.traversability(ground, cell=CELL, **{
    k: tv.DEFAULTS[k] for k in ("step_win", "step_max", "slope_max", "rough_max")})
observed = np.isfinite(ground)
known = observed & feat["enough"]        # '모름'은 장애물도 주행가능도 아니다
obst = known & (~trav)

if wall is not None:                     # 센서 경로 — 나무를 장애물에 합치고 밀봉
    geom_src = wall
    sealed, info = rowmod.seal_rows(geom_src, CELL,
                                    geom=rowmod.estimate_row_geometry(geom_src, CELL))
    obst = obst | sealed
    print(f"   열 밀봉 적용 {info.get('length', 0):.2f} m" if info.get("applied")
          else "   열 밀봉 생략 (주기성 약함)")
else:                                    # 참값 경로 — 나무 위치를 기하로 찍는다
    X = g.xmin + (np.arange(g.shape[1]) + 0.5) * CELL
    Y = g.ymin + (np.arange(g.shape[0]) + 0.5) * CELL
    GX, GY = np.meshgrid(X, Y)
    tree_x = np.array([X0 + k * S for k in range(R)])
    n_tree = M.get("trees_per_row", 41)
    ts = M.get("tree_spacing", 1.5)
    tree_y = (np.arange(n_tree) - (n_tree - 1) / 2) * ts
    trunk = np.zeros_like(obst)
    for tx in tree_x:
        for ty in tree_y:
            trunk |= ((GX - tx) ** 2 + (GY - ty) ** 2) <= 0.35 ** 2
    obst = obst | trunk
    # 정합 기준 점군 — 줄기를 수직으로 세운 것 (NDT 가 열을 잡을 수 있게)
    zs = np.arange(0.3, 1.9, 0.15)
    pts = [[tx, ty, z] for tx in tree_x for ty in tree_y for z in zs]
    cloud_pts = np.asarray(pts, dtype=np.float32)

free = tv.drivable_mask(~obst, cell=CELL, inflate=INFLATE, keep_largest=True)
print(f"   장애물 {int(obst.sum()):,} 셀 · 주행가능 {free.mean():.1%}")

# ── 3. 통로 중심선·그래프 ───────────────────────────────────────────────────
dist = co.distance_field(free, cell=CELL)
sk = co.skeleton(free, min_branch_px=tv.DEFAULTS["min_corridor_px"])
segs = co.trace_segments(sk)
graph = co.RouteGraph(g, segs, dist)
alleys = co.alley_polylines(graph, min_length=10.0)


def flanked(al, obst_mask, grid, cell, spacing, need=0.80):
    """양옆이 나무 열로 둘러싸인 통로만 '작업 통로'로 친다.

    바깥 열 너머의 여백 띠도 길고 곧아서 중심선이 잡히지만, 그건 밭이 아니라
    가장자리다. 기대 개수를 하드코딩하는 대신 기하로 판정한다 — 열 간격이
    바뀌어도, 밭 모양이 달라져도 그대로 성립한다.
    """
    pts = al["pts"]
    lo, hi = spacing * 0.30, spacing * 0.80      # 옆 열이 있어야 할 거리 범위
    hits = 0
    for x, y in pts:
        both = True
        for sgn in (-1, +1):
            found = False
            for off in np.arange(lo, hi, cell):
                r, c = grid.to_idx(x + sgn * off, y)
                if grid.inside(r, c) and obst_mask[r, c]:
                    found = True
                    break
            both &= found
        hits += both
    return hits / max(len(pts), 1) >= need


def trim_to_column(al, half_len):
    """통로 중심선을 나무 열 구간으로 자른다.

    추적된 엣지는 양 끝이 선회 구역으로 휘어 들어간다. 그 굽은 부분까지
    '통로 중심선'이라고 부르면 (1) 정확도 지표가 굽은 만큼 나빠지고
    (2) 경로 추종이 통로 안에서 이미 선회를 시작한다. 직선 구간만 통로로
    보고, 통로와 통로를 잇는 일은 경로 계획이 맡는다.
    """
    pts = al["pts"]
    m = np.abs(pts[:, 1]) <= half_len
    if m.sum() < 2:
        return None
    pts = pts[m]
    return dict(pts=pts, width=al["width"],
                length=float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum()))


kept = [al for al in alleys if flanked(al, obst, g, CELL, S)]
kept = [t for t in (trim_to_column(al, L / 2) for al in kept) if t is not None]
dropped = len(alleys) - len(kept)
alleys = kept
print(f"   세그먼트 {len(segs)} · 노드 {len(graph.nodes)} · 통로 중심선 {len(alleys)}개"
      f" (기대 {R - 1})" + (f" · 가장자리 띠 {dropped}개 제외" if dropped else ""))

if len(alleys) != R - 1:
    print(f"\n✗ 통로 개수가 기대와 다르다 ({len(alleys)} ≠ {R-1}) — 번들을 만들지 않는다")
    sys.exit(1)

# 중심선 정확도 — 통로 중앙에서 얼마나 벗어났나
alley_x = np.array([X0 + (k + 0.5) * S for k in range(R - 1)])
errs = []
for al in alleys:
    xs = al["pts"][:, 0]
    errs.append(xs - alley_x[np.abs(xs[:, None] - alley_x).argmin(axis=1)])
err = np.concatenate(errs)
rmse_mm = float(np.sqrt((err ** 2).mean()) * 1000)
print(f"   중심선 오차 RMSE {rmse_mm:.0f} mm · 최대 {np.abs(err).max()*1000:.0f} mm")

# ── 4. 저장 ─────────────────────────────────────────────────────────────────
meta = mapbundle.save(
    a.out, cloud=cloud_pts, trav=free, origin=(g.xmin, g.ymin), cell=CELL,
    alleys=alleys,
    geom=dict(rows=R, alleys=R - 1, row_spacing=S, x0=X0,
              col_len=L, headland=HL, tree_spacing=M.get("tree_spacing", 1.5)),
    source=src, notes=f"중심선 RMSE {rmse_mm:.0f} mm")

print(f"\n── 번들 저장 → {a.out} ──")
print(f"   해시 {meta['hash']} · 점군 {meta['n_points']:,} · 통로 {meta['n_alleys']}")

# ── 5. 되읽어 검증 ──────────────────────────────────────────────────────────
b = mapbundle.Bundle(a.out)
ok = []
ok.append(("해시 일치", b.verify()))
ok.append(("통로 수", b.alley_count() == R - 1))
ok.append(("통로 중앙이 주행가능", all(
    b.is_drivable(X0 + (k + 0.5) * S, 0.0) for k in range(R - 1))))
ok.append(("나무 열 위는 주행불가", not any(
    b.is_drivable(X0 + k * S, 0.0) for k in range(R))))
ok.append(("밭 밖은 주행불가", not b.is_drivable(X0 - 50.0, 0.0)))
print("\n── 되읽기 검증 ──")
for name, v in ok:
    print(f"   {'✔' if v else '✗'} {name}")
print(f"\n{sum(v for _, v in ok)}/{len(ok)} 통과")
sys.exit(0 if all(v for _, v in ok) else 1)
