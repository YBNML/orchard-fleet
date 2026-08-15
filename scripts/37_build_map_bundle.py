#!/usr/bin/env python3
"""맵 번들 생성 — 검증된 통로 추출 파이프라인을 오프라인 산출물로 승격한다.

    python3 scripts/37_build_map_bundle.py --out maps/orchard_v1
    python3 scripts/37_build_map_bundle.py --cloud /tmp/sweep_cloud.npz --out maps/orchard_sensed
    python3 scripts/37_build_map_bundle.py --farm maps/orchard_real/farm.json \
        --terrain sim/models/orchard_terrain_real --out maps/orchard_real

입력을 두 가지로 받는다.
    참값 지형(기본)  sim/models/orchard_terrain 의 하이트맵. 맵핑 세션 품질과
                     무관하게 항법·경로 로직을 먼저 검증하려는 용도다.
    센서 점군        --cloud 로 준다. 실제 맵핑 세션 산출물에 해당한다.

두 경로 모두 같은 모듈(traversability → row_structure → corridors)을 쓴다.
14번 스크립트가 증명한 파이프라인을 그대로 쓰되, 그림 대신 번들을 낸다.

**기하의 출처 — `--farm` (2026-08-15, 스펙 ④ T4)**
    계단식 월드는 열이 `x0 + k·row_spacing` 에 균일하게 서고 블록이 y=0 대칭이라
    지형 메타(heightmap_meta.json)의 스칼라 몇 개로 기하가 완전히 결정된다.
    실사 정사영상 농장은 그렇지 않다 — 열 간격이 4.75~15.25 m 로 불균일하고
    (통로 20 은 15.25 m 농로), 열마다 길이가 다르며(128~141 m), 블록이 y=0
    대칭이 아니다(캐노피 y −97.9 ~ +104.8). 게다가 평탄 지형 메타에는 x0·
    row_length 자체가 없다. 그래서 `--farm` 을 주면 **기하를 farm.json 에서
    직접** 읽는다(스펙 ④: 기하는 코드가 아니라 데이터로 흐른다).

    두 경로가 공유하는 것은 격자·주행가능·스켈레톤·번들 저장이고, 갈리는 것은
    (1) 열·나무 위치 (2) 통로 구간 판정 (3) 통로 번호 매김 세 곳뿐이다.
    계단식 경로의 산출물은 바이트 단위로 불변이다.
"""
from __future__ import annotations

import argparse
import json
import math
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
ap.add_argument("--farm", default="",
                help="농장 매니페스트(실사 월드). 주면 기하를 지형 메타가 아니라 "
                     "여기서 읽는다 — 불균일 간격·열별 길이·비대칭 y 대응")
a = ap.parse_args()
CELL = a.cell
INFLATE = tv.DEFAULTS["inflate"]

M = json.load(open(f"{a.terrain}/heightmap_meta.json"))
PAD = 3.0

if a.farm:
    # ── 실사 농장 기하 (farm.json 이 유일한 출처) ───────────────────────────
    F = json.load(open(a.farm))
    R = int(F["rows"])
    S = float(F["row_spacing_m"])               # 대표 간격(스칼라) — 위상 모형용
    TS = float(F["tree_spacing_m"])
    HL = float(F["headland_m"])
    ROW_X = np.array([float(p[0]) for p in F["row_origins"]], float)
    X0 = float(ROW_X[0])                        # 위상 원점 = 열 0 의 x
    # 열별 캐노피 구간 — gen_world.farm_row_canopy 와 **같은 규약**이어야
    # 번들의 나무가 월드의 나무와 같은 자리에 선다.
    _y0 = np.array([float(p[1]) for p in F["row_origins"]], float)
    _yl = np.array([float(v) for v in F["row_lengths_m"]], float)
    CAN_Y0 = _y0 + HL                           # 캐노피 시작 (y 최소 쪽 = 지리적 북)
    CAN_Y1 = _y0 + _yl - HL                     # 캐노피 끝   (y 최대 쪽 = 지리적 남)
    ALLEY_X = (ROW_X[:-1] + ROW_X[1:]) / 2.0    # 통로 중심 x (실측, 불균일)
    # 통로 y 구간 — 이웃 두 열 중 **짧은 쪽**(안쪽)이 통로의 실질 끝이다
    # (control.launch.py `_farm_geom` · gen_world.farm_alley_spawn 과 같은 규약).
    # 순서는 robomw site_geom v2 계약 그대로 [남단(y 최대), 북단(y 최소)].
    ALLEY_S = np.minimum(CAN_Y1[:-1], CAN_Y1[1:])
    ALLEY_N = np.maximum(CAN_Y0[:-1], CAN_Y0[1:])
    # col_len 은 로컬라이저에서 `|y| <= col_len/2` 라는 **원점 대칭** 판정으로만
    # 쓰인다(rowlocalize.estimate 의 in_block·at_row_end, map_localizer 의 슬립
    # 횡단 게이트). 이 농장의 블록은 y=0 대칭이 아니므로 그 어휘로는 구간을
    # 정확히 못 적는다 — 전 캐노피를 덮는 최소값을 쓴다. 결과적으로 이 월드에서
    # '나무 구역 밖'(선회 구역) 억제는 사실상 비활성이 된다. 대안(대표 열
    # 길이 141 m)은 반대로 통로 한복판(y<-71)을 선회 구역으로 오분류해 보정을
    # 막는다 — 그쪽이 훨씬 나쁘다. 리포트에 기록한다.
    COL_LEN = 2.0 * float(max(abs(CAN_Y0.min()), abs(CAN_Y1.max())))
    Y_LO, Y_HI = float(CAN_Y0.min()), float(CAN_Y1.max())
    BOUNDS = (float(ROW_X.min()) - PAD, Y_LO - HL - PAD,
              float(ROW_X.max()) + PAD, Y_HI + HL + PAD)
    GEOM_SRC = f"farm.json {a.farm}"
else:
    S, X0, R = M["row_spacing"], M["x0"], M["rows"]
    L, HL = M["row_length"], M["headland"]
    TS = M.get("tree_spacing", 1.5)
    COL_LEN = L
    ROW_X = np.array([X0 + k * S for k in range(R)], float)
    ALLEY_X = np.array([X0 + (k + 0.5) * S for k in range(R - 1)], float)
    Y_LO, Y_HI = -L / 2.0, L / 2.0
    BOUNDS = (X0 - PAD, -(L / 2 + HL + PAD), X0 + (R - 1) * S + PAD, L / 2 + HL + PAD)
    GEOM_SRC = f"지형 메타 {a.terrain}"

print("맵 번들 생성")
print("=" * 78)
print(f"   기하: {GEOM_SRC} · 열 {R} · 통로 {R - 1} · 대표 간격 {S:.4f} m")
if a.farm:
    _sp = np.diff(ROW_X)
    print(f"         간격 실측 {_sp.min():.2f}~{_sp.max():.2f} m (중앙값 {np.median(_sp):.2f})"
          f" · 캐노피 y {Y_LO:.1f}~{Y_HI:.1f} · col_len {COL_LEN:.1f}")

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
    if a.farm:
        # 열마다 캐노피 구간이 다르다 — 주간 위상만 **전 열 공통 격자**(y = i·TS)에
        # 스냅한다(gen_world.build_farm_trees 와 같은 규약. 로컬라이저의 종위상
        # 추정이 공통 위상을 전제한다). 지터·결주는 넣지 않는다: 번들은 참값
        # 기하 기준이고, 그래야 재생성이 결정적이다.
        trees = []
        for r in range(R):
            ys = math.ceil(CAN_Y0[r] / TS) * TS
            nt = int(math.floor((CAN_Y1[r] - ys) / TS)) + 1
            for t in range(nt):
                trees.append((float(ROW_X[r]), ys + t * TS))
        tree_xy = np.asarray(trees, float)
    else:
        n_tree = M.get("trees_per_row", 41)
        tree_y = (np.arange(n_tree) - (n_tree - 1) / 2) * TS
        tree_xy = np.asarray([(tx, ty) for tx in ROW_X for ty in tree_y], float)
    trunk = np.zeros_like(obst)
    for tx, ty in tree_xy:
        trunk |= ((GX - tx) ** 2 + (GY - ty) ** 2) <= 0.35 ** 2
    obst = obst | trunk
    if a.farm:
        # 열을 **연속 띠로 밀봉**한다. 계단식 월드에서 통로 옆면을 곧게 세운
        # 것은 나무가 아니라 계단 둑이었다 — 평탄 월드에는 그 둑이 없어서
        # 줄기 원판만 찍으면 통로 경계가 1.5 m 마다 0.6 m 씩 물결친다. 그
        # 물결의 중심축은 직선이 아니라 지그재그라 세선화가 통로 하나를
        # 100개 넘는 조각으로 쪼갠다(실측: 엣지 3,000개·p90 길이 1.1 m,
        # 통로 중심선 0개). 물리적으로도 밀봉이 옳다 — gen_world 가 열마다
        # 지주와 3단 와이어를 세우므로(build_farm_row_details) 나무 사이는
        # 트렐리스로 막혀 있다. 센서 경로가 rowmod.seal_rows 로 하는 일을
        # 참값 경로에서는 기하로 한다.
        for r in range(R):
            m_ = tree_xy[:, 0] == ROW_X[r]
            if not m_.any():
                continue
            ys_, ye_ = tree_xy[m_, 1].min(), tree_xy[m_, 1].max()
            obst |= ((np.abs(GX - ROW_X[r]) <= 0.35)
                     & (GY >= ys_ - 0.35) & (GY <= ye_ + 0.35))
    # 정합 기준 점군 — 줄기를 수직으로 세운 것 (NDT 가 열을 잡을 수 있게)
    zs = np.arange(0.3, 1.9, 0.15)
    cloud_pts = np.asarray([[tx, ty, z] for tx, ty in tree_xy for z in zs],
                           dtype=np.float32)
    print(f"   나무 {len(tree_xy):,}그루 → 정합 점군 {len(cloud_pts):,} 점")

free = tv.drivable_mask(~obst, cell=CELL, inflate=INFLATE, keep_largest=True)
print(f"   장애물 {int(obst.sum()):,} 셀 · 주행가능 {free.mean():.1%}")

# ── 3. 통로 중심선·그래프 ───────────────────────────────────────────────────
dist = co.distance_field(free, cell=CELL)
sk = co.skeleton(free, min_branch_px=tv.DEFAULTS["min_corridor_px"])
segs = co.trace_segments(sk)
graph = co.RouteGraph(g, segs, dist)
alleys = co.alley_polylines(graph, min_length=10.0)


def column_runs(graph, alley_x, y_lo, y_hi, min_len=10.0):
    """모든 그래프 엣지에서 나무 열 구간의 연속 런을 뽑는다.

    선회 평지 패드(08-10)가 통로 쌍을 물리적으로 이어 자유공간이 사다리가
    아니라 뱀형이 됐다 — 스켈레톤 엣지 하나가 U-호 너머 통로 두 개에 걸치고,
    그런 엣지는 끝점 변위 기준(along_row)으로는 통로로 분류되지 않는다.
    분류 기준을 '엣지의 전체 모양'에서 '수목 구간 안의 직선 런'으로 바꾼다.

    구간 판정은 **가장 가까운 통로의 y 구간**으로 한다. 계단식은 전 통로가
    같은 [−L/2, +L/2] 라 종전 `|y| <= half_len` 과 정확히 동치다. 실사 농장은
    통로마다 구간이 다르고(열별 길이 128~141 m, 블록이 y=0 비대칭) 전 통로의
    합집합을 한 띠로 쓰면 **어떤 통로의 헤드랜드가 다른 통로의 열 구간 y** 에
    들어와, 나무 없는 개활지의 중심선 잡음이 통째로 통로 후보가 된다
    (실측: 그렇게 하면 세그먼트 3,000개 중 통로가 하나도 안 남는다).
    """
    ax = np.asarray(alley_x, float)
    ylo = np.asarray(y_lo, float) * np.ones_like(ax)
    yhi = np.asarray(y_hi, float) * np.ones_like(ax)
    runs = []
    for e in graph.edges:
        pts = e["pts"]
        if pts[0, 1] > pts[-1, 1]:
            pts = pts[::-1]
        kk = np.abs(pts[:, 0][:, None] - ax[None, :]).argmin(axis=1)
        m = (pts[:, 1] >= ylo[kk]) & (pts[:, 1] <= yhi[kk])
        i, n = 0, len(pts)
        while i < n:
            if not m[i]:
                i += 1
                continue
            j = i
            while j < n and m[j]:
                j += 1
            seg = pts[i:j]
            ln = float(np.linalg.norm(np.diff(seg, axis=0), axis=1).sum())
            if ln >= min_len:
                runs.append(dict(pts=seg, width=e["width"], length=ln))
            i = j
    return runs


def flanked(al, obst_mask, grid, cell, lo, hi, need=0.80):
    """양옆이 나무 열로 둘러싸인 통로만 '작업 통로'로 친다.

    바깥 열 너머의 여백 띠도 길고 곧아서 중심선이 잡히지만, 그건 밭이 아니라
    가장자리다. 기대 개수를 하드코딩하는 대신 기하로 판정한다 — 열 간격이
    바뀌어도, 밭 모양이 달라져도 그대로 성립한다.

    lo·hi 는 '옆 열이 있어야 할 거리 범위'다. 계단식은 균일 간격이라
    (0.30S, 0.80S) 한 쌍이면 되지만, 실사 농장은 통로 폭이 4.75~15.25 m 라
    한 쌍으로는 농로(통로 20, 반폭 7.6 m)를 놓친다 — 호출측이 실측 간격의
    최솟값·최댓값에서 만든다.
    """
    pts = al["pts"]
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


if a.farm:
    runs = column_runs(graph, ALLEY_X, ALLEY_N, ALLEY_S)
    _sp = np.diff(ROW_X)
    FLO, FHI = 0.30 * float(_sp.min()), 0.80 * float(_sp.max())
else:
    runs = column_runs(graph, ALLEY_X, Y_LO, Y_HI)
    FLO, FHI = 0.30 * S, 0.80 * S
kept = [r for r in runs if flanked(r, obst, g, CELL, FLO, FHI)]
dropped = len(runs) - len(kept)
# 같은 통로에 걸린 런들을 병합 (분기점이 통로 중간을 끊을 수 있다)
by_k = {}
for r in kept:
    xm = float(np.mean(r["pts"][:, 0]))
    # 통로 번호는 **가장 가까운 실측 통로 중심**으로 매긴다. 균일 격자 역산
    # (round((x-x0)/S - 0.5))은 불균일 농장에서 최대 8.6 m 어긋난다.
    k = int(np.abs(ALLEY_X - xm).argmin())
    by_k.setdefault(k, []).append(r)
alleys = []
for k in sorted(by_k):
    pts = np.concatenate([p["pts"] for p in by_k[k]])
    pts = pts[np.argsort(pts[:, 1])]
    alleys.append(dict(
        pts=pts,
        width=float(np.median([p["width"] for p in by_k[k]])),
        length=float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())))
print(f"   세그먼트 {len(segs)} · 노드 {len(graph.nodes)} · 통로 중심선 {len(alleys)}개"
      f" (기대 {R - 1})" + (f" · 가장자리 띠 {dropped}개 제외" if dropped else ""))

if len(alleys) != R - 1:
    print(f"\n✗ 통로 개수가 기대와 다르다 ({len(alleys)} ≠ {R-1}) — 번들을 만들지 않는다")
    sys.exit(1)

# 중심선 정확도 — 통로 중앙에서 얼마나 벗어났나
errs = []
for al in alleys:
    xs = al["pts"][:, 0]
    errs.append(xs - ALLEY_X[np.abs(xs[:, None] - ALLEY_X).argmin(axis=1)])
err = np.concatenate(errs)
rmse_mm = float(np.sqrt((err ** 2).mean()) * 1000)
print(f"   중심선 오차 RMSE {rmse_mm:.0f} mm · 최대 {np.abs(err).max()*1000:.0f} mm")

# ── 4. 저장 ─────────────────────────────────────────────────────────────────
geom = dict(rows=R, alleys=R - 1, row_spacing=S, x0=X0,
            col_len=COL_LEN, headland=HL, tree_spacing=TS)
if a.farm:
    # 불균일 기하를 번들에 **명시**한다 — robomw site_geom v2 와 같은 계약
    # (alley_centers_x · row_span_y = [[남단, 북단], …], 남단은 y 최대 쪽).
    # 지금의 map_localizer 는 스칼라(row_spacing·x0·col_len)만 소비하지만,
    # 그 스칼라가 이 농장에서 무엇을 근사하고 있는지 되읽을 수 있어야 한다.
    geom["alley_centers_x"] = [round(float(v), 4) for v in ALLEY_X]
    geom["row_span_y"] = [[round(float(s_), 4), round(float(n_), 4)]
                          for s_, n_ in zip(ALLEY_S, ALLEY_N)]
    geom["farm"] = a.farm
meta = mapbundle.save(
    a.out, cloud=cloud_pts, trav=free, origin=(g.xmin, g.ymin), cell=CELL,
    alleys=alleys, geom=geom,
    source=src + (f" + {GEOM_SRC}" if a.farm else ""),
    notes=f"중심선 RMSE {rmse_mm:.0f} mm")

print(f"\n── 번들 저장 → {a.out} ──")
print(f"   해시 {meta['hash']} · 점군 {meta['n_points']:,} · 통로 {meta['n_alleys']}")

# ── 5. 되읽어 검증 ──────────────────────────────────────────────────────────
b = mapbundle.Bundle(a.out)
# 통로 중앙·열 위를 물어볼 y — 계단식은 0(블록 중앙), 실사는 통로마다 다른
# 캐노피 구간의 중점을 쓴다(y=0 은 어떤 열에서는 캐노피 밖이다).
if a.farm:
    mid_alley = [(float(ALLEY_S[k]) + float(ALLEY_N[k])) / 2.0 for k in range(R - 1)]
    mid_row = [(float(CAN_Y0[r]) + float(CAN_Y1[r])) / 2.0 for r in range(R)]
else:
    mid_alley = [0.0] * (R - 1)
    mid_row = [0.0] * R
ok = []
ok.append(("해시 일치", b.verify()))
ok.append(("통로 수", b.alley_count() == R - 1))
ok.append(("통로 중앙이 주행가능", all(
    b.is_drivable(ALLEY_X[k], mid_alley[k]) for k in range(R - 1))))
ok.append(("나무 열 위는 주행불가", not any(
    b.is_drivable(ROW_X[r], mid_row[r]) for r in range(R))))
ok.append(("밭 밖은 주행불가", not b.is_drivable(float(ROW_X[0]) - 50.0, mid_row[0])))
print("\n── 되읽기 검증 ──")
for name, v in ok:
    print(f"   {'✔' if v else '✗'} {name}")
print(f"\n{sum(v for _, v in ok)}/{len(ok)} 통과")
sys.exit(0 if all(v for _, v in ok) else 1)
