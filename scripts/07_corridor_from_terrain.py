#!/usr/bin/env python3
"""
1단계 검증 — 참값 지형에서 주행가능 맵·중심선·경로 그래프가 나오는가

    python3 scripts/07_corridor_from_terrain.py

목적: 알고리즘 자체를 먼저 검증한다. 입력이 완벽한 지형이므로 여기서 실패하면
그것은 맵 품질 문제가 아니라 **알고리즘 문제**다. 3단계에서 센싱한 맵에
같은 코드를 돌려 이 결과와 비교하면, 그 차이가 곧 "맵에서 통로 찾기"의 난이도다.

핵심 확인:
  · 단차 둑이 주행 불가로 찍히는가 (규칙이 아니라 지형 계산으로)
  · 자유공간 위상이 사다리(빗) 모양인가 — 통로가 선회 구간에서만 연결
  · **통로 0 → 통로 5 최단경로가 선회 구간을 경유하는가** (경유점 없이)
  · 중심선이 참값 통로 중앙과 얼마나 일치하는가
"""
import json
import sys

import numpy as np

sys.path.insert(0, "ros2_ws/src/orchard_sim")
from orchard_sim import corridors as co  # noqa: E402
from orchard_sim import traversability as tv  # noqa: E402

BASE = "sim/models/orchard_terrain"
M = json.load(open(f"{BASE}/heightmap_meta.json"))
S, X0, R = M["row_spacing"], M["x0"], M["rows"]
L, HL = M["row_length"], M["headland"]

# 과수원 블록 + 선회 구간 여유만 본다 (지형 전체 120 m 는 불필요)
PAD = 3.0
BOUNDS = (X0 - PAD, -(L / 2 + HL + PAD), X0 + (R - 1) * S + PAD, L / 2 + HL + PAD)
CELL = 0.10

print(f"과수원 {R}열 × {M['trees_per_row']}주 · 열간 {S} m · 열길이 {L} m · 선회 {HL} m")
print(f"해석 영역 x[{BOUNDS[0]:.1f}, {BOUNDS[2]:.1f}]  y[{BOUNDS[1]:.1f}, {BOUNDS[3]:.1f}]  셀 {CELL} m\n")

fails = []

# ── 1. 고도격자 + 주행가능 판정 ─────────────────────────────────────────────
g, ground = tv.dem_from_terrain(f"{BASE}/heightmap.npy", f"{BASE}/heightmap_meta.json",
                                cell=CELL, bounds=BOUNDS)
trav, feat = tv.traversability(ground, cell=CELL, step_win=tv.DEFAULTS["step_win"],
                               step_max=tv.DEFAULTS["step_max"],
                               slope_max=tv.DEFAULTS["slope_max"],
                               rough_max=tv.DEFAULTS["rough_max"])
print("── 1. 주행가능 판정 (지형 계산만, 규칙 없음) ──")
print(f"   격자 {g.shape[0]}×{g.shape[1]}  주행가능 {trav.mean():.1%}")

# 통로 중앙 / 단차 둑 위에서 각각 어떻게 찍혔는지
def sample(x, y):
    r, c = g.to_idx(x, y)
    if not g.inside(r, c):
        return None
    return dict(trav=bool(trav[r, c]), step=float(feat["step"][r, c]),
                slope=float(feat["slope"][r, c]))

print(f"\n   {'위치':<22} {'단차[m]':>9} {'경사':>8}  판정")
print("   " + "─" * 54)
alley_ok, bank_ok = 0, 0
for k in range(R - 1):
    s = sample(X0 + (k + 0.5) * S, 0.0)
    if s:
        alley_ok += s["trav"]
        if k < 3:
            print(f"   통로 {k} 중앙 (y=0)      {s['step']:9.3f} {s['slope']:8.1%}  "
                  f"{'주행가능' if s['trav'] else '불가'}")
for r_ in range(1, R - 1):
    s = sample(X0 + r_ * S, 0.0)
    if s:
        bank_ok += (not s["trav"])
        if r_ < 4:
            print(f"   수목열 {r_} 둑 (y=0)     {s['step']:9.3f} {s['slope']:8.1%}  "
                  f"{'주행가능' if s['trav'] else '불가'}")
print(f"\n   통로 중앙 {alley_ok}/{R-1} 주행가능,  단차 둑 {bank_ok}/{R-2} 주행불가")
if alley_ok < R - 1:
    fails.append(f"통로 중앙 {R-1-alley_ok}개가 주행불가로 잘못 찍힘")
if bank_ok < R - 2:
    fails.append(f"단차 둑 {R-2-bank_ok}개가 주행가능으로 잘못 찍힘 — 횡단이 열려버린다")

# ── 2. 자유공간 + 위상 ──────────────────────────────────────────────────────
free = tv.drivable_mask(trav, cell=CELL, inflate=tv.DEFAULTS["inflate"])
dist = co.distance_field(free, CELL)
print(f"\n── 2. 자유공간 (로봇 반경 {tv.DEFAULTS['inflate']} m 팽창 후) ──")
print(f"   자유 셀 {free.mean():.1%}  최대 여유폭 {dist.max()*2:.2f} m")
if not free.any():
    fails.append("자유공간이 비었다")
    print("\n✗ 실패:"); [print("   ·", f) for f in fails]; sys.exit(1)

# ── 3. 스켈레톤 → 경로 그래프 ───────────────────────────────────────────────
sk = co.skeleton(free, min_branch_px=tv.DEFAULTS["min_corridor_px"])
segs = co.trace_segments(sk)
graph = co.RouteGraph(g, segs, dist)
alleys = co.alley_polylines(graph, min_length=10.0)
print(f"\n── 3. 경로 그래프 ──")
print(f"   스켈레톤 {sk.sum():,} 셀 · 세그먼트 {len(segs)} · 노드 {len(graph.nodes)}")
print(f"   통로 엣지 {len(graph.alleys())} · 선회 엣지 "
      f"{len(graph.edges)-len(graph.alleys())}")
print(f"   길이 10 m 이상 통로 중심선 {len(alleys)}개 (기대 {R-1})")
if len(alleys) < R - 2:
    fails.append(f"통로 중심선 {len(alleys)}개만 추출됨 (기대 {R-1})")

# ── 4. 중심선 정확도 (참값 통로 중앙 대비) ─────────────────────────────────
print(f"\n── 4. 중심선 정확도 ──")
print(f"   {'통로':>5} {'참값 x':>9} {'추출 x':>9} {'횡오차 RMSE':>12} {'최대':>8} {'폭':>7} {'길이':>8}")
print("   " + "─" * 62)
errs_all = []
outside = []
for a in alleys:
    xt = a["x_mean"]
    k = int(round((xt - X0) / S - 0.5))
    true_x = X0 + (k + 0.5) * S
    if not (0 <= k <= R - 2):
        # 과수원 블록 밖에도 주행 가능한 띠가 있다. 알고리즘이 틀린 게 아니라
        # 실제로 그곳이 평탄해서 나온 것이므로, 평가에서 빼고 따로 보고한다.
        outside.append((k, xt, a["length"]))
        continue
    # 나무 구역 안쪽만 평가 (선회 구간은 중심선 개념이 다르다)
    m = np.abs(a["pts"][:, 1]) <= L / 2
    if m.sum() < 5:
        continue
    e = a["pts"][m, 0] - true_x
    rmse = float(np.sqrt((e ** 2).mean()))
    errs_all.append(e)
    print(f"   {k:>5} {true_x:>9.2f} {xt:>9.2f} {rmse*1000:>9.1f} mm "
          f"{np.abs(e).max()*1000:>6.0f} mm {a['width']:>6.2f} {a['length']:>7.1f}")
if errs_all:
    E = np.concatenate(errs_all)
    rmse_all = float(np.sqrt((E ** 2).mean()))
    print(f"\n   전체 횡오차 RMSE {rmse_all*1000:.1f} mm · 최대 {np.abs(E).max()*1000:.0f} mm")
    if rmse_all > 0.15:
        fails.append(f"중심선 횡오차 RMSE {rmse_all*1000:.0f} mm 가 과다")
if outside:
    print(f"\n   [참고] 과수원 블록 밖 주행가능 띠 {len(outside)}개: "
          + ", ".join(f"x={x:.1f}({l:.0f} m)" for _, x, l in outside))
    print(f"   알고리즘 오류가 아니라 그곳 지형이 실제로 평탄해서 나온 것이다.")

# ── 5. 핵심 — 통로 간 이동이 규칙 없이 선회 구간을 경유하는가 ────────────
print(f"\n── 5. 통로 간 이동 (규칙 없이 위상에서 나오는가) ──")

# (a) 나무 구역 안에서 통로를 가로지르는 엣지가 하나라도 있으면 안 된다.
#     있다면 단차 둑이 안 막힌 것이고, 그러면 플래너가 둑을 타넘으려 든다.
inside_cross = []
for e in graph.edges:
    p = e["pts"]
    m = np.abs(p[:, 1]) <= L / 2 - 1.0
    if m.sum() < 2:
        continue
    dx = float(p[m, 0].max() - p[m, 0].min())
    if dx > S * 0.6:          # 열간의 60% 넘게 x 로 이동 = 둑을 넘었다는 뜻
        inside_cross.append((dx, float(np.abs(p[m, 1]).max())))
print(f"   나무 구역 안에서 통로를 가로지르는 엣지: {len(inside_cross)}개")
if inside_cross:
    fails.append(f"나무 구역 안 횡단 엣지 {len(inside_cross)}개 — 단차 둑이 안 막혔다")
    for dx, ay in inside_cross[:3]:
        print(f"     x 이동 {dx:.1f} m at |y|<={ay:.1f} m  ✘")
else:
    print(f"   ✔ 없음 — 둑이 막혀 통로가 서로 격리됐다")

# (b) 실제 경로: 통로 A 끝 → 통로 B 끝. 두 통로 모두 과수원 블록 안에서 고른다.
inblock = [a for a in alleys
           if 0 <= int(round((a["x_mean"] - X0) / S - 0.5)) <= R - 2]
if len(inblock) >= 2:
    a0, a1 = inblock[0], inblock[min(5, len(inblock) - 1)]
    k0 = int(round((a0["x_mean"] - X0) / S - 0.5))
    k1 = int(round((a1["x_mean"] - X0) / S - 0.5))
    # 각 통로의 남쪽 끝점을 노드로 잡는다 (엣지 끝이 곧 노드다)
    n0 = graph.node_nearest(a0["pts"][0, 0], a0["pts"][0, 1])
    n1 = graph.node_nearest(a1["pts"][0, 0], a1["pts"][0, 1])
    path_e, plen = graph.shortest_path(n0, n1)
    if path_e is None:
        fails.append("통로 간 경로가 없다 — 그래프가 끊겼다")
        print("   ✘ 경로 없음")
    else:
        pts = np.vstack([graph.edges[e]["pts"] for e in path_e])
        max_absy = float(np.abs(pts[:, 1]).max())
        direct = abs(a1["x_mean"] - a0["x_mean"])
        kinds = [graph.edges[e]["kind"] for e in path_e]
        print(f"\n   통로 {k0} 남단 → 통로 {k1} 남단")
        print(f"   경로 길이 {plen:.1f} m  (통로 간 직선 거리 {direct:.1f} m)")
        print(f"   경유 엣지 {len(path_e)}개: {kinds}")
        print(f"   경로가 도달한 최대 |y| = {max_absy:.1f} m  (나무 구역 끝 {L/2:.1f} m)")
        if max_absy > L / 2:
            print(f"   ✔ 선회 구간을 경유한다 — 경유점을 넣지 않았는데도")
        else:
            fails.append("경로가 나무 구역 안에서 횡단했다")
            print("   ✘ 나무 구역 안에서 횡단")

    # (c) 모든 통로 쌍이 서로 도달 가능한가 (그래프가 하나로 연결됐는가)
    reach = 0
    pairs = 0
    for i in range(len(inblock)):
        for j in range(i + 1, len(inblock)):
            ni = graph.node_nearest(inblock[i]["pts"][0, 0], inblock[i]["pts"][0, 1])
            nj = graph.node_nearest(inblock[j]["pts"][0, 0], inblock[j]["pts"][0, 1])
            pe, _ = graph.shortest_path(ni, nj)
            pairs += 1
            reach += pe is not None
    print(f"\n   통로 쌍 도달성 {reach}/{pairs}"
          f"  {'✔ 전부 연결' if reach == pairs else '✘ 일부 고립'}")
    if reach < pairs:
        fails.append(f"통로 쌍 {pairs-reach}개가 서로 도달 불가")

# ── 6. 산출물 저장 ──────────────────────────────────────────────────────────
np.savez_compressed(
    "/tmp/corridor_truth.npz",
    ground=ground, trav=trav, free=free, dist=dist, skeleton=sk,
    xmin=g.xmin, ymin=g.ymin, cell=g.cell,
    alley_x=np.array([a["x_mean"] for a in alleys], np.float32),
    alley_pts=np.array([len(a["pts"]) for a in alleys], np.int32),
)
print(f"\n   → /tmp/corridor_truth.npz (3단계에서 센싱 맵 결과와 비교)")

print()
if fails:
    print("✗ 실패 항목:")
    for f in fails:
        print(f"   · {f}")
    sys.exit(1)
print("✔ 1단계 통과 — 주행가능 맵·중심선·사다리 위상이 지형 계산만으로 나온다")
