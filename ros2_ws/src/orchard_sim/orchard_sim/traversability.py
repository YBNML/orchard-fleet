"""
2.5D 고도격자 → 주행가능 맵 → 통로 중심선 → 경로 그래프

설계 원칙 (사용자 요구): **통로 간 횡단 금지를 규칙으로 넣지 않는다.**
주행 가능 여부는 지형에서 계산되고, 그 결과 자유공간의 위상이 사다리(빗) 모양이
되어 통로들이 오직 양끝 선회 구간에서만 연결된다. 그러면 어떤 그래프 탐색을 써도
통로 간 이동은 저절로 선회 구간을 경유한다 — 경유점을 손으로 넣을 필요가 없다.

입력은 고도격자 하나뿐이라, 참값 지형(해석적)과 센싱한 점군(DEM) 양쪽에
**같은 코드**를 돌릴 수 있다. 그래야 "알고리즘 문제"와 "맵 품질 문제"가 분리된다.

핵심 판정 (Scout Mini 기준):
    지상고 0.115 m, 외접원 지름 0.834 m, 등판 30°
    단차 임계 0.15 m — 둑은 0.22~0.41 m(불가), 램프는 0.10 m(가능), 통로는 0.02 m
    경사 임계 0.25 (14°) — 횡방향 안정성 기준. 등판 한계보다 보수적으로 둔다
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

# ── Scout Mini 제원에서 유도한 판정 임계 ────────────────────────────────────
GROUND_CLEARANCE = 0.115
FOOTPRINT_DIAG = 0.834

DEFAULTS = dict(
    cell=0.10,              # 격자 [m]
    step_win=7,             # 단차 평가 창 (7×0.10 = 0.7 m ≈ 로봇 발자국)
    step_max=0.15,          # 창 내 최대 고저차 [m]
    slope_max=0.25,         # 최대 경사 (14°)
    rough_max=0.05,         # 거칠기(국소 표준편차) [m]
    obstacle_min_agl=0.30,  # 벽 판정 하한 (지역 지면 기준) [m]
    obstacle_max_agl=1.30,  # 벽 판정 상한 [m]
    clutter_min_agl=0.12,   # 잡초·낙과 대역 하한 [m]
    inflate=0.42,           # 로봇 반경만큼 팽창 [m]
    min_corridor_px=8,      # 스켈레톤 가지치기 최소 길이 [셀]
)


class Grid:
    """월드 좌표 ↔ 격자 인덱스. 모든 레이어가 이 격자를 공유한다."""

    def __init__(self, xmin, ymin, cell, shape):
        self.xmin, self.ymin, self.cell = float(xmin), float(ymin), float(cell)
        self.shape = shape                      # (rows=y, cols=x)

    def to_idx(self, x, y):
        c = np.floor((np.asarray(x) - self.xmin) / self.cell).astype(int)
        r = np.floor((np.asarray(y) - self.ymin) / self.cell).astype(int)
        return r, c

    def to_world(self, r, c):
        x = self.xmin + (np.asarray(c) + 0.5) * self.cell
        y = self.ymin + (np.asarray(r) + 0.5) * self.cell
        return x, y

    def inside(self, r, c):
        return (r >= 0) & (r < self.shape[0]) & (c >= 0) & (c < self.shape[1])


# ═══════════════════════════════════════════════════════════════════════════
# 1. 고도격자
# ═══════════════════════════════════════════════════════════════════════════
def dem_from_points(points, cell=0.10, percentile=20.0, min_count=5,
                    gate=0.15, bounds=None):
    """점군 → 2.5D 지면 고도격자.

    셀별 z 의 저분위수를 지면 추정으로 쓴다. 최솟값은 거리 잡음을 물고,
    25분위(CMU terrain_analysis 기본)는 360° 라이다 기준이라 70.4° 쐐기로
    비스듬히 훑는 우리 조건에서는 지면 표본이 부족하다. 20분위가 절충이다.

    그다음 **높이 게이트 3×3 중앙값**을 건다 — 이게 계단식 대응의 핵심이다.
    이웃과의 차이가 gate 이하일 때만 평활에 참여시킨다:
      · 8.1° 램프는 0.3 m 창에서 0.043 m 변화 → 통과 (제대로 평활됨)
      · 58% 둑은 0.174 m 변화 → 배제 (둑이 가짜 램프로 뭉개지지 않는다)
    게이트가 없으면 둑이 완만하게 번져서 주행 가능한 것처럼 보인다.
    """
    P = np.asarray(points, np.float32)
    if bounds is None:
        xmin, ymin = P[:, 0].min() - cell, P[:, 1].min() - cell
        xmax, ymax = P[:, 0].max() + cell, P[:, 1].max() + cell
    else:
        xmin, ymin, xmax, ymax = bounds
    nx = int(np.ceil((xmax - xmin) / cell))
    ny = int(np.ceil((ymax - ymin) / cell))
    g = Grid(xmin, ymin, cell, (ny, nx))

    r, c = g.to_idx(P[:, 0], P[:, 1])
    ok = g.inside(r, c)
    r, c, z = r[ok], c[ok], P[ok, 2]
    flat = r * nx + c

    order = np.lexsort((z, flat))
    flat_s, z_s = flat[order], z[order]
    uniq, start, cnt = np.unique(flat_s, return_index=True, return_counts=True)

    ground = np.full(ny * nx, np.nan, np.float32)
    count = np.zeros(ny * nx, np.int32)
    relief = np.full(ny * nx, np.nan, np.float32)
    # 분위수는 정렬된 구간에서 인덱스로 바로 뽑는다 (그룹별 루프 없이)
    q_idx = start + np.minimum((cnt * percentile / 100.0).astype(int), cnt - 1)
    p10 = start + np.minimum((cnt * 0.10).astype(int), cnt - 1)
    p90 = start + np.minimum((cnt * 0.90).astype(int), cnt - 1)
    ground[uniq] = z_s[q_idx]
    relief[uniq] = z_s[p90] - z_s[p10]
    count[uniq] = cnt

    ground = ground.reshape(ny, nx)
    relief = relief.reshape(ny, nx)
    count = count.reshape(ny, nx)
    ground[count < min_count] = np.nan

    return g, _gated_median(ground, gate), relief, count


def _gated_median(ground, gate):
    """높이 게이트 3×3 중앙값. 게이트 밖 이웃은 평활에 넣지 않는다."""
    out = ground.copy()
    valid = np.isfinite(ground)
    filled = np.where(valid, ground, 0.0)
    stack, wstack = [], []
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            stack.append(np.roll(np.roll(filled, dr, 0), dc, 1))
            wstack.append(np.roll(np.roll(valid, dr, 0), dc, 1))
    S = np.stack(stack)                       # (9, ny, nx)
    W = np.stack(wstack)
    near = np.abs(S - ground[None]) <= gate
    use = W & near & np.isfinite(ground)[None]
    S = np.where(use, S, np.nan)
    with np.errstate(all="ignore"):
        med = np.nanmedian(S, axis=0)
    out[valid] = med[valid]
    return out


def dem_from_terrain(hm_path, meta_path, cell=0.10, bounds=None):
    """참값 지형(하이트맵)에서 같은 형식의 고도격자를 만든다.

    센싱한 맵과 **같은 다운스트림 코드**를 돌리기 위한 것이다.
    알고리즘 문제와 맵 품질 문제를 분리하려면 이 경로가 반드시 필요하다.
    """
    import json
    H = np.load(hm_path)
    m = json.load(open(meta_path))
    N = H.shape[0]
    half, E = m["half"], m["size_x"]
    if bounds is None:
        bounds = (-half, -half, half, half)
    xmin, ymin, xmax, ymax = bounds
    nx = int(np.ceil((xmax - xmin) / cell))
    ny = int(np.ceil((ymax - ymin) / cell))
    g = Grid(xmin, ymin, cell, (ny, nx))

    xs = xmin + (np.arange(nx) + 0.5) * cell
    ys = ymin + (np.arange(ny) + 0.5) * cell
    X, Y = np.meshgrid(xs, ys)
    fc = np.clip((X + half) / E * (N - 1), 0, N - 1)
    fr = np.clip((Y + half) / E * (N - 1), 0, N - 1)
    c0, r0 = fc.astype(int), fr.astype(int)
    c1, r1 = np.minimum(c0 + 1, N - 1), np.minimum(r0 + 1, N - 1)
    dc, dr = fc - c0, fr - r0
    top = H[r0, c0] * (1 - dc) + H[r0, c1] * dc
    bot = H[r1, c0] * (1 - dc) + H[r1, c1] * dc
    ground = (top * (1 - dr) + bot * dr).astype(np.float32)
    return g, ground


# ═══════════════════════════════════════════════════════════════════════════
# 2. 주행가능 판정
# ═══════════════════════════════════════════════════════════════════════════
def traversability(ground, cell=0.10, step_win=7, step_max=0.15,
                   slope_max=0.25, rough_max=0.05):
    """고도격자 → (주행가능 불리언, 지표 dict).

    **셀 대 셀 차이가 아니라 창(window) 최대-최소를 쓴다.** 0.10 m 셀 하나에
    58% 둑은 5.8 cm 밖에 안 담겨 어떤 임계값으로도 못 잡는다. 0.7 m 창에서는
    같은 둑이 0.41 m 가 되어 통로(0.02 m)와 11배 차이가 난다.
    """
    valid = np.isfinite(ground)
    filled = np.where(valid, ground, np.nanmedian(ground[valid]) if valid.any() else 0.0)

    step = (ndimage.maximum_filter(filled, step_win)
            - ndimage.minimum_filter(filled, step_win))
    gy, gx = np.gradient(filled, cell)
    slope = np.hypot(gx, gy)
    mean = ndimage.uniform_filter(filled, step_win)
    rough = np.sqrt(np.maximum(
        ndimage.uniform_filter((filled - mean) ** 2, step_win), 0.0))
    # 음형 장애물: 이웃보다 내가 높으면 곧 낭떠러지 가장자리다.
    # "z 위는 장애물" 규칙이 절대 못 잡는 것 — 계단식에서 옆 테라스가 26~50 cm 낮다.
    drop = filled - ndimage.minimum_filter(filled, step_win)

    trav = valid & (step <= step_max) & (slope <= slope_max) & (rough <= rough_max)
    return trav, dict(step=step, slope=slope, rough=rough, drop=drop, valid=valid)


def obstacle_layers(points, g, ground, min_agl=0.30, max_agl=1.30,
                    clutter_min=0.12, min_count=3):
    """지역 지면 기준 높이(AGL)로 벽 / 잡동사니를 분리한다.

    전역 z 임계가 아니라 **지역 지면 기준**이라는 점이 핵심이다. 계단식에서는
    통로마다 지면이 26~50 cm 다르므로 전역 임계로는 어느 통로에서든 틀린다.
    """
    P = np.asarray(points, np.float32)
    r, c = g.to_idx(P[:, 0], P[:, 1])
    ok = g.inside(r, c)
    r, c, z = r[ok], c[ok], P[ok, 2]
    gz = ground[r, c]
    fin = np.isfinite(gz)
    r, c, agl = r[fin], c[fin], z[fin] - gz[fin]

    def count_band(lo, hi):
        m = (agl >= lo) & (agl < hi)
        out = np.zeros(g.shape, np.int32)
        np.add.at(out, (r[m], c[m]), 1)
        return out

    wall = count_band(min_agl, max_agl) >= min_count
    clutter = count_band(clutter_min, min_agl) >= min_count
    return wall, clutter


def drivable_mask(trav, wall=None, cell=0.10, inflate=0.42, keep_largest=True):
    """주행가능 + 벽 제거 + 로봇 반경 팽창 → 최종 자유공간.

    팽창은 로봇 외접원 반경(0.417 m)만큼 준다. 이 한 줄이 "통로 간 횡단 금지"를
    강제하는 실질적 장치다 — 둑이 이미 불가로 찍혔고 팽창까지 되면 통로들은
    선회 구간을 통하지 않고는 연결되지 않는다.
    """
    free = trav.copy()
    if wall is not None:
        free &= ~wall
    rad = max(int(round(inflate / cell)), 1)
    free = ndimage.binary_erosion(free, _disk(rad), border_value=0)
    if keep_largest and free.any():
        lab, n = ndimage.label(free)
        if n > 1:
            sizes = ndimage.sum(free, lab, range(1, n + 1))
            free = lab == (int(np.argmax(sizes)) + 1)
    return free


def _disk(rad):
    y, x = np.ogrid[-rad:rad + 1, -rad:rad + 1]
    return x * x + y * y <= rad * rad
