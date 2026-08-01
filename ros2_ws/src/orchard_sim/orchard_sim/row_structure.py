"""
열 구조 추정 — 지형에 기대지 않고 통로를 갈라놓기

배경 (2026-07-26 실측, scripts/11_tree_vs_bank.py):
지금까지의 깔끔한 결과는 계단식 단차 둑 덕분이었다. 지형을 평평하게 두고 현실적인
나무(위치 흔들림 ±0.15 m, 수관 반경 편차, 결주)만 놓으면 자유공간이 **한 덩어리**로
이어져 버린다 — 결주 0% 에서도 그렇다. 분기점 46→697, 통로 간 횡단 0→16.

원인은 단순하다. 결주 한 그루가 생기면 이웃 나무 중심 간격이 2배가 되고,
로봇 반경만큼 팽창해도 약 1 m 폭의 구멍이 남는다. 스켈레톤은 그 구멍으로 새어
옆 통로와 고리를 만든다.

그래서 지형 대신 **열 구조 자체**를 쓴다. 과수원은 나무가 열로 심겨 있고, 그
열 방향·간격·위상은 장애물 마스크에서 추정할 수 있다. 추정한 열 방향으로만
선분 구조요소를 써서 닫으면 결주 구멍은 메워지고 통로는 안 붙는다 — 구조요소에
횡방향 폭이 없기 때문이다.

들어가는 가정은 하나뿐이고, 그마저 데이터로 검증한다:
    "장애물이 어떤 방향으로 주기적인 열을 이룬다"
열 구조가 약하면 score 가 낮게 나오고, 그때는 닫기를 건너뛴다 (원 마스크 그대로).
좌표계·간격·방향을 하드코딩하지 않으므로 열 간격이 다르거나 밭이 비스듬해도 된다.
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

try:
    from scipy.signal import find_peaks
    HAVE_SIGNAL = True
except ImportError:                                    # pragma: no cover
    HAVE_SIGNAL = False


# ═══════════════════════════════════════════════════════════════════════════
# 1. 열 방향 · 간격 · 위상 추정
# ═══════════════════════════════════════════════════════════════════════════
def _perp_histogram(X, Y, theta, cell, half_span):
    """열 방향 theta 의 **수직** 축에 투영한 히스토그램.

    열 방향이 맞으면 한 열의 셀들이 같은 칸으로 모여 뾰족한 봉우리가 선다.
    어긋나면 번져서 평평해진다. 이 뾰족함이 곧 목적함수다.
    """
    p = -np.sin(theta) * X + np.cos(theta) * Y
    nb = int(np.ceil(2 * half_span / cell))
    h, edges = np.histogram(p, bins=nb, range=(-half_span, half_span))
    return h.astype(np.float64), edges


def _concentration(h):
    """봉우리가 얼마나 뾰족한가 (역참여비). 균등분포면 낮고, 몰려 있으면 높다."""
    s = h.sum()
    if s <= 0:
        return 0.0
    hn = h / s
    return float((hn ** 2).sum())


def estimate_row_geometry(obst, cell=0.10, theta_step_deg=0.5,
                          spacing_range=(1.2, 12.0), max_samples=40000,
                          seed=0):
    """장애물 마스크에서 열 방향·간격·위상을 추정한다.

    반환 dict:
        theta        열 방향 [rad] (격자 x 축 기준, 0~pi)
        spacing      열 간격 [m]
        offsets      각 열의 수직축 위치 [m] (중심 기준)
        score        최적 방향의 뾰족함
        score_ratio  최악 방향 대비 배수 — 열 구조가 실제로 있는지의 지표
        cx, cy       기준 원점 (격자 미터 좌표)
        hist, edges  최적 방향의 투영 히스토그램
    """
    rs, cs = np.nonzero(obst)
    if rs.size < 50:
        raise ValueError("장애물 셀이 너무 적어 열 구조를 추정할 수 없습니다")

    if rs.size > max_samples:                      # 방향 탐색만 표본으로
        idx = np.random.default_rng(seed).choice(rs.size, max_samples, replace=False)
        rs_s, cs_s = rs[idx], cs[idx]
    else:
        rs_s, cs_s = rs, cs

    X_all, Y_all = cs * cell, rs * cell
    cx, cy = float(X_all.mean()), float(Y_all.mean())
    X, Y = cs_s * cell - cx, rs_s * cell - cy
    half_span = float(np.hypot(X, Y).max()) + cell

    def scan(th_list):
        return np.array([_concentration(_perp_histogram(X, Y, t, cell, half_span)[0])
                         for t in th_list])

    # 성긴 탐색 → 최적 부근만 촘촘히. 전 구간을 촘촘히 훑는 것보다 훨씬 싸고
    # 각도 분해능은 오히려 좋다.
    coarse = np.deg2rad(np.arange(0.0, 180.0, 1.0))
    s_coarse = scan(coarse)
    i0 = int(np.argmax(s_coarse))
    span = np.deg2rad(1.0)
    fine = np.linspace(coarse[i0] - span, coarse[i0] + span,
                       int(2 * span / np.deg2rad(theta_step_deg)) + 1)
    s_fine = scan(fine)

    thetas = np.concatenate([coarse, fine])
    scores = np.concatenate([s_coarse, s_fine])
    i_best = int(np.argmax(s_fine))
    theta = float(fine[i_best]) % np.pi
    score = float(s_fine[i_best])
    score_ratio = score / max(float(s_coarse.min()), 1e-12)

    # 전체 셀로 최종 히스토그램을 다시 만든다 (간격·위상은 정밀해야 하므로)
    Xf, Yf = X_all - cx, Y_all - cy
    half_f = float(np.hypot(Xf, Yf).max()) + cell
    hist, edges = _perp_histogram(Xf, Yf, theta, cell, half_f)
    centers = 0.5 * (edges[:-1] + edges[1:])

    # ── 간격: 자기상관의 첫 유의 봉우리 ────────────────────────────────────
    hc = hist - hist.mean()
    ac = np.correlate(hc, hc, mode="full")[hist.size - 1:]
    ac = ac / max(ac[0], 1e-12)
    lo = max(int(round(spacing_range[0] / cell)), 1)
    hi = min(int(round(spacing_range[1] / cell)), ac.size - 1)
    spacing = float(np.nan)
    if hi > lo:
        spacing = float((lo + int(np.argmax(ac[lo:hi]))) * cell)

    # ── 위상: 히스토그램 봉우리 위치 ───────────────────────────────────────
    offsets = np.array([])
    if HAVE_SIGNAL and np.isfinite(spacing) and spacing > 0:
        dist = max(int(round(0.6 * spacing / cell)), 1)
        pk, _ = find_peaks(hist, distance=dist, height=0.15 * hist.max())
        offsets = centers[pk]

    return dict(theta=theta, spacing=spacing, offsets=offsets, score=score,
                score_ratio=float(score_ratio), cx=cx, cy=cy,
                hist=hist, edges=edges, thetas=thetas, scores=scores)


# ═══════════════════════════════════════════════════════════════════════════
# 2. 열 방향 결손(결주) 길이 추정
# ═══════════════════════════════════════════════════════════════════════════
def row_gap_stats(obst, cell, geom, percentile=99.0, band_frac=0.35,
                  min_row_cells=20):
    """열마다 '열을 따라 얼마나 길게 비어 있나'를 재서 닫기 길이를 정한다.

    닫기 길이를 상수로 박으면 그게 곧 하드코딩이다. 대신 실제로 관측된 결손
    길이 분포의 분위수를 쓴다 — 그 과수원이 실제로 얼마나 비어 있는지에서
    길이가 나온다.
    """
    th, sp = geom["theta"], geom["spacing"]
    if not np.isfinite(sp) or geom["offsets"].size == 0:
        return dict(gaps=np.array([]), length=float("nan"), n_rows=0)

    rs, cs = np.nonzero(obst)
    X, Y = cs * cell - geom["cx"], rs * cell - geom["cy"]
    p = -np.sin(th) * X + np.cos(th) * Y          # 열 가로지르는 좌표
    q = np.cos(th) * X + np.sin(th) * Y           # 열을 따라가는 좌표

    gaps, n_rows = [], 0
    for off in geom["offsets"]:
        m = np.abs(p - off) <= band_frac * sp
        if m.sum() < min_row_cells:
            continue
        n_rows += 1
        qq = np.sort(q[m])
        # cell 격자로 양자화해 중복 제거 → 인접 간 빈틈이 곧 결손
        qb = np.unique(np.round(qq / cell).astype(np.int64)) * cell
        if qb.size < 2:
            continue
        d = np.diff(qb)
        gaps.append(d[d > 1.5 * cell])

    gaps = np.concatenate(gaps) if gaps else np.array([])
    length = float(np.percentile(gaps, percentile)) if gaps.size else 0.0
    return dict(gaps=gaps, length=length, n_rows=n_rows,
                pct={p: float(np.percentile(gaps, p)) for p in (50, 90, 95, 99)}
                if gaps.size else {},
                gmax=float(gaps.max()) if gaps.size else 0.0)


# ═══════════════════════════════════════════════════════════════════════════
# 3. 방향성 닫힘
# ═══════════════════════════════════════════════════════════════════════════
def line_element(theta, length_m, cell):
    """열 방향 선분 구조요소. 횡방향 폭이 0 이라 이웃 열을 붙이지 않는다."""
    n = max(int(round(length_m / cell)), 1)
    half = n // 2
    t = np.arange(-half, half + 1)
    dc = np.rint(t * np.cos(theta)).astype(int)
    dr = np.rint(t * np.sin(theta)).astype(int)
    R = int(max(np.abs(dc).max(), np.abs(dr).max()))
    se = np.zeros((2 * R + 1, 2 * R + 1), bool)
    se[dr + R, dc + R] = True
    return se


def oriented_closing(mask, theta, length_m, cell):
    """열 방향으로만 팽창→침식. 결손은 메우고 열 굵기는 되돌린다."""
    se = line_element(theta, length_m, cell)
    if se.sum() <= 1:
        return mask.copy()
    return ndimage.binary_closing(mask, structure=se, border_value=0)


def _pq_grids(shape, cell, geom):
    """격자 전체의 (열 가로지르는 좌표 p, 열 따라가는 좌표 q)."""
    r = np.arange(shape[0])[:, None] * cell - geom["cy"]
    c = np.arange(shape[1])[None, :] * cell - geom["cx"]
    th = geom["theta"]
    return (-np.sin(th) * c + np.cos(th) * r,
            np.cos(th) * c + np.sin(th) * r)


def row_fragmentation(mask, cell, geom, band_frac=0.35, min_cells=20):
    """열마다 장애물이 몇 조각으로 끊겨 있는가. 1.0 이면 완전히 이어진 열이다.

    참값이 없어도 계산된다 — 추정한 열 위치만 있으면 된다. 그래서 닫기 길이를
    '충분한가'로 자동 판정하는 데 쓸 수 있다.
    """
    if geom["offsets"].size == 0 or not np.isfinite(geom["spacing"]):
        return float("nan"), []
    P, _ = _pq_grids(mask.shape, cell, geom)
    conn = np.ones((3, 3), np.uint8)
    frags = []
    for off in geom["offsets"]:
        sub = mask & (np.abs(P - off) <= band_frac * geom["spacing"])
        if sub.sum() < min_cells:
            continue
        lab, n = ndimage.label(sub, conn)
        if n == 0:
            continue
        sizes = ndimage.sum(sub, lab, range(1, n + 1))
        frags.append(int((sizes >= min_cells).sum()))
    return (float(np.mean(frags)) if frags else float("nan")), frags


# ═══════════════════════════════════════════════════════════════════════════
# 4. 통합 — 자동 밀봉
# ═══════════════════════════════════════════════════════════════════════════
def seal_rows(obst, cell=0.10, min_score_ratio=3.0, gap_percentile=99.0,
              max_length_factor=1.5, geom=None):
    """장애물 마스크의 열 결손을 자동으로 메운다.

    열 구조가 뚜렷하지 않으면(score_ratio 미달) 아무것도 하지 않고 원본을
    돌려준다 — 과수원이 아닌 곳에서 없는 벽을 지어내지 않기 위해서다.

    반환: (밀봉된 마스크, 진단 dict)
    """
    geom = geom or estimate_row_geometry(obst, cell)
    info = dict(geom=geom, applied=False, length=0.0, reason="")

    if geom["score_ratio"] < min_score_ratio:
        info["reason"] = (f"열 구조 약함 (score_ratio {geom['score_ratio']:.1f} "
                          f"< {min_score_ratio}) — 닫기 생략")
        return obst.copy(), info

    gs = row_gap_stats(obst, cell, geom, percentile=gap_percentile)
    info.update(gap_stats=gs)
    if not np.isfinite(gs["length"]) or gs["length"] <= cell:
        info["reason"] = "결손 없음 — 닫기 불필요"
        return obst.copy(), info

    # ── 닫기 길이를 자동으로 정한다 ────────────────────────────────────────
    # 길이를 상수로 박거나 분위수 하나로 정하면, 결주가 연달아 난 자리에서
    # 부족해 구멍이 남는다. 대신 '열이 한 조각으로 이어질 때까지' 올린다.
    # 이 판정에는 참값이 필요 없다 — 추정한 열 위치만으로 계산된다.
    #
    # 닫기 길이를 키워도 이웃 열이 붙지는 않는다. 구조요소가 횡방향 폭이 0인
    # 선분이라 열 방향으로만 메우기 때문이다. 그래서 넉넉히 잡아도 안전하다.
    cap = (max_length_factor * geom["spacing"]
           if np.isfinite(geom["spacing"]) else float("inf"))
    ladder = sorted({round(min(v, cap), 2) for v in
                     (gs["pct"].get(90, 0), gs["pct"].get(95, 0),
                      gs["pct"].get(99, 0), gs["gmax"], cap) if v > cell})

    length, frag, trials = float(min(gs["length"], cap)), float("nan"), []
    sealed = obst
    for cand in ladder:
        sealed = oriented_closing(obst, geom["theta"], cand, cell)
        frag, _ = row_fragmentation(sealed, cell, geom)
        trials.append((cand, frag))
        if np.isfinite(frag) and frag <= 1.05:      # 열마다 사실상 한 조각
            length = cand
            break
        length = cand

    info.update(applied=True, length=length, frag=frag, trials=trials,
                capped=length >= cap,
                reason=(f"열 방향 {np.rad2deg(geom['theta']):.1f}° 로 {length:.2f} m "
                        f"닫기 (열당 조각 {frag:.2f})"))
    return sealed, info
