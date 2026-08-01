"""열 상대 로컬리제이션 — 사전 맵 위에서 위치를 잡는다 (순수 계산, ROS 무관).

왜 일반 NDT/ICP 가 아니라 이 방식인가
    과수원은 **주기 구조**다. 열 간격 3.5 m, 나무 간격 1.5 m 로 같은 모양이
    반복된다. 여기에 일반 스캔 정합을 걸면 방향마다 성질이 완전히 다르다.

        횡방향(x, 열을 가로지르는)  나무 열이 뚜렷한 선을 이룬다 → 강한 제약
        요(yaw)                     열 선의 기울기로 직접 읽힌다 → 강한 제약
        종방향(y, 열을 따라가는)    1.5 m 마다 같은 그림 → **주기적 모호성**

    이 비대칭을 무시하고 "정합이 다 잡아준다"고 두면, 종방향이 한 칸(1.5 m)
    미끄러진 해로 수렴해도 정합 점수는 똑같이 좋다. 그래서 방향을 나눠서 다룬다.

        횡·요   매 스캔 보정한다 (주기 위상으로 직접 계산 — 반복 최적화 없음)
        종      **보정하지 않는다.** 처음에는 위상으로 묶으려 했으나 실측에서
                기각됐다(2026-08-02, scripts/40_probe_agl_band.py) — 줄기가
                너무 가늘어(반경 0.035 m) 원거리에서 거의 안 맞고, 대신 잡히는
                수관은 열 방향으로 연속이라 -0.34~+0.41 m 편의가 실린다.
                종방향은 오도메트리에 맡기고 절대 기준은 열 끝에서 잡는다.

    이것이 원래 실패했던 LIO 와의 차이다. LIO 는 선회부에서 횡·요 제약이
    통째로 사라져 오차가 **누적**됐다. 여기서는 열이 보이는 한 횡·요가 매번
    절대 기준에 다시 붙으므로 누적되지 않는다.

한계 (설계 제약으로 명시)
    위상 보정은 ±(간격/2) 안에서만 뜻이 있다. 즉 보정 사이의 오도메트리 표류가
    횡 1.75 m · 종 0.75 m 를 넘으면 엉뚱한 열/나무에 붙는다. 보정 주기와
    오도메트리 품질이 이 한계를 지켜야 한다 — gate() 가 이를 감시한다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class RowFix:
    dx: float = 0.0            # 횡 보정 (밭 좌표 x, m)
    dy: float = 0.0            # 종 보정 (m) — 위상 잠금만, 절대 아님
    dyaw: float = 0.0          # 요 보정 (rad)
    quality: float = 0.0       # 0~1. 위상 집중도
    n_struct: int = 0          # 구조점 수
    y_span: float = 0.0        # 구조점의 종방향 퍼짐 (요 추정 신뢰의 근거)
    lateral_ok: bool = False   # 횡·요를 믿어도 되는가
    longitudinal_ok: bool = False   # 종 위상을 믿어도 되는가 (아래 상수 참조)
    lon_quality: float = 0.0   # 종 위상 집중도 (진단용)
    at_row_end: bool = False   # 열 끝(절대 기준을 잡을 수 있는 자리)
    yaw_saturated: bool = False   # 요 해가 탐색 범위 경계에 붙었다 = 범위 밖일 가능성
    in_block: bool = True      # 나무 구역 안인가 (선회 구역이면 False)


# 종 위상을 채택할 최소 집중도. 실측상 이 값에 도달하는 대역이 없어서 사실상
# 꺼져 있다 — 값을 낮추면 편의가 실린 보정이 들어온다. 근거는 estimate() 주석.
LONGITUDINAL_MIN_QUALITY = 0.85


def _wrap(v, period):
    """[-period/2, period/2) 로 접는다."""
    return (v + period / 2.0) % period - period / 2.0


def _phase(vals, period):
    """주기 위상 추정 — 원형 평균. 히스토그램 최빈보다 잡음에 강하다.

    반환 (offset, concentration). concentration 은 0(균일)~1(한 점에 몰림).
    """
    if len(vals) == 0:
        return 0.0, 0.0
    a = 2.0 * math.pi * (np.asarray(vals) % period) / period
    c, s = np.cos(a).mean(), np.sin(a).mean()
    r = math.hypot(c, s)
    off = math.atan2(s, c) * period / (2.0 * math.pi)
    return off, r


def structure_points(points_base, agl=(0.35, 1.30), max_range=25.0,
                     n_bins=12, ground_pct=15.0):
    """스캔에서 **줄기**만 남긴다 (지면·수관 제거).

    z 를 그대로 자르면 안 된다 — 점군은 센서 기준이라 지면이 z≈-0.6 에 있고,
    계단식 지형에서는 그 값이 거리에 따라 달라진다. 실제로 z∈[0.3,1.9] 로
    자르면 줄기가 아니라 **수관**이 남는데, 수관은 열 방향으로 연속이라
    종방향 위상이 통째로 무의미해진다(이번에 실제로 겪은 결함이다).

    그래서 거리 구간마다 지면 높이를 스캔 자체에서 추정하고, 그 위 높이(AGL)로
    자른다. 줄기 대(0.35~1.30 m)는 traversability.obstacle_layers 와 같은 값이다.
    """
    p = np.asarray(points_base, dtype=float)
    if p.size == 0:
        return p.reshape(0, 3)
    d = np.hypot(p[:, 0], p[:, 1])
    near = (d <= max_range) & (d > 0.8)
    if near.sum() < 50:
        return p[:0]
    p, d = p[near], d[near]

    edges = np.linspace(d.min(), d.max(), n_bins + 1)
    keep = np.zeros(len(p), dtype=bool)
    for i in range(n_bins):
        m = (d >= edges[i]) & (d < edges[i + 1] if i < n_bins - 1 else d <= edges[i + 1])
        if m.sum() < 20:
            continue
        gz = np.percentile(p[m, 2], ground_pct)      # 이 거리대의 지면 높이
        h = p[m, 2] - gz
        sel = (h >= agl[0]) & (h <= agl[1])
        idx = np.flatnonzero(m)[sel]
        keep[idx] = True
    return p[keep]


def estimate(points_base, pose, geom, *,
             min_struct=120, min_quality=0.25,
             yaw_range_deg=12.0, coarse=49, fine=25) -> RowFix:
    """사전 맵의 열 기하에 대해 현재 스캔의 어긋남을 계산한다.

    pose : (x, y, yaw) 현재 추정 (오도메트리가 밀고 온 값)
    geom : {"x0","row_spacing","tree_spacing","col_len"}

    **요를 먼저 푼다.** 요가 틀어진 채로 횡 위상을 재면 무너진다 — 25 m 앞의
    점은 요 6° 오차에 횡으로 2.6 m 밀리고, 그건 열 간격(3.5 m)의 3/4 이라
    위상이 여러 주기에 걸쳐 뭉개진다. 그래서 사전 요를 믿지 않고, 위상
    집중도가 가장 높아지는 요를 직접 찾는다. 집중도는 요가 맞을 때만
    날카로워지므로 이 1차원 탐색이 요와 횡을 함께 푼다.
    """
    fix = RowFix()
    sp = structure_points(points_base)
    fix.n_struct = int(len(sp))
    if fix.n_struct < min_struct:
        return fix

    x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    px, py = sp[:, 0], sp[:, 1]
    S = float(geom["row_spacing"])
    x0 = float(geom["x0"])

    def score(dyaw):
        c, s = math.cos(yaw + dyaw), math.sin(yaw + dyaw)
        wx = x + c * px - s * py
        off, conc = _phase(wx - x0, S)
        return conc, off

    def search(lo, hi, n):
        best = (-1.0, 0.0, 0.0)          # (집중도, dyaw, off)
        for dy in np.linspace(lo, hi, n):
            conc, off = score(float(dy))
            if conc > best[0]:
                best = (conc, float(dy), off)
        return best

    rad = math.radians(yaw_range_deg)
    conc, dyaw, off = search(-rad, rad, coarse)
    span = 2 * rad / (coarse - 1)        # 성긴 격자 한 칸 주변을 다시 훑는다
    conc, dyaw, off = search(dyaw - span, dyaw + span, fine)

    fix.quality = float(conc)
    fix.dyaw = float(dyaw)
    # 해가 탐색 경계에 붙었다면 진짜 해는 범위 밖일 수 있다. 그 값을 그대로
    # 쓰면 자세가 엉뚱한 쪽으로 끌려간다 — 실제로 선회 중에 요 오차 19.5°,
    # 종방향 21 m 발산을 겪었다. 경계 해는 채택하지 않는다.
    fix.yaw_saturated = abs(dyaw) > 0.85 * rad
    fix.dx = -_wrap(off, S)              # 관측이 +δ 로 밀려 있으면 자세를 -δ 로 당긴다
    fix.lateral_ok = (fix.quality >= min_quality and fix.n_struct >= min_struct)

    # 종방향 통계는 최적 요에서 다시 잡는다
    c, s = math.cos(yaw + dyaw), math.sin(yaw + dyaw)
    wy = y + s * px + c * py
    fix.y_span = float(wy.max() - wy.min()) if len(wy) else 0.0

    # ── 종 보정: 실측으로 기각됐다 ──────────────────────────────────────────
    # 2026-08-02 실측(scripts/40_probe_agl_band.py): 참값 자세에서도 종 위상이
    # 어느 높이 대역에서든 -0.34 ~ +0.41 m 로 흔들리고 집중도가 0.04~0.55 에
    # 그친다. 원인은 기하다 — 줄기 반경이 0.035 m 뿐이라 25 m 밖에서는 거의
    # 맞지 않고, 높은 대역에서 잡히는 것은 **수관**인데 수관은 열 방향으로
    # 연속이고 앞면만 보여 계통 편의가 생긴다.
    #
    # 편의가 실린 보정은 보정을 안 하느니만 못하다. 그래서 종방향은 위상으로
    # 잡지 않고 오도메트리에 맡기며, 절대 기준은 열이 끝나는 지점에서 잡는다.
    # (값은 진단용으로 계속 계산해 둔다 — 다른 수종·다른 센서에서는 달라진다)
    T = float(geom.get("tree_spacing", 1.5))
    off_y, conc_y = _phase(wy, T)
    fix.dy = -_wrap(off_y, T)
    fix.lon_quality = float(conc_y)
    fix.longitudinal_ok = bool(conc_y >= LONGITUDINAL_MIN_QUALITY)

    # ── 열 끝 판정 ──────────────────────────────────────────────────────────
    # 종방향은 나무 간격(1.5 m)으로 앨리어싱되므로 위상만으로는 절대 위치를
    # 못 잡는다. 열이 끝나는 자리(구조점이 사라지는 y)는 그 예외다 — 여기가
    # 종방향의 유일한 절대 기준이다.
    half = float(geom.get("col_len", 0.0)) / 2.0
    if half > 0 and len(wy):
        fix.at_row_end = bool(wy.max() < half - 1.0 or wy.min() > -half + 1.0)
        # 선회 구역에서는 열이 시야에서 사라진다 — 여기서 잡히는 '열'은 대개
        # 다른 각도에서 본 같은 나무들이라 위상이 엉뚱하게 맞는다.
        fix.in_block = bool(abs(y) <= half + 1.0)
    return fix


def gate(fix: RowFix, drift_since_fix_m: float, geom, *, max_jump_m=0.8):
    """이 보정을 써도 되는가.

    보정을 채택하는 조건은 넷이다. 하나라도 어기면 **보정하지 않고** 오도메트리로
    간다 — 틀린 보정은 보정을 안 하느니만 못하다. 실제로 이 넷 중 둘이 없을 때
    선회 구간에서 요 19.5° · 종방향 21 m 로 발산했다(2026-08-02 실측).

      1. 구조가 충분한가          집중도·구조점
      2. 요 해가 범위 안인가      경계에 붙은 해는 진짜 해가 밖에 있다는 신호
      3. 나무 구역 안인가         선회 구역의 '열'은 다른 각도에서 본 같은 나무다
      4. 표류가 위상 한계 안인가  ±(열 간격/2) 을 넘으면 엉뚱한 열에 붙는다
      5. 보정량이 상식적인가      갑자기 크게 튀는 보정은 잘못 붙은 것이다
    """
    if not fix.lateral_ok:
        return False, f"구조 부족 (점 {fix.n_struct}, 집중도 {fix.quality:.2f})"
    if fix.yaw_saturated:
        return False, f"요 해가 탐색 경계 ({math.degrees(fix.dyaw):+.1f}°)"
    if not fix.in_block:
        # 헤드랜드에서도 열 끝 줄기들이 시야에 들면 횡위상은 성립한다 —
        # 위상은 월드 x 기준이라 로봇 방위와 무관하고, 요가 자이로로 정직한
        # 지금은 '다른 각도에서 본 같은 나무' 오인 위험도 낮다. 실측: 횡단
        # 클라임 중 병진 슬립 65% 로 추정이 3 m 달아나는데 그때 유일한 절대
        # 기준이 이 횡위상이었다(08-02). 단 고신뢰·소보정만 받는다.
        if fix.quality < 0.5 or abs(fix.dx) > 0.5:
            return False, "선회 구역 (나무 구역 밖)"
    limit = float(geom["row_spacing"]) / 2.0
    if drift_since_fix_m > limit:
        return False, f"보정 간 표류 {drift_since_fix_m:.2f} m > 한계 {limit:.2f} m"
    if abs(fix.dx) > max_jump_m:
        return False, f"보정량 과다 (dx {fix.dx:+.2f} m)"
    return True, ""


def scan_travel(prev_pts, cur_pts, *, span_m=0.7, bin_m=0.05,
                r_min=1.0, r_max=20.0):
    """두 스캔(로봇 기준) 사이의 **전진 변위**를 추정한다 — 슬립 감지용.

    로봇이 d 만큼 전진하면 정지물의 로봇 기준 전방좌표는 d 만큼 줄어든다.
    두 스캔의 전방좌표 히스토그램을 상관시켜 그 d 를 찾는다. 오도메트리가
    "이만큼 갔다"고 보고하는데 스캔이 "안 갔다"고 하면 바퀴가 헛도는 것이다
    — 실제로 나무에 박힌 채 오도메트리만 35 m 달아난 사고를 겪었다(08-02).

    과수원은 1.5 m 주기 구조라 상관에도 주기마다 봉우리가 선다. 그래서 탐색
    폭(span_m)을 반주기(0.75 m) 아래로 잡아야 해가 유일하다 — 호출자는
    오도메트리 변위가 그 안일 때(0.5 m 안팎)마다 불러야 한다.

    회전이 섞이면 1차원 상관이 성립하지 않는다. 호출자가 요 변화 몇 도
    이내일 때만 부르는 것이 전제다.

    반환 (travel_m, confidence 0~1). 점이 부족하면 (0.0, 0.0).
    """
    a = np.asarray(prev_pts, dtype=float)
    b = np.asarray(cur_pts, dtype=float)
    if len(a) < 100 or len(b) < 100:
        return 0.0, 0.0
    edges = np.arange(r_min, r_max + bin_m, bin_m)

    def hist(p):
        x = p[:, 0]
        h, _ = np.histogram(x[(x >= r_min) & (x <= r_max)], bins=edges)
        h = h.astype(float)
        return h - h.mean()

    ha, hb = hist(a), hist(b)
    K = max(1, int(round(span_m / bin_m)))
    best_k, best_r = 0, -1.0
    for k in range(-K, K + 1):
        # 전진 d=k·bin 이면 현재 히스토그램은 이전 것을 왼쪽으로 민 것:
        #   hb[i] ≈ ha[i + k]
        u = ha[k:] if k >= 0 else ha[:k]
        v = hb[:len(ha) - k] if k >= 0 else hb[-k:]
        den = math.sqrt(float((u * u).sum()) * float((v * v).sum()))
        r = float((u * v).sum()) / den if den > 1e-9 else 0.0
        if r > best_r:
            best_r, best_k = r, k
    return best_k * bin_m, max(0.0, best_r)
