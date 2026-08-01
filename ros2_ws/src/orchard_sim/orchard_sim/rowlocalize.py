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
        종      위상으로 **묶어만 둔다**(±0.75 m 안에 가둠). 절대 기준은
                열 끝(선회 구역 진입)에서 나무가 끊기는 지점으로 잡는다.

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
    longitudinal_ok: bool = False   # 종 위상을 믿어도 되는가
    at_row_end: bool = False   # 열 끝(절대 기준을 잡을 수 있는 자리)


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


def structure_points(points_base, agl=(0.30, 1.90), max_range=25.0):
    """스캔에서 '열을 이루는' 점만 남긴다 (지면·하늘 제거).

    points_base: (N,3) 로봇 기준 좌표. z 는 기체 기준 높이.
    """
    p = np.asarray(points_base, dtype=float)
    if p.size == 0:
        return p.reshape(0, 3)
    d = np.hypot(p[:, 0], p[:, 1])
    m = (p[:, 2] >= agl[0]) & (p[:, 2] <= agl[1]) & (d <= max_range) & (d > 0.5)
    return p[m]


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
    fix.dx = -_wrap(off, S)              # 관측이 +δ 로 밀려 있으면 자세를 -δ 로 당긴다
    fix.lateral_ok = (fix.quality >= min_quality and fix.n_struct >= min_struct)

    # 종방향 통계는 최적 요에서 다시 잡는다
    c, s = math.cos(yaw + dyaw), math.sin(yaw + dyaw)
    wy = y + s * px + c * py
    fix.y_span = float(wy.max() - wy.min()) if len(wy) else 0.0

    # ── 종 보정: 나무 간격 위상 (잠금만) ────────────────────────────────────
    T = float(geom.get("tree_spacing", 1.5))
    off_y, conc_y = _phase(wy, T)
    fix.dy = -_wrap(off_y, T)
    fix.longitudinal_ok = bool(conc_y >= min_quality)

    # ── 열 끝 판정 ──────────────────────────────────────────────────────────
    # 종방향은 나무 간격(1.5 m)으로 앨리어싱되므로 위상만으로는 절대 위치를
    # 못 잡는다. 열이 끝나는 자리(구조점이 사라지는 y)는 그 예외다 — 여기가
    # 종방향의 유일한 절대 기준이다.
    half = float(geom.get("col_len", 0.0)) / 2.0
    if half > 0 and len(wy):
        fix.at_row_end = bool(wy.max() < half - 1.0 or wy.min() > -half + 1.0)
    return fix


def gate(fix: RowFix, drift_since_fix_m: float, geom) -> tuple[bool, str]:
    """이 보정을 써도 되는가.

    위상 보정은 ±(간격/2) 안에서만 뜻이 있다. 마지막 보정 이후 오도메트리가
    그보다 많이 밀렸다면 엉뚱한 열에 붙을 수 있으므로 **쓰지 않는다** —
    틀린 보정은 보정을 안 하느니만 못하다.
    """
    if not fix.lateral_ok:
        return False, f"구조 부족 (점 {fix.n_struct}, 집중도 {fix.quality:.2f})"
    limit = float(geom["row_spacing"]) / 2.0
    if drift_since_fix_m > limit:
        return False, f"보정 간 표류 {drift_since_fix_m:.2f} m > 한계 {limit:.2f} m"
    return True, ""
