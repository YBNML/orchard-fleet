#!/usr/bin/env python3
"""M3 결과 차트 생성 — 워드 보고서에 넣을 그림들

    python3 scripts/41_m3_figures.py

입력 (있는 것만 그린다)
    /tmp/agl_band.npz   높이 대역 실험 (40번)
    /tmp/loc_run.npz    주행 중 오차 기록 (39번)
출력
    docs/figures/m3/*.png
"""
from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import numpy as np                                    # noqa: E402
from matplotlib import font_manager                   # noqa: E402
import matplotlib.pyplot as plt                       # noqa: E402

F = "Noto Sans CJK KR"
_KR = Path("/tmp/NotoSansCJKkr-Regular.otf")
if not any(F in f.name for f in font_manager.fontManager.ttflist):
    if not _KR.exists():
        from fontTools.ttLib import TTCollection
        tc = TTCollection("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
        for _f in tc.fonts:
            if "KR" in (_f["name"].getDebugName(4) or ""):
                _f.save(str(_KR)); break
    font_manager.fontManager.addfont(str(_KR))
plt.rcParams["font.family"] = F
plt.rcParams["axes.unicode_minus"] = False

OUT = Path("docs/figures/m3"); OUT.mkdir(parents=True, exist_ok=True)
INK, LEAF, GOLD, DANGER, PLAN = "#221E18", "#2E6B37", "#8A5D12", "#A62A21", "#4C5B8C"
made = []


def save(fig, name):
    p = OUT / name
    fig.savefig(p, dpi=150, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    made.append(str(p))
    print(f"  {p}")


# ── 1. 높이 대역 실험 ───────────────────────────────────────────────────────
if Path("/tmp/agl_band.npz").exists():
    d = np.load("/tmp/agl_band.npz")
    lab = [f"{lo:.2f}~{hi:.2f}" for lo, hi in zip(d["lo"], d["hi"])]
    xi = np.arange(len(lab))
    fig, ax = plt.subplots(1, 2, figsize=(11.5, 4.0))

    ax[0].axhline(0, color=INK, lw=0.8)
    ax[0].plot(xi, d["ox"], "o-", color=LEAF, label="횡 (열 간격 3.5 m)")
    ax[0].plot(xi, d["oy"], "s--", color=DANGER, label="종 (나무 간격 1.5 m)")
    ax[0].fill_between(xi, -0.05, 0.05, color=LEAF, alpha=.12)
    ax[0].set_xticks(xi); ax[0].set_xticklabels(lab, rotation=45, ha="right", fontsize=8)
    ax[0].set_ylabel("참값 자세에서의 위상 오차 (m)")
    ax[0].set_title("높이 대역별 위상 편의 — 0 이어야 정상", fontsize=11)
    ax[0].legend(fontsize=9); ax[0].grid(alpha=.25)

    ax[1].plot(xi, d["cx"], "o-", color=LEAF, label="횡")
    ax[1].plot(xi, d["cy"], "s--", color=DANGER, label="종")
    ax[1].axhline(0.25, color=GOLD, ls=":", label="채택 문턱 0.25")
    ax[1].set_xticks(xi); ax[1].set_xticklabels(lab, rotation=45, ha="right", fontsize=8)
    ax[1].set_ylabel("위상 집중도 (0~1)")
    ax[1].set_title("높이 대역별 위상 집중도 — 높을수록 또렷", fontsize=11)
    ax[1].legend(fontsize=9); ax[1].grid(alpha=.25)
    fig.suptitle("실험: 어느 높이가 '열'을 보여주는가 (실측 라이다, 참값 자세)",
                 fontsize=12.5, y=1.02)
    save(fig, "agl_band.png")

# ── 2. 주행 중 오차 ─────────────────────────────────────────────────────────
if Path("/tmp/loc_run.npz").exists():
    d = np.load("/tmp/loc_run.npz")
    t, ex, ey, eyaw = d["t"], d["ex"], d["ey"], d["eyaw"]
    err = np.hypot(ex, ey)

    fig, ax = plt.subplots(2, 1, figsize=(11.0, 6.2), sharex=True,
                           gridspec_kw=dict(height_ratios=[2, 1]))
    ax[0].axhline(0.30, color=DANGER, ls="--", lw=1.2, label="M3 예산 0.30 m")
    ax[0].plot(t, err, color=INK, lw=1.0, label="위치 오차")
    ax[0].plot(t, np.abs(ex), color=LEAF, lw=0.9, alpha=.8, label="횡 |x|")
    ax[0].plot(t, np.abs(ey), color=PLAN, lw=0.9, alpha=.8, label="종 |y|")
    ax[0].set_ylabel("오차 (m)")
    ax[0].set_title(f"주행 중 위치 오차 — RMS {np.sqrt((err**2).mean()):.3f} m · "
                    f"최대 {err.max():.3f} m", fontsize=12)
    ax[0].legend(fontsize=9, ncol=4); ax[0].grid(alpha=.25)

    ax[1].plot(t, np.degrees(eyaw), color=GOLD, lw=1.0)
    ax[1].axhline(0, color=INK, lw=0.6)
    ax[1].set_ylabel("방위 오차 (°)"); ax[1].set_xlabel("시간 (초)")
    ax[1].grid(alpha=.25)
    save(fig, "live_error.png")

    # 궤적
    fig, ax = plt.subplots(figsize=(6.4, 7.6))
    ax.plot(d["gx"], d["gy"], color=INK, lw=2.0, label="참값")
    ax.plot(d["px"], d["py"], color=LEAF, lw=1.2, ls="--", label="추정")
    ax.set_aspect("equal"); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title("주행 궤적 — 참값 대 추정", fontsize=12)
    ax.legend(fontsize=9); ax.grid(alpha=.25)
    save(fig, "live_track.png")

    # 전반/후반 누적 여부
    h = len(err) // 2
    fig, ax = plt.subplots(figsize=(5.2, 4.0))
    vals = [np.sqrt((err[:h] ** 2).mean()), np.sqrt((err[h:] ** 2).mean())]
    ax.bar(["전반", "후반"], vals, color=[LEAF, PLAN], width=.55)
    ax.axhline(0.30, color=DANGER, ls="--", label="M3 예산")
    for i, v in enumerate(vals):
        ax.text(i, v + 0.005, f"{v:.3f} m", ha="center", fontsize=10, fontweight="bold")
    ax.set_ylabel("위치 오차 RMS (m)")
    ax.set_title("누적되는가 — 전반 대 후반", fontsize=12)
    ax.legend(fontsize=9); ax.grid(alpha=.25, axis="y")
    save(fig, "live_accum.png")

print(f"\n그림 {len(made)}개 생성")
