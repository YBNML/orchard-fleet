#!/usr/bin/env python3
"""헤드랜드 실패 해부도 — run3 기록에서 3단계(정상/스키드/환상)를 그린다

    python3 scripts/45_skid_anatomy_figure.py [/tmp/loc_run3.npz]
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import numpy as np                                    # noqa: E402
from matplotlib import font_manager                   # noqa: E402
import matplotlib.pyplot as plt                       # noqa: E402

F = "Noto Sans CJK KR"
_KR = Path("/tmp/NotoSansCJKkr-Regular.otf")
if not any(F in f.name for f in font_manager.fontManager.ttflist):
    font_manager.fontManager.addfont(str(_KR))
plt.rcParams["font.family"] = F
plt.rcParams["axes.unicode_minus"] = False

SRC = sys.argv[1] if len(sys.argv) > 1 else "/tmp/loc_run3.npz"
OUT = Path("docs/figures/m3"); OUT.mkdir(parents=True, exist_ok=True)
INK, LEAF, GOLD, DANGER, PLAN = "#221E18", "#2E6B37", "#8A5D12", "#A62A21", "#4C5B8C"

d = np.load(SRC)
t = d["t"] - d["t"][0]
gy, py = d["gy"], d["py"]

# 구간 경계: 참값이 통로를 벗어난 때(램프 진입), 참값이 멈춘 때(둑에 박힘)
i_ramp = int(np.argmax(gy > 31.2))
gmove = np.hypot(np.diff(d["gx"]), np.diff(d["gy"]))
cum = np.concatenate([[0], np.cumsum(gmove)])
i_stuck = int(np.searchsorted(cum, cum[-1] - 0.3))

fig, ax = plt.subplots(figsize=(11.5, 5.6))
ax.plot(t, gy, color=INK, lw=2.2, label="참값 y (지면)")
ax.plot(t, py, color=LEAF, lw=1.4, ls="--", label="추정 y (오도메트리+보정)")
ax.axvspan(0, t[i_ramp], color=LEAF, alpha=.08)
ax.axvspan(t[i_ramp], t[i_stuck], color=GOLD, alpha=.15)
ax.axvspan(t[i_stuck], t[-1], color=DANGER, alpha=.10)
ym = ax.get_ylim()

BOX = dict(facecolor="white", alpha=0.88, edgecolor="none", pad=2.5)

def label(x0, x1, txt, sub, ytxt=None):
    yy = ym[1] - 3 if ytxt is None else ytxt
    ax.text((x0 + x1) / 2, yy, txt, ha="center", fontsize=11,
            fontweight="bold", color=INK, bbox=BOX)
    ax.text((x0 + x1) / 2, yy - 5.5, sub, ha="center", fontsize=8.5,
            color=INK, bbox=BOX)

label(0, t[i_ramp], "① 통로 안 — 정상",
      f"종오차 최대 {np.abs(gy[:i_ramp]-py[:i_ramp]).max():.2f} m", ytxt=10.0)
label(t[i_ramp], t[i_stuck], "② 램프 — 궤도 미끄러짐",
      "바퀴 3.0 m vs 지면 5.8 m — 오도메트리가 덜 센다", ytxt=14.0)
label(t[i_stuck], t[-1], "③ 둑에 박힘 — 환상 주행",
      "참값 정지, 추정만 28.6 m 질주\n(코앞이 흙 → 구조점 0 → 감시 마비)",
      ytxt=26.0)
ax.axhline(34.32, color=PLAN, lw=0.9, ls=":")
ax.text(3, 32.6, "출구 웨이포인트 y=34.32", ha="left", fontsize=8,
        color=PLAN, bbox=BOX)
ax.axhline(38.9, color=DANGER, lw=0.9, ls=":")
ax.text(3, 40.0, "둑 (주행불가 경계)", ha="left", fontsize=8,
        color=DANGER, bbox=BOX)
ax.set_xlabel("시간 (초)"); ax.set_ylabel("y (m)")
ax.set_title("헤드랜드 실패 해부 — 통로 안은 완벽했고, 램프가 죽였다", fontsize=13)
ax.legend(fontsize=9, loc="lower left"); ax.grid(alpha=.25)
p = OUT / "skid_anatomy.png"
fig.savefig(p, dpi=150, bbox_inches="tight", facecolor="white")
print(f"생성: {p}")
