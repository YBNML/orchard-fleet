#!/usr/bin/env python3
"""BOUSTROPHEDON — 측량 도판 형식의 단일 플레이트 (300 dpi PNG)"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.patches import Arc, Rectangle, Circle
from matplotlib.path import Path as MPath
import matplotlib.patches as mpatches

FD = "/home/myhome/.claude/skills/canvas-design/canvas-fonts"
GLOOCK = fm.FontProperties(fname=f"{FD}/Gloock-Regular.ttf")
MONO = fm.FontProperties(fname=f"{FD}/GeistMono-Regular.ttf")
CRIM_I = fm.FontProperties(fname=f"{FD}/CrimsonPro-Italic.ttf")
KR = fm.FontProperties(fname="/tmp/NotoSansCJKkr-Regular.otf")

PAPER, INK = "#F1EFE7", "#241F19"
LEAF, GOLD, VERM, LOAM = "#3A5232", "#8A5D12", "#B7351F", "#5C4A33"

rng = np.random.default_rng(41)          # PL. 41

W, H = 100.0, 140.0
fig = plt.figure(figsize=(8.0, 11.2), dpi=300)
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, W); ax.set_ylim(0, H); ax.axis("off")
ax.add_patch(Rectangle((0, 0), W, H, fc=PAPER, ec="none", zorder=0))

# ── 종이 결 — 아주 미세한 입자 ──────────────────────────────────────────
gx = rng.uniform(0, W, 5200); gy = rng.uniform(0, H, 5200)
ax.scatter(gx, gy, s=0.15, c=INK, alpha=0.05, linewidths=0, zorder=1)

# ── 도판 틀 ─────────────────────────────────────────────────────────────
ax.add_patch(Rectangle((6, 6), 88, 128, fill=False, ec=INK, lw=0.7, zorder=6))
ax.add_patch(Rectangle((6.9, 6.9), 86.2, 126.2, fill=False, ec=INK,
                       lw=0.3, alpha=0.55, zorder=6))

# ── 머리글 ──────────────────────────────────────────────────────────────
ax.text(9.5, 129.6, "FIELD SURVEY · TERRACED ORCHARD · NINE PASSES",
        fontproperties=MONO, fontsize=5.4, color=INK, alpha=0.85, zorder=7)
ax.text(90.5, 129.6, "PL. 41", fontproperties=MONO, fontsize=5.4,
        color=INK, ha="right", alpha=0.85, zorder=7)
ax.plot([9.5, 90.5], [128.2, 128.2], color=INK, lw=0.35, alpha=0.6, zorder=7)

# ── 시야 문양 (우상) — 70.4° 쐐기에서 온원으로 ──────────────────────────
cx0, cy0, r0 = 85.0, 121.5, 3.4
ax.add_patch(Circle((cx0, cy0), r0, fill=False, ec=INK, lw=0.55, zorder=7))
th = np.radians(np.linspace(90 - 35.2, 90 + 35.2, 40))
ax.plot(cx0 + r0 * 0.72 * np.cos(th), cy0 + r0 * 0.72 * np.sin(th),
        color=INK, lw=0.45, ls=(0, (1.6, 1.6)), alpha=0.4, zorder=7)
for a in (90 - 35.2, 90 + 35.2):
    ar = np.radians(a)
    ax.plot([cx0, cx0 + r0 * 0.72 * np.cos(ar)],
            [cy0, cy0 + r0 * 0.72 * np.sin(ar)],
            color=INK, lw=0.45, ls=(0, (1.6, 1.6)), alpha=0.4, zorder=7)
ax.text(cx0 - r0 - 1.4, cy0, "70.4° → 360°", fontproperties=MONO,
        fontsize=3.9, color=INK, ha="right", va="center", alpha=0.65,
        zorder=7)

# ── 본 밭 기하 ──────────────────────────────────────────────────────────
XL, XR = 14.0, 86.0            # 열 0 … 열 9
ROWS = np.linspace(XL, XR, 10)
Y0, Y1 = 34.0, 108.0           # 블록(나무 구간)
GT = (110.5, 113.5)            # 황금 대역 (상)
GB = (28.5, 31.5)              # 황금 대역 (하)
APEX_T, APEX_B = 112.0, 30.0
CENTERS = (ROWS[:-1] + ROWS[1:]) / 2

# 테라스 계단 — 통로마다 아주 옅게 짙어지는 흙
for k in range(9):
    ax.add_patch(Rectangle((ROWS[k], Y0 - 1.0), 8, (Y1 - Y0) + 2,
                 fc=LOAM, ec="none", alpha=0.012 + 0.0045 * k, zorder=2))

# 황금 대역
for (a, b) in (GT, GB):
    ax.add_patch(Rectangle((XL - 1.5, a), (XR - XL) + 3, b - a,
                 fc=GOLD, ec="none", alpha=0.055, zorder=2))
    for yy in (a, b):
        ax.plot([XL - 1.5, XR + 1.5], [yy, yy], color=GOLD, lw=0.3,
                alpha=0.4, zorder=3)

# 나무 — 열마다 41점, 하나하나
ty = np.linspace(Y0, Y1, 41)
for x in ROWS:
    jx = x + rng.normal(0, 0.06, 41)
    jy = ty + rng.normal(0, 0.05, 41)
    ax.scatter(jx, jy, s=1.9, c=LEAF, alpha=0.85, linewidths=0, zorder=4)

# 벽(앵커) — 통로×단마다 다른 자리의 짧은 담 (교정표의 기억)
wall_j = rng.uniform(-0.9, 0.9, (9, 2))
for k, cxa in enumerate(CENTERS):
    ax.plot([cxa - 2.2, cxa + 2.2],
            [115.6 + wall_j[k][0]] * 2, color=INK, lw=0.8, alpha=0.4, zorder=4)
    ax.plot([cxa - 2.2, cxa + 2.2],
            [25.2 + wall_j[k][1]] * 2, color=INK, lw=0.8, alpha=0.4, zorder=4)

# ── 보스트로피돈 경로 — 진행 방향으로 떨리는 단 하나의 선 ────────────────
def jitter(n, amp=0.14, seed_off=0):
    r = np.cumsum(rng.normal(0, 1, n)); r -= np.linspace(r[0], r[-1], n)
    m = np.abs(r).max() or 1.0
    return r / m * amp

LEG_T, LEG_B = Y1, Y0 - 1.0        # 다리 끝 (블록 살짝 밖)
segs = []          # (points, color)
for k in range(9):
    cxa = CENTERS[k]
    up = (k % 2 == 0)
    n = 120
    if up:
        ys = np.linspace(26.0 if k == 0 else LEG_B, LEG_T, n)
    else:
        ys = np.linspace(LEG_T, LEG_B, n)
    xs = np.full(n, cxa) + jitter(n)
    segs.append((np.column_stack([xs, ys]), INK))
    if k < 8:                                   # 선회 호 — 반원, 정점은 황금 대역
        nxt = CENTERS[k + 1]
        mid = (cxa + nxt) / 2
        rr = (nxt - cxa) / 2
        if up:
            base, apex = LEG_T, APEX_T
            tt = np.linspace(np.pi, 0, 70)
        else:
            base, apex = LEG_B, APEX_B
            tt = np.linspace(np.pi, 2 * np.pi, 70)
        axs = mid + rr * np.cos(tt) + jitter(70, 0.10)
        ays = base + np.abs(np.sin(tt)) * (apex - base) + jitter(70, 0.07)
        segs.append((np.column_stack([axs, ays]), GOLD))

for pts, c in segs:
    ax.plot(pts[:, 0], pts[:, 1], color=c, lw=1.05,
            solid_capstyle="round", zorder=5)

# 시점·종점
ax.scatter([CENTERS[0]], [26.0], s=7, c=INK, zorder=6)
ax.add_patch(Circle((CENTERS[8], Y1 + 1.6), 0.75, fill=False, ec=INK,
                    lw=0.8, zorder=6))

# 상처 셋 — 아주 작은 주필 ×
for (wx, wy) in ((CENTERS[1] + 1.6, 29.1), (CENTERS[0] + 0.6, 114.3),
                 ((ROWS[2] + 0.35), 61.0)):
    ax.text(wx, wy, "×", fontproperties=MONO, fontsize=4.6, color=VERM,
            ha="center", va="center", alpha=0.9, zorder=6)

for k, cxa in enumerate(CENTERS):
    ax.text(cxa, 118.2, f"k{k}", fontproperties=MONO, fontsize=3.4,
            color=INK, ha="center", alpha=0.5, zorder=5)

# ── 좌측 자 ─────────────────────────────────────────────────────────────
ax.plot([11, 11], [Y0, Y1], color=INK, lw=0.4, alpha=0.7, zorder=5)
for m, lab in ((-30, "-30"), (-20, ""), (-10, ""), (0, "0"),
               (10, ""), (20, ""), (30, "+30")):
    yy = np.interp(m, [-30, 30], [Y0, Y1])
    tick = 1.1 if lab else 0.6
    ax.plot([11 - tick, 11], [yy, yy], color=INK, lw=0.4, alpha=0.7, zorder=5)
    if lab:
        ax.text(9.4, yy, lab, fontproperties=MONO, fontsize=3.8,
                color=INK, ha="right", va="center", alpha=0.7, zorder=5)
ax.text(9.4, (Y0 + Y1) / 2, "M", fontproperties=MONO, fontsize=3.8,
        color=INK, ha="right", va="center", alpha=0.45, zorder=5)

# ── 우측 괄호 — 황금 대역 ───────────────────────────────────────────────
bx = 88.6
ax.plot([bx, bx + 0.9, bx + 0.9, bx], [GT[0], GT[0], GT[1], GT[1]],
        color=GOLD, lw=0.55, zorder=5)
ax.text(bx + 2.1, sum(GT) / 2, "34.5–35.5", fontproperties=MONO,
        fontsize=3.6, color=GOLD, va="center", ha="center",
        rotation=90, zorder=5)

# ── 하단 — 표제석 ───────────────────────────────────────────────────────
ax.text(50, 18.6, "행 과 회 귀", fontproperties=KR, fontsize=5.2,
        color=INK, ha="center", alpha=0.62, zorder=7)
ax.text(50, 13.6, "B O U S T R O P H E D O N", fontproperties=GLOOCK,
        fontsize=17.5, color=INK, ha="center", zorder=7)
ax.text(50, 10.6, "the line that learns by returning",
        fontproperties=CRIM_I, fontsize=6.2, color=INK, ha="center",
        alpha=0.7, zorder=7)
ax.text(50, 8.3, "GRID 3.50 × 1.50 M   ·   RMS 0.09 M",
        fontproperties=MONO, fontsize=4.2, color=INK, ha="center",
        alpha=0.6, zorder=7)

out = "/home/myhome/YBNML/docs/design/art/boustrophedon.png"
fig.savefig(out, dpi=300, facecolor=PAPER)
print("생성:", out)
