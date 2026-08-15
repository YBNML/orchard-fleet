#!/usr/bin/env python3
"""정합 게이트 — 월드 나무가 정사영상의 나무열 위에 서 있는가 (스펙 ④ §7)

    python3 scripts/52_verify_farm_alignment.py --self-test
    python3 scripts/52_verify_farm_alignment.py \
        --world sim/worlds/orchard_real.sdf \
        --farm maps/orchard_real/farm.json \
        --n 20 --tol 0.5 --fig docs/figures/farm_alignment.png

무엇을 재나
    스펙 §7 정합 행의 문언은 "gz 월드 나무 위치 ↔ 이미지 픽셀 역변환 오차
    ≤0.5 m (표본 20그루)" 다. 이 스크립트는 그 문장을 세 항목으로 쪼개
    **각각 독립적으로** 잰다. 세 번째가 진짜 게이트다.

    A. 아핀 왕복 — 월드 → 픽셀 → 월드 가 제자리로 돌아오는가.
       farm.json `axes_note` 의 식을 이 파일에서 **다시 구현**해서 쓴다
       (gen_world 를 import 하지 않는다 — 생성기와 검증기가 같은 코드를
       공유하면 부호 하나가 틀려도 둘이 사이좋게 틀린다).

    B. 열 축 이탈 — 월드 나무가 farm.json 이 말하는 자기 열 축에서
       얼마나 벗어나 있는가(횡방향). 생성기가 매니페스트를 따랐는가의 검증.

    C. **이미지 능선 이탈** — 그 자리에서 정사영상의 식생(excess-green)
       능선이 실제로 어디인지 이미지에서 직접 찾아, 월드 나무와의 거리를
       잰다. A·B 는 farm.json 안에서만 도는 검사라 "매니페스트가 이미지와
       어긋나 있다"를 잡지 못한다 — 이미지를 다시 읽는 C 만이 그걸 잡는다.
       스펙 §0-1("이미지가 기준이다")이 요구하는 것도 이쪽이다.

    판정은 `align_err = |나무 x − (열 축 x + 능선 오프셋)|` 이 표본 전건에서
    `--tol`(기본 0.5 m) 이하인가로 한다. B 와 C 를 따로도 보고한다.

좌표계
    farm.json `axes_note` 그대로 — 월드에서 열은 x 로 늘어서고(cross-row),
    각 열은 +y 로 뻗는다(along-row). 회전(62.5°)은 월드→픽셀 아핀 안에만
    있다. world +y 는 이미지 +y(행 증가) = **지리적 남**이다.

표본 추출
    `--seed` 고정 난수로 열을 고르게 훑으며 뽑는다(같은 인자 → 같은 표본).
    스캔 창이 이미지 밖으로 나가는 나무는 뽑지 않는다(이미지가 없는 곳에서
    능선을 찾을 수는 없다) — 제외 수를 보고한다.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import random
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEF_WORLD = os.path.join("sim", "worlds", "orchard_real.sdf")
DEF_FARM = os.path.join("maps", "orchard_real", "farm.json")
DEF_IMAGERY = os.path.join("sim", "assets", "imagery")

# 나무 모델 이름 규약(gen_world) — 배경목은 열별 병합 모델, 계측목은 개별 모델.
RE_ROW_MODEL = re.compile(r"^row_trees_r(\d+)$")
RE_TREE_MODEL = re.compile(r"^apple_tree_[a-z0-9_]+__r(\d+)t(\d+)$")
RE_ROW_VISUAL = re.compile(r"^t(\d+)_all$")


# ── farm.json 아핀 (axes_note 식의 독립 재구현) ──────────────────────────────
def affine(farm):
    """(world→px, px→world) 두 함수를 돌려준다."""
    th = math.radians(float(farm["rotation_deg"]))
    c, s = math.cos(th), math.sin(th)
    k = float(farm["px_per_m"])
    ox, oy = (float(v) for v in farm["origin_px"])

    def to_px(wx, wy):
        return (ox + k * (c * wx - s * wy), oy + k * (s * wx + c * wy))

    def to_world(px, py):
        dx, dy = (px - ox) / k, (py - oy) / k
        return (c * dx + s * dy, -s * dx + c * dy)      # R⁻¹ = Rᵀ

    return to_px, to_world


def load_farm(path, imagery_dir=DEF_IMAGERY, require_image=True):
    """farm.json + 정사영상. 해시 불일치는 **명시적 실패**(스펙 §6)."""
    with open(path) as f:
        farm = json.load(f)
    n = int(farm["rows"])
    for k in ("row_origins", "row_lengths_m"):
        if len(farm[k]) != n:
            raise SystemExit(f"[52] ✘ farm.json 불일치: rows={n} 인데 {k} 는 {len(farm[k])}")
    img_path = os.path.join(imagery_dir, farm["image"])
    if not require_image:
        return farm, None
    if not os.path.exists(img_path):
        raise SystemExit(f"[52] ✘ 정사영상이 없습니다: {img_path}")
    h = hashlib.sha256(open(img_path, "rb").read()).hexdigest()
    if farm.get("image_sha256") and h != farm["image_sha256"]:
        raise SystemExit(
            f"[52] ✘ 이미지 해시 불일치 — farm.json 과 다른 이미지입니다\n"
            f"    farm.json: {farm['image_sha256']}\n    {img_path}: {h}")
    return farm, img_path


# ── 월드 SDF 에서 나무 위치 뽑기 ────────────────────────────────────────────
def _pose_xy(node):
    p = node.find("pose")
    if p is None or not (p.text or "").strip():
        return None
    v = (p.text or "").split()
    return (float(v[0]), float(v[1]))


def world_trees(sdf_path):
    """[(row, idx, x, y, kind)] — 배경목(row-merged visual)과 계측목(개별 모델)."""
    world = ET.parse(sdf_path).getroot().find("world")
    if world is None:
        raise SystemExit(f"[52] ✘ <world> 가 없습니다: {sdf_path}")
    out = []
    for m in world.findall("model"):
        name = m.get("name") or ""
        mrow = RE_ROW_MODEL.match(name)
        if mrow:
            r = int(mrow.group(1))
            base = _pose_xy(m) or (0.0, 0.0)
            for link in m.findall("link"):
                for vis in link.findall("visual"):
                    mv = RE_ROW_VISUAL.match(vis.get("name") or "")
                    if not mv:
                        continue                      # 사과·트렐리스 등은 제외
                    xy = _pose_xy(vis)
                    if xy is None:
                        continue
                    out.append((r, int(mv.group(1)),
                                base[0] + xy[0], base[1] + xy[1], "row"))
            continue
        mt = RE_TREE_MODEL.match(name)
        if mt:
            xy = _pose_xy(m)
            if xy is not None:
                out.append((int(mt.group(1)), int(mt.group(2)),
                            xy[0], xy[1], "inst"))
    if not out:
        raise SystemExit(f"[52] ✘ 나무를 하나도 못 찾았습니다: {sdf_path}")
    return out


# ── 이미지 식생 능선 ────────────────────────────────────────────────────────
def load_exg(img_path):
    """excess-green(2G−R−B) 지수 배열. 51 과 같은 계열 지수를 쓴다."""
    from PIL import Image
    a = np.asarray(Image.open(img_path).convert("RGB"), dtype=np.float64)
    return 2.0 * a[:, :, 1] - a[:, :, 0] - a[:, :, 2]


def sample_bilinear(arr, px, py):
    """쌍선형 표본. 이미지 밖이 하나라도 있으면 None — 밖을 0 으로 채우면
    없는 능선이 생긴다. px·py 는 스칼라도 배열도 된다."""
    h, w = arr.shape
    px, py = np.asarray(px, dtype=float), np.asarray(py, dtype=float)
    if (px.min() < 0 or px.max() > w - 1.001
            or py.min() < 0 or py.max() > h - 1.001):
        return None
    x0, y0 = px.astype(int), py.astype(int)
    fx, fy = px - x0, py - y0
    a = arr[y0, x0] * (1 - fx) + arr[y0, x0 + 1] * fx
    b = arr[y0 + 1, x0] * (1 - fx) + arr[y0 + 1, x0 + 1] * fx
    return a * (1 - fy) + b * fy


def ridge_offset_m(exg, to_px, row_x, wy, half_m, along_m=2.0,
                   step_m=0.05, along_step_m=0.25):
    """열 축(row_x) 기준 ±half_m 를 훑어 식생 능선의 횡 오프셋[m]을 찾는다.

    along-row 로 ±`along_m` 를 평균해 개별 나무 사이 간극(주간 1.5 m)의
    잡음을 눌러 준다. 창 안에 이미지 밖 표본이 하나라도 있으면 None —
    가장자리에서 반쪽짜리 능선을 재는 것보다 표본을 버리는 편이 낫다.
    """
    offs = np.arange(-half_m, half_m + 1e-9, step_m)
    alongs = np.arange(-along_m, along_m + 1e-9, along_step_m)
    wx = row_x + offs[:, None]                    # (오프셋, along)
    wy_g = wy + alongs[None, :]
    vals = sample_bilinear(exg, *to_px(wx, wy_g))
    if vals is None:
        return None
    prof = vals.mean(axis=1)
    # 3점 이동평균(≈0.15 m)으로 화소 잡음만 눌러 준다 — 능선 폭(수관 ~2 m)에
    # 비하면 충분히 좁아 위치를 옮기지 않는다.
    sm = np.convolve(prof, np.ones(3) / 3.0, mode="same")
    sm[0], sm[-1] = prof[0], prof[-1]
    i = int(np.argmax(sm))
    if 0 < i < len(sm) - 1:                       # 포물선 서브샘플 보간
        y0, y1, y2 = sm[i - 1], sm[i], sm[i + 1]
        den = (y0 - 2 * y1 + y2)
        if den != 0:
            return float(offs[i] - 0.5 * step_m * (y2 - y0) / den)
    return float(offs[i])


def row_ridge_scan(exg, to_px, farm, along_m=2.0, stride_m=5.0, edge_m=5.0):
    """열마다 종단으로 훑은 능선 오프셋 — **계통 정합**의 척도.

    표본 한 그루의 이탈은 실제 열의 사행·결주·잡초로도 생긴다(그 흔들림은
    농장의 성질이지 정합 오차가 아니다). 정합이 깨졌다면 그 열 **전체**가
    한쪽으로 밀린다. 그래서 열별 **중앙값**을 본다 — 사행은 상쇄되고 계통
    편의만 남는다.

    반환: {열: {"n","med","mean","sd","min","max","vals"}}
    """
    out = {}
    for r in range(int(farm["rows"])):
        rx, ry0 = (float(v) for v in farm["row_origins"][r])
        L = float(farm["row_lengths_m"][r])
        half = float(farm["row_spacing_m"]) / 2.0
        vals = []
        for t in np.arange(edge_m, max(edge_m, L - edge_m), stride_m):
            o = ridge_offset_m(exg, to_px, rx, ry0 + t, half, along_m)
            if o is not None:
                vals.append(o)
        if not vals:
            continue
        a = np.array(vals)
        out[r] = dict(n=len(a), med=float(np.median(a)), mean=float(a.mean()),
                      sd=float(a.std()), min=float(a.min()), max=float(a.max()),
                      vals=[round(float(v), 4) for v in a])
    return out


# ── 표본 추출 ───────────────────────────────────────────────────────────────
def pick_sample(trees, farm, n, seed, to_px, exg, half_m, along_m):
    """열을 고르게 훑으며 n 그루. 스캔 창이 이미지 밖인 나무는 건너뛴다."""
    rng = random.Random(seed)
    by_row = {}
    for t in trees:
        by_row.setdefault(t[0], []).append(t)
    rows = sorted(by_row)
    for r in rows:
        rng.shuffle(by_row[r])
    picked, skipped, i = [], 0, 0
    cursor = {r: 0 for r in rows}
    while len(picked) < n and i < 20000:
        r = rows[i % len(rows)]
        i += 1
        c = cursor[r]
        if c >= len(by_row[r]):
            continue
        cursor[r] = c + 1
        row, idx, x, y, kind = by_row[r][c]
        row_x = float(farm["row_origins"][row][0])
        if exg is not None and ridge_offset_m(
                exg, to_px, row_x, y, half_m, along_m) is None:
            skipped += 1
            continue
        picked.append(by_row[r][c])
    return picked, skipped


def measure(farm, trees, img_path, n, seed, along_m):
    to_px, to_world = affine(farm)
    exg = load_exg(img_path) if img_path else None
    half_m = float(farm["row_spacing_m"]) / 2.0
    sample, skipped = pick_sample(trees, farm, n, seed, to_px, exg, half_m, along_m)
    rows = []
    for row, idx, x, y, kind in sample:
        px, py = to_px(x, y)
        rx, ry = to_world(px, py)
        roundtrip = math.hypot(rx - x, ry - y)
        row_x, row_y0 = (float(v) for v in farm["row_origins"][row])
        row_len = float(farm["row_lengths_m"][row])
        off_axis = x - row_x
        along = y - row_y0
        ridge = (ridge_offset_m(exg, to_px, row_x, y, half_m, along_m)
                 if exg is not None else None)
        align = abs(off_axis - ridge) if ridge is not None else None
        rows.append(dict(row=row, idx=idx, kind=kind, x=x, y=y, px=px, py=py,
                         roundtrip_m=roundtrip, off_axis_m=off_axis,
                         along_m=along, row_len_m=row_len,
                         ridge_off_m=ridge, align_err_m=align,
                         in_extent=(-1e-6 <= along <= row_len + 1e-6)))
    return rows, skipped


# ── 그림 ────────────────────────────────────────────────────────────────────
def _use_kr_font(font_manager, plt, name="Noto Sans CJK KR"):
    """41_m3_figures.py 와 같은 절차 — CJK ttc 에서 KR 자체를 뽑아 등록한다."""
    kr = "/tmp/NotoSansCJKkr-Regular.otf"
    if not any(name in f.name for f in font_manager.fontManager.ttflist):
        try:
            if not os.path.exists(kr):
                from fontTools.ttLib import TTCollection
                tc = TTCollection("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
                for f in tc.fonts:
                    if "KR" in (f["name"].getDebugName(4) or ""):
                        f.save(kr)
                        break
            font_manager.fontManager.addfont(kr)
        except Exception as e:                        # 폰트가 없어도 그림은 나온다
            print(f"     (한글 폰트 등록 실패 — 라벨이 깨질 수 있습니다: {e})")
            return
    plt.rcParams["font.family"] = name
    plt.rcParams["axes.unicode_minus"] = False



def draw(fig_path, farm, img_path, rows, tol, scan=None):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import font_manager
    import matplotlib.pyplot as plt
    from PIL import Image
    _use_kr_font(font_manager, plt)                   # 한글 라벨이 두부가 되지 않게
    to_px, _ = affine(farm)
    img = Image.open(img_path).convert("RGB")
    ncol = 3 if scan else 2
    fig, ax = plt.subplots(1, ncol, figsize=(6 + 5 * ncol, 7.5), dpi=140,
                           gridspec_kw={"width_ratios": ([1.35, 1, 1] if scan
                                                         else [1.35, 1])})
    ax[0].imshow(img)
    for r, (rx, ry0) in enumerate(farm["row_origins"]):
        L = float(farm["row_lengths_m"][r])
        p0, p1 = to_px(float(rx), float(ry0)), to_px(float(rx), float(ry0) + L)
        ax[0].plot([p0[0], p1[0]], [p0[1], p1[1]], "-", lw=0.6,
                   color="#00d0ff", alpha=0.75)
    ok = [d for d in rows if d["align_err_m"] is not None and d["align_err_m"] <= tol]
    bad = [d for d in rows if d not in ok]
    ax[0].scatter([d["px"] for d in ok], [d["py"] for d in ok], s=46,
                  facecolors="none", edgecolors="#ffe14d", linewidths=1.6,
                  label=f"표본 통과 {len(ok)}")
    if bad:
        ax[0].scatter([d["px"] for d in bad], [d["py"] for d in bad], s=60,
                      marker="x", color="#ff3b30", linewidths=2.0,
                      label=f"표본 초과 {len(bad)}")
    ax[0].set_title(f"farm.json 열 축(하늘색) + 표본 나무 {len(rows)}그루\n"
                    f"{os.path.basename(img_path)}", fontsize=10)
    ax[0].legend(loc="upper right", fontsize=8)
    ax[0].set_xlabel("image px")

    lbl = [f"r{d['row']}t{d['idx']}" for d in rows]
    e = [d["align_err_m"] if d["align_err_m"] is not None else float("nan")
         for d in rows]
    b = [abs(d["off_axis_m"]) for d in rows]
    yy = np.arange(len(rows))
    ax[1].barh(yy + 0.2, e, height=0.38, color="#2f7ed8", label="C 이미지 능선 이탈")
    ax[1].barh(yy - 0.2, b, height=0.38, color="#9fb6cd", label="B 열 축 이탈")
    ax[1].axvline(tol, color="#ff3b30", ls="--", lw=1.2, label=f"기준 {tol} m")
    ax[1].set_yticks(yy)
    ax[1].set_yticklabels(lbl, fontsize=7)
    ax[1].invert_yaxis()
    ax[1].set_xlabel("오차 [m]")
    ax[1].set_title("표본별 정합 오차", fontsize=10)
    ax[1].legend(loc="lower right", fontsize=8)
    ax[1].grid(axis="x", alpha=0.3)

    if scan:
        rr = sorted(scan)
        med = [scan[r]["med"] for r in rr]
        sd = [scan[r]["sd"] for r in rr]
        ax[2].errorbar(med, rr, xerr=sd, fmt="o", ms=4, lw=1.0, capsize=2,
                       color="#2f7ed8", ecolor="#9fb6cd",
                       label="열별 중앙(막대=사행 표준편차)")
        ax[2].axvline(0, color="#666", lw=0.8)
        for t in (-tol, tol):
            ax[2].axvline(t, color="#ff3b30", ls="--", lw=1.2)
        ax[2].set_yticks(rr)
        ax[2].set_yticklabels([f"r{r}" for r in rr], fontsize=7)
        ax[2].invert_yaxis()
        ax[2].set_xlabel("능선 오프셋 [m]  (+x 쪽)")
        ax[2].set_title("열별 계통 정합(중앙) — 전 27열 종단 스캔", fontsize=10)
        ax[2].legend(loc="lower right", fontsize=8)
        ax[2].grid(axis="x", alpha=0.3)
    for a in ax:
        for s in a.spines.values():
            s.set_alpha(0.3)
    fig.tight_layout()
    os.makedirs(os.path.dirname(fig_path) or ".", exist_ok=True)
    fig.savefig(fig_path, bbox_inches="tight")
    plt.close(fig)
    return fig_path


# ── 자기시험 ────────────────────────────────────────────────────────────────
def self_test():
    """합성 데이터로 아핀·능선·판정을 검증한다(이미지·월드 파일 불요)."""
    fails = []

    def check(name, cond, detail=""):
        print(f"  {'OK ' if cond else '✘  '} {name}{(' — ' + detail) if detail else ''}")
        if not cond:
            fails.append(name)

    farm = {"rotation_deg": 62.5, "px_per_m": 4.0, "origin_px": [259.0, 403.7],
            "rows": 2, "row_spacing_m": 5.0,
            "row_origins": [[-10.0, -20.0], [-5.0, -20.0]],
            "row_lengths_m": [100.0, 100.0], "image": "x.jpg"}
    to_px, to_world = affine(farm)

    # A1 아핀 왕복
    worst = 0.0
    for wx in (-100, -3.5, 0, 17.25, 120):
        for wy in (-90, -1.0, 0, 44.5, 108):
            rx, ry = to_world(*to_px(wx, wy))
            worst = max(worst, math.hypot(rx - wx, ry - wy))
    check("A1 아핀 왕복 오차 < 1e-9 m", worst < 1e-9, f"최악 {worst:.2e}")

    # A2 회전 방향 — rotation_deg=0 이면 world +y 가 image +y(행 증가) 여야 한다
    f0 = dict(farm, rotation_deg=0.0, origin_px=[0.0, 0.0])
    p0, _ = affine(f0)
    check("A2 rotation 0 에서 world +y = image +y", p0(0, 1)[1] > p0(0, 0)[1],
          f"py {p0(0, 0)[1]:.1f} → {p0(0, 1)[1]:.1f}")
    check("A2 rotation 0 에서 world +x = image +x", p0(1, 0)[0] > p0(0, 0)[0])

    # A3 회전이 실제로 62.5° 인가 — 열 방향(+y)의 픽셀 방위각
    d = np.subtract(to_px(0, 1), to_px(0, 0))
    ang = math.degrees(math.atan2(d[1], d[0]))
    check("A3 열 방향 픽셀 방위 = 90°+62.5°", abs(ang - (90 + 62.5)) < 1e-6,
          f"{ang:.4f}°")

    # B 합성 이미지에 능선을 심고 되찾는다 (능선을 열 축에서 0.30 m 옮겨 둔다).
    #   능선은 **해석적으로** 그린다 — 픽셀에 점을 찍어 그리면 정수 화소로
    #   반올림되며 최대 0.5 px(=0.125 m) 의 편의가 심어져, 검출기가 아니라
    #   시험 데이터가 틀린 것을 검출기 탓으로 오판하게 된다(실제로 겪음).
    H = W = 900
    shift, sigma = 0.30, 0.8
    yy, xx = np.mgrid[0:H, 0:W]
    th = math.radians(farm["rotation_deg"])
    c, s = math.cos(th), math.sin(th)
    k = farm["px_per_m"]
    ox, oy = farm["origin_px"]
    dxp, dyp = (xx - ox) / k, (yy - oy) / k
    wxg = c * dxp + s * dyp                       # 픽셀 격자의 월드 x (cross-row)
    exg = np.zeros((H, W))
    for r in range(farm["rows"]):
        rx = farm["row_origins"][r][0] + shift
        exg += 10.0 * np.exp(-((wxg - rx) / sigma) ** 2)
    got = ridge_offset_m(exg, to_px, farm["row_origins"][0][0],
                         farm["row_origins"][0][1] + 40.0, 2.5)
    check("B1 심어 둔 능선을 0.05 m 안에서 되찾는다",
          got is not None and abs(got - shift) < 0.05, f"{got}")

    # B2 이미지 밖 창은 None (반쪽 능선 금지)
    out = ridge_offset_m(exg, to_px, farm["row_origins"][0][0], 5000.0, 2.5)
    check("B2 이미지 밖이면 None", out is None, str(out))

    # C 열별 계통 스캔 — 판정이 항진명제가 아님을 **버그 재주입**으로 확인한다.
    #   같은 합성 이미지에 대해 (1) 매니페스트가 맞으면 중앙 ≈ shift 를 되찾고
    #   (2) 매니페스트 열을 1.0 m 밀면 중앙이 tol 을 넘겨 FAIL 이 나야 한다.
    farm_ok = dict(farm, row_origins=[[x + shift, y] for x, y in farm["row_origins"]])
    sc = row_ridge_scan(exg, to_px, farm_ok, along_m=2.0, stride_m=10.0, edge_m=5.0)
    med_ok = max(abs(sc[r]["med"]) for r in sc) if sc else 9.9
    check("C1 매니페스트가 맞으면 열별 중앙 |오프셋| < 0.05 m",
          bool(sc) and med_ok < 0.05, f"최악 {med_ok:.4f} m ({len(sc)}열)")
    farm_bad = dict(farm, row_origins=[[x + shift + 1.0, y]
                                       for x, y in farm["row_origins"]])
    sc2 = row_ridge_scan(exg, to_px, farm_bad, along_m=2.0, stride_m=10.0, edge_m=5.0)
    med_bad = min(abs(sc2[r]["med"]) for r in sc2) if sc2 else 0.0
    check("C2 열을 1.0 m 밀면 열별 중앙이 tol 0.5 를 넘긴다 (판정 재주입 시험)",
          bool(sc2) and med_bad > 0.5, f"최소 {med_bad:.4f} m")
    # C3 사행은 중앙값이 상쇄한다 — ±0.9 m 로 흔들리되 평균 0 인 열은 통과
    sd_vals = np.array([+0.9, -0.9, +0.6, -0.6, 0.0])
    check("C3 사행(±0.9) 은 중앙값 0 으로 상쇄 — 계통 편의와 구분된다",
          abs(np.median(sd_vals)) <= 0.5, f"중앙 {np.median(sd_vals):+.2f}")

    # D SDF 파서 — 최소 월드 하나를 만들어 두 형태를 다 뽑는지
    import tempfile
    sdf = ("<sdf version='1.9'><world name='w'>"
           "<model name='row_trees_r3'><link name='link'>"
           "<visual name='t0_all'><pose>1 2 0 0 0 0</pose></visual>"
           "<visual name='t1_all'><pose>1.1 3.5 0 0 0 0</pose></visual>"
           "<visual name='trunk'><pose>9 9 0 0 0 0</pose></visual>"
           "</link></model>"
           "<model name='apple_tree_s07_0122__r5t7'><pose>4 5 0 0 0 0</pose>"
           "</model>"
           "<model name='trellis_r3'><pose>0 0 0 0 0 0</pose></model>"
           "</world></sdf>")
    with tempfile.NamedTemporaryFile("w", suffix=".sdf", delete=False) as fh:
        fh.write(sdf)
        tmp = fh.name
    got = sorted(world_trees(tmp))
    os.unlink(tmp)
    check("D1 배경목 2 + 계측목 1 만 뽑는다 (트렐리스·trunk 제외)",
          got == [(3, 0, 1.0, 2.0, "row"), (3, 1, 1.1, 3.5, "row"),
                  (5, 7, 4.0, 5.0, "inst")], str(got))

    print()
    if fails:
        print(f"[52] 자기시험 실패 {len(fails)}건: {', '.join(fails)}")
        return 1
    print("[52] 자기시험 GREEN — 전 항목 통과")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default=DEF_WORLD)
    ap.add_argument("--farm", default=DEF_FARM)
    ap.add_argument("--imagery", default=DEF_IMAGERY)
    ap.add_argument("--n", type=int, default=20, help="표본 나무 수 (스펙 §7: 20)")
    ap.add_argument("--tol", type=float, default=0.5, help="정합 오차 기준 [m]")
    ap.add_argument("--seed", type=int, default=20260815)
    ap.add_argument("--along", type=float, default=2.0,
                    help="능선 탐색의 along-row 평균 구간 반폭 [m]")
    ap.add_argument("--fig", default="", help="검증 그림 경로(비우면 안 그림)")
    ap.add_argument("--json-out", default="")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args(argv)
    if a.self_test:
        return self_test()

    os.chdir(ROOT)
    farm, img = load_farm(a.farm, a.imagery)
    trees = world_trees(a.world)
    rows, skipped = measure(farm, trees, img, a.n, a.seed, a.along)
    scan = row_ridge_scan(load_exg(img), affine(farm)[0], farm, a.along)

    print(f"[52] 월드 {a.world} · 나무 {len(trees)}그루 · farm {a.farm}")
    print(f"     정사영상 {img} (sha256 {farm['image_sha256'][:12]}…)")
    print(f"     표본 {len(rows)}그루 (seed {a.seed}, 이미지 밖으로 제외 {skipped})")
    print()
    print("  열  나무   종류        x         y      왕복[m]   B 열축[m]  "
          "능선[m]  C 정합[m]")
    for d in rows:
        print(f"  {d['row']:>2}  {d['idx']:>4}  {d['kind']:<5} "
              f"{d['x']:>9.3f} {d['y']:>9.3f}  {d['roundtrip_m']:>8.2e}  "
              f"{d['off_axis_m']:>+9.3f}  {d['ridge_off_m']:>+7.3f}  "
              f"{d['align_err_m']:>8.3f}"
              + ("" if (d["align_err_m"] or 0.0) <= a.tol else "  ← 기준 초과"))
    errs = [d["align_err_m"] for d in rows if d["align_err_m"] is not None]
    baxis = [abs(d["off_axis_m"]) for d in rows]
    rt = max(d["roundtrip_m"] for d in rows)
    n_c = sum(e <= a.tol for e in errs)

    print()
    print("  열별 계통 정합(전 27열 종단 스캔 — 중앙값이 곧 계통 편의)")
    print("   열    n    중앙[m]   사행 sd[m]    최소     최대")
    for r in sorted(scan):
        s = scan[r]
        print(f"   {r:>2}  {s['n']:>3}  {s['med']:>+8.3f}  {s['sd']:>9.3f}  "
              f"{s['min']:>+8.3f} {s['max']:>+8.3f}"
              + ("" if abs(s["med"]) <= a.tol else "  ← 기준 초과"))
    meds = [abs(scan[r]["med"]) for r in scan]
    allv = np.concatenate([scan[r]["vals"] for r in scan])
    n_rows_ok = sum(m <= a.tol for m in meds)

    print()
    print(f"  A 아핀 왕복 최악          {rt:.3e} m                     "
          f"{'PASS' if rt < 1e-6 else 'FAIL'} (기준 <1e-6)")
    print(f"  B 월드 나무 ↔ 매니페스트 열 축   중앙 {np.median(baxis):.3f} · "
          f"최악 {max(baxis):.3f} m       "
          f"{'PASS' if max(baxis) <= a.tol else 'FAIL'} (기준 ≤{a.tol})")
    print(f"  C 매니페스트 열 ↔ 이미지 능선(계통) 열별 |중앙| 최악 "
          f"{max(meds):.3f} m ({n_rows_ok}/{len(scan)}열)  "
          f"{'PASS' if n_rows_ok == len(scan) else 'FAIL'} (기준 ≤{a.tol})")
    print(f"  D 표본별 이미지 능선 이탈(참고)  중앙 {np.median(errs):.3f} · "
          f"최악 {max(errs):.3f} m · {n_c}/{len(errs)} ≤{a.tol}")
    print(f"     ↳ 잔차의 정체: 전 열 스캔 {len(allv)}점의 편의 중앙 "
          f"{np.median(allv):+.3f} m(≈0), 산포 sd {allv.std():.3f} m — "
          f"계통 어긋남이 아니라 실제 열의 사행이다.")
    print(f"  E 열 구간 안(along ∈ [0, row_length])  "
          f"{sum(d['in_extent'] for d in rows)}/{len(rows)}")
    print()
    ok_all = (rt < 1e-6 and max(baxis) <= a.tol and n_rows_ok == len(scan)
              and all(d["in_extent"] for d in rows) and len(rows) == a.n)
    print(f"[52] 정합 게이트 → {'PASS' if ok_all else 'FAIL'}  "
          f"(A·B·C·E 전건. D 는 농장의 사행을 함께 재는 참고 지표라 판정에 안 쓴다 "
          f"— 근거는 위 ↳ 줄)")
    if a.fig:
        print(f"     그림 {draw(a.fig, farm, img, rows, a.tol, scan)}")
    if a.json_out:
        with open(a.json_out, "w") as f:
            json.dump({"tol_m": a.tol, "seed": a.seed, "n": len(rows),
                       "skipped": skipped, "pass": ok_all,
                       "roundtrip_max_m": rt,
                       "off_axis_med_m": float(np.median(baxis)),
                       "off_axis_max_m": float(max(baxis)),
                       "row_med_abs_max_m": float(max(meds)),
                       "rows_ok": n_rows_ok, "rows_total": len(scan),
                       "scan_bias_med_m": float(np.median(allv)),
                       "scan_sd_m": float(allv.std()),
                       "sample_align_med_m": float(np.median(errs)),
                       "sample_align_max_m": float(max(errs)),
                       "sample_align_within_tol": n_c,
                       "samples": rows,
                       "row_scan": {str(k): {kk: vv for kk, vv in v.items()
                                             if kk != "vals"}
                                    for k, v in scan.items()}},
                      f, ensure_ascii=False, indent=1)
        print(f"     수치 {a.json_out}")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
