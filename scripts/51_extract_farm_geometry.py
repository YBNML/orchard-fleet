#!/usr/bin/env python3
"""실사 정사영상에서 과수원 열 기하를 반자동 추출해 farm.json 을 만든다

    python3 scripts/51_extract_farm_geometry.py --self-test
    python3 scripts/51_extract_farm_geometry.py \
        --image sim/assets/imagery/orchard_ortho.jpg \
        --meta sim/assets/imagery/orchard_ortho_meta.json \
        --out-json maps/orchard_real/farm.json \
        --out-overlay maps/orchard_real/geometry_overlay.png

무엇을 하나
    T1 이 확보한 정사영상(orchard_ortho.jpg)에서 사람이 다시 자로 재지 않고도
    열 축·간격·시작/끝을 뽑아 farm.json(스펙 §2, docs/superpowers/specs/
    2026-08-14-photoreal-world-design.md)을 만든다. 이미지 처리는 numpy/PIL
    만 쓴다(OpenCV 금지, scipy 도 안 씀 — 설치돼 있어도 규율 밖).

어떻게 하나 (Step 2 브리프 그대로)
    1. 식생 대비: excess-green 지수(2G-R-B) — T1 의 measure_rows.py 와 동일
       계열 지수(녹색 채널 우세, 조도 성분을 어느 정도 상쇄).
    2. 열 방향: 0.5° 격자 탐색으로 "열에 수직인 축으로 투영한 밝기 프로파일의
       분산"을 최대화하는 각도를 찾는다 — 열 방향이 맞을수록 나무열이 뚜렷한
       띠로 겹쳐 프로파일 분산이 커지고, 어긋날수록 옆 열과 섞여 뭉개진다.
    3. 간격: 그 최적 각도에서의 투영 프로파일을 자기상관해 첫 유의미한 피크의
       (서브픽셀 보간한) 지연(lag)을 열 간격으로 삼는다.
    4. 각 열 위치(row_origins): 같은 프로파일에서 극대점(피크)을 열 중심으로
       찾는다 — 격자로 이상화하지 않고 검출값을 그대로 쓴다(간격이 완전
       균일하지 않아도 그대로 반영됨).
    5. "온전한 열" 선별(rows/row_origins 에 남길 열) — 처음엔 along-row
       캐노피 임계로 직접 시작/끝을 재려 했으나, 실제 orchard_ortho.jpg 에
       적용해보니 개별 나무 캐노피 사이 간극·블록 내부 이질적 밝은 패치
       때문에 열마다 임계값에 따라 길이가 17m~135m 로 요동치는 잡음이 커
       버렸다(스크래치 디버그로 확인). 대신 순수 기하 기준으로 바꿨다 —
       각 열(대각선)이 "이미지 사각형 프레임"과 겹치는 along-row 가용 구간
       길이(avail_len_px, 캐노피와 무관)를 재서, 그 값이 후보 열 전체
       최댓값의 90% 이상인 열만 "온전"하다고 본다(모서리 열일수록 대각선이
       프레임을 일찍 벗어나 가용 구간이 자연히 짧아짐 — 크롭 밖에서 이어질
       수도 있으니 "경계에 잘린 열"로 제외). 상세 기준은 select_full_rows
       독스트링.
    6. 각 열의 시작/끝: 온전한 열들에서만, 그 가용 구간 양 끝과 실제
       캐노피(임계 판정, 개별 나무 간극을 메우는 폭으로 스무딩)가 시작되는
       지점 사이의 여백을 재 headland_m(중앙값)을 추정하고, 가용 구간
       양 끝에서 그 여백만큼 안쪽을 열의 시작/끝(row_length_m)으로 삼는다
       — canopy_edge_buffer 독스트링 참고.
    7. 월드 원점(origin_px)은 검출된 "온전한 열" 블록의 기하 중심 부근에 둔다
       (컨트롤러 지시).

좌표계 (axes_note 로 farm.json 에도 그대로 기록)
    월드→픽셀 아핀은
        px = origin_px + px_per_m * R(rotation_deg) @ (wx, wy)
        R(θ) = [[cosθ, -sinθ], [sinθ, cosθ]]  (표준 회전행렬, 반사 없음)
    즉 rotation_deg=0 일 때 world +x 는 image +x(오른쪽), world +y 는
    image +y(아래, 픽셀 행 증가) 방향과 나란하다. orchard_ortho_meta.json 의
    wms_request_bbox_epsg25831 주석대로 이 이미지의 맨 위 행(row 0)이 진짜
    지리적 "북"(최대 northing)이므로, **world +y 는 image +y 와 같은 방향
    (남쪽으로 내려가는 방향)이고 진짜 지리적 북이 아니다** — "북향"이라는
    이름이 관례로 쓰이더라도 이 매니페스트에서는 좌표계가 진북과 반대
    부호임을 소비자(T3 gen_world, T6 대시보드)가 반드시 알아야 한다.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

# ---------------------------------------------------------------------------
# 기본 지오메트리 도우미
# ---------------------------------------------------------------------------


def excess_green(arr_rgb: np.ndarray) -> np.ndarray:
    """식생 대비 지수(2G-R-B) — 녹색 채널 우세, 조도 성분 일부 상쇄."""
    a = arr_rgb.astype(np.float64)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    return 2.0 * g - r - b


def pixel_grid(h: int, w: int):
    ys, xs = np.mgrid[0:h, 0:w]
    return xs.astype(np.float64), ys.astype(np.float64)


def proj_s(xs, ys, theta_rad):
    """열간(cross-row, world x) 축 투영 좌표 — u_perp(θ)=(cosθ, sinθ)."""
    return xs * math.cos(theta_rad) + ys * math.sin(theta_rad)


def proj_t(xs, ys, theta_rad):
    """열대(along-row, world y) 축 투영 좌표 — u_row(θ)=(-sinθ, cosθ)."""
    return -xs * math.sin(theta_rad) + ys * math.cos(theta_rad)


def st_to_px(s, t, theta_rad):
    """(s,t) 투영좌표 → 픽셀좌표 (proj_s/proj_t 의 역변환, R(θ) 자체)."""
    c, sn = math.cos(theta_rad), math.sin(theta_rad)
    x = s * c - t * sn
    y = s * sn + t * c
    return x, y


def binned_profile(coord: np.ndarray, values: np.ndarray, bin_px: float = 1.0):
    """coord 를 bin_px 간격으로 양자화해 values 의 평균 프로파일을 만든다."""
    bins = np.round(coord / bin_px).astype(np.int64)
    bmin = int(bins.min())
    bins0 = (bins - bmin).ravel()
    nbins = int(bins0.max()) + 1
    sums = np.bincount(bins0, weights=values.ravel(), minlength=nbins)
    counts = np.bincount(bins0, minlength=nbins)
    mean = np.divide(sums, counts, out=np.zeros_like(sums), where=counts > 0)
    centers = (np.arange(nbins) + bmin) * bin_px
    return centers, mean, counts


# ---------------------------------------------------------------------------
# Step 2: 열 방향 (분산 최대화, 0.5° 격자)
# ---------------------------------------------------------------------------


def search_row_angle(idx, xs, ys, angle_lo=-90.0, angle_hi=90.0,
                      coarse_step=2.0, fine_step=0.5):
    """열에 수직인 축으로 투영한 프로파일의 분산이 최대가 되는 각도를 찾는다.

    coarse_step 격자로 전체 범위를 먼저 훑고(성능), best 주변 ±coarse_step 을
    fine_step(브리프 요구: 0.5°) 격자로 재탐색한다 — 최종 해상도는 항상
    fine_step. 탐색 범위는 -90..90(전 방향, 선의 방향은 mod 180 이라 이걸로
    모든 경우를 덮는다) — 애초 이 태스크의 지시문은 "열 방향이 대략 이미지
    수직축에서 약간 기울어짐"이라 했지만, 실제 orchard_ortho.jpg 를 육안
    확인하고 T1 의 독립적인 FFT 측정(scratchpad/measure_rows.py, row line
    orientation ~152 deg = 이 스크립트 좌표계로 환산하면 ~62-65 deg)과
    대조해보니 실제로는 대각선(수직에서 약 60도 이상 기울어짐)이었다 —
    좁은 범위로 탐색을 제한했다면 이 진짜 피크를 놓쳤을 것이다(실제로 처음
    -60..60 범위로 구현했을 때 진짜 피크가 범위 밖에 있어 경계값 60에
    걸려버리는 버그가 있었다). 브리프의 사전 설명보다 실측·교차검증을
    우선한다.
    """

    def variance_at(theta_deg):
        th = math.radians(theta_deg)
        coord = proj_s(xs, ys, th)
        _, mean, counts = binned_profile(coord, idx)
        good = counts > counts.max() * 0.5
        if good.sum() < 3:
            return -np.inf
        return float(mean[good].var())

    coarse_angles = np.arange(angle_lo, angle_hi + 1e-9, coarse_step)
    coarse_scores = np.array([variance_at(a) for a in coarse_angles])
    best_coarse = float(coarse_angles[int(np.argmax(coarse_scores))])

    fine_angles = np.arange(best_coarse - coarse_step, best_coarse + coarse_step + 1e-9,
                             fine_step)
    fine_angles = fine_angles[(fine_angles >= angle_lo) & (fine_angles <= angle_hi)]
    fine_scores = np.array([variance_at(a) for a in fine_angles])
    best_fine = float(fine_angles[int(np.argmax(fine_scores))])
    return best_fine


# ---------------------------------------------------------------------------
# Step 3: 간격 (자기상관 피크, 서브픽셀 보간)
# ---------------------------------------------------------------------------


def detect_spacing_px(centers, mean, counts, px_per_m, min_m=1.5, max_m=8.0):
    """cross-row 프로파일 자기상관의 첫 유의미한 피크 lag(px, 서브픽셀)."""
    good = counts > counts.max() * 0.5
    p = mean[good].astype(np.float64)
    p = p - p.mean()
    if p.std() == 0:
        raise ValueError("cross-row 프로파일이 평탄함 — 자기상관 불가")

    ac = np.correlate(p, p, mode="full")
    mid = len(ac) // 2
    ac_pos = ac[mid:]
    ac_pos = ac_pos / ac_pos[0]

    min_lag = max(1, int(round(min_m * px_per_m)))
    max_lag = min(len(ac_pos) - 2, int(round(max_m * px_per_m)))
    if max_lag <= min_lag + 1:
        raise ValueError("자기상관 탐색 대역이 프로파일 길이에 비해 좁음")

    band = ac_pos[min_lag:max_lag + 1]
    thresh = band.max() * 0.5
    peak_i = None
    for i in range(1, len(band) - 1):
        if band[i] >= band[i - 1] and band[i] >= band[i + 1] and band[i] >= thresh:
            peak_i = i
            break
    if peak_i is None:
        peak_i = int(np.argmax(band))
    lag = min_lag + peak_i

    # 서브픽셀 보간 (포물선 피팅) — self-test 허용오차(±0.07m=±0.28px)가
    # 정수 픽셀 해상도보다 좁아 필요.
    l, c, r = ac_pos[lag - 1], ac_pos[lag], ac_pos[lag + 1]
    denom = l - 2 * c + r
    delta = 0.5 * (l - r) / denom if denom != 0 else 0.0
    delta = max(-0.5, min(0.5, delta))
    return lag + delta


# ---------------------------------------------------------------------------
# Step 4: 열 위치(row_origins) — 격자 이상화 없이 검출값 그대로
# ---------------------------------------------------------------------------


def find_row_peaks(centers, mean, counts, spacing_px):
    good = counts > counts.max() * 0.5
    c = centers[good]
    m = mean[good]

    win = max(1, int(round(spacing_px / 6)))
    if win > 1:
        kernel = np.ones(win) / win
        m_s = np.convolve(m, kernel, mode="same")
    else:
        m_s = m

    baseline = float(np.median(m_s))
    spread = float(np.std(m_s))
    min_sep = max(2.0, spacing_px * 0.6)

    peak_idx = []
    n = len(m_s)
    for i in range(1, n - 1):
        if m_s[i] >= m_s[i - 1] and m_s[i] >= m_s[i + 1] and m_s[i] > baseline + 0.3 * spread:
            if peak_idx and (c[i] - c[peak_idx[-1]]) < min_sep:
                if m_s[i] > m_s[peak_idx[-1]]:
                    peak_idx[-1] = i
                continue
            peak_idx.append(i)
    return c[peak_idx]


# ---------------------------------------------------------------------------
# Step 5: 열 시작/끝 — 가용 구간(순수 기하) + 캐노피 임계(헤드랜드 여백 실측)
# ---------------------------------------------------------------------------
#
# 실제 이미지에 처음 적용해보니(§ 스크래치 디버그) 이 크롭은 캐노피 바로
# 가장자리까지 바짝 잘려 있어(T1 보고서 "가장자리에 소량 맥락만 포함") 열
# 내부의 캐노피 유무를 픽셀 단위로 임계 판정해 "온전한 열"을 가르는 방식은
# 너무 잡음이 심했다(개별 나무 캐노피 사이 간극·블록 중앙의 이질적 밝은
# 패치 때문에 같은 열도 임계값에 따라 길이가 17m~135m로 요동침). 대신
# 아래 두 단계로 나눈다:
#   1) row_available_range: 그 열(대각선)이 "사각형 이미지 프레임"과 실제로
#      겹치는 along-row 구간 — 순수 기하이며 캐노피와 무관해 잡음이 없다.
#      모서리에 가까운 열일수록 대각선이 프레임 밖으로 일찍 빠져나가 이
#      구간이 자연히 짧아진다.
#   2) select_full_rows: 그 가용 구간 길이가 후보 열 전체 최댓값의 90%
#      이상인 열만 "온전"하다고 본다 — 후보 대부분이 이미지 안에서 실제
#      캐노피로 덮여 있으므로(§ 오버레이 시각 확인), 가용 구간이 짧다는
#      것은 크롭 경계가 그 열을 실제로 잘랐다는 뜻이고, 크롭 밖에서 그
#      열이 계속되는지 이 이미지만으로는 알 수 없으므로 제외한다.
#   3) canopy_edge_buffer: "온전한" 열들에서만, 가용 구간 양 끝과 실제
#      캐노피(임계 판정)가 시작되는 지점 사이의 여백을 재서 headland_m 을
#      추정한다(중앙값 — 블록 내부 이질적 패치로 인한 이상치에 강건).


def row_available_range(t_coord, mask):
    """s_i 근방 띠가 이미지 사각형과 겹치는 along-row 범위(순수 기하)."""
    if mask.sum() < 10:
        return None
    tv = t_coord[mask]
    return float(tv.min()), float(tv.max())


def canopy_edge_buffer(idx, t_coord, mask, t_lo, t_hi, spacing_px, bin_px=1.0):
    """[t_lo,t_hi] 가용 구간 양 끝에서 실제 캐노피 임계까지의 여백(px)."""
    centers, mean, counts = binned_profile(t_coord[mask], idx[mask], bin_px)
    good = counts > 0
    c = centers[good]
    m = mean[good]
    order = np.argsort(c)
    c, m = c[order], m[order]
    if len(m) < 5:
        return None

    win = max(1, int(round(spacing_px)))  # 개별 나무 캐노피 간극을 메우는 폭
    if win > 1:
        kernel = np.ones(win) / win
        ms = np.convolve(m, kernel, mode="same")
    else:
        ms = m
    lo, hi = np.percentile(ms, [5, 95])
    if hi <= lo:
        return None
    thresh = lo + 0.35 * (hi - lo)
    canopy = np.where(ms > thresh)[0]
    if len(canopy) == 0:
        return None
    start_buf = c[canopy[0]] - t_lo
    end_buf = t_hi - c[canopy[-1]]
    return float(start_buf), float(end_buf)


def select_full_rows(row_records, avail_frac=0.9):
    """온전한 열만 남긴다.

    기준(브리프 요구 — 경계 잘린 열 제외 기준을 문서화, 위 docstring 참고):
    그 열의 가용 구간 길이(avail_len_px — 대각선이 이미지 사각형과 겹치는
    길이, 캐노피와 무관한 순수 기하)가 후보 열 전체 중 최댓값의 avail_frac
    (기본 90%) 이상인 열만 채택한다. 모서리 열은 대각선이 프레임을 일찍
    벗어나 avail_len_px 가 짧아지므로 자연히 제외된다 — 크롭 밖에서 그
    열이 계속되는지 알 수 없어 "경계에 잘린 열"로 취급.
    """
    if not row_records:
        return []
    max_avail = max(r["avail_len_px"] for r in row_records)
    return [r for r in row_records if r["avail_len_px"] >= avail_frac * max_avail]


# ---------------------------------------------------------------------------
# 전체 파이프라인
# ---------------------------------------------------------------------------


def extract_geometry(arr_rgb: np.ndarray, px_per_m: float, min_spacing_m=1.5,
                      max_spacing_m=8.0):
    """이미지 배열(H,W,3) → 기하 dict. farm.json/오버레이 둘 다 이걸 소비."""
    h, w = arr_rgb.shape[:2]
    idx = excess_green(arr_rgb)
    xs, ys = pixel_grid(h, w)

    theta_deg = search_row_angle(idx, xs, ys)
    theta_rad = math.radians(theta_deg)

    s_coord = proj_s(xs, ys, theta_rad)
    t_coord = proj_t(xs, ys, theta_rad)
    s_centers, s_mean, s_counts = binned_profile(s_coord, idx)

    spacing_px = detect_spacing_px(s_centers, s_mean, s_counts, px_per_m,
                                    min_m=min_spacing_m, max_m=max_spacing_m)
    spacing_m = spacing_px / px_per_m

    row_s_positions = find_row_peaks(s_centers, s_mean, s_counts, spacing_px)

    row_records = []
    for s_i in row_s_positions:
        mask = np.abs(s_coord - float(s_i)) < spacing_px * 0.35
        avail = row_available_range(t_coord, mask)
        if avail is None:
            continue
        t_lo, t_hi = avail
        row_records.append({
            "s": float(s_i),
            "mask": mask,
            "avail_t_lo": t_lo,
            "avail_t_hi": t_hi,
            "avail_len_px": t_hi - t_lo,
        })

    full_rows = select_full_rows(row_records)

    # headland_m 실측 — "온전한" 열에서만 가용구간 끝~실제 캐노피 여백을 재고
    # 중앙값을 쓴다(블록 내부 이질적 패치로 인한 개별 이상치에 강건).
    buffers = []
    for r in full_rows:
        b = canopy_edge_buffer(idx, t_coord, r["mask"], r["avail_t_lo"], r["avail_t_hi"],
                                spacing_px)
        if b is not None:
            buffers.append(b[0])
            buffers.append(b[1])
    headland_px = float(np.median(buffers)) if buffers else 0.0

    for r in full_rows:
        r["t_start"] = r["avail_t_lo"] + headland_px
        r["t_end"] = r["avail_t_hi"] - headland_px
        r["length_px"] = r["t_end"] - r["t_start"]

    for r in row_records:
        r.pop("mask", None)

    return {
        "theta_deg": theta_deg,
        "spacing_px": spacing_px,
        "spacing_m": spacing_m,
        "s_profile": (s_centers, s_mean, s_counts),
        "row_records": row_records,
        "full_rows": full_rows,
        "headland_px": headland_px,
        "headland_m": headland_px / px_per_m,
    }


# ---------------------------------------------------------------------------
# farm.json 조립
# ---------------------------------------------------------------------------


def build_farm_manifest(geo, image_name, px_per_m, image_sha256, headland_m,
                         tree_spacing_m, image_w, image_h):
    theta_rad = math.radians(geo["theta_deg"])
    full_rows = sorted(geo["full_rows"], key=lambda r: r["s"])
    if not full_rows:
        raise ValueError("온전한 열이 하나도 검출되지 않음")

    s_center = float(np.mean([r["s"] for r in full_rows]))
    t_starts = [r["t_start"] for r in full_rows]
    t_ends = [r["t_end"] for r in full_rows]
    t_center = float((np.median(t_starts) + np.median(t_ends)) / 2.0)

    origin_px_xy = st_to_px(s_center, t_center, theta_rad)
    origin_px = [float(origin_px_xy[0]), float(origin_px_xy[1])]

    def to_world(s, t):
        wx = (s - s_center) / px_per_m
        wy = (t - t_center) / px_per_m
        return [wx, wy]

    row_origins = [to_world(r["s"], r["t_start"]) for r in full_rows]
    row_lengths_m = [(r["t_end"] - r["t_start"]) / px_per_m for r in full_rows]
    row_length_m = float(np.median(row_lengths_m))

    # bounds_m = 이미지 전체(0,0)~(image_w,image_h) 픽셀 사각형을 이 아핀으로
    # 월드좌표에 투영한 바운딩 박스 — 컨트롤러 지시: 열 캐노피+headland_m 만의
    # 타이트한 박스가 아니라, 이미지에 담긴 나지 여백·농로까지 포함해 T3/T4 가
    # 미션 스탠드포인트·선회 패드를 놓을 여지를 준다(§bounds_note).
    corners_px = [(0.0, 0.0), (float(image_w), 0.0),
                  (float(image_w), float(image_h)), (0.0, float(image_h))]
    corners_world = [to_world(proj_s(cx, cy, theta_rad), proj_t(cx, cy, theta_rad))
                      for cx, cy in corners_px]
    xs_full = [c[0] for c in corners_world]
    ys_full = [c[1] for c in corners_world]
    min_x, max_x = min(xs_full), max(xs_full)
    min_y, max_y = min(ys_full), max(ys_full)

    manifest = {
        "image": image_name,
        "px_per_m": px_per_m,
        "origin_px": origin_px,
        "rotation_deg": round(geo["theta_deg"], 3),
        "rows": len(full_rows),
        "row_spacing_m": round(geo["spacing_m"], 4),
        "row_length_m": round(row_length_m, 3),
        "tree_spacing_m": tree_spacing_m,
        "row_origins": [[round(x, 4), round(y, 4)] for x, y in row_origins],
        "headland_m": headland_m,
        "bounds_m": [[round(min_x, 3), round(min_y, 3)], [round(max_x, 3), round(max_y, 3)]],
        "terrain": "flat",
        "image_sha256": image_sha256,
        "axes_note": (
            "px = origin_px + px_per_m * R(rotation_deg) @ (wx, wy), "
            "R(theta) = [[cos,-sin],[sin,cos]] (표준 회전, 반사 없음). "
            "rotation_deg=0 이면 world +x = image +x(오른쪽), "
            "world +y = image +y(아래, 픽셀 행 증가) 와 같은 방향. "
            "orchard_ortho_meta.json 의 wms_request_bbox_epsg25831 에 따르면 "
            "이 이미지의 row 0(맨 위)이 진짜 지리적 북(최대 northing)이다 — "
            "따라서 world +y 는 image +y 와 나란하므로 진짜 지리적 북이 "
            "아니라 남쪽으로 내려가는 방향이다. row_origins 는 각 열에서 "
            "t(along-row) 값이 더 작은 끝점(이미지 위쪽에 가까운 끝, 즉 이 "
            "규약상 world y 가 더 작은 끝)이며, 열은 거기서 +y 로 "
            "row_length_m 만큼 뻗는다. T3/T6 는 이 부호를 그대로 따라야 "
            "하며, 진짜 지리적 북이 필요하면 별도로 wy 를 뒤집어야 한다."
        ),
        "bounds_note": (
            "bounds_m 은 (row_origins/headland_m 기반의 타이트한 열 경계가 "
            "아니라) orchard_ortho.jpg 이미지 전체 픽셀 사각형 "
            "(0,0)~(image_w,image_h) 을 이 아핀으로 월드좌표에 투영한 "
            "바운딩 박스다 — 이미지에는 열 캐노피 바깥으로 나지 여백·농로·"
            "인접 휴경지가 더 담겨 있다(geometry_overlay.png 참고). T3/T4 가 "
            "미션 스탠드포인트·선회 패드를 그 여백에 배치할 수 있도록 "
            "일부러 넉넉하게 잡았다. **headland_m 과 혼동하지 말 것**: "
            "headland_m 은 그와 전혀 다른, 훨씬 좁은 개념이다 — '온전한 "
            "열'에서 실측한 '실제 캐노피 가장자리'와 '그 열이 이미지 "
            "프레임과 만나는 지점' 사이 거리일 뿐(row_selection_note "
            "참고), bounds_m 가장자리까지의 여유를 뜻하지 않는다. 열 자체의 "
            "타이트한 경계가 필요하면 row_origins ± row_length_m(+y 방향) "
            "± row_spacing_m/2(x 방향)로 별도 계산하라."
        ),
        "row_selection_note": (
            "rows/row_origins 는 '온전한 열'만 포함한다 — 기준(순수 기하, "
            "캐노피 잡음 배제): 그 열(대각선)이 이미지 사각형 프레임과 "
            "겹치는 along-row 가용 구간 길이가 후보 열 전체 최댓값의 90% "
            "이상인 열만 채택(모서리 열은 대각선이 프레임을 일찍 벗어나 "
            "가용 구간이 짧아짐 — 크롭 밖 연속 여부를 알 수 없어 제외). "
            "row_length_m/headland_m 은 그 온전한 열들에서만 캐노피 임계로 "
            "추가 실측(스크립트 canopy_edge_buffer 참고). row_origins "
            "좌표(각 열의 cross-row 위치)는 검출된 실측 위치 그대로이며 "
            "격자로 이상화하지 않았다(열 간격이 완전히 균일하지 않을 수 "
            "있음)."
        ),
    }
    return manifest


# ---------------------------------------------------------------------------
# 오버레이 렌더
# ---------------------------------------------------------------------------


def render_overlay(arr_rgb, geo, manifest, out_path):
    h, w = arr_rgb.shape[:2]
    theta_rad = math.radians(geo["theta_deg"])
    im = Image.fromarray(arr_rgb.astype(np.uint8), mode="RGB").convert("RGB")
    draw = ImageDraw.Draw(im)

    full_s = {round(r["s"], 3) for r in geo["full_rows"]}
    for r in geo["row_records"]:
        is_full = round(r["s"], 3) in full_s
        if is_full:
            color = (255, 60, 60)
            t0, t1 = r["t_start"], r["t_end"]
        else:
            # 제외된(경계 잘린) 열 — 실제 채택된 구간이 없으니 가용 구간
            # 전체(순수 기하)를 옅은 색으로 참고 표시만 한다.
            color = (255, 210, 0)
            t0, t1 = r["avail_t_lo"], r["avail_t_hi"]
        x0, y0 = st_to_px(r["s"], t0, theta_rad)
        x1, y1 = st_to_px(r["s"], t1, theta_rad)
        draw.line([(x0, y0), (x1, y1)], fill=color, width=2)
        # 끝점 표시
        for (px_, py_) in ((x0, y0), (x1, y1)):
            draw.ellipse([px_ - 3, py_ - 3, px_ + 3, py_ + 3], outline=color, width=1)

    # headland 경계(열 시작/끝 공통 대역) — 파란 점선 느낌으로 짧은 세그먼트 반복
    if geo["full_rows"]:
        t_starts = [r["t_start"] for r in geo["full_rows"]]
        t_ends = [r["t_end"] for r in geo["full_rows"]]
        s_vals = [r["s"] for r in geo["full_rows"]]
        s_lo, s_hi = min(s_vals) - geo["spacing_m"] * manifest["px_per_m"] / 2, \
            max(s_vals) + geo["spacing_m"] * manifest["px_per_m"] / 2
        for t_val, label in ((float(np.median(t_starts)), "row start"),
                              (float(np.median(t_ends)), "row end")):
            x0, y0 = st_to_px(s_lo, t_val, theta_rad)
            x1, y1 = st_to_px(s_hi, t_val, theta_rad)
            draw.line([(x0, y0), (x1, y1)], fill=(60, 140, 255), width=2)

    # bounds_m 블록 경계(월드 아핀으로 픽셀에 되돌려 그림) — 초록 사각형
    (bx0, by0), (bx1, by1) = manifest["bounds_m"]
    corners_world = [(bx0, by0), (bx1, by0), (bx1, by1), (bx0, by1)]
    px_per_m = manifest["px_per_m"]
    ox, oy = manifest["origin_px"]

    def world_to_px(wx, wy):
        c, sn = math.cos(theta_rad), math.sin(theta_rad)
        dx = px_per_m * (wx * c - wy * sn)
        dy = px_per_m * (wx * sn + wy * c)
        return ox + dx, oy + dy

    corners_px = [world_to_px(wx, wy) for wx, wy in corners_world]
    draw.line(corners_px + [corners_px[0]], fill=(50, 220, 90), width=2)

    # origin 십자
    draw.line([(ox - 10, oy), (ox + 10, oy)], fill=(0, 255, 255), width=2)
    draw.line([(ox, oy - 10), (ox, oy + 10)], fill=(0, 255, 255), width=2)

    n_full = len(geo["full_rows"])
    n_excl = len(geo["row_records"]) - n_full
    legend = [
        ("red = full row (used)", (255, 60, 60)),
        ("yellow = excluded (boundary-clipped)", (255, 210, 0)),
        ("blue = row start/end (headland edge)", (60, 140, 255)),
        ("green = bounds_m", (50, 220, 90)),
        ("cyan + = origin_px (world 0,0)", (0, 255, 255)),
    ]
    ly = 4
    for text, color in legend:
        draw.rectangle([0, ly - 2, w, ly + 12], fill=(0, 0, 0))
        draw.text((4, ly), text, fill=color)
        ly += 14

    caption = (f"rows(full)={n_full} excluded={n_excl} "
               f"spacing={geo['spacing_m']:.3f}m rot={geo['theta_deg']:.2f}deg "
               f"headland={manifest['headland_m']:.2f}m row_len={manifest['row_length_m']:.1f}m "
               f"origin_px=({ox:.1f},{oy:.1f})")
    draw.rectangle([0, h - 16, w, h], fill=(0, 0, 0))
    draw.text((3, h - 14), caption, fill=(255, 255, 255))

    im.save(out_path)


# ---------------------------------------------------------------------------
# self-test (합성 이미지)
# ---------------------------------------------------------------------------


def make_synthetic_image(spacing_px=14.0, angle_deg=7.0, px_per_m=4.0,
                          base_w=340, base_h=560, headland_frac=0.15, seed=7):
    """세로 줄무늬(간격 spacing_px)를 만들어 angle_deg 만큼 회전한 합성 이미지.

    실제 열 방향 검출과는 독립적인 경로(PIL.Image.rotate)로 회전을 걸어
    self-test 가 진짜 블랙박스 검증이 되게 한다(검출 공식을 그대로 재사용해
    생성하면 버그가 상쇄될 수 있음).
    """
    rng = np.random.default_rng(seed)
    canopy = np.array([55, 120, 45], dtype=np.float64)
    soil = np.array([150, 120, 90], dtype=np.float64)

    base = np.empty((base_h, base_w, 3), dtype=np.float64)
    col_phase = (np.arange(base_w) % spacing_px) < (spacing_px * 0.5)
    for x in range(base_w):
        base[:, x, :] = canopy if col_phase[x] else soil
    noise = rng.normal(0, 6.0, size=base.shape)
    base = np.clip(base + noise, 0, 255)

    # headland: 위/아래 일부는 무조건 나지(soil) — along-row 임계 로직도
    # 자연스럽게 같이 돈다(self-test 는 spacing/rotation 만 단언하지만
    # 파이프라인 전체가 죽지 않아야 함).
    hl = int(base_h * headland_frac)
    base[:hl, :, :] = soil
    base[-hl:, :, :] = soil

    base_img = Image.fromarray(base.astype(np.uint8), mode="RGB")
    fill = tuple(int(v) for v in soil)
    rotated = base_img.rotate(angle_deg, expand=True, resample=Image.BICUBIC,
                               fillcolor=fill)
    arr = np.asarray(rotated).astype(np.uint8)
    return arr, px_per_m


def run_self_test():
    print("[self-test] 합성 줄무늬 이미지(간격14px=3.5m@4px/m, 회전7deg) 생성")
    true_angle = 7.0
    arr, px_per_m = make_synthetic_image(spacing_px=14.0, angle_deg=true_angle,
                                          px_per_m=4.0)
    geo = extract_geometry(arr, px_per_m)

    print(f"[self-test] 복원 rotation_deg={geo['theta_deg']:.3f} "
          f"(기대: {true_angle} 근방)")
    print(f"[self-test] 복원 row_spacing_m={geo['spacing_m']:.4f} (기대: 3.5 근방)")
    print(f"[self-test] 검출 열 수(전체 후보)={len(geo['row_records'])}, "
          f"온전한 열={len(geo['full_rows'])}")

    ok = True
    if not (3.43 <= geo["spacing_m"] <= 3.57):
        print(f"[self-test] FAIL row_spacing_m {geo['spacing_m']:.4f} "
              f"not in [3.43,3.57]")
        ok = False
    # rotation_deg 는 부호가 make_synthetic_image 의 PIL.rotate 관례에 따라
    # +angle 또는 -angle 로 나올 수 있다 — 절대값으로 판정하되 부호 자체는
    # 아래에서 별도 출력해 실제 이미지 적용 시 관례를 고정한다.
    measured = geo["theta_deg"]
    if not (6.5 <= abs(measured) <= 7.5):
        print(f"[self-test] FAIL rotation_deg {measured:.3f} abs not in [6.5,7.5]")
        ok = False

    if ok:
        print("[self-test] GREEN — row_spacing_m, rotation_deg 복원 성공")
        print(f"[self-test] 부호 관례: PIL.Image.rotate(+{true_angle}) 이미지에서 "
              f"검출된 theta_deg 부호={'+' if measured >= 0 else '-'} "
              "(실제 이미지 rotation_deg 도 이 관례를 그대로 따름)")
    else:
        print("[self-test] RED")
    return ok


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def sha256_of_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--image", default="sim/assets/imagery/orchard_ortho.jpg")
    ap.add_argument("--meta", default="sim/assets/imagery/orchard_ortho_meta.json")
    ap.add_argument("--out-json", default="maps/orchard_real/farm.json")
    ap.add_argument("--out-overlay", default="maps/orchard_real/geometry_overlay.png")
    ap.add_argument("--headland-m", type=float, default=None,
                     help="비우면 이미지에서 실측한 값 사용")
    ap.add_argument("--tree-spacing-m", type=float, default=1.50,
                     help="이미지에서 개별 나무를 검출하지 않으므로(스펙 §3) "
                          "기존 계단식 월드 기본값(gen_world DEFAULTS)을 이어씀")
    ap.add_argument("--write", action="store_true",
                     help="farm.json/오버레이를 실제로 쓴다 (기본은 dry-run — "
                          "승인 게이트: 오버레이만 만들고 멈춘다)")
    args = ap.parse_args()

    if args.self_test:
        ok = run_self_test()
        sys.exit(0 if ok else 1)

    if not os.path.exists(args.image):
        print(f"[extract] 이미지 없음: {args.image}", file=sys.stderr)
        sys.exit(2)
    with open(args.meta) as f:
        meta = json.load(f)

    image_sha256 = sha256_of_file(args.image)
    if meta.get("sha256") and meta["sha256"] != image_sha256:
        print(f"[extract] 경고: meta sha256({meta['sha256']}) != 실제 파일 sha256"
              f"({image_sha256})", file=sys.stderr)

    px_per_m = round(1.0 / meta["gsd_m"], 6)
    im = Image.open(args.image).convert("RGB")
    arr = np.asarray(im)

    print(f"[extract] 이미지={args.image} size={im.size} px_per_m={px_per_m}")
    geo = extract_geometry(arr, px_per_m)
    print(f"[extract] rotation_deg={geo['theta_deg']:.3f} "
          f"row_spacing_m={geo['spacing_m']:.4f}")
    print(f"[extract] 후보 열={len(geo['row_records'])} "
          f"온전한 열={len(geo['full_rows'])}")

    if not geo["full_rows"]:
        print("[extract] 온전한 열이 하나도 없음 — 중단", file=sys.stderr)
        sys.exit(1)

    # headland_m: --headland-m 없으면 extract_geometry 가 실측한 값을 쓴다
    # (온전한 열들의 '가용 구간 끝'~'실제 캐노피 시작' 여백의 중앙값 —
    # canopy_edge_buffer 독스트링 참고). 이 크롭은 T1 이 캐노피 바로
    # 가장자리까지 바짝 잘라서(§ 리포트) 값이 작게(수 m) 나올 수 있다 —
    # gen_world DEFAULTS 의 계단식 월드 기본값(6.0m)보다 작다면 T3 가 실제
    # 선회에 필요한 여유를 이 값 그대로 쓸지 판단해야 한다(findings 인계).
    if args.headland_m is not None:
        headland_m = args.headland_m
        headland_source = "--headland-m 인자로 지정"
    else:
        headland_m = round(geo["headland_m"], 3)
        headland_source = ("이미지 실측 — 온전한 열의 '가용 구간 끝'과 실제 "
                            "캐노피 시작 사이 여백의 중앙값(canopy_edge_buffer)")
    print(f"[extract] headland_m={headland_m} ({headland_source})")

    manifest = build_farm_manifest(geo, os.path.basename(args.image), px_per_m,
                                    image_sha256, headland_m, args.tree_spacing_m,
                                    im.size[0], im.size[1])

    os.makedirs(os.path.dirname(args.out_overlay) or ".", exist_ok=True)
    render_overlay(arr, geo, manifest, args.out_overlay)
    print(f"[extract] 오버레이 저장: {args.out_overlay}")

    print(json.dumps(manifest, ensure_ascii=False, indent=2))

    if args.write:
        os.makedirs(os.path.dirname(args.out_json) or ".", exist_ok=True)
        with open(args.out_json, "w") as f:
            json.dump(manifest, f, ensure_ascii=False, indent=2)
            f.write("\n")
        print(f"[extract] farm.json 저장: {args.out_json}")
    else:
        print("[extract] --write 없이 실행됨 — farm.json 미저장(승인 게이트). "
              "오버레이를 확인 후 --write 로 재실행하라.")


if __name__ == "__main__":
    main()
