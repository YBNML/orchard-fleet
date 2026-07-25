"""
Livox 인터페이스 계약 단위테스트 (설계서 §6)

    cd ros2_ws/src/orchard_sim && python3 -m pytest test/ -v

설계서가 지목한 함정을 회귀로 고정한다:
  · 원형 FOV 마스크 — 정사각 격자의 모서리 21%(=1-π/4)를 잘라야 실제 MID-70 이 된다
  · 필드 불일치 — CustomMsg 는 uint8 reflectivity, PointCloud2 는 float32 intensity
    "한쪽에 맞춰 짠 코드를 옮기면 조용히 절단된다" → 양쪽 경로를 모두 테스트한다
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from orchard_sim import livox_contract as lc  # noqa: E402


# ── 원형 FOV 마스크 ─────────────────────────────────────────────────────────
def _square_grid(n=113, half=lc.FOV_HALF_ANGLE_RAD, dist=10.0):
    """gz gpu_lidar 가 쏘는 (방위각, 고도각) 정사각 격자를 재현한다."""
    a = np.linspace(-half, half, n)
    az, el = np.meshgrid(a, a, indexing="ij")
    x = dist * np.cos(el) * np.cos(az)
    y = dist * np.cos(el) * np.sin(az)
    z = dist * np.sin(el)
    return x.ravel(), y.ravel(), z.ravel()


def test_circular_mask_removes_square_corners():
    """정사각 격자에 원형 마스크를 씌우면 모서리가 잘려 약 77% 만 남는다.

    연속 극한은 π/4 = 78.54% 지만, 유한 격자에서는 가우스 원 문제의 경계항
    O(r)/r² = O(1/n) 만큼 낮게 나온다. 실제 운용 격자 113×113 에서는
    반지름 56 격자칸이므로 π·56²/113² = 77.16% 가 정확한 기댓값이다.
    이 값을 회귀로 고정한다 — 마스크가 빠지면 100%, 반각을 잘못 쓰면 크게 벗어난다.
    """
    n = 113
    x, y, z = _square_grid(n)
    kept = lc.circular_fov_mask(x, y, z).mean()
    r = (n - 1) / 2.0
    expected = math.pi * r * r / (n * n)          # = 0.7716
    assert kept == pytest.approx(expected, abs=0.005), \
        f"남은 비율 {kept:.4f} (기대 {expected:.4f}) — 원형 마스크가 모서리를 제대로 자르지 않았다"
    assert kept < 1.0, "마스크가 아무것도 자르지 않았다"


def test_circular_mask_converges_to_quarter_pi():
    """격자를 촘촘히 할수록 연속 극한 π/4 에 수렴해야 한다 (기하가 맞다는 확인)."""
    ratios = [lc.circular_fov_mask(*_square_grid(n)).mean() for n in (113, 401, 801)]
    assert ratios[0] < ratios[1] < ratios[2] < math.pi / 4, "단조 수렴하지 않는다"
    assert ratios[-1] == pytest.approx(math.pi / 4, abs=0.005)


def test_circular_mask_keeps_boresight_drops_corner():
    """중심축은 반드시 통과, 정사각 모서리는 반드시 탈락."""
    h = lc.FOV_HALF_ANGLE_RAD
    # 중심축
    assert lc.circular_fov_mask(np.array([10.0]), np.array([0.0]), np.array([0.0]))[0]
    # 모서리: az=el=h → 반경 h*sqrt(2) > h
    d = 10.0
    x = np.array([d * math.cos(h) * math.cos(h)])
    y = np.array([d * math.cos(h) * math.sin(h)])
    z = np.array([d * math.sin(h)])
    assert not lc.circular_fov_mask(x, y, z)[0]


def test_fov_half_angle_matches_spec():
    """MID-70 원형 FOV 70.4° → 반각 35.2°."""
    assert math.degrees(lc.FOV_HALF_ANGLE_RAD) == pytest.approx(35.2, abs=0.02)


# ── 무효점 제거 ─────────────────────────────────────────────────────────────
def test_valid_range_mask_drops_nonfinite_and_out_of_spec():
    x = np.array([1.0, np.nan, np.inf, 0.01, 200.0, 50.0], dtype=np.float32)
    y = np.zeros(6, np.float32)
    z = np.zeros(6, np.float32)
    m = lc.valid_range_mask(x, y, z)
    assert list(m) == [True, False, False, False, False, True], \
        "NaN/inf/최소거리미만/최대거리초과를 걸러야 한다"


# ── intensity 스케일 ────────────────────────────────────────────────────────
@pytest.mark.parametrize("raw,expect_max", [
    (np.array([0.0, 0.5, 1.0], np.float32), 255.0),      # 0~1 정규화 입력
    (np.array([0.0, 128.0, 255.0], np.float32), 255.0),  # 이미 0~255
    (np.array([0.0, 500.0, 1000.0], np.float32), 255.0),  # 범위 초과 → 정규화
])
def test_scale_intensity_lands_in_0_255(raw, expect_max):
    out = lc.scale_intensity(raw)
    assert out.min() >= 0.0 and out.max() == pytest.approx(expect_max, abs=1e-3)


def test_scale_intensity_explicit_max():
    out = lc.scale_intensity(np.array([0.0, 5.0, 10.0], np.float32), in_max=10.0)
    assert list(np.round(out, 3)) == [0.0, 127.5, 255.0]


# ── PointXYZRTLT 왕복 ───────────────────────────────────────────────────────
def test_pointxyzrtlt_roundtrip_preserves_values():
    n = 7
    x = np.arange(n, dtype=np.float32)
    y = np.arange(n, dtype=np.float32) * -2.0
    z = np.arange(n, dtype=np.float32) * 0.5
    inten = np.linspace(0, 255, n).astype(np.float32)
    ts = 1234.5 + np.arange(n) * 1e-3

    data = lc.pack_pointxyzrtlt(x, y, z, inten, ts)
    assert len(data) == n * lc.PXYZRTLT_POINT_STEP

    r = lc.unpack_pointxyzrtlt(data, n)
    np.testing.assert_allclose(r["x"], x)
    np.testing.assert_allclose(r["y"], y)
    np.testing.assert_allclose(r["z"], z)
    np.testing.assert_allclose(r["intensity"], inten)
    np.testing.assert_allclose(r["timestamp"], ts, rtol=0, atol=1e-9)
    assert (r["line"] == lc.LINE_ID).all(), "MID-70 은 단일 레이저라 line 이 항상 0"
    assert (r["tag"] == 0).all()


def test_pointxyzrtlt_field_layout_is_pinned():
    """바이트 배치가 바뀌면 하류 소비자가 조용히 깨진다. 회귀로 고정한다."""
    names = [f[0] for f in lc.PXYZRTLT_FIELDS]
    offs = [f[1] for f in lc.PXYZRTLT_FIELDS]
    assert names == ["x", "y", "z", "intensity", "tag", "line", "timestamp"]
    assert offs == [0, 4, 8, 12, 16, 17, 24]
    assert lc.PXYZRTLT_POINT_STEP == 32
    # timestamp 는 float64 이므로 8바이트 정렬이어야 한다
    assert offs[names.index("timestamp")] % 8 == 0


# ── 필드 불일치 함정 (설계서 §6) ────────────────────────────────────────────
def test_intensity_to_reflectivity_rounds_and_clips():
    """float32 0~255 → uint8. 잘못 옮기면 조용히 절단되는 지점."""
    inten = np.array([-5.0, 0.0, 0.4, 0.6, 127.5, 254.9, 300.0], np.float32)
    refl = lc.intensity_to_reflectivity(inten)
    assert refl.dtype == np.uint8
    assert list(refl) == [0, 0, 0, 1, 128, 255, 255]


def test_reflectivity_never_silently_truncates_high_values():
    """255 를 넘는 값이 0 으로 랩어라운드되지 않아야 한다 (uint8 캐스팅 함정)."""
    refl = lc.intensity_to_reflectivity(np.array([256.0, 511.0, 1000.0]))
    assert list(refl) == [255, 255, 255]


# ── 점별 시각 ───────────────────────────────────────────────────────────────
def test_per_point_timestamps_span_frame_period():
    ts = lc.per_point_timestamps(100, 10.0, 0.1)
    assert ts[0] == pytest.approx(10.0)
    assert ts[-1] < 10.1 and ts[-1] >= 10.099 - 1e-9
    assert np.all(np.diff(ts) > 0), "점별 시각은 단조증가해야 한다 (디스큐잉 전제)"


def test_offset_time_ns_is_uint32_relative():
    ts = lc.per_point_timestamps(5, 100.0, 0.1)
    off = lc.offset_time_ns(ts, 100.0)
    assert off.dtype == np.uint32
    assert off[0] == 0
    assert off[-1] == pytest.approx(0.08 * 1e9, rel=1e-6)


def test_offset_time_clips_negative_to_zero():
    """timebase 보다 이른 점이 들어와도 uint32 언더플로가 나면 안 된다."""
    off = lc.offset_time_ns(np.array([99.0, 100.0]), 100.0)
    assert off[0] == 0


# ── 프레임 처리 통합 ────────────────────────────────────────────────────────
def test_process_frame_matches_mid70_emission_budget():
    """113×113 @ 10 Hz 원형 마스크 → **발사** 예산이 100 kpts/s 여야 한다.

    사양의 100 kpts/s 는 발사율이지 수신율이 아니다. 하늘로 쏜 광선은 무반사이므로
    실제 씬에서 kept 는 이보다 적다 (과수원 실측 약 73 kpts/s). 둘을 구분해 검사한다.
    """
    x, y, z = _square_grid(113)
    inten = np.full(x.shape, 0.5, np.float32)
    r = lc.process_frame(x, y, z, inten, 0.0, 0.1)
    emit_rate = r["emitted"] / 0.1
    assert 90_000 <= emit_rate <= 110_000, \
        f"발사율 {emit_rate:,.0f} pts/s — MID-70 사양 100 kpts/s 와 10% 이상 어긋난다"
    # 이 합성 격자는 전부 유효 거리라 kept == emitted
    assert r["kept"] == r["emitted"]


def test_process_frame_separates_emitted_from_kept():
    """무반사(inf)가 섞이면 emitted 는 그대로, kept 만 줄어야 한다."""
    x, y, z = _square_grid(51)
    x = x.copy()
    x[::3] = np.inf                      # 3점 중 1점을 무반사로
    inten = np.ones(x.shape, np.float32)
    r = lc.process_frame(x, y, z, inten, 0.0, 0.1)
    assert r["kept"] < r["emitted"], "무반사가 kept 에서 빠져야 한다"
    assert r["emitted"] > 0


def test_intensity_mode_range_is_nonzero_and_monotone():
    """gz 가 intensity 0 만 줄 때 쓰는 합성 모드. 거리에 단조감소해야 한다."""
    d = np.array([1.0, 10.0, 45.0, 89.0], np.float32)
    zeros = np.zeros(4, np.float32)
    r = lc.process_frame(d, zeros, zeros, zeros, 0.0, 0.1, intensity_mode="range")
    assert r["intensity"].max() > 0, "합성 모드인데 전부 0 이다"
    assert np.all(np.diff(r["intensity"]) < 0), "거리가 멀수록 낮아져야 한다"
    assert r["intensity"].max() <= 255.0 and r["intensity"].min() >= 0.0


def test_intensity_mode_passthrough_keeps_zeros():
    """gz 가 0 을 주면 기본 모드는 0 을 유지한다 — 없는 정보를 지어내지 않는다."""
    d = np.array([5.0, 20.0], np.float32)
    zeros = np.zeros(2, np.float32)
    r = lc.process_frame(d, zeros, zeros, zeros, 0.0, 0.1)
    assert np.all(r["intensity"] == 0.0)


def test_process_frame_drops_invalid_before_masking():
    x = np.array([10.0, np.nan, 10.0], np.float32)
    y = np.array([0.0, 0.0, 0.0], np.float32)
    z = np.array([0.0, 0.0, 0.0], np.float32)
    r = lc.process_frame(x, y, z, np.ones(3, np.float32), 0.0, 0.1)
    assert r["kept"] == 2 and r["total"] == 3


def test_process_frame_without_mask_keeps_corners():
    x, y, z = _square_grid(51)
    inten = np.ones(x.shape, np.float32)
    on = lc.process_frame(x, y, z, inten, 0.0, 0.1, apply_fov_mask=True)
    off = lc.process_frame(x, y, z, inten, 0.0, 0.1, apply_fov_mask=False)
    assert off["kept"] > on["kept"], "마스크를 끄면 모서리가 남아야 한다"
    assert off["kept"] == x.size
