"""
Livox MID-70 인터페이스 계약 — 순수 변환 로직 (설계서 §6)

ROS 노드와 분리해 두어 단위테스트가 가능하다. 설계서가 명시한 "필드 불일치 함정"
(CustomMsg 는 uint8 reflectivity, PointCloud2 는 float32 intensity 0~255) 때문에
두 경로를 모두 여기서 고정하고 양쪽 다 테스트한다.

계약:
    토픽      /livox/lidar
    frame_id  livox_frame
    타입      sensor_msgs/msg/PointCloud2, PointXYZRTLT 배치
                x, y, z      float32  (m)
                intensity    float32  (0.0~255.0)
                tag          uint8
                line         uint8    (MID-70 은 단일 레이저라 항상 0)
                timestamp    float64  (절대 시각 [s])
    주기      10 Hz
    부가      /livox/lidar_custom  livox_ros_driver2/msg/CustomMsg (FAST-LIO2 용)
"""
from __future__ import annotations

import numpy as np

# ── MID-70 실측 사양 (Mid-70 User Manual v1.2) ──────────────────────────────
FOV_HALF_ANGLE_RAD = 0.61436     # 원형 FOV 70.4° 의 반각 = 35.2°
RANGE_MIN_M = 0.05
RANGE_MAX_M = 90.0
LINE_ID = 0                      # 단일 레이저
POINT_RATE_HZ = 100_000

# PointXYZRTLT 바이트 배치 — 실제 Livox 드라이버와 동일하게 고정한다
PXYZRTLT_POINT_STEP = 32
PXYZRTLT_FIELDS = (
    # (이름, offset, datatype 상수, count)
    ("x", 0, 7, 1),           # FLOAT32
    ("y", 4, 7, 1),
    ("z", 8, 7, 1),
    ("intensity", 12, 7, 1),  # FLOAT32 0~255
    ("tag", 16, 2, 1),        # UINT8
    ("line", 17, 2, 1),       # UINT8
    ("timestamp", 24, 8, 1),  # FLOAT64  (8바이트 정렬 위해 offset 24)
)


def circular_fov_mask(x, y, z, half_angle_rad=FOV_HALF_ANGLE_RAD):
    """MID-70 의 **원형** FOV 마스크.

    gz 의 gpu_lidar 는 (방위각, 고도각) 정사각 격자를 쏘지만 실제 MID-70 은 원형이다.
    정사각 격자의 모서리 약 21%(=1-π/4)를 잘라내야 실제와 같은 점 분포가 된다.
    잘리는 모서리는 하필 과수원 로봇이 수간과 지면을 찾는 방향이라 그냥 두면 안 된다.
    """
    r_xy = np.hypot(x, y)
    az = np.arctan2(y, x)
    el = np.arctan2(z, r_xy)
    return np.hypot(az, el) <= half_angle_rad


def valid_range_mask(x, y, z, rmin=RANGE_MIN_M, rmax=RANGE_MAX_M):
    """무반사(NaN/inf)와 사양 밖 거리를 걸러낸다. gz 는 is_dense=False 로 내보낸다."""
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    d = np.sqrt(np.where(finite, x * x + y * y + z * z, 0.0))
    return finite & (d >= rmin) & (d <= rmax)


def scale_intensity(raw, in_max=None):
    """gz 의 intensity 를 Livox 규약인 0~255 float 로 옮긴다.

    gz gpu_lidar 의 intensity 스케일은 렌더 백엔드/재질에 따라 다르다.
    in_max 를 주면 그 값을 255 로 사상하고, 없으면 관측 최댓값으로 정규화한다.
    이미 0~255 범위로 보이면 건드리지 않는다.
    """
    r = np.asarray(raw, dtype=np.float32)
    r = np.where(np.isfinite(r), r, 0.0)
    if in_max is None:
        m = float(r.max()) if r.size else 0.0
        if m <= 1.0 + 1e-6:          # 0~1 정규화 값
            return np.clip(r * 255.0, 0.0, 255.0)
        if m <= 255.0 + 1e-3:        # 이미 0~255
            return np.clip(r, 0.0, 255.0)
        return np.clip(r / m * 255.0, 0.0, 255.0)
    if in_max <= 0:
        return np.zeros_like(r)
    return np.clip(r / float(in_max) * 255.0, 0.0, 255.0)


def per_point_timestamps(n, frame_start_s, period_s):
    """스캔 순서에 따라 프레임 구간에 점별 시각을 고르게 편다.

    FAST-LIO2 의 모션 디스큐잉이 점별 시각을 요구한다. gz 는 프레임 시각 하나만
    주므로 여기서 합성한다 — 실제 로제트 스캔의 시간 분포와는 다르며,
    그 차이는 설계서 §6 의 "실제 MID-70 과의 차이"에 기록돼 있다.
    """
    if n <= 0:
        return np.zeros(0, dtype=np.float64)
    frac = np.arange(n, dtype=np.float64) / float(n)
    return frame_start_s + frac * period_s


def pack_pointxyzrtlt(x, y, z, intensity, timestamps, tag=0, line=LINE_ID):
    """PointXYZRTLT 바이트 버퍼를 만든다 (PointCloud2.data 로 그대로 들어간다)."""
    n = x.shape[0]
    buf = np.zeros(n * PXYZRTLT_POINT_STEP, dtype=np.uint8)
    v = buf.view(np.uint8).reshape(n, PXYZRTLT_POINT_STEP)

    v[:, 0:4] = np.ascontiguousarray(x, dtype=np.float32).view(np.uint8).reshape(n, 4)
    v[:, 4:8] = np.ascontiguousarray(y, dtype=np.float32).view(np.uint8).reshape(n, 4)
    v[:, 8:12] = np.ascontiguousarray(z, dtype=np.float32).view(np.uint8).reshape(n, 4)
    v[:, 12:16] = np.ascontiguousarray(intensity, dtype=np.float32).view(np.uint8).reshape(n, 4)
    v[:, 16] = np.uint8(tag)
    v[:, 17] = np.uint8(line)
    v[:, 24:32] = np.ascontiguousarray(timestamps, dtype=np.float64).view(np.uint8).reshape(n, 8)
    return buf.tobytes()


def unpack_pointxyzrtlt(data, n):
    """pack_pointxyzrtlt 의 역변환 — 테스트와 소비자 검증용."""
    v = np.frombuffer(data, dtype=np.uint8).reshape(n, PXYZRTLT_POINT_STEP)
    return dict(
        x=v[:, 0:4].copy().view(np.float32).ravel(),
        y=v[:, 4:8].copy().view(np.float32).ravel(),
        z=v[:, 8:12].copy().view(np.float32).ravel(),
        intensity=v[:, 12:16].copy().view(np.float32).ravel(),
        tag=v[:, 16].copy(),
        line=v[:, 17].copy(),
        timestamp=v[:, 24:32].copy().view(np.float64).ravel(),
    )


def intensity_to_reflectivity(intensity):
    """float32 0~255  →  uint8 0~255.

    설계서가 지목한 필드 불일치 함정. 한쪽에 맞춰 짠 코드를 그대로 옮기면
    조용히 절단되므로 여기서 한 번만 변환하고 테스트로 고정한다.
    """
    return np.clip(np.rint(np.asarray(intensity, dtype=np.float64)), 0, 255).astype(np.uint8)


def offset_time_ns(timestamps, timebase_s):
    """CustomPoint.offset_time — timebase 기준 상대 시각 [ns], uint32."""
    d = (np.asarray(timestamps, dtype=np.float64) - float(timebase_s)) * 1e9
    return np.clip(np.rint(d), 0, np.iinfo(np.uint32).max).astype(np.uint32)


def synth_intensity_from_range(x, y, z, rmax=RANGE_MAX_M):
    """거리 기반 합성 intensity.

    **이것은 반사율이 아니다.** gz gpu_lidar 가 반사강도를 모델링하지 않아
    intensity 가 전부 0 으로 나오는 상황(2026-07-25 실측)에서, 시각화나
    거리 가중 휴리스틱을 위해 0 이 아닌 값이 필요할 때만 쓴다.
    재질 단서로 해석하는 알고리즘에는 쓰면 안 된다.
    """
    d = np.sqrt(x * x + y * y + z * z)
    v = 255.0 * np.clip(1.0 - d / float(rmax), 0.0, 1.0)
    return v.astype(np.float32)


def process_frame(x, y, z, intensity, frame_start_s, period_s,
                  intensity_in_max=None, apply_fov_mask=True,
                  intensity_mode="passthrough"):
    """원시 gz 점군 → 계약에 맞는 배열들. 브리지 노드가 이 함수만 부른다.

    intensity_mode
        passthrough : gz 값을 0~255 로 스케일만 (gz 가 0 이면 결과도 0) — 기본
        range       : 거리 기반 합성 (반사율 아님, synth_intensity_from_range 참조)

    반환: dict(x, y, z, intensity, timestamp, kept, total, emitted)
        emitted : FOV 마스크만 통과한 점 수. **발사 예산** 확인용이며,
                  kept(실제 반사 수신)와 구분된다 — 하늘로 쏜 광선은 무반사다.
    """
    x = np.asarray(x, dtype=np.float32).ravel()
    y = np.asarray(y, dtype=np.float32).ravel()
    z = np.asarray(z, dtype=np.float32).ravel()
    inten = np.asarray(intensity, dtype=np.float32).ravel()
    total = x.shape[0]

    fov = circular_fov_mask(x, y, z) if apply_fov_mask else np.ones(total, bool)
    m = valid_range_mask(x, y, z) & fov

    x, y, z, inten = x[m], y[m], z[m], inten[m]
    if intensity_mode == "range":
        inten = synth_intensity_from_range(x, y, z)
    else:
        inten = scale_intensity(inten, intensity_in_max)
    ts = per_point_timestamps(x.shape[0], frame_start_s, period_s)
    return dict(x=x, y=y, z=z, intensity=inten, timestamp=ts,
                kept=int(x.shape[0]), total=int(total),
                emitted=int(fov.sum()))
