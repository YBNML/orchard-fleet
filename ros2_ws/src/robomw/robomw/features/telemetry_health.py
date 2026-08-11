"""센서 주기·경고 텔레메트리 (health 토픽).

임계값을 코드에 박지 않고 파라미터로 뺐다. 센서 구성이 바뀌면(라이다 교체,
IMU 추가) 임계도 함께 바뀌어야 하는데, 그때 이 파일을 안 고치게 하려는 것이다.
"""
from __future__ import annotations

import math

from robomw.core.base import Feature


class TelemetryHealth(Feature):
    name = "telemetry_health"
    version = "1.0"
    summary = "센서 주기·SLAM 이격·경고"
    topics = ("health",)

    def setup(self, ctx):
        super().setup(ctx)
        self.period = 1.0 / float(ctx.param("health_hz", 1.0))
        # (블랙보드 rates 키, 표시명, 최소 Hz)
        self.limits = [
            ("lidar", "라이다", float(ctx.param("min_lidar_hz", 5.0))),
            ("imu", "IMU", float(ctx.param("min_imu_hz", 100.0))),
            ("lio", "SLAM 오도메트리", float(ctx.param("min_lio_hz", 5.0))),
        ]
        self.tilt_warn = float(ctx.param("tilt_warn_deg", 20.0))
        self._next = 0.0

    def telemetry(self, now):
        if now < self._next:
            return ()
        self._next = now + self.period
        bb = self.ctx.bb
        rates = dict(bb.rates)
        warn = []
        for key, label, lim in self.limits:
            hz = rates.get(key, 0.0)
            if hz < lim:
                warn.append(f"{label} 저조 ({hz:.0f} < {lim:.0f} Hz)")
        if bb.tilt_deg > self.tilt_warn:
            warn.append(f"기울기 {bb.tilt_deg:.0f}°")

        gap = None
        if bb.lio_pose and bb.pose:
            gap = round(math.hypot(bb.lio_pose[0] - bb.pose[0],
                                   bb.lio_pose[1] - bb.pose[1]), 2)
        pl = dict(warnings=warn, slam_gap=gap,
                  clients=bb.extra.get("clients", 0),
                  map_cells=bb.extra.get("map_cells", 0),
                  uptime=round(now, 1))
        for key, _label, _lim in self.limits:
            pl[f"{key}_hz"] = round(rates.get(key, 0.0), 1)
        return (("health", pl),)
