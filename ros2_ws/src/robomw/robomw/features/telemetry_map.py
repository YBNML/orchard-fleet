"""누적 장애물 격자 텔레메트리 (map 토픽).

관제 화면에는 '로봇을 막는 것'만 보이면 되므로 지면 기준 높이(AGL) 대역만
격자에 쌓는다. 점군을 그대로 보내지 않는 이유는 회선 예산이다 — 셀 인덱스만
보내면 프레임당 수 KB 로 끝난다.

**점군은 여기서 구독하지 않는다.** 이 기능은 ROS 를 모른다(코어 격리). 점군을
map 프레임 점 배열로 푸는 것은 기체측 어댑터의 일이고(orchard_sim/adapters/
ros_cloud.py), 이 기능은 `bb.extra["cloud_sinks"]` 에 콜백을 걸어 그 배열을
받는다. 그래서 라이다가 바뀌어도(점군 형식·TF 프레임 이름) 이 파일은 그대로다.
받는 배열은 (N, 3) map 프레임 좌표 + 그 프레임의 센서 높이(z)다 — AGL 대역·
격자 크기 같은 '무엇을 지도에 올릴지'는 여기서 정한다.
"""
from __future__ import annotations

import numpy as np

from robomw.core.base import Feature


class TelemetryMap(Feature):
    name = "telemetry_map"
    version = "1.0"
    summary = "누적 장애물 격자"
    topics = ("map",)

    def setup(self, ctx):
        super().setup(ctx)
        self.cell = float(ctx.param("map_cell", 0.25))
        self.max_points = int(ctx.param("map_max_points", 6000))
        self.period = float(ctx.param("map_period", 3.0))
        self.agl_lo = float(ctx.param("map_agl_min", 0.30))
        self.agl_hi = float(ctx.param("map_agl_max", 1.60))
        self.sensor_h = float(ctx.param("sensor_height", 0.645))
        self.cells = {}
        self._next = 0.0
        self._dirty = True
        # 점군 공급 신청. 어댑터는 이 목록을 프레임마다 호출한다 — 아무도
        # 신청하지 않으면 어댑터는 변환조차 하지 않는다(공짜로 뜨는 기능).
        ctx.bb.extra.setdefault("cloud_sinks", []).append(self.feed_points)

    def feed_points(self, pts, sensor_z):
        """map 프레임 점 배열 (N, 3) 을 격자에 쌓는다. 어댑터가 부른다."""
        agl = pts[:, 2] - sensor_z + self.sensor_h
        w = pts[(agl > self.agl_lo) & (agl < self.agl_hi)]
        if w.shape[0] == 0:
            return
        # numpy 정수는 JSON 으로 못 나간다 — 여기서 파이썬 int 로 바꿔둔다
        ij = np.floor(w[:, :2] / self.cell).astype(np.int32).tolist()
        with self.ctx.bb.lock:
            for k in map(tuple, ij):
                self.cells[k] = self.cells.get(k, 0) + 1
            self.ctx.bb.extra["map_cells"] = len(self.cells)
        self._dirty = True

    def telemetry(self, now):
        if now < self._next or not self._dirty:
            return ()
        self._next = now + self.period
        with self.ctx.bb.lock:
            top = sorted(self.cells.items(), key=lambda kv: -kv[1])[:self.max_points]
        self._dirty = False
        if not top:
            return ()
        flat = []
        for (ix, iy), _n in top:
            flat.append(ix)
            flat.append(iy)
        return (("map", dict(cell=self.cell, n=len(top), ij=flat)),)

    def teardown(self):
        try:
            self.ctx.bb.extra.get("cloud_sinks", []).remove(self.feed_points)
        except ValueError:
            pass
