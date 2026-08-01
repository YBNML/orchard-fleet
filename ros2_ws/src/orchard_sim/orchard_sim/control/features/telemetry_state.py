"""포즈·모드·궤적 텔레메트리 (state 토픽)."""
from __future__ import annotations

import time

from orchard_sim.control.base import Feature


class TelemetryState(Feature):
    name = "telemetry_state"
    version = "1.0"
    summary = "포즈·자세·모드·주행 궤적"
    topics = ("state",)

    def setup(self, ctx):
        super().setup(ctx)
        self.period = 1.0 / float(ctx.param("state_hz", 5.0))
        self.track_step = float(ctx.param("track_step", 0.25))
        self.track_max = int(ctx.param("track_max", 4000))
        self.track = []
        self._next = 0.0

    def tick(self, now):
        p = self.ctx.bb.pose
        if p is None:
            return None
        if not self.track or (abs(p[0] - self.track[-1][0])
                              + abs(p[1] - self.track[-1][1])) > self.track_step:
            self.track.append((round(p[0], 2), round(p[1], 2)))
            del self.track[:-self.track_max]
        return None

    def telemetry(self, now):
        if now < self._next:
            return ()
        self._next = now + self.period
        s = self.ctx.safety.snapshot()
        bb = self.ctx.bb
        pose = bb.pose
        mission = bb.extra.get("mission_status")
        mode = bb.extra.get("mode", "idle")
        if s["estop"]:
            mode = "estop"
        return (("state", dict(
            mode=mode, estop=s["estop"], estop_reason=s["estop_reason"],
            paused=s["paused"], gate=s["gate"],
            # 2단계 해제 — 관제 화면이 '무엇이 남았는지'를 그대로 보여준다
            estop_stage=s.get("estop_stage"),
            needs_remote_ok=s.get("needs_remote_ok"),
            needs_local_ok=s.get("needs_local_ok"),
            last_round_trip_s=s.get("last_round_trip_s"),
            service_mode=s.get("service_mode"),
            tilt_exposure_s=s.get("tilt_exposure_s"),
            pose=(None if pose is None else
                  dict(x=round(pose[0], 3), y=round(pose[1], 3),
                       yaw=round(pose[2], 4))),
            tilt=round(bb.tilt_deg, 1),
            lio=(None if bb.lio_pose is None else
                 [round(v, 3) for v in bb.lio_pose]),
            mission=mission,
            track=self.track[-1500:])),)
