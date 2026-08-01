"""
원격 수동 조종 기능.

데드맨은 여기서 구현하지 않는다. 속도 **요청**만 내고, 갱신이 끊기면 요청을
내지 않는다. 최종 판정은 안전 조정자가 한다 — 그래야 기능을 잘못 짜도
데드맨이 무력화되지 않는다.

우선순위를 임무(5)보다 높게(10) 둔 이유: 사람이 조종간을 잡고 있으면 그쪽이
이겨야 한다. 임무 중에 조종을 잡아도 별도 모드 전환 없이 개입이 먹는다.
"""
from __future__ import annotations

import time

from orchard_sim.control.base import Feature, VelocityRequest
from orchard_sim.link import protocol as P


class DriveTeleop(Feature):
    name = "drive_teleop"
    version = "1.0"
    summary = "원격 수동 조종 (데드맨)"
    commands = (P.CMD_SET_MODE,)
    topics = ()

    def setup(self, ctx):
        super().setup(ctx)
        self.vmax = float(ctx.param("teleop_max_v", 0.8))
        self.wmax = float(ctx.param("teleop_max_w", 1.2))
        self._v = self._w = 0.0
        self._at = 0.0

    def on_command(self, cmd, payload):
        if cmd == "teleop":
            # 조종 입력은 지연이 생명이라 명령 큐를 거치지 않고 바로 들어온다.
            v = max(-self.vmax, min(self.vmax, float(payload.get("v", 0.0))))
            w = max(-self.wmax, min(self.wmax, float(payload.get("w", 0.0))))
            self._v, self._w, self._at = v, w, time.monotonic()
            return True
        if cmd == P.CMD_SET_MODE:
            m = payload.get("mode", P.MODE_IDLE)
            s = self.ctx.safety.snapshot()
            if s["estop"]:
                self.ctx.event("rejected", "비상정지 상태 — 모드 변경 불가", "warn")
                return True
            if m in (P.MODE_IDLE, P.MODE_TELEOP):
                self.ctx.bb.extra["mode"] = m
                if m == P.MODE_TELEOP:
                    self.ctx.safety.set_paused(False)
                self.ctx.event("mode", f"모드 → {m}")
                return True
            return False        # 다른 모드는 다른 기능이 처리한다
        return False

    def tick(self, now):
        if self.ctx.bb.extra.get("mode") != P.MODE_TELEOP:
            return None
        # 갱신이 끊기면 요청 자체를 내지 않는다 → 조정자의 데드맨이 정지시킨다
        if (now - self._at) * 1000.0 > P.TELEOP_DEADMAN_MS:
            return None
        if self._v == 0.0 and self._w == 0.0:
            return None
        return VelocityRequest(self._v, self._w, priority=10, reason="teleop")
