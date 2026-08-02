"""
임무 주행 기능 — 통로 목록을 받아 보스트로피돈으로 훑는다.

경로 생성을 웨이포인트 목록으로 분리해 둔 이유: 나중에 통로 추출 결과(경로
그래프)에서 웨이포인트를 뽑도록 바꿀 때 `build_waypoints` 하나만 갈아끼우면
되게 하려는 것이다. 추종기(pure-pursuit 유사)는 그대로 둔다.

계단식 지형에서는 통로 사이를 직선으로 가로지를 수 없다(둑 경사 최대 60%).
그래서 통로 간 이동은 반드시 선회 구간을 경유하도록 웨이포인트를 만든다 —
이건 손으로 넣은 규칙이 아니라 지형 제약을 반영한 경로 설계다.
"""
from __future__ import annotations

import math
import time

from orchard_sim.control.base import Feature, VelocityRequest
from orchard_sim.link import protocol as P


class DriveMission(Feature):
    name = "drive_mission"
    version = "1.0"
    summary = "통로 목록 순회 (보스트로피돈)"
    commands = (P.CMD_MISSION_START, P.CMD_MISSION_PAUSE, P.CMD_MISSION_RESUME,
                P.CMD_MISSION_CANCEL)

    def setup(self, ctx):
        super().setup(ctx)
        pr = ctx.param
        self.R = int(pr("rows", 10))
        self.T = int(pr("trees_per_row", 41))
        self.S = float(pr("row_spacing", 3.5))
        self.tsp = float(pr("tree_spacing", 1.5))
        self.HL = float(pr("headland", 6.0))
        self.speed = float(pr("speed", 0.7))
        self.turn_speed = float(pr("turn_speed", 0.5))
        self.y_slow_in = float(pr("y_slow_in", 25.0))
        self.slow_factor = float(pr("slow_factor", 0.40))
        self.decel_dist = float(pr("decel_dist", 3.0))
        self.tol = float(pr("wp_tol", 0.5))
        self.x0 = -((self.R - 1) * self.S) / 2.0
        self.col_l = (self.T - 1) * self.tsp
        self.mission = None
        self._align_key = None          # 회전 슬립 감시 — (wp idx, 시작 시각)

    # ── 경로 ────────────────────────────────────────────────────────────────
    def cross_y(self, sign):
        """통로 간 횡이동을 허용하는 y — 램프 대역 **깊숙이**.

        0.72(±34.3): 칸 오차 1.5 m 에 램프 밖(31.8, 하네스 0/4)에서 건넌다.
        0.90(±35.4): 실측 오차가 2.4 m(칸+하강 슬립)라 32.5 — 여전히 밖.
        1.10(±36.6): 2.4 m 를 먹고도 34.2 — 검증 대역(33.5, 4/4) 안이다.
        오차를 고치는 대신 견딘다. 둑(38.7)까지 2.1 m 는 벽 도착 판정
        (여유<1.8 m)이 받친다 — 오차가 작아 실제로 36.6 까지 가면 벽
        판정이 출구를 제때 끝낸다.
        """
        return sign * (self.col_l / 2.0 + self.HL * 1.10)

    def build_waypoints(self, alleys):
        y_lo = -self.col_l / 2.0 - self.HL * 0.25
        y_hi = self.col_l / 2.0 + self.HL * 0.25
        wps = []
        for i, k in enumerate(alleys):
            cx = self.x0 + (k + 0.5) * self.S
            up = (i % 2 == 0)
            y_start, y_end = (y_lo, y_hi) if up else (y_hi, y_lo)
            if i > 0:
                yc = self.cross_y(-1.0 if up else 1.0)
                wps.append(dict(x=None, y=yc, kind="exit", alley=k))
                # 횡단은 x 만 가면 된다 — y 까지 고정하면 출구에서 조금만
                # 밀려 있어도 둑 모서리로 대각 진입해 램프에서 낀다 (실측:
                # 남동 대각 클라임 정체, 08-02). y 는 현재 위치를 따른다.
                wps.append(dict(x=cx, y=None, kind="cross", alley=k))
                wps.append(dict(x=cx, y=y_start, kind="enter", alley=k))
            wps.append(dict(x=cx, y=y_end, kind="traverse", alley=k))
        return wps

    # ── 명령 ────────────────────────────────────────────────────────────────
    def on_command(self, cmd, payload):
        s = self.ctx.safety
        if cmd == P.CMD_MISSION_START:
            alleys = [int(v) for v in (payload.get("alleys") or [])
                      if 0 <= int(v) <= self.R - 2]
            if not alleys:
                self.ctx.event("rejected", "유효한 통로가 없다", "warn")
                return True
            if s.snapshot()["estop"]:
                self.ctx.event("rejected", "비상정지 상태 — 임무 시작 불가", "warn")
                return True
            self.mission = dict(alleys=alleys, mode=payload.get("mode", "mapping"),
                                wps=self.build_waypoints(alleys), idx=0,
                                started=time.time())
            s.set_paused(False)
            self.ctx.bb.extra["mode"] = P.MODE_MISSION
            self.ctx.event("mission_started",
                           f"임무 시작 — 통로 {alleys} · 웨이포인트 "
                           f"{len(self.mission['wps'])}개")
            return True
        if cmd == P.CMD_MISSION_PAUSE:
            s.set_paused(True)
            self.ctx.event("mission_paused", "임무 일시정지")
            return True
        if cmd == P.CMD_MISSION_RESUME:
            if not s.set_paused(False):
                self.ctx.event("rejected", "비상정지 상태 — 먼저 해제해야 한다", "warn")
                return True
            if self.mission:
                self.ctx.bb.extra["mode"] = P.MODE_MISSION
            self.ctx.event("mission_resumed", "임무 재개")
            return True
        if cmd == P.CMD_MISSION_CANCEL:
            self.mission = None
            self.ctx.bb.extra["mode"] = P.MODE_IDLE
            self.ctx.event("mission_cancelled", "임무 취소")
            return True
        return False

    # ── 주행 ────────────────────────────────────────────────────────────────
    def speed_limit(self, y, dist):
        """통로 끝·선회 구간 감속.

        2026-07-30 실측으로 이 감속은 **LIO 표류에는 역효과**였다 (퇴화 구간
        체류 시간이 거리 27% → 시간 56% 로 늘어난다). 다만 주행 안정성에는
        도움이 되므로 남기고, 파라미터로 끌 수 있게 뒀다 (slow_factor=1.0).
        """
        v = self.speed
        if abs(y) >= self.y_slow_in:
            v = min(v, self.speed * self.slow_factor)
        if dist < self.decel_dist:
            v = min(v, max(self.speed * 0.25, self.speed * dist / self.decel_dist))
        return max(v, self.speed * 0.20)

    def tick(self, now):
        self._publish_status()
        m = self.mission
        p = self.ctx.bb.pose
        if m is None or p is None:
            return None
        if self.ctx.bb.extra.get("mode") != P.MODE_MISSION:
            return None
        if m["idx"] >= len(m["wps"]):
            self.mission = None
            self.ctx.bb.extra["mode"] = P.MODE_IDLE
            self.ctx.event("mission_done", "임무 완료")
            return None
        wp = m["wps"][m["idx"]]
        tx = p[0] if wp["x"] is None else wp["x"]
        ty = p[1] if wp.get("y") is None else wp["y"]
        dx, dy = tx - p[0], ty - p[1]
        dist = math.hypot(dx, dy)
        if dist < self.tol:
            m["idx"] += 1
            return None
        # 축분리 조준 — 진행축은 3 m 룩어헤드로 자르고 횡축은 이득을 준다.
        # 원거리 점 조준(횡편차 1.5 m → 조향 2°)도, 단순 룩어헤드(0.75 m →
        # 10°)도 복원력이 모자라 통로 폭 2.0 m 를 지키지 못했다 (실측:
        # 서쪽 모서리 → 과회전 → 동쪽 법면, 08-02). 편차 0.3 m 에 22°,
        # 0.75 m 에 45° 를 요구해야 계단 통로에서 선을 지킨다.
        L, G = 3.0, 4.0
        if abs(dy) >= abs(dx):          # 남북 구간: 기준선 x = 목표 x
            sdy = math.copysign(min(abs(dy), L), dy)
            sdx = math.copysign(min(abs(dx) * G, L), dx)
            cross = abs(dx)
        else:                           # 동서 구간: 기준선 y = 목표 y
            sdx = math.copysign(min(abs(dx), L), dx)
            sdy = math.copysign(min(abs(dy) * G, L), dy)
            cross = abs(dy)
        err = (math.atan2(sdy, sdx) - p[2] + math.pi) % (2 * math.pi) - math.pi
        # 헤드랜드 구간의 '벽 앞 도착' — 램프에서 궤도가 미끄러지면 오도메트리가
        # 실제보다 덜 세서, 추정으로는 '아직 못 왔는데' 몸은 둑 앞에 와 있다.
        # 사람이 운전하듯 벽이 코앞이면 그 웨이포인트는 온 것이다. 계속 밀면
        # 둑을 파고든다 — 실제로 4.3 m 지나쳐 박혔다(08-02).
        clearance = getattr(self.ctx.bb, "clearance", None)
        if (wp["kind"] in ("exit", "enter") and clearance is not None
                and clearance < 1.8 and abs(err) < 0.5):
            m["idx"] += 1
            self.ctx.event("mission",
                           f"전방 {clearance:.1f} m 벽 — {wp['kind']} 도착 처리")
            return None
        # 횡단은 피벗을 거의 끝내고 출발한다 — 34° 남긴 채 전진하면 대각
        # 성분이 열 끝 모서리 경사로 파고든다 (실측: 남단에서 3회 모두
        # x≈-8.9 지점 밀착, 08-02). 직진 구간은 0.6 rad 로 충분하다.
        align_th = 0.15 if wp["kind"] == "cross" else 0.6
        if abs(err) > align_th:
            # 방향 전환은 제자리 회전이 아니라 **호**로 돈다 — 램프에서
            # 제자리 회전은 궤도가 통째로 미끄러져 물리적으로 안 된다
            # (실측: 30초 명령에 회전 3° 미만, 08-02). 바퀴가 굴러가며
            # 돌면 접지력이 산다. 반경 = v/wz = 0.15/0.5 = 0.3 m.
            if self.ctx.safety.paused:
                self._align_key = None      # 정지 중엔 시간을 세지 않는다
                return None
            if self._align_key is None or self._align_key[0] != m["idx"]:
                self._align_key = (m["idx"], now)
            elif now - self._align_key[1] > 45.0:
                self._align_key = None
                self.ctx.safety.set_paused(True)
                self.ctx.event("assistance",
                               "방향 전환 30초째 미완 — 경사 회전 슬립",
                               level="critical", code="TRACTION_LOSS")
                return None
            # 순수 피벗으로 돈다. 호(전진·후진) 선회는 바퀴 횡마찰이 과대해
            # 제자리 회전이 안 되던 시절의 보정책이었는데, 회전이 느린 만큼
            # 병진이 계속 쌓여 헤드랜드에서 2.5 m 를 흘러내렸다 (08-02).
            # 마찰 모델을 실차에 맞춘 지금은 피벗이 전 지형에서 된다 —
            # 안 되면 30초 감시가 세우고 사람을 부른다.
            return VelocityRequest(0.0, self.turn_speed * (1 if err > 0 else -1),
                                   priority=5, reason="mission:align")
        self._align_key = None
        v = self.speed_limit(p[1], dist)
        if wp["kind"] == "cross":
            # 클라임은 관성이 살린다 — 서행 구간(0.28 m/s)에서 출발하면
            # 슬립 65% 오르막을 기다시피 오르다 코를 박는다 (실측: 횡단
            # 시도의 절반이 밀착 정지, 08-02). 전속으로 치고 오르고, 감속도
            # 면제한다 — 넘친 만큼은 다음 진입 웨이포인트가 되당긴다.
            v = self.speed
        elif cross > 0.5:
            v = min(v, self.speed * 0.4)    # 선을 벗어났으면 느리게 복귀한다
        return VelocityRequest(v * max(0.25, 1.0 - abs(err)), 1.4 * err,
                               priority=5, reason="mission:follow")

    def _publish_status(self):
        m = self.mission
        if m is None:
            self.ctx.bb.extra["mission_status"] = None
            return
        wp = m["wps"][m["idx"]] if m["idx"] < len(m["wps"]) else None
        self.ctx.bb.extra["mission_status"] = dict(
            alleys=m["alleys"], mode=m["mode"], idx=m["idx"], total=len(m["wps"]),
            alley=wp["alley"] if wp else None,
            phase=wp["kind"] if wp else "done",
            elapsed=round(time.time() - m["started"], 1))
