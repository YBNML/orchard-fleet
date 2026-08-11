"""
임무 주행 기능 — 통로 목록을 받아 보스트로피돈으로 훑는다.

경로 생성을 웨이포인트 목록으로 분리해 둔 이유: 나중에 통로 추출 결과(경로
그래프)에서 웨이포인트를 뽑도록 바꿀 때 `build_waypoints` 하나만 갈아끼우면
되게 하려는 것이다. 추종기(pure-pursuit 유사)는 그대로 둔다.

계단식 지형에서는 통로 사이를 직선으로 가로지를 수 없다(둑 경사 최대 60%).
그래서 통로 간 이동은 반드시 선회 구간을 경유하도록 웨이포인트를 만든다 —
이건 손으로 넣은 규칙이 아니라 지형 제약을 반영한 경로 설계다.

**완료 보고.** 임무는 명령 하나가 수십 분 뒤에 끝나는 일이라, 받았다(accepted)
와 끝났다(completed)가 다른 사건이다. 그래서 임무 내내 실적을 모아 두었다가
끝나는 순간 mission_start 의 cmd_id 로 결과를 낸다 — 무엇을 얼마나 훑었는지
(alleys_done·coverage), 얼마나 달렸는지(distance_m·duration_s), 사람 손이
몇 번 필요했는지(interventions). 관제는 이 한 건으로 임무를 마감한다.
보고 키는 계약(protocol.MISSION_REPORT_KEYS)이 정한다.
"""
from __future__ import annotations

import math
import time

from robomw.core.base import Feature, VelocityRequest
from robomw.link import protocol as P


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
        self._report = None             # 완료 보고 수집함 (임무 하나당 하나)
        self._last_pose = None          # 주행거리 적분용 직전 위치

    # ── 경로 ────────────────────────────────────────────────────────────────
    def cross_y(self, sign):
        """통로 간 횡이동을 허용하는 y — 램프 대역 **깊숙이**.

        하네스 소사 실측(08-03): 통과율 지도의 황금 대역은 |y|∈[34.5,35.5]
        (4개 횡단 모두 12/12), 가장자리(32.5·36.5)는 배신한다. 0.8333 은
        그 중심 35.0 이다. 목표=실제가 성립하는 근거는 캘리브레이션 벽
        앵커 — 통로×단별 실측 벽 위치로 접근 중 종오차를 소거한다
        (주입 2 m → 0.4 m 수렴 실측).
        """
        # 08-11: 남측도 0.6667 로 통일 (남 -34.0 / 북 +34.0). 종전 0.5(-33.0)의
        # 근거였던 '남측 겉보기 벽(램프 면)이 얕다'는 선회 평지 패드 조성으로
        # 소멸했다 (재교정 벽 테이블은 남측도 대부분 -37~-39 울타리선). 오히려
        # -33.0 출구는 tol 0.5 조기완료와 겹쳐 r=1 호가 y=-33 대역을 지나게
        # 만들었고, 그 선의 구조물과 기하적으로 교차했다 (run42 개입 1·2).
        f = 0.6667
        return sign * (self.col_l / 2.0 + self.HL * f)

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
    def _reject(self, payload, msg, code, reason=None):
        """임무 시작 거부 — 화면 이벤트와 명령 결과를 **둘 다** 낸다.

        예전에는 이벤트만 냈다. 그러면 cmd_id 를 붙여 보낸 관제에게는 라우터가
        내는 accepted 만 도착한다 — 거부가 수락으로 보이고, 관제는 로봇이
        임무를 도는 줄 알고 다음 절차로 넘어간다. 반대로 cmd_id 없이 보내던
        옛 클라이언트에게는 예전 그대로 rejected 이벤트만 간다.
        """
        self.ctx.event("rejected", msg, "warn")
        self.ctx.emit_cmd_result(payload.get("cmd_id"), P.CMD_MISSION_START,
                                 "rejected", code=code,
                                 data={"reason": reason or msg})
        return True

    def on_command(self, cmd, payload):
        s = self.ctx.safety
        if cmd == P.CMD_MISSION_START:
            alleys = [int(v) for v in (payload.get("alleys") or [])
                      if 0 <= int(v) <= self.R - 2]
            if not alleys:
                return self._reject(payload, "유효한 통로가 없다", "BAD_PARAM")
            # work(작업기) 는 선택 필드다. 있으면 계약이 검사하고, 통과한 것만
            # 블랙보드에 놓는다 — 실행(방제·예초 …)은 스펙 ② 몫이라 아직 없다.
            # 검사부터 넣는 이유: 형식이 틀린 값을 받아만 두면, 나중에 실행이
            # 붙는 날 현장에서 처음 터진다.
            work = payload.get("work")
            if work is not None:
                ok, why = P.validate_work(work)
                if not ok:
                    return self._reject(payload, f"work 스키마 오류 — {why}",
                                        "BAD_PARAM", why)
            if s.snapshot()["estop"]:
                return self._reject(payload, "비상정지 상태 — 임무 시작 불가",
                                    "ESTOPPED")
            if self.ctx.bb.pose is None:
                # 측위가 서기 전에는 임무를 만들지 않는다 (스펙 §5). 예전에는
                # 임무만 만들어 놓고 포즈가 올 때까지 조용히 서 있었다 —
                # 관제 화면에서는 '시작했다는데 안 간다'로만 보였고, 사람이
                # 로봇에 걸어가 보게 만드는 종류의 침묵이다.
                return self._reject(payload,
                                    "측위 미준비 — 로컬라이저가 아직 위치를 내지 못했다",
                                    "BUSY", "측위 미준비")
            if work is not None:
                self.ctx.bb.extra["work"] = work
            self.mission = dict(alleys=alleys, mode=payload.get("mode", "mapping"),
                                wps=self.build_waypoints(alleys), idx=0,
                                started=time.time(),
                                # 완료 보고를 이 명령의 결과로 돌려주기 위한 상관 키
                                cmd_id=payload.get("cmd_id"))
            self._begin_report()
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
            self._report = None         # 중간에 끊긴 실적은 보고하지 않는다
            self._last_pose = None
            self.ctx.bb.extra["mode"] = P.MODE_IDLE
            self.ctx.event("mission_cancelled", "임무 취소")
            return True
        return False

    # ── 완료 보고 ───────────────────────────────────────────────────────────
    def _begin_report(self):
        """실적 수집을 연다. 임무 하나가 곧 보고 하나다."""
        self._report = dict(alleys_done=[], distance_m=0.0, t0=time.time(),
                            interventions=0)
        self._last_pose = None
        # 개입 횟수는 호스트가 센다(로봇을 세운 개입 요청마다 +1 — 판정은
        # control_agent._is_intervention). 임무마다 0 에서 다시 시작해야
        # 이번 임무의 숫자가 된다.
        self.ctx.bb.extra["mission_interventions"] = 0

    def _track_distance(self, m, p):
        """주행 거리 적분 (완료 보고의 distance_m).

        한 틱(50 ms)에 0.5 m 을 넘는 변위는 실제 주행이 아니다 — 10 m/s 는 이
        기체가 못 내는 속도다. 로컬라이저 재초기화·텔레포트가 위치를 통째로
        옮긴 것이므로 버린다. 그대로 더하면 보고서의 주행거리가 수십 m 씩
        부풀어, 거리로 커버리지를 가늠하는 판단이 통째로 어긋난다.
        """
        if m is None or p is None or self._report is None:
            self._last_pose = None      # 임무 밖·측위 공백의 이동은 세지 않는다
            return
        q, self._last_pose = self._last_pose, (p[0], p[1])
        if q is None:
            return
        dd = math.hypot(p[0] - q[0], p[1] - q[1])
        if dd <= 0.5:
            self._report["distance_m"] += dd

    def _note_wp_done(self, wp):
        """웨이포인트 하나 완료. traverse 를 마쳤다는 것이 곧 통로 완주다."""
        if wp["kind"] == "traverse" and self._report is not None:
            self._report["alleys_done"].append(wp["alley"])

    def _finish(self, m):
        """완료 보고를 명령 결과(completed)로 낸다.

        키는 계약(P.MISSION_REPORT_KEYS)이 정한 다섯 개다 — 관제 UI 가 고정된
        칸을 그리므로 하나라도 빠지면 화면에 빈칸이 남는다.
        """
        rp, self._report = self._report, None
        self._last_pose = None
        if rp is None:
            return
        done = list(rp["alleys_done"])
        rp["interventions"] = int(self.ctx.bb.extra.get("mission_interventions", 0))
        data = dict(alleys_done=done,
                    distance_m=round(rp["distance_m"], 1),
                    duration_s=round(time.time() - rp["t0"], 1),
                    interventions=rp["interventions"],
                    coverage=round(len(done) / (len(m["alleys"]) or 1), 3))
        self.ctx.emit_cmd_result(m.get("cmd_id"), P.CMD_MISSION_START,
                                 "completed", data=data)

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
        self._track_distance(m, p)
        if m is None or p is None:
            return None
        if self.ctx.bb.extra.get("mode") != P.MODE_MISSION:
            return None
        if m["idx"] >= len(m["wps"]):
            self.mission = None
            self.ctx.bb.extra["mode"] = P.MODE_IDLE
            self.ctx.event("mission_done", "임무 완료")
            self._finish(m)
            return None
        wp = m["wps"][m["idx"]]
        tx = p[0] if wp["x"] is None else wp["x"]
        ty = p[1] if wp.get("y") is None else wp["y"]
        dx, dy = tx - p[0], ty - p[1]
        dist = math.hypot(dx, dy)
        if dist < self.tol:
            self._note_wp_done(wp)
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
        # 블록 안 traverse 에서는 피벗하지 않는다 — 횡표류가 계단 법면
        # 깔때기에 걸리면 err 가 0.6 을 넘는데, 그 자리(법면 위)의 제자리
        # 회전은 회전 슬립으로 쐐기가 된다 (실측: x=-9.1 근방 3회, 08-03).
        # 조향각을 눌러 굴러가며 복귀하면 접지가 산다.
        if (wp["kind"] == "traverse"
                and abs(p[1]) < self.col_l / 2.0 - 1.0
                and abs(err) > align_th):
            err = max(-0.55, min(0.55, err))
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
        # 횡단은 **방위 유지 조향** — est 위치 오차로 조향하지 않는다.
        # 등판 슬립 중 위상 보정이 거부되면 est 가 오염되고, 그 오염이
        # wz 요동으로 되먹임돼 접지를 무너뜨린다 (소거법 실측: 지형·피벗·
        # 정속·정wz·참값폐루프 전부 개루프 4/4~56/56 인데 임무만 정체,
        # 08-03). 횡단은 직선이다 — 자이로 방위만 물고 간다. est 는 도착
        # 판정에만 쓴다.
        if wp["kind"] == "cross":
            want = 0.0 if dx > 0 else math.pi
            err_h = (want - p[2] + math.pi) % (2 * math.pi) - math.pi
            return VelocityRequest(self.speed,
                                   max(-0.4, min(0.4, 1.0 * err_h)),
                                   priority=5, reason="mission:cross-hold")
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
