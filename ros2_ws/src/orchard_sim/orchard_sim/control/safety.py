"""
안전 조정자 — 코어. 기능이 우회할 수 없다.

기능이 늘고 바뀌어도 여기 있는 네 가지는 항상 성립해야 한다.

    비상정지 래치   해제는 **2단계** 다 — 관제 승인(원격)과 현장 확인(로컬)이 모두
                    있어야 풀린다. 어느 한쪽만으로는 절대 안 풀린다. 해제해도
                    자동 재개하지 않는다.

                    근거: ISO 13849-1 §5.2.2 는 리셋 액추에이터가 위험구역 밖의
                    "사람이 없음을 확인할 수 있는 시야가 확보된 위치"에 있을 것을
                    요구한다. 지도만 보는 관제실은 이 요건을 못 채우므로, 같은
                    조항이 허용하는 **특별 리셋 절차**로 대신한다 — 시야를 가진
                    사람이 현장에서 확인하고, 관제가 승인한다.
                    ISO 13850: 리셋은 고의적 수동 동작이어야 하고 리셋 자체가
                    재시동이 되어선 안 된다. ISO 3691-4/B56.5: 자동 재시동 금지.
    데드맨          속도 요청은 갱신이 끊기면 무효다. 회선이 끊겼을 때 **마지막
                    속도 명령이 살아 있는 것**이 가장 위험한 실패 모드다.
    링크두절 정지   관제 연결이 끊기면 스스로 선다. 임무는 지우지 않고 일시정지 —
                    통신이 돌아오면 이어서 할 수 있어야 한다.
    전복 감지       관제 지시를 기다리지 않고 로봇이 스스로 잡는다.

속도는 **단일 창구**로만 나간다. 기능이 여러 개여도 /cmd_vel 을 두고 다투지
않는다 — 조정자가 우선순위로 하나를 고르고 게이트를 통과시킨다.
"""
from __future__ import annotations

import threading
import time

from orchard_sim.link import protocol as P


# 해제 절차 단계 — 관제 화면이 "무엇이 남았는지"를 그대로 보여줄 수 있게 이름을 준다
STAGE_CLEAR = "clear"                      # 래치 없음
STAGE_LATCHED = "latched"                  # 정지 래치, 양쪽 다 미완
STAGE_AWAITING_LOCAL = "awaiting_local"    # 관제 승인됨 → 현장 확인 대기
STAGE_AWAITING_REMOTE = "awaiting_remote"  # 현장 확인됨 → 관제 승인 대기


class SafetyArbiter:

    def __init__(self, tilt_limit_deg=35.0, on_event=None, on_estop=None,
                 reset_window_s=600.0):
        self._lock = threading.RLock()
        self.tilt_limit = float(tilt_limit_deg)
        self._on_event = on_event or (lambda *a, **k: None)
        self._on_estop = on_estop or (lambda reason: None)
        self.reset_window_s = float(reset_window_s)

        self.estop = False
        self.estop_reason = ""
        self.paused = False
        self.link_ok = False
        self.last_client_seen = 0.0
        self._last_gate = ""

        # 2단계 해제 상태
        self.estop_stage = STAGE_CLEAR
        self._estop_at = 0.0        # 래치가 걸린 시각 (왕복시간 측정 기준)
        self._remote_ok_at = 0.0    # 관제 승인 시각
        self._local_ok_at = 0.0     # 현장 확인 시각
        self.last_round_trip_s = None   # 직전 해제의 estop→현장확인 소요

    # ── 상태 조회 ───────────────────────────────────────────────────────────
    def snapshot(self):
        with self._lock:
            self._expire_locked(time.monotonic())
            return dict(estop=self.estop, estop_reason=self.estop_reason,
                        paused=self.paused, link_ok=self.link_ok,
                        gate=self._last_gate,
                        estop_stage=self.estop_stage,
                        reset_window_s=self.reset_window_s,
                        needs_remote_ok=self.estop and not self._remote_ok_at,
                        needs_local_ok=self.estop and not self._local_ok_at,
                        last_round_trip_s=self.last_round_trip_s)

    # ── 비상정지 ────────────────────────────────────────────────────────────
    def trigger(self, reason: str):
        with self._lock:
            if self.estop:
                return False
            self.estop = True
            self.estop_reason = reason
            self.paused = True
            self.estop_stage = STAGE_LATCHED
            self._estop_at = time.monotonic()
            self._remote_ok_at = self._local_ok_at = 0.0
        self._on_event("estop", f"비상정지 — {reason}", "critical")
        self._on_estop(reason)
        return True

    # ── 2단계 해제 ──────────────────────────────────────────────────────────
    def _expire_locked(self, now: float):
        """두 단계 사이가 창을 넘으면 무효화한다 — 며칠 전 승인이 살아 있으면 안 된다."""
        if not self.estop:
            return
        for attr in ("_remote_ok_at", "_local_ok_at"):
            t = getattr(self, attr)
            if t and (now - t) > self.reset_window_s:
                setattr(self, attr, 0.0)
        if self.estop:
            if self._remote_ok_at and not self._local_ok_at:
                self.estop_stage = STAGE_AWAITING_LOCAL
            elif self._local_ok_at and not self._remote_ok_at:
                self.estop_stage = STAGE_AWAITING_REMOTE
            else:
                self.estop_stage = STAGE_LATCHED

    def _finish_locked(self, now: float):
        self.estop = False
        self.estop_reason = ""
        self.paused = True              # 해제 ≠ 재개
        self.estop_stage = STAGE_CLEAR
        self.last_round_trip_s = round(max(0.0, self._local_ok_at - self._estop_at), 1)
        self._remote_ok_at = self._local_ok_at = 0.0

    def request_clear(self, who: str = "관제"):
        """① 관제 승인. 이것만으로는 **절대** 풀리지 않는다.

        서버가 손상돼 이 명령을 마음대로 보내도, 현장 확인이 없으면 로봇은
        움직이지 않는다.
        """
        now = time.monotonic()
        with self._lock:
            if not self.estop:
                return False, "비상정지 상태가 아닙니다"
            self._expire_locked(now)
            self._remote_ok_at = now
            if self._local_ok_at:
                self._finish_locked(now)
                done = True
            else:
                self.estop_stage = STAGE_AWAITING_LOCAL
                done = False
        if done:
            self._on_event("estop_cleared",
                           f"비상정지 해제 ({who} 승인 + 현장 확인) — 임무는 일시정지 상태")
            return True, "해제됨"
        self._on_event("estop_clear_requested",
                       f"{who} 해제 승인 — 현장 확인을 기다립니다", "warn")
        return True, "현장 확인 대기"

    def local_reset(self, who: str = "현장"):
        """② 현장 확인. 로봇 곁에서 위험구역을 눈으로 확인한 사람의 조작이다.

        규격이 요구하는 '시야 확보된 위치에서의 리셋'을 담당하는 쪽이다.
        """
        now = time.monotonic()
        with self._lock:
            if not self.estop:
                return False, "비상정지 상태가 아닙니다"
            self._expire_locked(now)
            self._local_ok_at = now
            if self._remote_ok_at:
                self._finish_locked(now)
                done = True
            else:
                self.estop_stage = STAGE_AWAITING_REMOTE
                done = False
        if done:
            self._on_event("estop_cleared",
                           "비상정지 해제 (관제 승인 + 현장 확인) — 임무는 일시정지 상태")
            return True, "해제됨"
        self._on_event("local_reset", f"{who} 확인 완료 — 관제 승인을 기다립니다", "warn")
        return True, "관제 승인 대기"

    def cancel_clear(self, who: str = "관제"):
        """해제 절차 취소 — 다시 래치 상태로 되돌린다."""
        with self._lock:
            if not self.estop:
                return False
            had = bool(self._remote_ok_at or self._local_ok_at)
            self._remote_ok_at = self._local_ok_at = 0.0
            self.estop_stage = STAGE_LATCHED
        if had:
            self._on_event("estop_clear_canceled", f"{who} 해제 절차 취소", "warn")
        return True

    def set_paused(self, val: bool) -> bool:
        """일시정지 설정. 비상정지 중에는 해제(False)를 거부한다."""
        with self._lock:
            if self.estop and not val:
                return False
            self.paused = bool(val)
            return True

    # ── 링크 ────────────────────────────────────────────────────────────────
    def note_client(self, now=None):
        with self._lock:
            self.last_client_seen = now if now is not None else time.monotonic()

    def update_link(self, n_clients: int, now: float):
        """관제 연결 상태를 갱신하고, 끊겼으면 임무를 일시정지시킨다."""
        with self._lock:
            idle_ms = (now - self.last_client_seen) * 1000.0
            ok = n_clients > 0 and idle_ms <= P.LINK_LOSS_STOP_MS
            changed = ok != self.link_ok
            self.link_ok = ok
            newly_lost = changed and not ok
            if newly_lost and not self.paused:
                self.paused = True
        if newly_lost:
            self._on_event("link_lost", "관제 연결 끊김 — 정지 후 임무 일시정지", "warn")
        return ok

    # ── 자세 ────────────────────────────────────────────────────────────────
    def check_attitude(self, tilt_deg: float):
        if tilt_deg is not None and tilt_deg > self.tilt_limit:
            self.trigger(f"전복 감지 (기울기 {tilt_deg:.0f}°)")
            return False
        return True

    # ── 속도 게이트 ─────────────────────────────────────────────────────────
    def arbitrate(self, requests, now: float):
        """요청 목록 → (v, w, 사유). 게이트에 막히면 (0, 0, 사유).

        requests: [(VelocityRequest, 요청시각)] — 시각은 데드맨 판정용.
        """
        with self._lock:
            if self.estop:
                self._last_gate = "estop"
                return 0.0, 0.0, "estop"
            if not self.link_ok:
                self._last_gate = "link"
                return 0.0, 0.0, "link"
            if self.paused:
                self._last_gate = "paused"
                return 0.0, 0.0, "paused"

        fresh = [(r, t) for (r, t) in requests
                 if r is not None and (now - t) * 1000.0 <= P.TELEOP_DEADMAN_MS]
        if not fresh:
            with self._lock:
                self._last_gate = "deadman" if requests else "idle"
            return 0.0, 0.0, self._last_gate

        best = max(fresh, key=lambda rt: rt[0].priority)[0]
        with self._lock:
            self._last_gate = ""
        return float(best.v), float(best.w), best.reason
