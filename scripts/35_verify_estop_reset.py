#!/usr/bin/env python3
"""비상정지 2단계 해제 절차 검증 (ISO 13849-1 §5.2.2 특별 리셋 절차)

    python3 scripts/35_verify_estop_reset.py

증명해야 할 것은 하나다 — **어느 한쪽만으로는 절대 안 풀린다.**
관제 서버가 통째로 손상돼 clear_estop_request 를 무한히 보내도, 위험구역을
눈으로 확인한 사람이 기체에서 누르기 전에는 로봇이 움직이지 않아야 한다.
"""
from __future__ import annotations

import sys
import time

sys.path.insert(0, "ros2_ws/src/orchard_sim")

from orchard_sim.control.safety import (STAGE_AWAITING_LOCAL,  # noqa: E402
                                        STAGE_AWAITING_REMOTE, STAGE_CLEAR,
                                        STAGE_LATCHED, SafetyArbiter)

OK, NG = "\033[92m✔\033[0m", "\033[91m✗\033[0m"
res = []


def check(name, cond, detail=""):
    res.append(bool(cond))
    print(f"   {OK if cond else NG} {name}" + (f"  — {detail}" if detail else ""))


def mk(window=600.0):
    events = []
    s = SafetyArbiter(on_event=lambda k, m, lv="info": events.append(k),
                      reset_window_s=window)
    s.link_ok = True
    return s, events


print("비상정지 2단계 해제 검증")
print("=" * 74)

# ── 1. 원격 단독으로는 안 풀린다 (가장 중요) ────────────────────────────────
print("\n── 1. 관제 승인만으로는 풀리지 않는다 ──")
s, ev = mk()
s.trigger("시험")
check("정지 직후 래치", s.estop and s.snapshot()["estop_stage"] == STAGE_LATCHED)
for i in range(50):                       # 손상된 서버가 50번 두드려도
    s.request_clear("공격자")
check("관제 승인 50회 반복 후에도 여전히 정지", s.estop is True,
      f"stage={s.snapshot()['estop_stage']}")
check("단계가 '현장 확인 대기'", s.snapshot()["estop_stage"] == STAGE_AWAITING_LOCAL)
check("속도 게이트가 계속 estop", s.arbitrate([], time.monotonic())[2] == "estop")

# ── 2. 현장 확인이 더해지면 풀린다 ──────────────────────────────────────────
print("\n── 2. 현장 확인이 더해지면 풀린다 ──")
ok, why = s.local_reset("현장")
check("현장 확인 후 해제됨", ok and s.estop is False, why)
check("해제해도 임무는 일시정지 유지 (자동 재개 없음)", s.paused is True)
check("단계 clear", s.snapshot()["estop_stage"] == STAGE_CLEAR)
check("왕복시간이 기록됨", s.snapshot()["last_round_trip_s"] is not None,
      f"{s.snapshot()['last_round_trip_s']}초")

# ── 3. 반대 순서도 성립한다 ─────────────────────────────────────────────────
print("\n── 3. 현장 확인이 먼저여도 성립 ──")
s, ev = mk()
s.trigger("시험")
s.local_reset("현장")
check("현장 확인만으로는 안 풀림", s.estop is True)
check("단계가 '관제 승인 대기'", s.snapshot()["estop_stage"] == STAGE_AWAITING_REMOTE)
for i in range(20):
    s.local_reset("현장")                 # 현장에서 계속 눌러도
check("현장 확인 20회 반복 후에도 정지", s.estop is True)
s.request_clear("관제")
check("관제 승인 후 해제", s.estop is False)

# ── 4. 창 만료 ──────────────────────────────────────────────────────────────
print("\n── 4. 두 단계 사이가 벌어지면 무효 ──")
s, ev = mk(window=0.3)
s.trigger("시험")
s.request_clear("관제")
time.sleep(0.45)                          # 창을 넘긴다
check("만료 후 단계가 래치로 복귀", s.snapshot()["estop_stage"] == STAGE_LATCHED)
s.local_reset("현장")
check("만료된 승인으로는 안 풀림", s.estop is True,
      f"stage={s.snapshot()['estop_stage']}")
s.request_clear("관제")                    # 창 안에서 다시
check("다시 승인하면 풀림", s.estop is False)

# ── 5. 취소 ─────────────────────────────────────────────────────────────────
print("\n── 5. 해제 절차 취소 ──")
s, ev = mk()
s.trigger("시험")
s.request_clear("관제")
s.cancel_clear("관제")
check("취소 후 래치로 복귀", s.snapshot()["estop_stage"] == STAGE_LATCHED)
s.local_reset("현장")
check("취소된 승인은 되살아나지 않음", s.estop is True)

# ── 6. 정지 중에는 어떤 요청도 통과 못 한다 ─────────────────────────────────
print("\n── 6. 래치 중 속도 게이트 ──")
s, ev = mk()
s.trigger("시험")


class R:                                   # 최고 우선순위 요청을 흉내
    priority, v, w, reason = 99, 1.0, 1.0, "공격"


now = time.monotonic()
check("최고 우선순위 요청도 막힘", s.arbitrate([(R(), now)], now)[:2] == (0.0, 0.0))
check("재정지는 중복 트리거되지 않음", s.trigger("두 번째") is False)

# ── 7. 이벤트 기록 ──────────────────────────────────────────────────────────
print("\n── 7. 절차가 이벤트로 남는가 ──")
s, ev = mk()
s.trigger("시험")
s.request_clear("관제")
s.local_reset("현장")
check("estop → 승인 → 해제가 순서대로 기록",
      ev[:1] == ["estop"] and "estop_clear_requested" in ev and "estop_cleared" in ev,
      str(ev))

print("\n" + "=" * 74)
n_ok, n = sum(res), len(res)
print(f"{n_ok}/{n} 통과")
sys.exit(0 if n_ok == n else 1)
