"""BT 엔진 — asyncio 1 Hz 틱 · 영속 · 재기동 복원.

한 틱은 RUNNING 인스턴스를 id 순으로 한 번씩 tick 하는 것이다. 인스턴스마다
세션을 새로 열고 tree_json 을 되살려(from_state) 틱한 뒤 되적는다(to_state) —
**진행 상태가 곧 DB 행이라 재기동 복원에 별도 로직이 필요 없다**. 엔진을 새로
만들어도 RUNNING 행을 읽어 하던 자리에서 이어 달린다.

바깥 세계와 닿는 부분은 EngineCtx 하나로 모았다. 임무 발진·승격·취소는 전부
`mission_ops` 의 공용 경로를 부른다 — REST 와 같은 함수다(T4 이관 (a)):
비인접 통로 검증·"잠금 없이 나가는 임무 없음"·발진 전 커밋 같은 불변이 BT
경로에서 조용히 빠지는 것을 막는다.
"""
from __future__ import annotations

import asyncio
import contextlib
import datetime as dt
import logging

from .. import audit, interventions, mission_ops, missions
from ..missions import InvalidTransition
from ..mission_ops import MissionOpError
from ..models import BTInstance, Mission, Robot, Track
from ..traffic import AlleyLocks, conflict
from . import nodes, presets

log = logging.getLogger("fleet_server.bt")

_ACTIVE = "RUNNING"
_RESULT_STATE = {"success": "SUCCESS", "failure": "FAILED"}

# 좀비 감시 대상 — 둘 다 "이미 로봇에 나간" 임무다. QUEUED_LOCK 은 나간 적이
# 없고(잠금 대기), PAUSED 는 사람이 세운 것이라 대상이 아니다.
_STALL_WATCHED = ("QUEUED", "RUNNING")
_WHY_OFFLINE = "링크 단절"
_WHY_IDLE = "로봇 mode=idle"


def mission_ids(state: dict | None) -> list[int]:
    """트리에서 Action 이 붙든 mission_id 를 모은다(취소·정리용)."""
    if not state:
        return []
    if state.get("kind") == "action":
        mid = state.get("mission_id")
        return [mid] if mid is not None else []
    kids = state.get("children") or ([state["child"]] if state.get("child") else [])
    return [mid for k in kids for mid in mission_ids(k)]


class EngineCtx:
    """노드가 보는 바깥 세계 — DB 세션 하나와 FleetPort 하나에 묶인다.

    인스턴스 단위로 만들어지므로 farm·발주자(created_by)를 안다: BT 가 만드는
    임무의 감사 기록은 그 인스턴스를 만든 사람 앞으로 남는다(role="bt").
    """

    def __init__(self, db, fleet, inst: BTInstance):
        self.db, self.fleet, self.inst = db, fleet, inst

    # ── 조건 ────────────────────────────────────────────────────────────────
    async def alley_free(self, alleys) -> bool:
        """같은 farm 의 활성 잠금과 겹치지 않는가 — AlleyLock 규칙 그대로."""
        return not any(conflict(alleys, row["alleys"])
                       for row in AlleyLocks.list_active(self.db)
                       if row["farm_id"] == self.inst.farm_id)

    async def robot_idle(self, robot: str) -> bool:
        """활성 임무(QUEUED/QUEUED_LOCK/RUNNING/PAUSED)가 없으면 놀고 있는 것 —
        REST 가 이중시작을 막는 기준과 같은 잣대를 쓴다."""
        return (self.db.query(Mission)
                .filter(Mission.robot_id == robot,
                        Mission.state.in_(mission_ops.ACTIVE_MISSION_STATES))
                .first()) is None

    async def robot_online(self, robot: str) -> bool:
        return bool(self.fleet.robot_status(robot).online)

    # ── 임무 ────────────────────────────────────────────────────────────────
    async def start_mission(self, spec: dict) -> int | None:
        robot = self.db.get(Robot, spec.get("robot"))
        if robot is None or robot.farm_id != self.inst.farm_id:
            self._audit("rejected", str(spec.get("robot")), "로봇 없음/농장 불일치")
            return None
        try:
            ms, lock_reason = await mission_ops.create_and_dispatch(
                self.db, self.fleet, robot=robot, alleys=spec.get("alleys"),
                work=spec.get("work"), created_by=self.inst.created_by)
        except MissionOpError as e:
            self._audit("rejected", robot.id, e.detail)
            return None                        # 발진 거부 — Action 실패(Retry 가 받는다)
        if lock_reason is not None:            # QUEUED_LOCK — 실패가 아니라 대기
            self._audit("rejected", robot.id, f"mission={ms.id} {lock_reason}")
        else:
            self._audit("accepted", robot.id,
                        f"mission={ms.id} alleys={spec.get('alleys')}")
        return ms.id

    async def mission_status(self, mission_id: int) -> str:
        ms = self.db.get(Mission, mission_id)
        return ms.state if ms is not None else "FAILED"   # 사라진 임무는 실패로 본다

    async def promote(self, mission_id: int) -> bool:
        ms = self.db.get(Mission, mission_id)
        if ms is None:
            return False
        try:
            ok = await mission_ops.promote_locked(self.db, self.fleet, ms)
        except InvalidTransition:              # 동시에 다른 세션이 전이시켰다
            return False
        if ok:
            self._audit("accepted", ms.robot_id, f"mission={ms.id} 잠금 승격 발진")
        return ok

    def _audit(self, result: str, target: str, detail: str) -> None:
        audit.record(self.db, action="mission_start", result=result,
                     user_id=self.inst.created_by, role="bt", target=target,
                     detail=f"BT#{self.inst.id}({self.inst.preset}) {detail}")


class BTEngine:
    """`BTEngine(session_factory, fleet)` — create/cancel + 1 Hz 틱 태스크."""

    def __init__(self, session_factory, fleet, *, period_s: float = 1.0,
                 max_tick_errors: int = 5, stall_ticks: int = 30,
                 queued_stall_ticks: int = 60, track_fresh_s: float = 20.0):
        self._factory = session_factory
        self.fleet = fleet                      # lifespan 이 레거시 포트로 교체할 수 있다
        self.period_s = period_s
        self.max_tick_errors = max_tick_errors  # 연속 틱 예외 한계(넘으면 종착)
        self.stall_ticks = stall_ticks          # RUNNING 임무 정체 판정 틱 수
        self.queued_stall_ticks = queued_stall_ticks   # QUEUED 는 더 길게 본다
        self.track_fresh_s = track_fresh_s      # 이보다 낡은 텔레메트리는 증거로 안 친다
        self._task: asyncio.Task | None = None
        self._lock = asyncio.Lock()             # 틱과 취소를 직렬화
        # 연속 실패/정체 누적기는 메모리에만 둔다. 목적이 "폭주 차단"이라 재기동에
        # 초기화돼도 해롭지 않고(다시 관찰하면 된다), 컬럼 추가는 이 저장소에
        # 마이그레이션 기구가 없어 기존 배포본 DB 를 깨뜨린다(T4 이연 항목).
        self._tick_errors: dict[int, int] = {}
        self._stalled: dict[int, int] = {}
        self._notified: set[int] = set()         # 정체 알림을 이미 낸 임무(에피소드당 1회)

    # ── 생성·취소 ───────────────────────────────────────────────────────────
    def create(self, preset: str, params: dict, *, created_by: int) -> list[int]:
        return self.create_from_plans(preset, params,
                                      presets.build(preset, params),
                                      created_by=created_by)

    def create_from_plans(self, preset: str, params: dict,
                          plans: list[presets.Plan], *, created_by: int) -> list[int]:
        """계획(프리셋 산출물) → bt_instances 행. 프리셋 1회 호출이 인스턴스 N개다
        (분담 정찰은 2개 — 로봇마다 하나)."""
        ids: list[int] = []
        with self._factory() as db:
            for plan in plans:
                robot = db.get(Robot, plan.robot_id)
                if robot is None:
                    raise presets.PresetError(f"로봇이 없습니다: {plan.robot_id}")
                row = BTInstance(preset=preset, params_json=dict(params or {}),
                                 robot_id=robot.id, farm_id=robot.farm_id,
                                 state=_ACTIVE, tree_json=plan.tree.to_state(),
                                 created_by=created_by)
                db.add(row)
                db.flush()
                ids.append(row.id)
            db.commit()
        return ids

    async def cancel(self, instance_id: int) -> BTInstance | None:
        """인스턴스 중단 + 그 트리가 띄운 활성 임무 회수.

        임무를 함께 취소하지 않으면 로봇은 계속 달리고 통로 잠금도 남는다 —
        QUEUED_LOCK 으로 대기 중이던 임무도 여기서 정리된다(로봇당 활성 임무
        1개 규칙 때문에 남겨두면 그 로봇에 새 임무를 낼 수 없다)."""
        async with self._lock:
            with self._factory() as db:
                inst = db.get(BTInstance, instance_id)
                if inst is None:
                    return None
                if inst.state == _ACTIVE:
                    await self._cancel_missions(db, inst)
                    inst.state = "CANCELED"
                    inst.updated_at = dt.datetime.now(dt.UTC)
                    db.commit()
                return inst

    async def _cancel_missions(self, db, inst: BTInstance) -> None:
        for mid in mission_ids(inst.tree_json):
            ms = db.get(Mission, mid)
            if ms is None or ms.state not in mission_ops.ACTIVE_MISSION_STATES:
                continue
            try:
                await mission_ops.apply_verb(db, self.fleet, ms, "cancel")
            except (MissionOpError, InvalidTransition) as e:
                log.warning("BT#%s 임무 %s 취소 실패: %s", inst.id, mid, e)

    # ── 틱 ──────────────────────────────────────────────────────────────────
    async def tick_once(self) -> None:
        async with self._lock:
            with self._factory() as db:
                ids = [r.id for r in db.query(BTInstance)
                       .filter(BTInstance.state == _ACTIVE)
                       .order_by(BTInstance.id).all()]
            for iid in ids:
                await self._tick_instance(iid)
            try:
                self._sweep_stalled_missions()
            except Exception:                   # 감시가 터져도 인스턴스 틱은 계속돈다
                log.exception("좀비 임무 감시 실패")

    async def _tick_instance(self, instance_id: int) -> None:
        """인스턴스 한 번 틱. 예외는 **이번 틱의 실패**일 뿐 종착이 아니다.

        예전에는 예외를 곧장 FAILED 로 종착시켰는데, 그러면 이미 발진한 임무가
        QUEUED 인 채 통로 잠금을 쥐고 남고(로봇은 달리고 있을 수도 있다) BT
        cancel 은 그 인스턴스가 RUNNING 이 아니라 아무것도 회수하지 못했다.
        게다가 tree_json 저장이 try 안이라 방금 붙인 mission_id 까지 유실됐다.
        지금은 진행 상태를 어떤 경로에서도 저장하고, 연속 실패가 한계를 넘을
        때만 종착시킨다(그때도 tree_json 은 보존 — 사람이 회수할 근거다).
        """
        with self._factory() as db:
            inst = db.get(BTInstance, instance_id)
            if inst is None or inst.state != _ACTIVE:
                return                          # 그 사이 취소됨
            try:
                tree = nodes.from_state(inst.tree_json)
            except Exception as e:              # 트리를 못 읽는다 — 재시도해도 같다
                log.exception("BT#%s 트리 복원 실패", instance_id)
                inst.state = "FAILED"
                inst.note = f"트리 복원 실패: {e}"[:160]
                inst.updated_at = dt.datetime.now(dt.UTC)
                db.commit()
                return
            result: str | None = None
            try:
                result = await tree.tick(EngineCtx(db, self.fleet, inst))
            except Exception as e:              # 한 인스턴스의 사고로 큐 전체가 멈추면 안 된다
                log.exception("BT#%s 틱 실패", instance_id)
                errors = self._tick_errors.get(instance_id, 0) + 1
                self._tick_errors[instance_id] = errors
                inst.note = f"엔진 예외({errors}/{self.max_tick_errors}): {e}"[:160]
                if errors >= self.max_tick_errors:
                    inst.state = "FAILED"       # 폭주 차단 — 여기서만 종착시킨다
            else:
                self._tick_errors.pop(instance_id, None)
                inst.state = _RESULT_STATE.get(result, _ACTIVE)
            inst.tree_json = tree.to_state()    # 예외 경로에서도 진행 상태 보존
            inst.updated_at = dt.datetime.now(dt.UTC)
            db.commit()

    # ── 좀비 임무 감시 ──────────────────────────────────────────────────────
    def _sweep_stalled_missions(self) -> None:
        """로봇이 들고 있지 않은 임무를 정리한다 — 증거의 종류에 따라 다르게.

        로봇이 임무 도중 재기동하면 완료/취소 보고가 영영 오지 않는다. 서버
        임무는 RUNNING(또는 수락 신호까지 잃었으면 QUEUED)으로 굳고, 통로 잠금은
        잡힌 채, BT Action 은 영원히 running 이다. 서버가 이미 받고 있는 것만으로
        그 괴리를 본다 — 로봇 계약은 건드리지 않는다.

        **적극적 증거(온라인 ∧ 최신 mode=idle) → 종착.** 로봇이 살아서 "나는
        임무 중이 아니다"라고 말하고 있다. 관문 경유 fail 이므로 잠금이 풀린다.
        RUNNING 은 stall_ticks, QUEUED 는 queued_stall_ticks(더 길게 — 수락 직후
        구간의 idle 을 오귀속하지 않기 위해).

        **증거의 부재(오프라인) → 알림만, 자동 종착 없음(라운드 2).** 로봇의 링크
        단절 정책은 임무를 버리지 않는다(robomw safety.py — paused 만 세우고 해제는
        명시적 resume 뿐). 여기서 임무를 실패시켜 잠금을 풀면, 로봇이 통로 안에
        임무를 쥐고 서 있는데 서버는 그 통로를 다른 로봇에게 내준다 — 링크 복구
        후 사람이 resume 하면 서버가 비었다고 믿는 통로를 달린다. T4 C3 가 일부러
        사람에게 맡긴 판단이라, 개입 큐로 부르고 기다린다(회수는 사람의 오프라인
        cancel — 그 경로가 잠금까지 푼다).
        """
        with self._factory() as db:
            rows = db.query(Mission).filter(Mission.state.in_(_STALL_WATCHED)).all()
            live_ids = {ms.id for ms in rows}
            for mid in list(self._stalled):
                if mid not in live_ids:
                    del self._stalled[mid]      # 종착한 임무의 누적은 버린다
                    self._notified.discard(mid)
            for ms in rows:
                why = self._robot_lost_mission(db, ms)
                if why is None:                 # 정상 — 에피소드 종료
                    self._stalled.pop(ms.id, None)
                    self._notified.discard(ms.id)
                    continue
                n = self._stalled.get(ms.id, 0) + 1
                self._stalled[ms.id] = n
                limit = (self.stall_ticks if ms.state == "RUNNING"
                         else self.queued_stall_ticks)
                if n < limit:
                    continue
                if why == _WHY_OFFLINE:         # 증거의 부재 — 사람에게 넘긴다
                    self._notify_link_stall(db, ms, n)
                    continue
                reason = f"로봇이 임무를 들고 있지 않음 — {why} {n}틱 연속"
                try:
                    missions.apply(db, ms, "fail", payload={"reason": reason,
                                                            "source": "bt_watchdog"})
                    log.warning("임무 %s 좀비 판정 — FAILED (%s)", ms.id, reason)
                except InvalidTransition:
                    pass                        # 그 사이 정상 종착 — 무시
                self._stalled.pop(ms.id, None)

    def _notify_link_stall(self, db, ms: Mission, ticks: int) -> None:
        """정체 에피소드당 개입 알림 1회 — 틱마다 큐를 두드리지 않는다.

        코드는 기존 LINK_LOST_POLICY 를 쓴다(코드표 확장 없이). 로봇이 스스로
        내는 같은 사유의 호출은 링크가 끊긴 동안 서버에 닿을 수 없으므로, 이
        서버측 관측이 그 티켓을 대신 연다 — 링크가 돌아와 로봇 호출이 도착하면
        open_or_bump 가 같은 건을 잇는다."""
        if ms.id in self._notified:
            return
        self._notified.add(ms.id)
        interventions.open_or_bump(
            db, robot_id=ms.robot_id, farm_id=ms.farm_id, code="LINK_LOST_POLICY",
            msg=f"임무#{ms.id} 진행 중 링크 단절 — 서버가 종착 신호를 받지 못한다",
            severity="warn",
            context={"mission_id": ms.id, "state": ms.state, "ticks": ticks,
                     "source": "bt_watchdog"})
        log.warning("임무 %s 링크 단절 정체 %d틱 — 개입 알림(자동 종착 안 함)",
                    ms.id, ticks)

    def _robot_lost_mission(self, db, ms: Mission) -> str | None:
        """이 임무를 로봇이 더는 들고 있지 않다고 볼 근거(사유) 또는 None(보류)."""
        if not self.fleet.robot_status(ms.robot_id).online:
            return _WHY_OFFLINE                 # 종착 신호가 올 길이 없다
        row = (db.query(Track).filter(Track.robot_id == ms.robot_id)
               .order_by(Track.ts.desc()).first())
        if row is None:
            return None                         # 아직 아무 보고도 없다 — 판단 보류
        # 낡은 증거는 증거가 아니다: tel/state 만 멈추고 링크(ping)는 살아 있는
        # 경우, 임무 이전의 idle 행이 계속 "최신"으로 읽혀 주행 중인 임무를
        # 죽인다. 신선도 하한을 못 넘으면 판단을 보류한다.
        ts = row.ts if row.ts.tzinfo else row.ts.replace(tzinfo=dt.UTC)
        if (dt.datetime.now(dt.UTC) - ts).total_seconds() > self.track_fresh_s:
            return None
        return _WHY_IDLE if str(row.mode or "") == "idle" else None

    # ── 수명주기(서버 lifespan) ─────────────────────────────────────────────
    def restore(self) -> int:
        """기동 시 이어받을 RUNNING 인스턴스 수. 상태는 tree_json 에 이미 있으므로
        되살릴 것은 없다 — Action 은 다음 틱에 mission_id 로 임무 상태를 다시
        판정한다(서버가 죽은 사이 끝났으면 그 틱에 성공/실패가 확정된다)."""
        with self._factory() as db:
            n = db.query(BTInstance).filter(BTInstance.state == _ACTIVE).count()
        if n:
            log.info("BT 인스턴스 %d개 복원(RUNNING)", n)
        return n

    async def start(self) -> None:
        if self._task is not None and not self._task.done():
            return
        self._task = asyncio.create_task(self._run(), name="bt-engine")

    async def _run(self) -> None:
        while True:
            try:
                await self.tick_once()
            except asyncio.CancelledError:
                raise
            except Exception:                   # 틱 루프는 무슨 일이 있어도 살아 있어야 한다
                log.exception("BT 엔진 틱 오류")
            await asyncio.sleep(self.period_s)

    async def stop(self) -> None:
        task, self._task = self._task, None
        if task is None:
            return
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await task
