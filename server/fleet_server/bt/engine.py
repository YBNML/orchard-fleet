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

from .. import audit, mission_ops
from ..missions import InvalidTransition
from ..mission_ops import MissionOpError
from ..models import BTInstance, Mission, Robot
from ..traffic import AlleyLocks, conflict
from . import nodes, presets

log = logging.getLogger("fleet_server.bt")

_ACTIVE = "RUNNING"
_RESULT_STATE = {"success": "SUCCESS", "failure": "FAILED"}


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

    def __init__(self, session_factory, fleet, *, period_s: float = 1.0):
        self._factory = session_factory
        self.fleet = fleet                      # lifespan 이 레거시 포트로 교체할 수 있다
        self.period_s = period_s
        self._task: asyncio.Task | None = None
        self._lock = asyncio.Lock()             # 틱과 취소를 직렬화

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

    async def _tick_instance(self, instance_id: int) -> None:
        with self._factory() as db:
            inst = db.get(BTInstance, instance_id)
            if inst is None or inst.state != _ACTIVE:
                return                          # 그 사이 취소됨
            note = ""
            try:
                tree = nodes.from_state(inst.tree_json)
                result = await tree.tick(EngineCtx(db, self.fleet, inst))
                inst.tree_json = tree.to_state()
            except Exception as e:              # 한 인스턴스의 사고로 큐 전체가 멈추면 안 된다
                log.exception("BT#%s 틱 실패", instance_id)
                result, note = "failure", f"엔진 예외: {e}"[:160]
            inst.state = _RESULT_STATE.get(result, _ACTIVE)
            inst.note = note or inst.note
            inst.updated_at = dt.datetime.now(dt.UTC)
            db.commit()

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
