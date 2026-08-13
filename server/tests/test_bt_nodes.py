"""BT 노드 5종의 의미론 — 순수 로직(DB·HTTP 없음), Fake ctx 의 반환 시퀀스로 단언.

ctx 콜백은 전부 코루틴이다: 실제 발진(start_mission)이 `await fleet.send_command`
를 타야 하므로 tick 도 async 다. 노드는 여전히 DB·HTTP 를 모른다 — 콜백만 부른다.
"""
from fleet_server.bt import nodes as N


class FakeCtx:
    """반환 시퀀스를 조작하는 가짜 컨텍스트. status/start 는 소진되면 마지막 값을 유지."""

    def __init__(self, *, free=True, idle=True, online=True,
                 start_ids=None, status=None, promote_ok=False):
        self.free, self.idle, self.online = free, idle, online
        self._start_ids = list(start_ids) if start_ids is not None else [1]
        self._status = list(status) if status is not None else ["QUEUED"]
        self.promote_ok = promote_ok
        self.started: list[dict] = []
        self.promoted: list[int] = []

    @staticmethod
    def _next(seq):
        return seq.pop(0) if len(seq) > 1 else seq[0]

    async def alley_free(self, alleys):
        return self.free

    async def robot_idle(self, robot):
        return self.idle

    async def robot_online(self, robot):
        return self.online

    async def start_mission(self, spec):
        self.started.append(spec)
        return self._next(self._start_ids)

    async def mission_status(self, mission_id):
        return self._next(self._status)

    async def promote(self, mission_id):
        self.promoted.append(mission_id)
        return self.promote_ok


def _action(robot="scout01", alleys=(0,)):
    return N.Action({"robot": robot, "alleys": list(alleys)})


# ── 브리프 Step 1 의 5종 ──────────────────────────────────────────────────────

async def test_sequence_fails_fast():
    """첫 실패에서 중단 — 뒤 자식은 아예 틱되지 않는다."""
    ctx = FakeCtx(start_ids=[None])                  # 첫 Action 의 발진이 거부됨
    seq = N.Sequence([_action(alleys=[0]), _action(alleys=[1])])
    assert await seq.tick(ctx) == "failure"
    assert [s["alleys"] for s in ctx.started] == [[0]]   # 두 번째 Action 미실행


async def test_retry_counts():
    """n회 시도 후 실패 확정 — 그 전까지는 running(다음 틱에 재시도)."""
    ctx = FakeCtx(start_ids=[None])
    node = N.Retry(2, _action())
    assert await node.tick(ctx) == "running"          # 1회 실패 → 재시도 예약
    assert await node.tick(ctx) == "failure"          # 2회째 실패 → 확정
    assert len(ctx.started) == 2


async def test_condition_waits():
    """불충족은 실패가 아니라 running(대기) — 충족되면 success."""
    ctx = FakeCtx(free=False)
    cond = N.Condition("alley_free", [0, 1])
    assert await cond.tick(ctx) == "running"
    ctx.free = True
    assert await cond.tick(ctx) == "success"


async def test_action_lifecycle():
    """start→running(mission_id 부착)→DONE=success / FAILED=failure."""
    ctx = FakeCtx(start_ids=[7], status=["RUNNING", "DONE"])
    act = _action()
    assert await act.tick(ctx) == "running"
    assert act.to_state()["mission_id"] == 7
    assert await act.tick(ctx) == "running"           # RUNNING 보고 중
    assert await act.tick(ctx) == "success"           # DONE
    fctx, fail = FakeCtx(start_ids=[8], status=["FAILED"]), _action()
    assert await fail.tick(fctx) == "running"
    assert await fail.tick(fctx) == "failure"         # 로봇 거부·실패 보고


async def test_state_roundtrip():
    """to_state→from_state 후 동일 거동 — Sequence 진행 위치·Action mission_id 유지."""
    ctx = FakeCtx(start_ids=[5], status=["RUNNING", "DONE"])
    seq = N.Sequence([N.Condition("robot_online", "scout01"), _action()])
    assert await seq.tick(ctx) == "running"           # 조건 통과 → Action 발진
    revived = N.from_state(seq.to_state())
    assert await revived.tick(ctx) == "running"       # 발진 재요청 없음(mission_id 재부착)
    assert len(ctx.started) == 1
    assert await revived.tick(ctx) == "success"


# ── 나머지 의미론 ────────────────────────────────────────────────────────────

async def test_selector_takes_first_non_failure():
    ctx = FakeCtx(start_ids=[None, 3], status=["RUNNING"])
    sel = N.Selector([_action(alleys=[0]), _action(alleys=[1])])
    assert await sel.tick(ctx) == "running"           # 첫 자식 실패 → 둘째로
    assert [s["alleys"] for s in ctx.started] == [[0], [1]]


async def test_selector_fails_when_all_children_fail():
    ctx = FakeCtx(start_ids=[None])
    sel = N.Selector([_action(alleys=[0]), _action(alleys=[1])])
    assert await sel.tick(ctx) == "failure"


async def test_sequence_completes_when_all_children_succeed():
    ctx = FakeCtx(start_ids=[9], status=["DONE"])
    seq = N.Sequence([N.Condition("robot_idle", "scout01"), _action()])
    assert await seq.tick(ctx) == "running"
    assert await seq.tick(ctx) == "success"


async def test_action_running_while_queued_lock_and_asks_for_promotion():
    """T4 이관 (c) — QUEUED_LOCK 은 실패가 아니다. running 을 유지하며 틱마다
    승격(잠금 재획득)을 시도한다."""
    ctx = FakeCtx(start_ids=[4], status=["QUEUED_LOCK"])
    act = _action()
    assert await act.tick(ctx) == "running"           # 발진 시도 → QUEUED_LOCK id 부착
    assert await act.tick(ctx) == "running"
    assert ctx.promoted == [4]
    assert await act.tick(ctx) == "running"
    assert ctx.promoted == [4, 4]                     # 매 틱 재시도


async def test_action_failure_when_start_rejected():
    """None = 발진 거부(활성 임무 409·오프라인 등) → 실패. Retry 규칙이 받는다."""
    ctx = FakeCtx(start_ids=[None])
    assert await _action().tick(ctx) == "failure"


async def test_action_canceled_mission_is_failure():
    ctx = FakeCtx(start_ids=[2], status=["CANCELED"])
    act = _action()
    await act.tick(ctx)
    assert await act.tick(ctx) == "failure"


async def test_retry_starts_a_new_mission_on_each_attempt():
    """재시도는 새 임무여야 한다 — 실패한 mission_id 를 붙든 채 재시도하면 안 된다."""
    ctx = FakeCtx(start_ids=[11, 12], status=["FAILED"])
    node = N.Retry(2, _action())
    assert await node.tick(ctx) == "running"          # 발진(11)
    assert await node.tick(ctx) == "running"          # 11 FAILED → 재시도 예약
    assert await node.tick(ctx) == "running"          # 발진(12)
    assert len(ctx.started) == 2


async def test_condition_unknown_kind_is_rejected():
    try:
        N.Condition("weather_nice", None)
    except ValueError:
        return
    raise AssertionError("알 수 없는 조건 종류는 거부해야 한다")


async def test_state_roundtrip_preserves_retry_counter():
    ctx = FakeCtx(start_ids=[None])
    node = N.Retry(2, _action())
    assert await node.tick(ctx) == "running"
    revived = N.from_state(node.to_state())
    assert await revived.tick(ctx) == "failure"       # 남은 시도 1회를 그대로 이어감


async def test_node_states_expose_last_result():
    """GET /bt 가 보여줄 트리 — 각 노드의 마지막 판정이 상태에 실린다."""
    ctx = FakeCtx(free=False)
    seq = N.Sequence([N.Condition("alley_free", [0, 1]), _action()])
    await seq.tick(ctx)
    st = seq.to_state()
    assert st["last"] == "running"
    assert st["children"][0]["last"] == "running"
    assert st["children"][1]["last"] is None          # 아직 틱되지 않음
