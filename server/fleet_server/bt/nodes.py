"""BT 노드 5종 — 순수 로직. DB·HTTP·FastAPI 를 모른다.

바깥 세계는 전부 `ctx` 콜백으로만 만진다:

    ctx.alley_free(alleys) -> bool        · ctx.robot_idle(robot) -> bool
    ctx.robot_online(robot) -> bool       · ctx.start_mission(spec) -> mission_id|None
    ctx.mission_status(mission_id) -> str · ctx.promote(mission_id) -> bool

**tick 은 코루틴이다(브리프의 `tick(ctx)` 서명에서 async 로).** 실제 발진은
`await fleet.send_command` 를 타야 하는데, 서버는 단일 이벤트루프 위에서
돌기 때문에 동기 tick 안에서 그 await 를 부를 방법이 없다(스레드로 넘기면
DB 세션과 루프를 가로질러야 한다). 노드가 async 여도 "순수"는 유지된다 —
노드는 여전히 콜백만 부르고 무엇이 어떻게 저장·전송되는지 모른다.

**`start_mission` 이 None 을 돌려주면 실패다(브리프 주석에서 이탈).** 브리프는
None 을 "잠금 대기"로 적었지만, T4 확정 사실에 따르면 잠금 대기는 임무 행이
QUEUED_LOCK 으로 *존재하는* 상태다 — 그 mission_id 를 Action 이 들고 있어야
틱마다 승격(잠금 재획득)을 시도할 수 있다. 그래서 잠금 대기는 mission_id 를
돌려주고(=running 유지), None 은 발진 자체가 거부된 경우(로봇에 활성 임무
409·오프라인·권한/검증 400)만 뜻한다. 이중시작 BUSY 는 그 경로로 실패해
Retry 규칙을 탄다.

직렬화: `to_state()` 는 진행 위치(Sequence 인덱스·Retry 시도수·Action
mission_id)를 포함한 JSON 트리, `from_state()` 는 그 역이다. 서버 재기동은
이 왕복만으로 이어 달린다.
"""
from __future__ import annotations

RUNNING, SUCCESS, FAILURE = "running", "success", "failure"

# 임무 상태 → Action 판정. DB 는 완료를 "DONE" 으로 적고(missions.TRANSITIONS)
# 스펙 문서는 COMPLETED 라 부른다 — 둘 다 성공으로 받는다.
_SUCCESS_STATES = frozenset({"DONE", "COMPLETED"})
_FAILURE_STATES = frozenset({"FAILED", "CANCELED"})
_LOCK_WAIT_STATE = "QUEUED_LOCK"


class Node:
    """모든 노드의 공통 뼈대 — 마지막 판정(last)을 상태에 남긴다(GET /bt 트리)."""

    kind = ""

    def __init__(self) -> None:
        self.last: str | None = None

    async def tick(self, ctx) -> str:
        self.last = await self._tick(ctx)
        return self.last

    async def _tick(self, ctx) -> str:               # pragma: no cover - 추상
        raise NotImplementedError

    def reset(self) -> None:
        """재시도를 위해 진행 상태를 초기화한다(last 는 관측값이라 남긴다)."""

    def to_state(self) -> dict:
        raise NotImplementedError                     # pragma: no cover - 추상

    def _base(self) -> dict:
        return {"kind": self.kind, "last": self.last}


class _Composite(Node):
    """자식 목록을 순서대로 도는 노드(Sequence·Selector) 공통."""

    def __init__(self, children: list[Node], i: int = 0, last: str | None = None):
        super().__init__()
        self.children = list(children)
        self.i = i
        self.last = last

    def reset(self) -> None:
        self.i = 0
        for c in self.children:
            c.reset()

    def to_state(self) -> dict:
        return {**self._base(), "i": self.i,
                "children": [c.to_state() for c in self.children]}


class Sequence(_Composite):
    """모두 성공해야 성공. 첫 실패에서 즉시 실패(빠른 실패), running 은 그대로 전달.

    진행 위치(i)는 틱을 넘어 유지된다 — 이미 통과한 Condition 을 매 틱 다시
    묻지 않는다(통과 후 로봇이 바빠졌다고 임무가 취소되면 안 된다)."""

    kind = "sequence"

    async def _tick(self, ctx) -> str:
        while self.i < len(self.children):
            r = await self.children[self.i].tick(ctx)
            if r == RUNNING:
                return RUNNING
            if r == FAILURE:
                return FAILURE
            self.i += 1
        return SUCCESS


class Selector(_Composite):
    """하나라도 성공하면 성공. 자식이 실패하면 다음 자식으로, 전부 실패면 실패."""

    kind = "selector"

    async def _tick(self, ctx) -> str:
        while self.i < len(self.children):
            r = await self.children[self.i].tick(ctx)
            if r in (RUNNING, SUCCESS):
                return r
            self.i += 1
        return FAILURE


class Retry(Node):
    """자식을 최대 n회 시도한다 — n번째 실패에서 실패 확정.

    실패 즉시 다시 틱하지 않고 running 을 돌려준다(다음 틱에 재시도). 1 Hz
    엔진에서 이는 곧 1초 백오프이고, 같은 틱 안에서 실패를 n번 몰아쳐 로봇에
    같은 명령을 연타하는 것을 막는다."""

    kind = "retry"

    def __init__(self, n: int, child: Node, tries: int = 0, last: str | None = None):
        super().__init__()
        if n < 1:
            raise ValueError("Retry n 은 1 이상이어야 합니다")
        self.n = int(n)
        self.child = child
        self.tries = tries
        self.last = last

    async def _tick(self, ctx) -> str:
        r = await self.child.tick(ctx)
        if r != FAILURE:
            return r
        self.tries += 1
        if self.tries >= self.n:
            return FAILURE
        self.child.reset()                            # 다음 시도는 새 임무로
        return RUNNING

    def reset(self) -> None:
        self.tries = 0
        self.child.reset()

    def to_state(self) -> dict:
        return {**self._base(), "n": self.n, "tries": self.tries,
                "child": self.child.to_state()}


class Condition(Node):
    """게이트 — 불충족은 실패가 아니라 running(대기)이다.

    실패로 처리하면 "로봇이 아직 안 켜졌다"가 임무 취소가 된다. BT 는 조건이
    충족될 때까지 그 자리에서 기다리는 것이 옳다(교통 정리도 같은 이유로
    alley_free 게이트를 Action 앞에 둔다)."""

    kind = "condition"
    KINDS = ("alley_free", "robot_idle", "robot_online")

    def __init__(self, cond: str, arg, last: str | None = None):
        super().__init__()
        if cond not in self.KINDS:
            raise ValueError(f"알 수 없는 조건 종류: {cond}")
        self.cond = cond
        self.arg = arg
        self.last = last

    async def _tick(self, ctx) -> str:
        ok = await getattr(ctx, self.cond)(self.arg)
        return SUCCESS if ok else RUNNING

    def to_state(self) -> dict:
        return {**self._base(), "cond": self.cond, "arg": self.arg}


class Action(Node):
    """임무 하나의 수명주기 — 발진(start_mission) 후 임무 상태를 판정한다.

    spec = {"robot": ..., "alleys"?: [...], "work"?: {...}} — REST 의 임무
    본문과 같은 모양이다(엔진이 그대로 공용 발진 경로에 넘긴다).

    QUEUED_LOCK 이면 running 을 유지하며 매 틱 승격을 시도한다 — 서버에는
    QUEUED_LOCK→발진 승격 경로가 없고(T4), 그것이 엔진의 몫이다."""

    kind = "action"

    def __init__(self, spec: dict, mission_id: int | None = None,
                 last: str | None = None):
        super().__init__()
        self.spec = dict(spec)
        self.mission_id = mission_id
        self.last = last

    async def _tick(self, ctx) -> str:
        if self.mission_id is None:
            mid = await ctx.start_mission(dict(self.spec))
            if mid is None:
                return FAILURE                        # 발진 거부 — Retry 가 받는다
            self.mission_id = mid
            return RUNNING
        state = (await ctx.mission_status(self.mission_id) or "").upper()
        if state in _SUCCESS_STATES:
            return SUCCESS
        if state in _FAILURE_STATES:
            return FAILURE
        if state == _LOCK_WAIT_STATE:
            await ctx.promote(self.mission_id)        # 잠금 재획득 시도(성공하면 발진)
        return RUNNING

    def reset(self) -> None:
        self.mission_id = None                        # 재시도는 반드시 새 임무로

    def to_state(self) -> dict:
        return {**self._base(), "spec": dict(self.spec), "mission_id": self.mission_id}


def from_state(state: dict) -> Node:
    """to_state() 의 역 — 진행 위치까지 그대로 되살린다."""
    kind = state["kind"]
    last = state.get("last")
    if kind == "sequence":
        return Sequence([from_state(c) for c in state["children"]],
                        i=state.get("i", 0), last=last)
    if kind == "selector":
        return Selector([from_state(c) for c in state["children"]],
                        i=state.get("i", 0), last=last)
    if kind == "retry":
        return Retry(state["n"], from_state(state["child"]),
                     tries=state.get("tries", 0), last=last)
    if kind == "condition":
        return Condition(state["cond"], state.get("arg"), last=last)
    if kind == "action":
        return Action(state["spec"], mission_id=state.get("mission_id"), last=last)
    raise ValueError(f"알 수 없는 노드 종류: {kind}")
