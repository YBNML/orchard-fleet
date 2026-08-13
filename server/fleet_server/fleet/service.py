"""텔레메트리 허브 — FleetPort 수신을 DB 수집·최신값 캐시·구독자 팬아웃으로.

**로봇 보고 → 서버 임무 상태기계**의 입구가 여기다. 경로가 셋이다:

1. `evt` 의 `cmd_result(cmd="mission_start")` — cmd_id(f"m{id}")로 정확히 상관되는
   정본. accepted→RUNNING · completed→DONE(완료 보고 data 포함) · rejected→FAILED.
2. `evt` 의 임무 kind(`mission_started`/`mission_done`/`mission_cancelled`) —
   상관 키가 없어 "그 로봇의 최신 활성 임무" 휴리스틱을 쓰는 보완재.
3. `mission` 채널 `{"state": ...}` — 계약(T_MISSION)에는 있지만 **현 로봇 스택은
   아무도 발행하지 않는다**. 그래서 1·2가 없던 시절 실기 임무는 QUEUED 에서
   벗어나지 못했다(임무 14~21 전부 started_at NULL) — 완료가 없으니 AlleyLock 도
   풀리지 않았고, BT Action 은 성공 판정을 받을 수 없었다.

전이는 전부 `missions.apply` 관문을 지난다 — 종착 시 통로 잠금을 푸는 훅이 그
관문 안에 있다. 중복 신호(1과 2가 같은 사건을 두 번 알린다)는 관문이 불가 전이를
막아 자연히 멱등이 된다.
"""
from __future__ import annotations

import re
from typing import Callable

from .. import ingest, missions
from ..models import Mission, MissionEvent

_ROBOT_STATE_EVENT = {"running": "start", "paused": "pause", "done": "complete",
                      "canceled": "cancel", "failed": "fail"}

# 로봇이 실제로 내는 임무 evt kind → 임무 전이.
#
# 이름은 상상하지 않고 로봇 소스에서 확정했다 — robomw
# `profiles/orchard/mission.py` 의 `ctx.event(...)` 호출부(mission_started /
# mission_cancelled / mission_done)와 실기 events 테이블에 남은 payload 다.
# **채널 "mission"(T_MISSION) 은 계약에는 있지만 어떤 노드도 발행하지 않는다**
# — 그래서 _sync_mission 만 있던 시절 실기 임무는 QUEUED 에서 영영 못 벗어났고
# (임무 14~21 전부 started_at NULL), 완료가 없으니 AlleyLock 도 안 풀렸다.
# 이 매핑이 그 구멍을 로봇 계약 변경 없이 서버 쪽에서 메운다.
#
# **`mission_cancelled` 는 일부러 뺐다(리뷰 I-3).** 취소는 언제나 서버가 먼저
# 아는 사건이다 — REST·BT 는 apply_verb 로 전이를 커밋한 뒤에 명령을 보낸다.
# 그러니 이 kind 로 새로 얻는 것은 없는데, 상관 키가 없어 지연 도착하면
# "그 로봇의 최신 활성 임무" = **직후에 발진한 다음 임무**를 취소해 버린다
# (QUEUED 에서 cancel 은 합법 전이라 조용히 성공하고 통로 잠금까지 풀린다).
# 얻는 것 없이 잃을 것만 있는 매핑이라 넣지 않는다 — 취소 종착은 서버 주도
# apply_verb 와 cmd_result 상관 경로가 이미 맡는다.
_EVT_KIND_EVENT = {"mission_started": "start", "mission_done": "complete"}

# cmd_result(cmd="mission_start") 의 status → 임무 전이. 이쪽은 cmd_id 가
# 상관 키라 휴리스틱이 필요 없다(서버가 f"m{id}" 로 짓는다 — I4 와 같은 규약).
# 실기 payload 예: {"kind":"cmd_result","cmd_id":"m21","cmd":"mission_start",
#   "status":"completed","code":"OK","data":{"alleys_done":[7,6,5],
#   "distance_m":244.7,"duration_s":3963.8,"interventions":0,"coverage":1.0}}
_CMD_STATUS_EVENT = {"accepted": "start", "completed": "complete",
                     "rejected": "fail"}
_ACTIVE_FOR_ROBOT_REPORT = ["QUEUED", "RUNNING", "PAUSED"]

# 리뷰 라운드 1 (I4) — mission_start 의 cmd_id 는 mission_routes.create_mission
# 이 항상 f"m{mission.id}" 로 짓는다(검증 동사 cmd_id 는 "m{id}-{verb}" 라
# 대시가 있어 이 정규식에 안 걸린다 — mission_start 만 골라낸다). 이 관례를
# 파싱해 mission_id 를 되찾는다 — 재기동으로 잃어도 되는 인메모리 상관표
# 대신 이미 존재하는 결정적 규약을 재사용한 것(재기동 후 유실되는 경우는
# C3 의 로컬 cancel 로 회수 가능하다).
_MSTART_CMD_ID_RE = re.compile(r"^m(\d+)$")


class FleetService:
    def __init__(self, session_factory):
        self._factory = session_factory
        self.latest: dict[str, dict[str, dict]] = {}
        self._subs: list[Callable[[str, str, dict], None]] = []

    def attach(self, fleet) -> None:
        fleet.set_telemetry_handler(self.on_telemetry)

    def subscribe(self, cb: Callable[[str, str, dict], None]):
        self._subs.append(cb)
        def unsub():
            if cb in self._subs:
                self._subs.remove(cb)
        return unsub

    def on_telemetry(self, robot_id: str, channel: str, payload: dict,
                     seq: int | None) -> None:
        self.latest.setdefault(robot_id, {})[channel] = payload
        if channel == "tel/state":
            with self._factory() as db:
                ingest.track(db, robot_id, payload)
        elif channel == "evt":
            with self._factory() as db:
                fresh = ingest.event(db, robot_id, channel, seq, payload)
                if fresh:                   # 재전송 방어는 ingest 의 seq 중복 제거가 맡는다
                    self._route_intervention(db, robot_id, payload)
                    self._consume_cmd_result(db, robot_id, payload)
                    self._consume_mission_evt(db, robot_id, payload)
        elif channel == "mission":
            self._sync_mission(robot_id, payload)
        for cb in list(self._subs):
            cb(robot_id, channel, payload)

    # 로봇 이벤트 → 개입 큐. 이벤트가 곧 티켓이 되는 지점이다.
    #   kind 가 코드표에 있으면 그것을, 아니면 payload.code 를 본다 —
    #   구형 에이전트(kind="estop")와 신형(code="ESTOP_REMOTE") 둘 다 받는다.
    _KIND_TO_CODE = {"estop": "ESTOP_REMOTE", "estop_cleared": None,
                     "link_lost": "LINK_LOST_POLICY", "tilt": "TILT_LIMIT"}

    def _route_intervention(self, db, robot_id: str, payload: dict) -> None:
        from .. import interventions, stopcodes
        from ..models import Robot
        kind = str(payload.get("kind", ""))
        code = payload.get("code") or self._KIND_TO_CODE.get(kind)
        if kind in ("estop_cleared", "resolved") and payload.get("code"):
            interventions.auto_resolve(db, robot_id, payload["code"])
            return
        if not code or not stopcodes.is_intervention(code):
            return
        robot = db.get(Robot, robot_id)
        if robot is None:
            return
        interventions.open_or_bump(
            db, robot_id=robot_id, farm_id=robot.farm_id, code=code,
            msg=str(payload.get("msg", ""))[:256],
            severity=str(payload.get("severity", "warn")),
            context={k: payload[k] for k in ("x", "y", "alley") if k in payload})

    def _consume_cmd_result(self, db, robot_id: str, payload: dict) -> None:
        """mission_start 의 명령 결과로 임무 상태기계를 돌린다 — 상관은 cmd_id.

        accepted → RUNNING(수락 시점이 곧 발진), completed → DONE(완료 보고
        data 를 그 전이 이벤트에 싣는다), rejected → FAILED(T4 I4: BUSY 등으로
        거부되면 서버 임무가 QUEUED 에 멈추고 AlleyLock 이 고착된다).

        전이는 반드시 missions.apply 관문을 지난다 — 종착 시 통로 잠금을 푸는
        훅이 그 관문 안에 있다(missions.py). 여기서 상태를 직접 쓰면 그 해제가
        빠진다.
        """
        if payload.get("kind") != "cmd_result" or payload.get("cmd") != "mission_start":
            return
        event = _CMD_STATUS_EVENT.get(str(payload.get("status", "")))
        if event is None:
            return
        m = _MSTART_CMD_ID_RE.match(str(payload.get("cmd_id", "")))
        if not m:
            return                              # 서버가 낸 임무가 아니다(스크립트 cmd_id)
        ms = db.get(Mission, int(m.group(1)))
        if ms is None or ms.robot_id != robot_id:
            return
        data = payload.get("data") or {}
        if event == "fail":
            ev_payload = {"reason": data.get("reason") or payload.get("code") or "거부",
                          "code": payload.get("code")}
        else:
            ev_payload = dict(data)
        try:
            missions.apply(db, ms, event, payload=ev_payload)
            return
        except missions.InvalidTransition:
            pass
        # 여기부터는 전이가 불가했던 경우다. 두 갈래가 있다.
        if event == "complete" and ms.state == "QUEUED" and self._catch_up(db, ms, ev_payload):
            return
        # 이미 그 상태다 — 중복 신호(evt kind 가 먼저 왔거나 재전송)라 전이는
        # 없던 일로 한다. 다만 완료 보고(운행 실적)까지 버리면 안 된다.
        if event == "complete" and data:
            self._record_report(db, ms, data)

    @staticmethod
    def _catch_up(db, ms: Mission, ev_payload: dict) -> bool:
        """수락 신호를 놓친 임무의 완료 보고를 받아 따라잡는다.

        실기에서 실제로 났다(임무 23): 로봇의 accepted 가 수집 단계에서
        유실돼(§11 seq 중복 판정) 서버 임무는 QUEUED 인데 완주 보고만 도착했다.
        QUEUED→complete 는 불가 전이라 임무는 그대로 굳고 통로 잠금을 영원히
        쥔다 — Action 도 영원히 running 이다. **cmd_id 로 정확히 상관된** 완료
        보고는 그 임무의 것이 확실하므로, 잃어버린 start 를 먼저 적용한 뒤
        종착시킨다(둘 다 관문 경유 — 잠금 해제 훅을 탄다). 상관 키가 없는
        kind 경로에는 이 권한을 주지 않는다."""
        try:
            missions.apply(db, ms, "start", payload={"catch_up": "수락 신호 유실"})
            missions.apply(db, ms, "complete", payload=ev_payload)
            return True
        except missions.InvalidTransition:
            return False

    @staticmethod
    def _record_report(db, ms: Mission, data: dict) -> None:
        """완료 보고(alleys_done·distance_m·duration_s·interventions·coverage)를
        임무 이벤트로 남긴다. 이미 적힌 임무면 다시 적지 않는다(재전송 멱등)."""
        for ev in db.query(MissionEvent).filter(
                MissionEvent.mission_id == ms.id,
                MissionEvent.kind.in_(("complete", "report"))):
            if (ev.payload_json or {}).get("distance_m") is not None:
                return
        db.add(MissionEvent(mission_id=ms.id, kind="report", payload_json=dict(data)))
        db.commit()

    def _consume_mission_evt(self, db, robot_id: str, payload: dict) -> None:
        """화면용 임무 이벤트(mission_started/done/cancelled)도 상태기계로 넘긴다.

        cmd_result 가 상관 키를 갖는 정본이고 이쪽은 보완재다 — 상관 키가 없어
        "그 로봇의 최신 활성 임무" 휴리스틱을 쓴다. 그 휴리스틱이 성립하는
        근거는 로봇당 활성 임무 1개 불변(T4 I7)이다. 두 신호가 겹쳐도 전이가
        한 번만 먹히므로(관문이 불가 전이를 막는다) 결과는 같다."""
        event = _EVT_KIND_EVENT.get(str(payload.get("kind", "")))
        if event is None:
            return
        ms = self._active_mission(db, robot_id)
        if ms is None:
            return
        try:
            missions.apply(db, ms, event, payload={"kind": payload.get("kind"),
                                                   "msg": payload.get("msg", "")})
        except missions.InvalidTransition:
            pass                                # 이미 같은 상태 등 — 무시

    @staticmethod
    def _active_mission(db, robot_id: str) -> Mission | None:
        """그 로봇의 최신 활성 임무. QUEUED_LOCK 은 로봇에 나간 적이 없으므로
        제외한다 — 로봇 보고가 가리킬 수 있는 임무가 아니다."""
        return (db.query(Mission)
                .filter(Mission.robot_id == robot_id,
                        Mission.state.in_(_ACTIVE_FOR_ROBOT_REPORT))
                .order_by(Mission.id.desc()).first())

    def _sync_mission(self, robot_id: str, payload: dict) -> None:
        ev = _ROBOT_STATE_EVENT.get(str(payload.get("state", "")).lower())
        if ev is None:
            return
        with self._factory() as db:
            ms = self._active_mission(db, robot_id)
            if ms is None:
                return
            try:
                missions.apply(db, ms, ev, payload=payload)
            except missions.InvalidTransition:
                pass                            # 이미 같은 상태 등 — 무시
