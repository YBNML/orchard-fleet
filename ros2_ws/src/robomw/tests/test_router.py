from robomw.core.router import CommandRouter
import robomw.link.protocol as P


class FakeReg:
    def __init__(self): self.calls = []
    def dispatch(self, cmd, payload): self.calls.append(cmd); return True


def mk(supported=lambda c: True):
    events = []
    reg = FakeReg()
    r = CommandRouter(reg, events.append, supported)
    return r, reg, events


def test_accepted_result_and_dispatch():
    r, reg, events = mk()
    res = r.handle("mission_pause", {"cmd_id": "c1"}, P.ROLE_OPERATOR)
    assert res["status"] == "accepted" and reg.calls == ["mission_pause"]
    assert events and events[-1]["kind"] == "cmd_result"


def test_denied_no_dispatch():
    r, reg, events = mk()
    res = r.handle("set_mode", {"cmd_id": "c2"}, P.ROLE_OBSERVER)
    assert res["status"] == "rejected" and res["code"] == "DENIED" and not reg.calls


def test_idempotent_replay():
    r, reg, events = mk()
    a = r.handle("mission_pause", {"cmd_id": "c3"}, P.ROLE_OPERATOR)
    b = r.handle("mission_pause", {"cmd_id": "c3"}, P.ROLE_OPERATOR)
    assert reg.calls == ["mission_pause"]          # 재실행 없음
    assert b == a                                   # 직전 결과 재발행


def test_unsupported():
    r, reg, events = mk(supported=lambda c: c != P.CMD_SELF_TEST)
    res = r.handle(P.CMD_SELF_TEST, {"cmd_id": "c4"}, P.ROLE_OPERATOR)
    assert res["status"] == "rejected" and res["code"] == "UNSUPPORTED" and not reg.calls


def test_no_cmd_id_legacy_path():
    r, reg, events = mk()
    assert r.handle("mission_pause", {}, P.ROLE_OPERATOR) is None
    assert reg.calls == ["mission_pause"] and not events


def test_dispatch_exception_becomes_internal():
    class BoomReg:
        def dispatch(self, cmd, payload): raise RuntimeError("boom")
    events = []
    r = CommandRouter(BoomReg(), events.append, lambda c: True)
    res = r.handle("mission_pause", {"cmd_id": "c9"}, P.ROLE_OPERATOR)
    assert res["status"] == "failed" and res["code"] == "INTERNAL"
    assert any(e.get("kind") == "assistance" for e in events)


def test_dispatch_exception_without_cmd_id_still_warns():
    """cmd_id 가 없어도 경보는 올라가야 한다 — 조용히 죽는 것이 가장 나쁘다."""
    class BoomReg:
        def dispatch(self, cmd, payload): raise RuntimeError("boom")
    events = []
    r = CommandRouter(BoomReg(), events.append, lambda c: True)
    assert r.handle("mission_pause", {}, P.ROLE_OPERATOR) is None
    assert [e["kind"] for e in events] == ["assistance"]


def test_feature_result_wins_over_accepted():
    """기능이 처리 중에 낸 답(거부)을 라우터가 accepted 로 덮지 않는다.

    측위 미준비 BUSY 거부가 이 경로를 탄다 — 덮이면 관제는 임무가 도는 줄 안다.
    """
    events = []
    r = CommandRouter(None, events.append, lambda c: True)

    class RejReg:
        def dispatch(self, cmd, payload):
            r.emit_result(payload["cmd_id"], cmd, "rejected", "BUSY",
                          {"reason": "측위 미준비"})
            return True                     # 기능이 처리는 했다
    r._reg = RejReg()
    res = r.handle(P.CMD_MISSION_START, {"cmd_id": "c10"}, P.ROLE_OPERATOR)
    assert res["status"] == "rejected" and res["code"] == "BUSY"
    assert [e["status"] for e in events] == ["rejected"]      # accepted 발행 없음


def test_emit_result_later_is_cached_and_replayed():
    """임무 완료 보고처럼 나중에 나오는 결과도 같은 cmd_id 캐시에 남는다."""
    r, reg, events = mk()
    r.handle(P.CMD_MISSION_START, {"cmd_id": "c11"}, P.ROLE_OPERATOR)
    late = r.emit_result("c11", P.CMD_MISSION_START, "completed",
                         data={"coverage": 1.0})
    assert late["status"] == "completed" and late["data"]["coverage"] == 1.0
    again = r.handle(P.CMD_MISSION_START, {"cmd_id": "c11"}, P.ROLE_OPERATOR)
    assert again == late and reg.calls == [P.CMD_MISSION_START]   # 재실행 없음
    assert r.emit_result(None, P.CMD_MISSION_START, "completed") is None
