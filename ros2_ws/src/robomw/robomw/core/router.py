"""명령 라우터 — 권한 재판정·cmd_id 멱등·cmd_result 발행 (스펙 §2.2).

입구(wsserver 콜백)의 1차 판정과 별개로 소비 시점에 2차 판정한다(기존
control_agent 이중 판정 규약 유지). cmd_id 가 있으면 결과를 이벤트로
발행하고 최근 32건을 캐시해 재수신 시 재실행 없이 재발행한다.
"""
from collections import OrderedDict

from robomw.link import protocol as P


class CommandRouter:
    def __init__(self, registry, emit_event, supported):
        self._reg = registry
        self._emit = emit_event
        self._supported = supported
        self._cache = OrderedDict()          # cmd_id -> cmd_result dict

    def _result(self, cmd_id, cmd, status, code, data=None):
        res = P.make_cmd_result(cmd_id, cmd, status, code, data)
        self._cache[cmd_id] = res
        while len(self._cache) > 32:
            self._cache.popitem(last=False)
        self._emit(res)
        return res

    def handle(self, cmd, payload, role):
        cmd_id = payload.get("cmd_id")
        if cmd_id and cmd_id in self._cache:
            res = self._cache[cmd_id]
            self._emit(res)
            return res
        ok, reason = P.authorize(role, cmd)
        if not ok:
            return self._result(cmd_id, cmd, "rejected", "DENIED",
                                {"reason": reason}) if cmd_id else None
        if not self._supported(cmd):
            return self._result(cmd_id, cmd, "rejected", "UNSUPPORTED") if cmd_id else None
        handled = self._reg.dispatch(cmd, payload)
        if cmd_id:
            if handled:
                return self._result(cmd_id, cmd, "accepted", "OK")
            return self._result(cmd_id, cmd, "rejected", "BAD_PARAM",
                                {"reason": "처리 기능 없음"})
        return None
