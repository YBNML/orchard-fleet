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

    def emit_result(self, cmd_id, cmd, status, code="OK", data=None):
        """명령이 끝난 **나중에** 나오는 결과 (임무 완료 보고 등).

        기능이 Context.emit_cmd_result 로 부르면 여기로 온다. 즉시 답하는
        결과와 같은 경로·같은 캐시를 쓰는 이유: 관제가 같은 cmd_id 를 다시
        물었을 때 마지막 답이 그대로 나와야 하고, 발행 형식이 두 벌이 되면
        언젠가 갈라진다.
        """
        if not cmd_id:
            return None
        return self._result(cmd_id, cmd, status, code, data)

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
        try:
            handled = self._reg.dispatch(cmd, payload)
        except Exception as e:
            # 어댑터·기능이 터져도 프로세스는 산다 (스펙 §5). 다만 **조용히
            # 삼키지 않는다** — 명령이 안 먹은 것을 관제가 모르면 운전자는
            # 로봇이 멈춘 줄 알고 걸어서 접근한다. 결과(INTERNAL)로 답하고,
            # 같은 사실을 개입 큐로도 올린다(assistance/critical).
            res = (self._result(cmd_id, cmd, "failed", "INTERNAL",
                                {"reason": str(e)[:200]}) if cmd_id else None)
            self._emit({"kind": "assistance",
                        "msg": f"명령 처리 내부 오류: {cmd}",
                        "level": "critical", "code": "CMD_INTERNAL"})
            return res
        if cmd_id:
            done = self._cache.get(cmd_id)
            if done is not None:
                # 기능이 처리 도중에 이미 답을 냈다 (측위 미준비 거부·잘못된
                # work·즉시 완료). 그 답이 최종이다 — 여기서 accepted 로
                # 덮으면 거부가 수락으로 뒤집혀 관제가 임무가 도는 줄 안다.
                # (입구에서 캐시를 확인하고 왔으므로 이 항목은 방금 생긴 것이다)
                return done
            if handled:
                return self._result(cmd_id, cmd, "accepted", "OK")
            return self._result(cmd_id, cmd, "rejected", "BAD_PARAM",
                                {"reason": "처리 기능 없음"})
        return None
