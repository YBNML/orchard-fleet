"""
유지보수 기능 — 자가진단(self_test)·재정위(relocalize)·블랙박스 덤프.

이번 태스크는 **self_test 만 구현**한다. commands 에는 세 명령을 다 선언해
두지만(관례상 "이 기능이 다룰 수 있는 명령"을 hello 에 그대로 드러내려는
것), relocalize·blackbox_dump 는 여전히 control_agent._cmd_supported 의
PENDING_CMDS 에 남아 있어 라우터가 여기 닿기 전에 UNSUPPORTED 로 되돌린다
(T3·T4 가 각각 구현하며 그때 PENDING_CMDS 에서 뺀다). 그래서 이 두 명령의
on_command 분기는 지금은 아무도 호출하지 않는 죽은 경로지만, 라우터가
먼저 열어주는 순간 바로 기능이 반응하도록 자리만 잡아 둔다.

self_test 는 **움직임을 만들지 않는다.** 이 파일에 구동 명령을 발행하는
코드가 없다 — drive 항목의 판정은 SDK Drive.limits() 조회뿐이다(실제 판정은
robomw ROS import 금지 원칙에 따라 bb.extra["sdk_diag"](ScoutDiag)가 한다).

robomw 는 ROS 를 모른다 — 이 파일이 아는 것은 bb.extra["sdk_diag"] 하나뿐이고,
그 객체가 무엇으로 구현됐는지(orchard_sim.adapters.scout_diag.ScoutDiag)는
호스트(control_agent)만 안다.
"""
from __future__ import annotations

from robomw.core.base import Feature
from robomw.link import protocol as P


class MaintenanceFeature(Feature):
    name = "maintenance"
    version = "1.0"
    summary = "자가진단·재정위·블랙박스 (self_test 만 구현)"
    commands = (P.CMD_SELF_TEST, P.CMD_RELOCALIZE, P.CMD_BLACKBOX_DUMP)
    topics = ()

    def _diag(self):
        return self.ctx.bb.extra.get("sdk_diag")

    def _in_mission(self):
        """임무 주행 중인가 — mode 와 mission 실존을 함께 본다.

        bb 에는 mission 객체 자체를 넣어 두지 않는다(그건 drive_mission 기능
        인스턴스 안에만 있는 `self.mission`). 그 존재를 블랙보드에서 볼 수 있는
        유일한 대리는 mission.py 가 매 틱 갱신하는
        bb.extra["mission_status"](임무 없음 → None, 있음 → dict)다 — 그래서
        mode==MODE_MISSION 과 함께 이걸 본다. mode 만으로도 사실상 같은 판정이
        나오지만(둘은 mission_start 에서 같은 순간에 세팅된다), 이렇게 이중으로
        보는 편이 둘 중 하나가 나중에 따로 바뀌어도 안전한 쪽으로 값싸다.
        """
        bb = self.ctx.bb
        return (bb.extra.get("mode") == P.MODE_MISSION
                and bb.extra.get("mission_status") is not None)

    def on_command(self, cmd, payload):
        if cmd == P.CMD_SELF_TEST:
            return self._self_test(payload)
        if cmd == P.CMD_RELOCALIZE:
            return False       # 스펙 ② T3 — 지금은 라우터가 UNSUPPORTED 로 막는다
        if cmd == P.CMD_BLACKBOX_DUMP:
            return False       # 스펙 ② T4 — 지금은 라우터가 UNSUPPORTED 로 막는다
        return False

    def _self_test(self, payload):
        cmd_id = payload.get("cmd_id")
        if self._in_mission():
            self.ctx.event("rejected", "주행 중 — 자가진단 불가", "warn")
            self.ctx.emit_cmd_result(cmd_id, P.CMD_SELF_TEST, "rejected",
                                     "BUSY", {"reason": "주행 중"})
            return True
        diag = self._diag()
        if diag is None:
            # 어댑터 미배선(테스트·미완성 호스트) — 조용히 성공한 척하지 않는다.
            self.ctx.emit_cmd_result(cmd_id, P.CMD_SELF_TEST, "failed",
                                     "INTERNAL", {"reason": "진단 어댑터 미배선"})
            return True
        items = payload.get("items") or None    # 빈 목록도 "전체"로 본다
        results = diag.self_test(items)
        data = dict(
            items=[dict(name=r.name, ok=r.ok, detail=r.detail) for r in results],
            all_ok=all(r.ok for r in results))
        self.ctx.emit_cmd_result(cmd_id, P.CMD_SELF_TEST, "completed", "OK", data)
        return True
