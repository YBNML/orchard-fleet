"""
유지보수 기능 — 자가진단(self_test)·재정위(relocalize)·블랙박스 덤프.

self_test 는 **움직임을 만들지 않는다.** 이 파일에 구동 명령을 발행하는
코드가 없다 — drive 항목의 판정은 SDK Drive.limits() 조회뿐이다(실제 판정은
robomw ROS import 금지 원칙에 따라 bb.extra["sdk_diag"](ScoutDiag)가 한다).

relocalize 는 **위치 추정만** 바꾼다. 기체는 그대로 서 있고 추정이 그 자리로
옮겨 갈 뿐이다 — 그래서 주행 중에는 거부한다(BUSY). 달리는 도중에 추정이
도약하면 추종기가 방금까지 쫓던 웨이포인트를 딴 곳으로 알고 조향한다.

blackbox_dump 는 **궤적·이벤트를 npz 덤프한다.** control_agent 의 1 Hz 타이머가
포즈를 피드하고, event() 호출이 이벤트를 피드한다. 최대 900초 윈도우.

robomw 는 ROS 를 모른다 — 이 파일이 아는 것은 블랙보드에 꽂힌 어댑터
(bb.extra["sdk_diag"]·bb.extra["sdk_loc"])와 현장 기하(bb.extra["site_geom"])
뿐이고, 그것들이 무엇으로 구현됐는지(orchard_sim.adapters.…)는 호스트
(control_agent)만 안다. **격자 상수를 이 파일에 적어 두지 않는 이유**가
그것이다 — 통로 간격·열 길이는 현장마다 다르고, hello 로 관제에 알리는 값과
여기서 쓰는 값이 갈라지면 운영자가 화면에서 고른 통로와 로봇이 뛰어드는
통로가 달라진다. 한 벌(site_geom)만 본다.
"""
from __future__ import annotations

import json
import math
import time

from robomw.core.base import Feature
from robomw.link import protocol as P
from robomw.sdk.types import Pose

# 격자 재정위의 단 정지선 — 열 끝(±col_len/2)에서 헤드랜드 쪽으로 이만큼
# 물러난 자리가 '통로 앞에 선' 자세다. 임무 경로의 진입 웨이포인트
# (mission.build_waypoints 의 y_lo·y_hi = ±(col_len/2 + headland·0.25))와 같은
# 대역이되, 그쪽은 지형(헤드랜드 폭)에 비례하고 이쪽은 고정이다 — 운영자가
# "3번 통로 남쪽 끝"이라고 말할 때 기대하는 자리는 헤드랜드 폭과 무관하게
# 열 끝 바로 앞이기 때문이다.
END_STANDOFF_M = 1.5


class MaintenanceFeature(Feature):
    name = "maintenance"
    version = "1.2"
    summary = "자가진단·재정위·블랙박스 덤프"
    commands = (P.CMD_SELF_TEST, P.CMD_RELOCALIZE, P.CMD_BLACKBOX_DUMP)
    topics = ()

    def _diag(self):
        return self.ctx.bb.extra.get("sdk_diag")

    def _loc(self):
        return self.ctx.bb.extra.get("sdk_loc")

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

    def _moving(self):
        """기체가 지금 움직이거나 곧 움직일 수 있는 상태인가. 아니면 None.

        **임무만 보면 안 된다.** 재정위는 확인을 기다리는 동안 호스트의 제어
        루프를 멈춰 세운다(어댑터 주석 참조) — 그 사이에는 속도 명령도,
        데드맨도, 비상정지 큐 소비도 돌지 않는다. 기체의 구동부는 마지막
        속도를 유지하므로(gz DiffDrive 에는 명령 만료가 없다) 텔레옵으로
        0.8 m/s 로 가는 중에 재정위가 들어오면 로봇은 눈을 감은 채 1.6 m 를
        더 간다. 그래서 '임무 중'이 아니라 **'유휴가 아니면'** 거부한다.

        두 겹으로 본다:
          mode  유휴가 아니면(임무·텔레옵) 언제든 바퀴가 돌 수 있다
          gate  안전 조정자가 마지막 중재에서 어떤 요청도 막지 않았다면("")
                방금 실제로 속도가 나갔다는 뜻이다 — 모드 표기가 늦거나
                (mission_start 직후 50 ms) 기능이 모드를 안 세운 경로까지 잡는다
        """
        mode = self.ctx.bb.extra.get("mode", P.MODE_IDLE)
        if mode != P.MODE_IDLE:
            return f"모드 {mode}"
        s = getattr(self.ctx, "safety", None)
        if s is not None and s.snapshot().get("gate") == "":
            return "속도 요청 통과 중"
        return None

    def on_command(self, cmd, payload):
        if cmd == P.CMD_SELF_TEST:
            return self._self_test(payload)
        if cmd == P.CMD_RELOCALIZE:
            return self._relocalize(payload)
        if cmd == P.CMD_BLACKBOX_DUMP:
            return self._blackbox_dump(payload)
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

    # ── 재정위 ──────────────────────────────────────────────────────────────
    def _grid_pose(self, alley, end):
        """{통로 번호, 단} → (x, y, yaw). 못 풀면 (None, 사유).

        기하는 호스트가 얹은 bb.extra["site_geom"](hello 의 site.geometry 와
        **같은 사전**)에서만 읽는다. 그 키가 없으면 계산하지 않는다 — 기본값을
        지어내면 화면이 말한 통로와 로봇이 믿는 통로가 갈라진다.
        """
        geom = self.ctx.bb.extra.get("site_geom")
        if not geom:
            return None, "현장 기하 미배선"
        try:
            n = int(geom["alleys"])
            S = float(geom["row_spacing"])
            x0 = float(geom["x0"])
            half = float(geom["col_len"]) / 2.0
        except (KeyError, TypeError, ValueError):
            return None, "현장 기하 형식 오류"
        try:
            k = int(alley)
        except (TypeError, ValueError):
            return None, "통로 번호가 정수가 아니다"
        if not 0 <= k < n:
            return None, f"통로 번호 범위 밖 (0~{n - 1})"
        if end not in ("north", "south"):
            return None, "단은 north 또는 south"
        sign = 1.0 if end == "north" else -1.0
        # 북단에 선 로봇은 남(-π/2)을, 남단에 선 로봇은 북(+π/2)을 본다 —
        # 어느 쪽이든 **통로 안쪽**이 정면이다(그 자리에서 바로 훑기 시작한다).
        return (x0 + (k + 0.5) * S,
                sign * (half + END_STANDOFF_M),
                -sign * math.pi / 2.0), None

    def _target_pose(self, payload):
        """payload → (x, y, yaw). 좌표 직접 지정이 우선, 없으면 격자 변환."""
        if any(kk in payload for kk in ("x", "y", "yaw")):
            try:
                return (float(payload["x"]), float(payload["y"]),
                        float(payload["yaw"])), None
            except (KeyError, TypeError, ValueError):
                return None, "x·y·yaw 를 모두 수로 줘야 한다"
        if "alley" in payload or "end" in payload:
            return self._grid_pose(payload.get("alley"), payload.get("end"))
        return None, "{x,y,yaw} 또는 {alley,end} 가 필요하다"

    def _relocalize(self, payload):
        """운영자가 준 자세로 측위를 다시 잡는다.

        이것은 **복구 명령**이다 — 로봇이 자기 위치를 잃었을 때(개입 큐의
        LOCALIZATION_LOST) 사람이 눈으로 확인한 자리를 알려 주는 것. 그래서
        결과를 정직하게 답해야 한다: 어댑터가 재초기화 후 측위 품질이 살아난
        것을 확인하지 못하면 completed 가 아니라 failed(TIMEOUT)다. 여기서
        성공한 척하면 관제는 로봇이 제자리를 안다고 믿고 임무를 재개한다.

        **completed 가 뜻하는 것의 한계.** 확인 잣대인 측위 품질은 스캔 구조의
        위상 집중도라 요 오차에는 민감하지만 위치 오지정에는 둔감하다 — 과수원은
        주기 구조라 한 통로 옆에서도 같은 값이 나온다(옆 통로 주기 모호성).
        즉 completed 는 "재초기화 뒤 측위가 건강하다"이지 "운영자가 준 자세가
        옳다"가 아니다. 자세가 옳은지는 사람이 눈으로 본 것이 근거다.
        """
        cmd_id = payload.get("cmd_id")
        why_moving = self._moving()
        if why_moving is not None:
            self.ctx.event("rejected", f"주행 중 — 재초기화 불가 ({why_moving})",
                           "warn")
            self.ctx.emit_cmd_result(cmd_id, P.CMD_RELOCALIZE, "rejected",
                                     "BUSY", {"reason": "주행 중 재초기화 불가",
                                              "state": why_moving})
            return True
        loc = self._loc()
        if loc is None:
            self.ctx.emit_cmd_result(cmd_id, P.CMD_RELOCALIZE, "failed",
                                     "INTERNAL", {"reason": "측위 어댑터 미배선"})
            return True
        xyz, why = self._target_pose(payload)
        if xyz is None:
            # 기하 미배선은 요청 탓이 아니라 호스트 탓이다 — 운영자가 payload 를
            # 고쳐도 안 되는 일이니 BAD_PARAM(요청 오류)으로 답하면 안 된다.
            host_fault = why.startswith("현장 기하")
            self.ctx.event("rejected", f"재정위 불가 — {why}", "warn")
            self.ctx.emit_cmd_result(
                cmd_id, P.CMD_RELOCALIZE,
                "failed" if host_fault else "rejected",
                "INTERNAL" if host_fault else "BAD_PARAM", {"reason": why})
            return True
        x, y, yaw = xyz
        ok = loc.reinit(Pose(x, y, yaw, 1.0))
        diag = dict(loc.diagnostics() or {})
        if not ok:
            self.ctx.event("rejected",
                           f"재정위 실패 — ({x:.2f}, {y:.2f}) 에서 측위 품질 미확인",
                           "warn")
            self.ctx.emit_cmd_result(cmd_id, P.CMD_RELOCALIZE, "failed",
                                     "TIMEOUT", diag)
            return True
        self.ctx.event("relocalized",
                       f"재정위 — ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)",
                       "info")
        self.ctx.emit_cmd_result(cmd_id, P.CMD_RELOCALIZE, "completed", "OK",
                                 {"quality": diag.get("quality")})
        return True

    # ── 블랙박스 덤프 ────────────────────────────────────────────────────────
    def _blackbox_dump(self, payload):
        """궤적·이벤트 npz 덤프.

        예외는 라우터가 INTERNAL 로 승격한다 (별도 try 불필요).
        """
        cmd_id = payload.get("cmd_id")
        window_s = payload.get("window_s", 900.0)
        diag = self._diag()
        if diag is None:
            self.ctx.emit_cmd_result(cmd_id, P.CMD_BLACKBOX_DUMP, "failed",
                                     "INTERNAL", {"reason": "진단 어댑터 미배선"})
            return True
        result = diag.blackbox_dump(window_s)
        if not result or "path" not in result:
            self.ctx.emit_cmd_result(cmd_id, P.CMD_BLACKBOX_DUMP, "failed",
                                     "INTERNAL", {"reason": "블랙박스 덤프 실패"})
            return True
        data = dict(
            path=result["path"],
            bytes=result.get("bytes", 0),
            events=result.get("events", 0),
            poses=result.get("poses", 0))
        self.ctx.emit_cmd_result(cmd_id, P.CMD_BLACKBOX_DUMP, "completed", "OK", data)
        return True
