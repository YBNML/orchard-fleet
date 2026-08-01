"""정지·개입 사유 코드표 — ISO 18497-3 위험 카탈로그 기반.

백지에서 만들지 않는다. 농업 자율주행 기계의 위험은 이미 목록화돼 있고,
그 목록을 그대로 코드로 쓰면 (1) 빠뜨리는 항목이 줄고 (2) 나중에 인증·감사
때 대응표를 새로 만들 필요가 없다.

각 코드는 네 가지를 말한다.
    label            사람이 읽는 이름 (화면·보고서에 그대로 쓴다)
    category         묶음 — 집계와 화면 분류에 쓴다
    needs_operator   사람의 판단이 필요한가 = **개입으로 센다**
    needs_site_visit 사람이 기체까지 가야 하는가 = 왕복 비용이 발생한다

`needs_site_visit` 이 지표의 핵심이다. 규격상 비상정지 해제는 현장 확인을
요구하므로(ISO 13849-1 §5.2.2), 관제가 아무리 빨라도 이 항목은 사람의 이동
시간만큼 걸린다 — 운영 원가를 결정하는 건 개입 횟수가 아니라 **이동 횟수**다.
"""
from __future__ import annotations

from dataclasses import dataclass

CAT_PERCEPTION = "인지"
CAT_SENSOR = "센서"
CAT_LOCALIZATION = "측위"
CAT_ATTITUDE = "자세"
CAT_TRACTION = "구동"
CAT_BOUNDARY = "경계"
CAT_POWER = "전원"
CAT_LINK = "통신"
CAT_COMMAND = "지시"
CAT_NORMAL = "정상"


@dataclass(frozen=True)
class StopCode:
    code: str
    label: str
    category: str
    needs_operator: bool
    needs_site_visit: bool


def _c(code, label, category, op=True, visit=False):
    return StopCode(code, label, category, op, visit)


CODES: dict[str, StopCode] = {c.code: c for c in (
    # ── 인지 (ISO 18497-3 의 감지 실패 계열) ────────────────────────────────
    _c("OBSTACLE_FRONT", "전방 장애물", CAT_PERCEPTION),
    _c("NEGATIVE_OBSTACLE", "전방 지면 소실 (법면·구덩이)", CAT_PERCEPTION, visit=True),
    _c("SLOPE_SCAN_DEVIATION", "경사로 스캔 평면 이탈", CAT_PERCEPTION),
    # ── 센서 ────────────────────────────────────────────────────────────────
    _c("SENSOR_CONTAMINATION", "센서 오염·가림", CAT_SENSOR, visit=True),
    _c("SENSOR_TIMEOUT", "센서 무응답", CAT_SENSOR, visit=True),
    # ── 측위 ────────────────────────────────────────────────────────────────
    _c("LOCALIZATION_LOST", "위치 상실 (정합 실패 지속)", CAT_LOCALIZATION),
    _c("LOCALIZATION_UNCERTAIN", "위치 불확실 (공분산 초과)", CAT_LOCALIZATION),
    # ── 자세·구동 ───────────────────────────────────────────────────────────
    _c("TILT_LIMIT", "기울기 한계 초과", CAT_ATTITUDE, visit=True),
    _c("TRACTION_LOSS", "구동 상실 (진흙·슬립)", CAT_TRACTION, visit=True),
    # ── 경계 ────────────────────────────────────────────────────────────────
    _c("GEOFENCE", "주행가능 영역 이탈 시도", CAT_BOUNDARY),
    # ── 전원·통신 ───────────────────────────────────────────────────────────
    _c("BATTERY_LOW", "저배터리 정지", CAT_POWER, visit=True),
    _c("LINK_LOST_POLICY", "링크 단절 정책 정지", CAT_LINK),
    # ── 지시 ────────────────────────────────────────────────────────────────
    # 비상정지는 규격상 현장 확인 없이 풀 수 없다 — 반드시 왕복이 생긴다.
    _c("ESTOP_REMOTE", "비상정지 (관제)", CAT_COMMAND, visit=True),
    _c("ESTOP_LOCAL", "비상정지 (현장)", CAT_COMMAND, visit=True),
    _c("OPERATOR_PAUSE", "지시 일시정지", CAT_COMMAND, op=False),
    # ── 정상 ────────────────────────────────────────────────────────────────
    _c("MISSION_DONE", "임무 완료", CAT_NORMAL, op=False),
)}

UNKNOWN = _c("UNKNOWN", "분류되지 않은 정지", CAT_PERCEPTION)


def get(code: str | None) -> StopCode:
    """미지 코드는 UNKNOWN 으로 떨어뜨린다 — 로봇이 새 코드를 보내도 집계가 안 깨진다."""
    return CODES.get((code or "").upper(), UNKNOWN)


def is_intervention(code: str | None) -> bool:
    """이 사건을 '개입'으로 셀 것인가.

    정의를 코드에 못 박아 둔다. 무엇을 개입으로 셀지 정의가 흔들리면 지표는
    곧바로 조작 가능한 숫자가 된다(캘리포니아 DMV 자율주행 보고 사례).
    개입 = 로봇이 스스로 계획을 이어가지 못해 사람의 판단·조작·이동을 요구한 사건.
    정상 종료와 지시에 의한 정지는 개입이 아니다.
    """
    return get(code).needs_operator


def as_list() -> list[dict]:
    """화면·문서용 목록 (코드표 자체를 UI 에서 보여줄 수 있게)."""
    return [dict(code=c.code, label=c.label, category=c.category,
                 needs_operator=c.needs_operator, needs_site_visit=c.needs_site_visit)
            for c in CODES.values()]
