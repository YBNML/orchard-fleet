"""운영 지표 — '가동률'을 헤드라인에서 내리고 개입 중심으로 바꾼 것.

왜 가동률이 아닌가: 상용 플릿에서 관제 조직의 KPI 는 예외 없이 '개입을 줄이고
가동률을 올린다'로 정의되며, 그중 관리 가능한 쪽은 개입이다. 가동률은 결과이고
개입은 원인이다. 게다가 농업 로봇의 가동률은 계절과 날씨가 지배해서 관제가
잘했는지 못했는지를 거의 말해주지 않는다.

다섯 지표:
    개입/작업면적   거리 기준은 통로 길이에 좌우돼 밭마다 비교가 안 된다
    MTBI            능동 운용시간 기준. 대기 시간을 빼지 않으면 부풀려진다
    처리시간 p50/p95 평균은 꼬리를 숨긴다. 관제가 아픈 건 꼬리 쪽이다
    현장 출동       규격상 사람이 반드시 이동해야 하는 건수와 그 왕복시간
    실가동일수      국내 농기계 임대 실적이 대당 연 11.4일 — 경제성의 실제 병목
"""
from __future__ import annotations

import datetime as dt

from sqlalchemy import func

from .models import Intervention, Mission, Track


def _pct(sorted_vals, q):
    if not sorted_vals:
        return None
    i = min(len(sorted_vals) - 1, max(0, int(round((len(sorted_vals) - 1) * q))))
    return round(sorted_vals[i], 1)


def summary(db, farm_ids=None, since: dt.datetime | None = None) -> dict:
    """farm_ids 가 None 이면 전체(admin). since 가 None 이면 전 기간."""
    iv = db.query(Intervention)
    ms = db.query(Mission)
    tr = db.query(Track)
    if farm_ids is not None:
        iv = iv.filter(Intervention.farm_id.in_(farm_ids))
        ms = ms.filter(Mission.farm_id.in_(farm_ids))
    if since is not None:
        iv = iv.filter(Intervention.opened_at >= since)
        ms = ms.filter(Mission.created_at >= since)
        tr = tr.filter(Track.ts >= since)

    rows = iv.all()
    n_iv = len(rows)
    n_visit = sum(1 for r in rows if r.needs_site_visit)
    n_open = sum(1 for r in rows if r.state in ("OPEN", "ACKED"))

    # 처리 시간 (초) — 해소된 건만
    ack_s, res_s = [], []
    for r in rows:
        if r.acked_at:
            ack_s.append((r.acked_at - r.opened_at).total_seconds())
        if r.resolved_at:
            res_s.append((r.resolved_at - r.opened_at).total_seconds())
    ack_s.sort(); res_s.sort()

    # 작업량 — 통로 수를 면적 대용으로 쓴다(같은 밭 안에서는 비례한다).
    # 진짜 면적이 필요해지면 맵 번들의 통로 길이 × 열 간격으로 바꾼다.
    alleys = 0
    active_s = 0.0
    work_days = set()
    for m in ms.all():
        alleys += len(((m.spec_json or {}).get("alleys") or []))
        if m.started_at:
            work_days.add(m.started_at.date().isoformat())
            end = m.ended_at or m.started_at
            active_s += max(0.0, (end - m.started_at).total_seconds())

    # 능동 운용시간이 없으면 '간격'이라는 말 자체가 성립하지 않는다 — 0 분으로
    # 표시하면 "1분에 한 번씩 터진다"는 정반대 인상을 준다.
    mtbi = round(active_s / n_iv / 60.0, 1) if (n_iv and active_s > 0) else None

    return dict(
        interventions=n_iv,
        open_now=n_open,
        per_alley=(round(n_iv / alleys, 3) if alleys else None),
        alleys_worked=alleys,
        mtbi_min=mtbi,
        active_min=round(active_s / 60.0, 1),
        ack_p50_s=_pct(ack_s, 0.5), ack_p95_s=_pct(ack_s, 0.95),
        resolve_p50_s=_pct(res_s, 0.5), resolve_p95_s=_pct(res_s, 0.95),
        site_visits=n_visit,
        site_visit_ratio=(round(n_visit / n_iv, 3) if n_iv else None),
        work_days=len(work_days),
        by_category=_by_category(rows),
    )


def _by_category(rows) -> list[dict]:
    agg: dict[str, int] = {}
    for r in rows:
        agg[r.category or "기타"] = agg.get(r.category or "기타", 0) + 1
    return [dict(category=k, count=v)
            for k, v in sorted(agg.items(), key=lambda kv: -kv[1])]
