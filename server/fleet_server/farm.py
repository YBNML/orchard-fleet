"""농장 매니페스트(farm.json) 로더 — Task 5.

기동 시 1회 읽어 app.state.farm 에 얹는다(정적 지리 데이터라 요청마다 다시
읽지 않는다). 파일이 없어도 서버는 뜬다 — 대시보드가 폴백 경로를 타도록
None 을 두고 경고만 남긴다(스펙 §6 은 "무음 불일치 금지" 지 "무음 부재 금지"
까지는 아니다 — 부재는 로그 + None 으로 명시적으로 알린다)."""
from __future__ import annotations

import json
import logging

from .config import Settings

log = logging.getLogger("fleet_server.farm")


def load_farm(settings: Settings) -> dict | None:
    path = settings.farm_manifest_path
    if not path.is_file():
        log.warning("농장 매니페스트를 찾을 수 없습니다(대시보드는 폴백 경로를 씁니다) — %s",
                   path)
        return None
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)
