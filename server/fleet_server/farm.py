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

# M1(수정 라운드1) — 이 서버가 farm.json 에서 실제로 읽는 필수 키. 없으면
# 매니페스트가 손상됐거나 다른 스펙 버전이라는 뜻이라 500 으로 죽이지 않고
# (기동은 계속돼야 한다) 등록 자체를 건너뛴다 — 파일 부재와 같은 폴백 경로다.
_REQUIRED_KEYS = ("rows", "row_spacing_m", "terrain")


def load_farm(settings: Settings) -> dict | None:
    path = settings.farm_manifest_path
    if not path.is_file():
        log.warning("농장 매니페스트를 찾을 수 없습니다(대시보드는 폴백 경로를 씁니다) — %s",
                   path)
        return None
    with path.open("r", encoding="utf-8") as f:
        farm = json.load(f)
    missing = [k for k in _REQUIRED_KEYS if k not in farm]
    if missing:
        log.warning("농장 매니페스트에 필수 키가 없어 등록하지 않습니다(대시보드는 폴백 "
                   "경로를 씁니다) — 결여 키 %s, 파일 %s", missing, path)
        return None
    return farm
