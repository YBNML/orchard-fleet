"""농장 매니페스트 API (Task 5) — 대시보드(T6)가 아핀·ortho_url·비균일 통로
기하를 이 API 하나에서 읽는다. T3 N-3: 로봇 hello 기하는 균일 격자가 남아
있다 — 대시보드 오버레이의 정확한 소스는 여기(farm.json 전문)다.

**단일 매니페스트 배치 한계(Task5 수정 라운드1 I2).** 이 서버는 농장을
여러 개 등록할 수 있지만(admin API 의 `/farms`), 지리 매니페스트는
`FLEET_FARM_MANIFEST` 파일 하나뿐이다 — 여러 농장이 각자 다른 지리를 가지는
다중 농장 배치의 정합은 이번 스펙 범위 밖이다(원장 이연). 응답에 실리는
`farm_id` 는 이 매니페스트가 속한다고 **설정으로 선언한** 농장(기본 1,
`FLEET_FARM_ID`)일 뿐 — admin API 의 실제 farms 테이블 id 와 자동으로
맞물리지 않는다. 단일 농장 배치를 전제한다."""
from __future__ import annotations

import hashlib
import logging

from fastapi import APIRouter, Depends, HTTPException, Request
from fastapi.responses import FileResponse

from ..deps import current_user
from ..models import User

log = logging.getLogger("fleet_server.farm")

router = APIRouter(tags=["farm"])          # /api/v1 프리픽스로 얹는다
assets_router = APIRouter(tags=["assets"])  # 이미지는 프리픽스 없이(정적 자산)


# 정사영상 출처 표기 — **라이선스 의무의 이행 지점**.
#
# sim/assets/imagery/LICENSE-DATA.md 는 "이 이미지를 사용하는 모든 산출물
# (시뮬레이션 월드, 스크린샷, **대시보드 배경** 등)"에 표기를 요구한다(CC BY 4.0).
# 문구는 그 문서의 축약형을 **그대로** 옮긴다 — 두 곳에서 따로 손보면 갈라진다.
#
# 화면이 아니라 API 가 들고 있는 이유: 다른 농장·다른 영상으로 바꿔도 표기가
# 데이터를 따라오게 하려는 것이다(기하와 같은 원칙). farm.json 에 `attribution`
# 이 있으면 그것을 우선한다.
ORTHO_ATTRIBUTION = ("PNOA cedido por © Instituto Geográfico Nacional · CC BY 4.0")


@router.get("/farm")
def get_farm(request: Request, user: User = Depends(current_user)) -> dict:
    """farm.json 전문 + ortho_url + attribution. 로그인 사용자면 전원 허용."""
    farm = request.app.state.farm
    if farm is None:
        raise HTTPException(404, "농장 매니페스트가 설정되지 않았습니다")
    settings = request.app.state.settings
    out = dict(farm)
    out["ortho_url"] = f"/assets/{farm['image']}"
    out["farm_id"] = settings.farm_id            # I2 — 설정으로 선언한 단일 농장 id
    out["attribution"] = farm.get("attribution") or ORTHO_ATTRIBUTION
    return out


@assets_router.get("/assets/{filename}")
def get_asset(filename: str, request: Request, user: User = Depends(current_user)):
    """농장 정사영상 서빙 — farm.json 이 가리키는 파일 하나만 내준다(경로 순회 방지).

    서빙 직전 실제 파일의 sha256 을 farm.json 의 image_sha256 과 대조한다 —
    불일치를 조용히 넘기지 않는다(스펙 §6): 500 을 내고 로그를 남긴다."""
    farm = request.app.state.farm
    if farm is None or filename != farm.get("image"):
        raise HTTPException(404, "이미지를 찾을 수 없습니다")
    settings = request.app.state.settings
    path = settings.imagery_dir / filename
    if not path.is_file():
        log.error("농장 이미지 파일이 없습니다 — %s", path)
        raise HTTPException(404, "이미지 파일이 없습니다")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    expected = farm.get("image_sha256")
    if expected and digest != expected:
        log.error("농장 이미지 sha256 불일치 — %s: 기대 %s 실측 %s", path, expected, digest)
        raise HTTPException(500, "이미지 무결성 불일치 — 관리자에게 문의하십시오")
    return FileResponse(path)
