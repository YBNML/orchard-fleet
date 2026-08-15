"""농장 매니페스트 API (Task 5) — 대시보드(T6)가 아핀·ortho_url·비균일 통로
기하를 이 API 하나에서 읽는다. T3 N-3: 로봇 hello 기하는 균일 격자가 남아
있다 — 대시보드 오버레이의 정확한 소스는 여기(farm.json 전문)다."""
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


@router.get("/farm")
def get_farm(request: Request, user: User = Depends(current_user)) -> dict:
    """farm.json 전문 + ortho_url. 로그인 사용자면 역할 무관하게 전원 허용."""
    farm = request.app.state.farm
    if farm is None:
        raise HTTPException(404, "농장 매니페스트가 설정되지 않았습니다")
    out = dict(farm)
    out["ortho_url"] = f"/assets/{farm['image']}"
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
