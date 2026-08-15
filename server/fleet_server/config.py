"""서버 설정 — 환경변수 FLEET_* 로 주입한다."""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent   # server/fleet_server/.. → 저장소 루트


@dataclass
class Settings:
    db_url: str = "sqlite:///fleet.db"
    session_secret: str = ""              # 비어 있으면 개발용 (부트스트랩에서 경고)
    session_ttl_s: int = 7 * 86400
    login_delay_s: float = 0.5            # 로그인 실패 지연 (스펙 §5)
    allowed_origins: list[str] = field(default_factory=list)   # WS Origin 허용 목록
    web_dir: Path = Path(__file__).resolve().parent.parent / "web"
    admin_login: str = ""                 # users 테이블이 빌 때만 부트스트랩
    admin_password: str = ""
    offline_after_s: float = 15.0         # 스펙 §3.1 — 오프라인 표시 15초
    event_ttl_days: int = 7               # Task 6 — 이벤트 보존정책(TTL 7일)
    event_ttl_safe_days: int = 90         # T6 리뷰 I2 — 안전·수명주기 kind 는 더 길게(기본 90일)
    farm_manifest_path: Path = _REPO_ROOT / "maps/orchard_real/farm.json"   # Task 5
    imagery_dir: Path = _REPO_ROOT / "sim/assets/imagery"                   # Task 5 — ortho 원본 위치
    farm_id: int = 1     # Task5 수정 라운드1 I2 — 단일 매니페스트 배치 전제(다중 농장 정합은 범위 밖)


def load_settings() -> Settings:
    s = Settings()
    s.db_url = os.environ.get("FLEET_DB_URL", s.db_url)
    s.session_secret = os.environ.get("FLEET_SESSION_SECRET", s.session_secret)
    s.session_ttl_s = int(os.environ.get("FLEET_SESSION_TTL_S", s.session_ttl_s))
    s.login_delay_s = float(os.environ.get("FLEET_LOGIN_DELAY_S", s.login_delay_s))
    raw = os.environ.get("FLEET_ALLOWED_ORIGINS", "")
    s.allowed_origins = [o.strip() for o in raw.split(",") if o.strip()]
    if os.environ.get("FLEET_WEB_DIR"):
        s.web_dir = Path(os.environ["FLEET_WEB_DIR"])
    s.admin_login = os.environ.get("FLEET_ADMIN_LOGIN", "")
    s.admin_password = os.environ.get("FLEET_ADMIN_PASSWORD", "")
    s.event_ttl_days = int(os.environ.get("FLEET_EVENT_TTL_DAYS", s.event_ttl_days))
    s.event_ttl_safe_days = int(os.environ.get("FLEET_EVENT_TTL_SAFE_DAYS",
                                               s.event_ttl_safe_days))
    if os.environ.get("FLEET_FARM_MANIFEST"):
        s.farm_manifest_path = Path(os.environ["FLEET_FARM_MANIFEST"])
    if os.environ.get("FLEET_IMAGERY_DIR"):
        s.imagery_dir = Path(os.environ["FLEET_IMAGERY_DIR"])
    s.farm_id = int(os.environ.get("FLEET_FARM_ID", s.farm_id))
    return s
