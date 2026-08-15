from __future__ import annotations

import contextlib

from fastapi import FastAPI
from fastapi.responses import FileResponse

from . import auth
from .config import Settings, load_settings
from .db import Base, make_engine, make_session_factory
from .event_retention import RetentionTask
from .farm import load_farm
from .fleet.port import InMemoryFleetPort
from .fleet.service import FleetService
from .migrations import ensure_events_epoch_column
from .models import User


def _bootstrap_admin(app: FastAPI) -> None:
    """users 가 비어 있으면 FLEET_ADMIN_* 로 최초 관리자를 만든다."""
    s: Settings = app.state.settings
    if not (s.admin_login and s.admin_password):
        return
    with app.state.session_factory() as db:
        if db.query(User).count() == 0:
            db.add(User(login=s.admin_login, pw_hash=auth.hash_password(s.admin_password),
                        role="admin", display_name="관리자"))
            db.commit()


def create_app(settings: Settings | None = None, engine=None, fleet=None) -> FastAPI:
    settings = settings or load_settings()
    engine = engine or make_engine(settings.db_url)
    ensure_events_epoch_column(engine)      # T6 리뷰 I4 — 구 스키마 자동 복구(create_all 전)
    Base.metadata.create_all(engine)
    session_factory = make_session_factory(engine)

    from .traffic import AlleyLocks
    with session_factory() as db:               # 재기동 시 RUNNING 임무 잠금 정합 확인
        AlleyLocks.restore(db)
        db.commit()

    use_legacy = fleet is None                  # 운영: lifespan 에서 레거시 기동

    @contextlib.asynccontextmanager
    async def lifespan(app: FastAPI):
        if use_legacy:
            from .fleet.legacy_ws import LegacyFleetPort
            from .models import Robot
            lp = LegacyFleetPort(settings.offline_after_s)
            app.state.fleet = lp
            app.state.fleet_service.attach(lp)
            app.state.bt_engine.fleet = lp      # 엔진도 같은 포트를 봐야 한다
            with session_factory() as db:
                for r in db.query(Robot).filter(Robot.conn_kind == "legacy_ws"):
                    lp.register_robot(r.id, r.farm_id, r.conn_kind, r.config_json)
        app.state.bt_engine.restore()           # RUNNING 인스턴스를 이어받는다
        await app.state.bt_engine.start()       # 1 Hz 틱
        await app.state.retention.start()       # 이벤트 보존정책 — 기동 시 1회 + 24h 주기
        yield
        await app.state.retention.stop()
        await app.state.bt_engine.stop()
        if use_legacy and hasattr(app.state.fleet, "shutdown"):
            await app.state.fleet.shutdown()

    app = FastAPI(title="과수원 통합관제 서버", lifespan=lifespan)
    app.state.settings = settings
    app.state.engine = engine
    app.state.session_factory = session_factory
    app.state.farm = load_farm(settings)         # Task 5 — 없으면 None(대시보드 폴백)
    app.state.fleet = fleet if fleet is not None else InMemoryFleetPort(settings.offline_after_s)
    app.state.fleet_service = FleetService(session_factory)
    if fleet is not None:
        app.state.fleet_service.attach(fleet)

    from .bt.engine import BTEngine
    app.state.bt_engine = BTEngine(session_factory, app.state.fleet, farm=app.state.farm)
    app.state.retention = RetentionTask(session_factory, settings.event_ttl_days,
                                        safe_ttl_days=settings.event_ttl_safe_days)

    from .api import admin_routes, auth_routes
    app.include_router(auth_routes.router, prefix="/api/v1")
    app.include_router(admin_routes.router, prefix="/api/v1")

    from .api import bt_routes, farm_routes, history_routes, mission_routes, ops_routes
    app.include_router(mission_routes.router, prefix="/api/v1")
    app.include_router(history_routes.router, prefix="/api/v1")
    app.include_router(ops_routes.router, prefix="/api/v1")
    app.include_router(bt_routes.router, prefix="/api/v1")
    app.include_router(farm_routes.router, prefix="/api/v1")
    app.include_router(farm_routes.assets_router)   # 정적 이미지 — /assets/... (프리픽스 없음)

    from . import ws as ws_module
    app.include_router(ws_module.router)

    _bootstrap_admin(app)

    @app.get("/", include_in_schema=False)
    def index():
        return FileResponse(settings.web_dir / "index.html")

    return app
