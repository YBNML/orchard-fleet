from __future__ import annotations

from fastapi import FastAPI
from fastapi.responses import FileResponse

from . import auth
from .config import Settings, load_settings
from .db import Base, make_engine, make_session_factory
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
    Base.metadata.create_all(engine)

    app = FastAPI(title="과수원 통합관제 서버")
    app.state.settings = settings
    app.state.engine = engine
    app.state.session_factory = make_session_factory(engine)

    from .fleet.port import InMemoryFleetPort
    app.state.fleet = fleet if fleet is not None else InMemoryFleetPort(settings.offline_after_s)

    from .api import admin_routes, auth_routes
    app.include_router(auth_routes.router, prefix="/api/v1")
    app.include_router(admin_routes.router, prefix="/api/v1")

    from .api import history_routes, mission_routes
    app.include_router(mission_routes.router, prefix="/api/v1")
    app.include_router(history_routes.router, prefix="/api/v1")

    _bootstrap_admin(app)

    @app.get("/", include_in_schema=False)
    def index():
        return FileResponse(settings.web_dir / "index.html")

    return app
