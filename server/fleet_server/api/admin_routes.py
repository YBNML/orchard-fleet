from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel

from .. import auth
from ..deps import (csrf_protect, current_user, farm_scope, get_db,
                    require_min_role)
from ..models import Farm, Robot, User, UserFarm

router = APIRouter(tags=["admin"])
_admin = Depends(require_min_role("admin"))
_csrf = Depends(csrf_protect)


# ── 농장 ──────────────────────────────────────────────────────────────────
class FarmBody(BaseModel):
    name: str


class FarmPatch(BaseModel):
    map_bundle_ref: str | None = None
    config_json: dict | None = None


@router.get("/farms")
def list_farms(db=Depends(get_db), user: User = Depends(current_user)):
    scope = farm_scope(db, user)
    q = db.query(Farm)
    if scope is not None:
        q = q.filter(Farm.id.in_(scope))
    return [{"id": f.id, "name": f.name, "map_bundle_ref": f.map_bundle_ref}
            for f in q.order_by(Farm.id)]


@router.post("/farms", dependencies=[_admin, _csrf])
def create_farm(body: FarmBody, db=Depends(get_db)):
    f = Farm(name=body.name)
    db.add(f); db.commit()
    return {"id": f.id, "name": f.name}


@router.patch("/farms/{farm_id}", dependencies=[_admin, _csrf])
def patch_farm(farm_id: int, body: FarmPatch, db=Depends(get_db)):
    f = db.get(Farm, farm_id)
    if f is None:
        raise HTTPException(404, "농장이 없습니다")
    if body.map_bundle_ref is not None:
        f.map_bundle_ref = body.map_bundle_ref
    if body.config_json is not None:
        f.config_json = body.config_json
    db.commit()
    return {"ok": True}


# ── 로봇 ──────────────────────────────────────────────────────────────────
class RobotBody(BaseModel):
    id: str
    farm_id: int
    name: str
    kind: str = "orchard"
    conn_kind: str = "legacy_ws"
    config_json: dict = {}


def _robot_out(r: Robot, admin: bool) -> dict:
    out = {"id": r.id, "farm_id": r.farm_id, "name": r.name, "kind": r.kind,
           "conn_kind": r.conn_kind,
           "last_seen": r.last_seen.isoformat() if r.last_seen else None}
    if admin:
        out["config_json"] = r.config_json      # 접속 정보는 admin 에게만
    return out


@router.get("/robots")
def list_robots(farm_id: int | None = None, db=Depends(get_db),
                user: User = Depends(current_user)):
    scope = farm_scope(db, user)
    if farm_id is not None and scope is not None and farm_id not in scope:
        raise HTTPException(403, "해당 농장 권한이 없습니다")
    q = db.query(Robot)
    if farm_id is not None:
        q = q.filter(Robot.farm_id == farm_id)
    elif scope is not None:
        q = q.filter(Robot.farm_id.in_(scope))
    admin = auth.normalize_role(user.role) == "admin"
    return [_robot_out(r, admin) for r in q.order_by(Robot.id)]


@router.post("/robots", dependencies=[_admin, _csrf])
def create_robot(body: RobotBody, db=Depends(get_db)):
    if db.get(Farm, body.farm_id) is None:
        raise HTTPException(404, "농장이 없습니다")
    r = Robot(**body.model_dump())
    db.add(r); db.commit()
    return _robot_out(r, admin=True)


@router.patch("/robots/{robot_id}", dependencies=[_admin, _csrf])
def patch_robot(robot_id: str, body: dict, db=Depends(get_db)):
    r = db.get(Robot, robot_id)
    if r is None:
        raise HTTPException(404, "로봇이 없습니다")
    for k in ("name", "kind", "conn_kind", "config_json", "farm_id"):
        if k in body:
            setattr(r, k, body[k])
    db.commit()
    return {"ok": True}


# ── 사용자 ────────────────────────────────────────────────────────────────
class UserBody(BaseModel):
    login: str
    password: str
    role: str = "observer"
    display_name: str = ""
    farm_ids: list[int] = []


@router.get("/users", dependencies=[_admin])
def list_users(db=Depends(get_db)):
    return [{"id": u.id, "login": u.login, "role": u.role,
             "display_name": u.display_name, "disabled": u.disabled}
            for u in db.query(User).order_by(User.id)]


@router.post("/users", dependencies=[_admin, _csrf])
def create_user(body: UserBody, db=Depends(get_db)):
    u = User(login=body.login, pw_hash=auth.hash_password(body.password),
             role=auth.normalize_role(body.role), display_name=body.display_name)
    db.add(u); db.flush()
    for fid in body.farm_ids:
        db.add(UserFarm(user_id=u.id, farm_id=fid))
    db.commit()
    return {"id": u.id, "login": u.login, "role": u.role}


@router.patch("/users/{user_id}", dependencies=[_admin, _csrf])
def patch_user(user_id: int, body: dict, db=Depends(get_db)):
    u = db.get(User, user_id)
    if u is None:
        raise HTTPException(404, "사용자가 없습니다")
    if "disabled" in body:
        u.disabled = bool(body["disabled"])
    if "role" in body:
        u.role = auth.normalize_role(body["role"])
    if "password" in body:
        u.pw_hash = auth.hash_password(body["password"])
    if "farm_ids" in body:
        db.query(UserFarm).filter(UserFarm.user_id == u.id).delete()
        for fid in body["farm_ids"]:
            db.add(UserFarm(user_id=u.id, farm_id=fid))
    db.commit()
    return {"ok": True}
