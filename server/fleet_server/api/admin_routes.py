from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import BaseModel

from .. import audit, auth
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


@router.post("/farms", dependencies=[_csrf])
def create_farm(body: FarmBody, db=Depends(get_db),
                user: User = Depends(require_min_role("admin"))):
    f = Farm(name=body.name)
    db.add(f); db.commit()
    audit.record(db, action="farm_create", result="accepted",
                 user_id=user.id, role=user.role, target=str(f.id), detail=f.name)
    return {"id": f.id, "name": f.name}


@router.patch("/farms/{farm_id}", dependencies=[_csrf])
def patch_farm(farm_id: int, body: FarmPatch, db=Depends(get_db),
               user: User = Depends(require_min_role("admin"))):
    f = db.get(Farm, farm_id)
    if f is None:
        audit.record(db, action="farm_patch", result="rejected",
                     user_id=user.id, role=user.role, target=str(farm_id),
                     detail="대상 없음")
        raise HTTPException(404, "농장이 없습니다")
    changed = []
    if body.map_bundle_ref is not None:
        f.map_bundle_ref = body.map_bundle_ref
        changed.append("map_bundle_ref")
    if body.config_json is not None:
        f.config_json = body.config_json
        changed.append("config_json")
    db.commit()
    audit.record(db, action="farm_patch", result="accepted",
                 user_id=user.id, role=user.role, target=str(f.id),
                 detail=",".join(changed))
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


@router.post("/robots", dependencies=[_csrf])
def create_robot(body: RobotBody, db=Depends(get_db),
                 user: User = Depends(require_min_role("admin"))):
    if db.get(Farm, body.farm_id) is None:
        audit.record(db, action="robot_create", result="rejected",
                     user_id=user.id, role=user.role, target=body.id,
                     detail="농장 없음")
        raise HTTPException(404, "농장이 없습니다")
    r = Robot(**body.model_dump())
    db.add(r); db.commit()
    audit.record(db, action="robot_create", result="accepted",
                 user_id=user.id, role=user.role, target=str(r.id), detail=r.name)
    return _robot_out(r, admin=True)


@router.patch("/robots/{robot_id}", dependencies=[_csrf])
def patch_robot(robot_id: str, body: dict, db=Depends(get_db),
                user: User = Depends(require_min_role("admin"))):
    r = db.get(Robot, robot_id)
    if r is None:
        audit.record(db, action="robot_patch", result="rejected",
                     user_id=user.id, role=user.role, target=robot_id,
                     detail="대상 없음")
        raise HTTPException(404, "로봇이 없습니다")
    for k in ("name", "kind", "conn_kind", "config_json", "farm_id"):
        if k in body:
            setattr(r, k, body[k])
    db.commit()
    audit.record(db, action="robot_patch", result="accepted",
                 user_id=user.id, role=user.role, target=robot_id,
                 detail=",".join(sorted(body.keys())))
    return {"ok": True}


@router.get("/robots/{robot_id}/status")
def robot_status(robot_id: str, request: Request, db=Depends(get_db),
                 user: User = Depends(current_user)):
    r = db.get(Robot, robot_id)
    if r is None:
        raise HTTPException(404, "로봇이 없습니다")
    scope = farm_scope(db, user)
    if scope is not None and r.farm_id not in scope:
        raise HTTPException(403, "해당 농장 권한이 없습니다")
    st = request.app.state.fleet.robot_status(robot_id)
    return {"online": st.online, "last_seen": st.last_seen}


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


@router.post("/users", dependencies=[_csrf])
def create_user(body: UserBody, db=Depends(get_db),
                user: User = Depends(require_min_role("admin"))):
    u = User(login=body.login, pw_hash=auth.hash_password(body.password),
             role=auth.normalize_role(body.role), display_name=body.display_name)
    db.add(u); db.flush()
    for fid in body.farm_ids:
        db.add(UserFarm(user_id=u.id, farm_id=fid))
    db.commit()
    audit.record(db, action="user_create", result="accepted",
                 user_id=user.id, role=user.role, target=str(u.id), detail=u.login)
    return {"id": u.id, "login": u.login, "role": u.role}


@router.patch("/users/{user_id}", dependencies=[_csrf])
def patch_user(user_id: int, body: dict, db=Depends(get_db),
               user: User = Depends(require_min_role("admin"))):
    u = db.get(User, user_id)
    if u is None:
        audit.record(db, action="user_patch", result="rejected",
                     user_id=user.id, role=user.role, target=str(user_id),
                     detail="대상 없음")
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
    audit.record(db, action="user_patch", result="accepted",
                 user_id=user.id, role=user.role, target=str(u.id),
                 detail=",".join(sorted(body.keys())))
    return {"ok": True}
