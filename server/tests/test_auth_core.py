import pytest

from fleet_server import auth


def test_password_roundtrip():
    h = auth.hash_password("비밀1234")
    assert h != "비밀1234"
    assert auth.verify_password("비밀1234", h)
    assert not auth.verify_password("오답", h)
    assert not auth.verify_password("비밀1234", "손상된해시")


# D9: estop·stop_all은 observer 포함 전 역할 허용, 해제는 admin만
@pytest.mark.parametrize("role,action,ok", [
    ("observer", "estop", True),
    ("observer", "stop_all", True),
    ("observer", "ping", True),
    ("observer", "mission_start", False),
    ("observer", "teleop", False),
    ("observer", "clear_estop", False),
    ("operator", "mission_start", True),
    ("operator", "teleop", True),
    ("operator", "clear_estop", False),
    ("admin", "clear_estop", True),
    ("admin", "set_mode", True),
])
def test_matrix(role, action, ok):
    assert auth.authorize(role, action) is ok


def test_fail_closed():
    assert auth.authorize("admin", "완전히_모르는_명령") is True   # 미지 명령 → admin 요구
    assert auth.authorize("operator", "완전히_모르는_명령") is False
    assert auth.normalize_role("이상한역할") == "observer"          # 미지 역할 → 강등
    assert auth.authorize("이상한역할", "mission_start") is False
    assert auth.authorize(None, "estop") is True                    # 강등돼도 estop은 가능(D9)
