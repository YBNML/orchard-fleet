"""Task 5 — 농장 매니페스트 API(GET /api/v1/farm) + 정적 이미지 서빙(sha256
무결성) + 프리셋 일반화(N_ALLEYS→farm, terrain 별 파리티) 테스트.

baseline(conftest._test_settings) 은 farm_manifest_path 를 존재하지 않는
경로로 못박아 둔다(Task 5) — 이 파일의 테스트만 tmp_path 에 직접 farm.json
을 써서 명시적으로 farm 이 있는 앱을 구성한다. 무관한 226개 baseline 테스트가
저장소의 실제 maps/orchard_real/farm.json 내용에 우연히 결합되지 않게
하려는 격리다."""
from __future__ import annotations

import hashlib
import json
import logging
import pathlib

import pytest
from fastapi.testclient import TestClient

from fleet_server.app import create_app
from fleet_server.bt import presets
from fleet_server.fleet.port import InMemoryFleetPort

from tests.conftest import _test_settings, do_login

IMG_BYTES = b"jpeg-fixture-bytes-not-a-real-image"


def _row_origins(rows: int, spacing: list[float]) -> list[list[float]]:
    """axes_note 관례 — cross-row(통로 폭) 축은 world x. spacing 은 통로 rows-1개의 폭."""
    xs = [0.0]
    for gap in spacing:
        xs.append(xs[-1] + gap)
    return [[x, 0.0] for x in xs]


def _first_action_alleys(tree) -> list[int]:
    st = tree.to_state()
    kids = st.get("children") or ([st["child"]] if st.get("child") else [])
    for k in kids:
        if k["kind"] == "action":
            return k["spec"]["alleys"]
    return []


def _write_farm(tmp_path, *, rows=10, terrain="flat", spacing=None,
                image="ortho_test.jpg", image_bytes=IMG_BYTES,
                image_sha256=None, with_row_origins=True):
    if spacing is None:
        spacing = [5.0] * (rows - 1)
    farm = {
        "image": image,
        "px_per_m": 4.0,
        "origin_px": [0.0, 0.0],
        "rotation_deg": 0.0,
        "rows": rows,
        "row_spacing_m": 5.0,
        "row_length_m": 100.0,
        "row_lengths_m": [100.0] * rows,
        "tree_spacing_m": 1.5,
        "headland_m": 2.0,
        "bounds_m": [[-10.0, -10.0], [110.0, 110.0]],
        "image_footprint_world": [[0, 0], [100, 0], [100, 100], [0, 100]],
        "terrain": terrain,
        "image_sha256": (image_sha256 if image_sha256 is not None
                         else hashlib.sha256(image_bytes).hexdigest()),
        "axes_note": "테스트 픽스처",
        "bounds_note": "테스트 픽스처",
        "row_selection_note": "테스트 픽스처",
    }
    if with_row_origins:
        farm["row_origins"] = _row_origins(rows, spacing)
    manifest_path = tmp_path / "farm.json"
    manifest_path.write_text(json.dumps(farm), encoding="utf-8")
    imagery_dir = tmp_path / "imagery"
    imagery_dir.mkdir(exist_ok=True)
    (imagery_dir / image).write_bytes(image_bytes)
    return manifest_path, imagery_dir, farm


def _app_with_farm(tmp_path, **farm_kw):
    manifest_path, imagery_dir, farm = _write_farm(tmp_path, **farm_kw)
    settings = _test_settings(farm_manifest_path=manifest_path, imagery_dir=imagery_dir)
    app = create_app(settings, fleet=InMemoryFleetPort())
    return app, farm


def _seed_bt(client) -> dict:
    """admin API 로 농장 하나·로봇 하나를 만든다(test_bt_engine._seed_api 관례).
    admin 은 role rank 상 operator 요건을 만족해 별도 operator 계정이 필요 없다."""
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    fa = client.post("/api/v1/farms", json={"name": "농장R"}, headers=h).json()
    client.post("/api/v1/robots", headers=h,
               json={"id": "scoutR", "farm_id": fa["id"], "name": "scoutR"})
    return h


@pytest.fixture()
def client_with_farm(tmp_path):
    app, _farm = _app_with_farm(tmp_path, rows=10, terrain="flat")
    return TestClient(app)


# ── GET /api/v1/farm ─────────────────────────────────────────────────────────

def test_farm_endpoint_serves_manifest(client_with_farm):
    do_login(client_with_farm)
    r = client_with_farm.get("/api/v1/farm")
    assert r.status_code == 200 and r.json()["rows"] == 10 and r.json()["terrain"] == "flat"


def test_farm_endpoint_includes_ortho_url(client_with_farm):
    do_login(client_with_farm)
    r = client_with_farm.get("/api/v1/farm")
    assert r.json()["ortho_url"] == "/assets/ortho_test.jpg"


def test_farm_endpoint_requires_login(client_with_farm):
    assert client_with_farm.get("/api/v1/farm").status_code == 401


def test_farm_endpoint_404_when_manifest_missing(tmp_path):
    settings = _test_settings(farm_manifest_path=tmp_path / "does_not_exist.json")
    app = create_app(settings, fleet=InMemoryFleetPort())
    client = TestClient(app)
    do_login(client)
    assert client.get("/api/v1/farm").status_code == 404
    # 서버는 뜬다 — 매니페스트 부재가 다른 API 를 막지 않는다(스펙: 대시보드 폴백 경로)
    assert client.get("/api/v1/stopcodes").status_code == 200


# ── 정적 이미지 서빙 + sha256 무결성(스펙 §6 — 무음 불일치 금지) ────────────────

def test_asset_serves_image_bytes(client_with_farm):
    do_login(client_with_farm)
    r = client_with_farm.get("/assets/ortho_test.jpg")
    assert r.status_code == 200
    assert r.content == IMG_BYTES


def test_asset_sha256_mismatch_returns_500_and_logs(tmp_path, caplog):
    app, _farm = _app_with_farm(tmp_path, rows=10, terrain="flat",
                                image_sha256="0" * 64)   # 고의로 farm.json 에 틀린 값
    client = TestClient(app)
    do_login(client)
    with caplog.at_level(logging.ERROR, logger="fleet_server.farm"):
        r = client.get("/assets/ortho_test.jpg")
    assert r.status_code == 500
    assert any("불일치" in rec.message for rec in caplog.records)


def test_asset_unknown_filename_404(client_with_farm):
    do_login(client_with_farm)
    assert client_with_farm.get("/assets/not_the_farm_image.jpg").status_code == 404


# ── 프리셋 — N_ALLEYS → farm 일반화 ───────────────────────────────────────────

def test_presets_use_farm_alley_count(tmp_path):
    """farm rows=8 → 통로 0..6 만 유효, 7 은 400."""
    app, _farm = _app_with_farm(tmp_path, rows=8, terrain="flat", with_row_origins=False)
    client = TestClient(app)
    h = _seed_bt(client)

    ok = client.post("/api/v1/bt", headers=h, json={
        "preset": "single_alley_loop", "params": {"robot": "scoutR", "alley": 6, "n": 1}})
    assert ok.status_code == 200, ok.text

    bad = client.post("/api/v1/bt", headers=h, json={
        "preset": "single_alley_loop", "params": {"robot": "scoutR", "alley": 7, "n": 1}})
    assert bad.status_code == 400, bad.text


# ── terrain 별 파리티 게이트 ───────────────────────────────────────────────────

def test_parity_gate_disabled_on_flat():
    """flat: split_k=4(구 불가 분할) 허용 — 폭 정보가 없는 farm 이라 최대 폭
    통로 제외(T7 ①)는 발동하지 않고, 전이 규칙(T7 ②)만 본다(모두 ±1)."""
    plans = presets.full_split_patrol("scout01", "scout02", split_k=4, n_alleys=9,
                                      farm={"terrain": "flat"})
    assert [p.robot_id for p in plans] == ["scout01", "scout02"]
    assert _first_action_alleys(plans[0].tree) == [0, 1, 2, 3]
    assert _first_action_alleys(plans[1].tree) == [5, 6, 7, 8]


def test_parity_gate_kept_on_terraced():
    """terraced: 기존 400 유지 — split_k=4 의 B=[5,6,7,8] 은 여전히 거부된다."""
    with pytest.raises(presets.PresetError):
        presets.full_split_patrol("scout01", "scout02", split_k=4, n_alleys=9,
                                  farm={"terrain": "terraced"})


def test_parity_gate_kept_when_farm_absent():
    """farm 자체가 없는 레거시 호출(orchard_v1) 도 여전히 terraced 취급 — 회귀 방지."""
    with pytest.raises(presets.PresetError):
        presets.full_split_patrol("scout01", "scout02", split_k=4)


# ── T7 인계 ①② 세부 검증 ──────────────────────────────────────────────────────

def test_flat_terrain_rejects_mission_touching_widest_alley():
    farm = {"terrain": "flat", "row_origins": _row_origins(6, [5, 5, 15, 5, 5])}
    with pytest.raises(presets.PresetError):     # 통로 2(폭 15) 가 최대 폭 통로
        presets.sequential_retry("scout01", [1, 2, 3], farm=farm)
    presets.sequential_retry("scout01", [0, 1], farm=farm)   # 버퍼를 안 건드리면 통과


def test_flat_terrain_rejects_transition_further_than_two():
    farm = {"terrain": "flat"}
    with pytest.raises(presets.PresetError):
        presets.sequential_retry("scout01", [0, 4], farm=farm)   # 전이 4 — 위반
    presets.sequential_retry("scout01", [0, 2], farm=farm)        # 전이 2 — 허용
    presets.sequential_retry("scout01", [0, 1], farm=farm)        # 전이 1 — 허용


def test_full_split_patrol_default_split_uses_widest_alley_when_farm_given():
    farm = {"terrain": "flat", "rows": 6, "row_origins": _row_origins(6, [5, 5, 15, 5, 5])}
    plans = presets.full_split_patrol("scout01", "scout02", farm=farm)   # split_k 미지정
    a_alleys, b_alleys = _first_action_alleys(plans[0].tree), _first_action_alleys(plans[1].tree)
    assert 2 not in a_alleys and 2 not in b_alleys   # 최대 폭 통로(버퍼) 는 어느 쪽에도 없다
    assert a_alleys == [0, 1] and b_alleys == [3, 4]


# ── 실사 farm.json 자체 회귀 — T7 인계가 못박은 사실 ───────────────────────────

def test_real_orchard_manifest_matches_t7_handoff_facts():
    """실사 농장은 rows=27(통로 26개)·flat·통로 20 이 최대 폭(15.25m 농로) 이고,
    그 통로가 기본 분할의 자연 버퍼가 된다(설계 지침) — 어긋나면 여기서 잡는다."""
    real_path = (pathlib.Path(__file__).resolve().parent.parent.parent
                / "maps/orchard_real/farm.json")
    real_farm = json.loads(real_path.read_text(encoding="utf-8"))

    assert presets.n_alleys_of(real_farm) == 26
    assert presets.terrain_of(real_farm) == "flat"
    assert presets.widest_alley(real_farm) == 20

    plans = presets.full_split_patrol("scout01", "scout02", farm=real_farm)
    assert 20 not in _first_action_alleys(plans[0].tree)
    assert 20 not in _first_action_alleys(plans[1].tree)
