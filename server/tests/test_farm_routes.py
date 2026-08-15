"""Task 5 — 농장 매니페스트 API(GET /api/v1/farm) + 정적 이미지 서빙(sha256
무결성) + 프리셋 일반화(N_ALLEYS→farm, terrain 별 파리티) 테스트.

**수정 라운드1 반영**: no_go_alleys(farm.json 수동 필드, C2) 기반 회피 —
widest argmax 는 더는 판정에 쓰이지 않는다(진단용으로만 남음). 전이 규칙은
인접(±1)만 허용한다(C1 — ±2 철회). full_split_patrol 기본 분할은 no_go 를
제외한 가장 큰 연속 블록의 중앙 통로다(C3). mission_ops 가 REST 직접 생성과
BT 발진 둘 다에서 no_go 를 검증한다(I1, 단일 출처).

baseline(conftest._test_settings) 은 farm_manifest_path 를 존재하지 않는
경로로 못박아 둔다(Task 5) — 이 파일의 테스트만 tmp_path 에 직접 farm.json
을 써서 명시적으로 farm 이 있는 앱을 구성한다. 무관한 baseline 테스트가
저장소의 실제 maps/orchard_real/farm.json 내용에 우연히 결합되지 않게
하려는 격리다."""
from __future__ import annotations

import hashlib
import json
import logging
import pathlib

import pytest
from fastapi.testclient import TestClient

from fleet_server import mission_ops
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
                image_sha256=None, with_row_origins=True,
                no_go_alleys=None, no_go_note=None):
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
    if no_go_alleys is not None:
        farm["no_go_alleys"] = no_go_alleys
        farm["no_go_note"] = no_go_note or "테스트 픽스처 — no_go"
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


def _seed_bt(client) -> tuple[dict, int]:
    """admin API 로 농장 하나·로봇 하나를 만든다(test_bt_engine._seed_api 관례).
    admin 은 role rank 상 operator 요건을 만족해 별도 operator 계정이 필요 없다.
    (헤더, farm_id) 를 돌려준다 — 추가 로봇을 같은 농장에 등록할 때 farm_id
    를 DB 자동증가값 추측 없이 쓸 수 있게."""
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    fa = client.post("/api/v1/farms", json={"name": "농장R"}, headers=h).json()
    client.post("/api/v1/robots", headers=h,
               json={"id": "scoutR", "farm_id": fa["id"], "name": "scoutR"})
    return h, fa["id"]


@pytest.fixture()
def client_with_farm(tmp_path):
    app, _farm = _app_with_farm(tmp_path, rows=10, terrain="flat")
    return TestClient(app)


# ── GET /api/v1/farm ─────────────────────────────────────────────────────────

def test_farm_endpoint_serves_manifest(client_with_farm):
    do_login(client_with_farm)
    r = client_with_farm.get("/api/v1/farm")
    assert r.status_code == 200 and r.json()["rows"] == 10 and r.json()["terrain"] == "flat"


def test_farm_endpoint_includes_ortho_url_and_farm_id(client_with_farm):
    do_login(client_with_farm)
    r = client_with_farm.get("/api/v1/farm")
    assert r.json()["ortho_url"] == "/assets/ortho_test.jpg"
    assert r.json()["farm_id"] == 1                # I2 — 기본 FLEET_FARM_ID=1


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


def test_farm_endpoint_404_when_required_schema_key_missing(tmp_path, caplog):
    """M1 — 필수 키(rows/row_spacing_m/terrain) 결여 시 500 이 아니라 미등록(404) +
    기동 경고. 서버는 계속 뜬다."""
    manifest_path = tmp_path / "farm.json"
    manifest_path.write_text(json.dumps({"image": "x.jpg"}), encoding="utf-8")   # rows 등 없음
    settings = _test_settings(farm_manifest_path=manifest_path)
    with caplog.at_level(logging.WARNING, logger="fleet_server.farm"):
        app = create_app(settings, fleet=InMemoryFleetPort())
    assert app.state.farm is None
    client = TestClient(app)
    do_login(client)
    assert client.get("/api/v1/farm").status_code == 404
    assert any("필수 키" in rec.message for rec in caplog.records)


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
    h, _fid = _seed_bt(client)

    ok = client.post("/api/v1/bt", headers=h, json={
        "preset": "single_alley_loop", "params": {"robot": "scoutR", "alley": 6, "n": 1}})
    assert ok.status_code == 200, ok.text

    bad = client.post("/api/v1/bt", headers=h, json={
        "preset": "single_alley_loop", "params": {"robot": "scoutR", "alley": 7, "n": 1}})
    assert bad.status_code == 400, bad.text


# ── terrain 별 파리티 게이트 ───────────────────────────────────────────────────

def test_parity_gate_disabled_on_flat():
    """flat: split_k=4(구 불가 분할) 허용 — flat 은 파리티 자체가 없다(no_go
    도 인접 규칙도 이 분할과 무관 — A=[0..3]·B=[5..8] 모두 내부 전이가 ±1)."""
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


# ── C1 — 전이는 인접(±1)만(±2 철회) ───────────────────────────────────────────

def test_flat_terrain_only_allows_adjacent_transitions():
    """수정 라운드1 C1 — 초판이 허용했던 '한 칸 건너(±2)'가 철회됐다. REST 직접
    생성 경로(mission_ops.alleys_sequence_valid)와 반드시 같은 규칙이어야
    "200 수락 후 조용한 FAILED" 가 다시 나지 않는다."""
    farm = {"terrain": "flat"}
    with pytest.raises(presets.PresetError):
        presets.sequential_retry("scout01", [0, 2], farm=farm)   # 전이 2 — 이제 거부
    presets.sequential_retry("scout01", [0, 1], farm=farm)        # 전이 1 — 허용


# ── C2 — no_go_alleys(수동 필드) 기반 회피, widest argmax 아님 ────────────────

def test_flat_terrain_rejects_mission_touching_no_go_alley():
    farm = {"terrain": "flat", "no_go_alleys": [2],
           # row_origins 은 통로 0 이 최대 폭(진단 widest_alley 는 0 을 가리킴)이 되게
           # 일부러 비대칭으로 둔다 — no_go 판정이 widest_alley 를 안 쓴다는 것을
           # 이 불일치 자체로 증명한다.
           "row_origins": _row_origins(4, [15, 5, 5])}
    assert presets.widest_alley(farm) == 0          # 진단값은 여전히 0(최대 폭)
    with pytest.raises(presets.PresetError):         # 그런데도 거부되는 것은 통로 2(no_go)
        presets.sequential_retry("scout01", [1, 2, 3], farm=farm)
    presets.sequential_retry("scout01", [0, 1], farm=farm)   # no_go 를 안 건드리면 통과


def test_full_split_patrol_default_split_uses_largest_no_go_free_block():
    """C3 — 기본 분할점은 no_go 를 뺀 가장 큰 연속 블록의 중앙. rows=8(통로 0..6)
    에서 no_go=[5] → 블록 [0..4](길이5)·[6](길이1) 중 큰 쪽 [0..4], 중앙 k=2."""
    farm = {"terrain": "flat", "rows": 8, "no_go_alleys": [5]}
    plans = presets.full_split_patrol("scout01", "scout02", farm=farm)   # split_k 미지정
    a, b = _first_action_alleys(plans[0].tree), _first_action_alleys(plans[1].tree)
    assert a == [0, 1] and b == [3, 4]
    assert 5 not in a and 5 not in b


def test_full_split_patrol_explicit_split_k_on_no_go_alley_400():
    farm = {"terrain": "flat", "rows": 8, "no_go_alleys": [5]}
    with pytest.raises(presets.PresetError):
        presets.full_split_patrol("scout01", "scout02", split_k=5, farm=farm)


def test_full_split_patrol_uniform_farm_center_splits_whole_range():
    """C3 — no_go 가 없는(균일) farm 은 '허위 버퍼' 없이 전체 범위의 중앙에서
    분할한다(예전의 상수 5 는 farm 이 아예 없을 때만 남는다)."""
    farm = {"terrain": "flat", "rows": 8}                # no_go 없음 → 블록 = [0..6] 전체
    plans = presets.full_split_patrol("scout01", "scout02", farm=farm)
    a, b = _first_action_alleys(plans[0].tree), _first_action_alleys(plans[1].tree)
    assert a == [0, 1, 2] and b == [4, 5, 6]              # 블록 길이7 → k=3


# ── I3 — sequential_retry 범위 검사 ───────────────────────────────────────────

def test_sequential_retry_rejects_out_of_range_alley():
    with pytest.raises(presets.PresetError):
        presets.sequential_retry("scout01", [6, 7], farm={"terrain": "flat", "rows": 8})


# ── M2 — alley_widths 는 abs() 를 쓴다 ────────────────────────────────────────

def test_alley_widths_uses_abs():
    farm = {"row_origins": [[10.0, 0.0], [4.0, 0.0], [9.0, 0.0]]}   # 역행(음수 차분)
    assert presets.alley_widths(farm) == [6.0, 5.0]


# ── I1 — mission_ops 단일 출처: REST 직접 생성 + BT 발진 둘 다 no_go 를 본다 ──

def _seed_mission(client, robot_id="scoutM"):
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    fa = client.post("/api/v1/farms", json={"name": "농장M"}, headers=h).json()
    client.post("/api/v1/robots", headers=h,
               json={"id": robot_id, "farm_id": fa["id"], "name": robot_id})
    csrf_op = do_login(client)
    return {"X-CSRF": csrf_op}


def test_rest_direct_mission_on_no_go_alley_400(tmp_path):
    app, _farm = _app_with_farm(tmp_path, rows=27, terrain="flat", with_row_origins=False,
                                no_go_alleys=[20, 23], no_go_note="테스트 — 20/23 회피")
    client = TestClient(app)
    h = _seed_mission(client)
    r = client.post("/api/v1/missions", headers=h,
                    json={"robot_id": "scoutM", "alleys": [20]})
    assert r.status_code == 400, r.text
    assert "20" in r.json()["detail"]


def test_rest_direct_wildcard_mission_400_when_farm_has_no_go(tmp_path):
    app, _farm = _app_with_farm(tmp_path, rows=27, terrain="flat", with_row_origins=False,
                                no_go_alleys=[20, 23], no_go_note="테스트 — 20/23 회피")
    client = TestClient(app)
    h = _seed_mission(client)
    r = client.post("/api/v1/missions", headers=h, json={"robot_id": "scoutM"})   # alleys 생략
    assert r.status_code == 400, r.text
    assert "전 통로 임무 불가" in r.json()["detail"]
    assert "20" in r.json()["detail"] and "23" in r.json()["detail"]


def test_rest_direct_mission_wildcard_ok_when_farm_absent(client, app):
    """하위호환 — farm 미등록(app.state.farm is None)이면 와일드카드는 예전대로 통과."""
    assert app.state.farm is None
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    fa = client.post("/api/v1/farms", json={"name": "농장W"}, headers=h).json()
    client.post("/api/v1/robots", headers=h,
               json={"id": "scoutW", "farm_id": fa["id"], "name": "scoutW"})
    app.state.fleet.feed("scoutW", "tel/state", {})
    r = client.post("/api/v1/missions", headers=h, json={"robot_id": "scoutW"})
    assert r.status_code == 200, r.text


def test_farm_no_go_check_unit():
    """mission_ops.farm_no_go_check 단위 — (a) 교차 거부 (b) 와일드카드 거부
    (c) farm 없으면 무조건 통과(하위호환)."""
    farm = {"no_go_alleys": [20, 23]}
    ok, why = mission_ops.farm_no_go_check([19, 20, 21], farm)
    assert not ok and "20" in why
    ok, why = mission_ops.farm_no_go_check(None, farm)
    assert not ok and "전 통로 임무 불가" in why
    ok, _ = mission_ops.farm_no_go_check([0, 1, 2], farm)
    assert ok
    ok, _ = mission_ops.farm_no_go_check(None, None)
    assert ok


# ── 실사 farm.json 자체 회귀 — T7 인계·수정 라운드1 이 못박은 사실 ───────────────

def test_real_orchard_manifest_matches_t7_handoff_facts():
    """실사 농장은 rows=27(통로 26개)·flat·no_go_alleys=[20,23](수동 큐레이션,
    T4 §4.2·§1.3 근거) 이고, 기본 분담은 그 둘을 뺀 최대 블록[0..19]의 중앙(k=10)
    에서 갈린다 — A=10개/B=9개, 어느 쪽도 no_go 를 포함하지 않으며 내부 전이는
    전부 ±1 이다(mission_ops.alleys_sequence_valid 와 같은 규칙으로 실제 발진
    가능)."""
    real_path = (pathlib.Path(__file__).resolve().parent.parent.parent
                / "maps/orchard_real/farm.json")
    real_farm = json.loads(real_path.read_text(encoding="utf-8"))

    assert presets.n_alleys_of(real_farm) == 26
    assert presets.terrain_of(real_farm) == "flat"
    assert presets.no_go_alleys_of(real_farm) == [20, 23]
    assert presets.widest_alley(real_farm) == 20        # 진단값(더는 판정에 안 쓰임)

    plans = presets.full_split_patrol("scout01", "scout02", farm=real_farm)
    a, b = _first_action_alleys(plans[0].tree), _first_action_alleys(plans[1].tree)
    assert a == list(range(0, 10)) and b == list(range(11, 20))
    assert len(a) == 10 and len(b) == 9
    assert 20 not in a and 23 not in a and 20 not in b and 23 not in b
    from fleet_server import mission_ops as _mo
    assert _mo.alleys_sequence_valid(a) and _mo.alleys_sequence_valid(b)


def test_real_orchard_manifest_rejects_alley_23_in_preset():
    real_path = (pathlib.Path(__file__).resolve().parent.parent.parent
                / "maps/orchard_real/farm.json")
    real_farm = json.loads(real_path.read_text(encoding="utf-8"))
    with pytest.raises(presets.PresetError):
        presets.single_alley_loop("scout01", 23, farm=real_farm)


# ── 종단(end-to-end) — 리뷰어 지적, 실사 farm 픽스처로 ─────────────────────────

def _real_farm_dict() -> dict:
    real_path = (pathlib.Path(__file__).resolve().parent.parent.parent
                / "maps/orchard_real/farm.json")
    return json.loads(real_path.read_text(encoding="utf-8"))


def test_e2e_default_split_patrol_passes_no_go_and_adjacency_and_launches(tmp_path):
    """실사 farm 픽스처 — 기본 분담(A/B)이 no_go 무교차·±1 이고, 그 목록으로
    /api/v1/bt 를 찔러 실제로 200(발진 승인)까지 간다."""
    real_farm = _real_farm_dict()
    manifest_path = tmp_path / "farm.json"
    manifest_path.write_text(json.dumps(real_farm), encoding="utf-8")
    settings = _test_settings(farm_manifest_path=manifest_path)
    app = create_app(settings, fleet=InMemoryFleetPort())
    client = TestClient(app)
    h, fid = _seed_bt(client)
    client.post("/api/v1/robots", headers=h,
               json={"id": "scoutR2", "farm_id": fid, "name": "scoutR2"})
    r = client.post("/api/v1/bt", headers=h, json={
        "preset": "full_split_patrol", "params": {"robot_a": "scoutR", "robot_b": "scoutR2"}})
    assert r.status_code == 200, r.text
    assert len(r.json()["ids"]) == 2


def test_e2e_alley_23_preset_400(tmp_path):
    real_farm = _real_farm_dict()
    manifest_path = tmp_path / "farm.json"
    manifest_path.write_text(json.dumps(real_farm), encoding="utf-8")
    settings = _test_settings(farm_manifest_path=manifest_path)
    app = create_app(settings, fleet=InMemoryFleetPort())
    client = TestClient(app)
    h, _fid = _seed_bt(client)
    r = client.post("/api/v1/bt", headers=h, json={
        "preset": "single_alley_loop", "params": {"robot": "scoutR", "alley": 23, "n": 1}})
    assert r.status_code == 400, r.text


def test_e2e_rest_direct_alley_20_400(tmp_path):
    real_farm = _real_farm_dict()
    manifest_path = tmp_path / "farm.json"
    manifest_path.write_text(json.dumps(real_farm), encoding="utf-8")
    settings = _test_settings(farm_manifest_path=manifest_path)
    app = create_app(settings, fleet=InMemoryFleetPort())
    client = TestClient(app)
    h = _seed_mission(client)
    r = client.post("/api/v1/missions", headers=h, json={"robot_id": "scoutM", "alleys": [20]})
    assert r.status_code == 400, r.text


def test_e2e_rest_wildcard_400(tmp_path):
    real_farm = _real_farm_dict()
    manifest_path = tmp_path / "farm.json"
    manifest_path.write_text(json.dumps(real_farm), encoding="utf-8")
    settings = _test_settings(farm_manifest_path=manifest_path)
    app = create_app(settings, fleet=InMemoryFleetPort())
    client = TestClient(app)
    h = _seed_mission(client)
    r = client.post("/api/v1/missions", headers=h, json={"robot_id": "scoutM"})
    assert r.status_code == 400, r.text
    assert "전 통로 임무 불가" in r.json()["detail"]


def test_e2e_uniform_farm_default_split_still_launches(tmp_path):
    """균일 farm(no_go 없음) 기본 분담도 정상 동작 — '허위 버퍼' 소멸을 API
    경로로도 확인한다."""
    app, _farm = _app_with_farm(tmp_path, rows=9, terrain="flat", with_row_origins=False)
    client = TestClient(app)
    h, fid = _seed_bt(client)
    client.post("/api/v1/robots", headers=h,
               json={"id": "scoutU2", "farm_id": fid, "name": "scoutU2"})
    r = client.post("/api/v1/bt", headers=h, json={
        "preset": "full_split_patrol", "params": {"robot_a": "scoutR", "robot_b": "scoutU2"}})
    assert r.status_code == 200, r.text
    assert len(r.json()["ids"]) == 2


# ── M4 — BT 발진 거부 사유가 inst.note 에 남는다 ──────────────────────────────

def test_bt_launch_rejection_records_note_on_instance(tmp_path):
    """레이스로 프리셋 검증을 지나친 뒤(직접 create_from_plans 로 우회 재현)
    실제 발진(mission_ops)이 no_go 로 거부되면 inst.note 에 사유가 남는다."""
    import asyncio

    from fleet_server.bt import nodes

    app, _farm = _app_with_farm(tmp_path, rows=27, terrain="flat", with_row_origins=False,
                                no_go_alleys=[20, 23])
    client = TestClient(app)
    h, _fid = _seed_bt(client)

    tree = nodes.Sequence([nodes.Action({"robot": "scoutR", "alleys": [20]})])
    ids = app.state.bt_engine.create_from_plans(
        "single_alley_loop", {"robot": "scoutR", "alley": 20},
        [presets.Plan("scoutR", tree)], created_by=1)
    asyncio.run(app.state.bt_engine.tick_once())

    rows = client.get("/api/v1/bt", headers=h).json()
    row = next(r for r in rows if r["id"] == ids[0])
    assert row["note"] and "발진 거부" in row["note"]


# ── 수정 라운드2 — N1·N2·N3(잔여 Minor) ───────────────────────────────────────

def test_n1_terraced_farm_still_rejects_no_go_alley():
    """N1 — terraced 분기도 no_go 교집합을 본다(현행 도달 불가지만 C1 과 같은
    종류의 발산을 봉인). [2,3,4,5] 는 parity_safe 로는 통과(오름차순·첫 통로
    짝수)하지만 통로 4 가 no_go 라 여전히 400 이어야 한다."""
    farm = {"terrain": "terraced", "no_go_alleys": [4]}
    assert presets.parity_safe([2, 3, 4, 5])          # parity 만 보면 통과하는 목록
    with pytest.raises(presets.PresetError):
        presets.sequential_retry("scout01", [2, 3, 4, 5], farm=farm)


def test_n2_explicit_n_alleys_mismatch_with_farm_400():
    """N2 — flat 경로에서 명시 n_alleys 가 farm 유도값과 다르면 조용히 무시하지
    않고 400 이다."""
    farm = {"terrain": "flat", "rows": 8}             # n_alleys_of == 7
    with pytest.raises(presets.PresetError, match="n_alleys 는 farm 기하와 불일치"):
        presets.full_split_patrol("scout01", "scout02", n_alleys=9, farm=farm)


def test_n3_load_farm_rejects_missing_image_key(tmp_path, caplog):
    manifest_path = tmp_path / "farm.json"
    manifest_path.write_text(json.dumps({"rows": 10, "row_spacing_m": 5.0,
                                         "terrain": "flat"}), encoding="utf-8")
    settings = _test_settings(farm_manifest_path=manifest_path)
    with caplog.at_level(logging.WARNING, logger="fleet_server.farm"):
        app = create_app(settings, fleet=InMemoryFleetPort())
    assert app.state.farm is None
    assert any("필수 키" in rec.message and "image" in rec.message
              for rec in caplog.records)


def test_n3_load_farm_rejects_non_integer_no_go_alleys(tmp_path, caplog):
    manifest_path = tmp_path / "farm.json"
    manifest_path.write_text(json.dumps({
        "image": "x.jpg", "rows": 10, "row_spacing_m": 5.0, "terrain": "flat",
        "no_go_alleys": ["스물", 23],                  # 비정수 섞임
    }), encoding="utf-8")
    settings = _test_settings(farm_manifest_path=manifest_path)
    with caplog.at_level(logging.WARNING, logger="fleet_server.farm"):
        app = create_app(settings, fleet=InMemoryFleetPort())
    assert app.state.farm is None
    assert any("no_go_alleys" in rec.message for rec in caplog.records)
