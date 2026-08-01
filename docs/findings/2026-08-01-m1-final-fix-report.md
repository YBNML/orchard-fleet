# M1 관제 서버 — 전체 브랜치 리뷰 최종 수정 리포트

베이스: `ca7f9d5..5c18668` (14 태스크 / 23 커밋) 최종 리뷰에서 나온 발견사항 전체를
단일 수정 웨이브로 처리했다. 아래는 항목별 수정 내용·추가 테스트·게이트 3종 출력이다.

---

## Critical 1 — 대시보드 XSS 체인

**수정:** `server/web/index.html` 스크립트 상단에 `esc()` 헬퍼를 추가하고, 로봇
유래 문자열을 innerHTML 로 넣는 모든 지점에 적용했다.

- `renderFleet()` — `r.id`, `modeName(r.state.mode)` (로봇이 알 수 없는 mode 문자열을
  보내면 매핑 없이 원문이 그대로 나가던 지점)
- `applyFeatures()` — hello 의 `f.name`/`f.version`/`f.summary`
- `syncHealth()` — `f(v,u,lim)` 헬퍼(로봇이 `lidar_hz` 등에 문자열을 보내면
  `v < lim` 비교가 항상 거짓이 되어 원문이 그대로 innerHTML 에 들어가던 지점),
  `warns` 헬스 경고 목록
- `logEvent()` — 이벤트 `msg`, 로봇 `id`

**남겨둔 innerHTML (판단 근거):**
- `box.innerHTML = ""` / `sel.innerHTML = ""` / `ul.innerHTML = ""` /
  `div.innerHTML = ""` 류의 클리어 — 정적 빈 문자열, 주입 지점 아님.
- `buildAlleys()` 의 `${k}` — 루프 인덱스(항상 정수, `for (let k=0;k<r.geom.alleys;k++)`),
  문자열이 아니라 로봇 유래 컨텐츠가 들어갈 수 없음.
- `featList` 의 "기능 목록 없음 (구형 에이전트)" — 정적 마크업.
- 사용자 목록(`refreshUsers`)·이력 옵션(`refreshHistory`)·이력 이벤트
  (`hist-events`)는 이미 `textContent`/`el.value` 로 렌더돼 있어 애초에 안전
  (innerHTML 을 쓰지 않음) — 확인만 하고 변경 없음.

**테스트:**
- `server/tests/test_dashboard_serving.py::test_xss_escape_helper_applied_at_robot_derived_sinks`
  — `esc()` 정의 존재 + 취약했던 각 지점에 `esc(...)` 적용 여부를 정적 마커로 확인.
- `scripts/34_verify_m1_security.py` 10-1~10-4 — 실제 가짜 로봇이 `<img src=x onerror=alert(1)>`
  를 evt msg 로 보낸 뒤, 서버 `/events` 가 원문을 그대로 담는지(서버는 필터링하지
  않음이 의도), 대시보드 HTML 에 esc() 헬퍼와 각 싱크의 적용 마커가 있는지,
  페이로드 원문이 정적 마크업에 없는지 확인.

---

## Critical 2 — tz-naive 직렬화로 KST 이력 재생 실패

**수정:** `server/fleet_server/timeutil.py` 신설, `iso_utc(dt_)` 헬퍼
(naive 는 UTC 로 간주해 접미사를 붙인다). 적용처:
- `api/mission_routes.py` — `_mission_out` 의 `created_at`/`started_at`/`ended_at`
- `api/history_routes.py` — tracks/events/audit 의 `ts`
- `api/admin_routes.py` — `_robot_out` 의 `last_seen`

**참고:** `Robot.last_seen` DB 컬럼은 현재 어디서도 기록되지 않아(레거시 어댑터는
presence 를 메모리 레지스트리에서만 관리) 실사용 경로에서는 항상 `null` 이다.
그래도 직렬화 지점은 스펙대로 고쳤고, `_robot_out` 을 직접 호출하는 유닛 테스트로
회귀를 잡았다. (last_seen 을 실제로 채우는 것은 이번 브리프 범위 밖.)

**테스트:**
- `server/tests/test_timeutil.py` — naive/aware/None 3종.
- `server/tests/test_admin_api.py::test_robot_out_last_seen_naive_gets_tz_suffix`
- `server/tests/test_mission_api.py::test_mission_timestamps_have_tz_suffix_after_fresh_query`
  — POST 응답(같은 세션)이 아니라 이후 **다른 세션의 GET**(SQLite 가 naive 로
  돌려주는 실제 버그 재현 조건)에서 접미사를 확인.
- `server/tests/test_history_api.py::test_history_routes` 에 tz 접미사 단언 추가.
- `scripts/33_verify_m1.py` 9-4~9-8 — `/missions`·`/tracks`·`/events`·`/audit`·
  `/robots` 응답 전부 tz 접미사 검사.

---

## Important 3 — WS 접속 시점 스냅샷이 살아있는 권한·토폴로지로 동작

**수정:** `server/fleet_server/ws.py` 를 재작성.
- `revalidate(force=False)` 클로저 추가 — cmd·teleop 처리 직전 세션·사용자·
  `robot_farm` 을 DB 로 다시 확인한다. 실패(세션 만료·계정 disabled)하면
  `{"type":"denied", ...}` 를 큐에 넣고 **큐가 실제로 비워질 때까지 기다린 뒤**
  (`queue.join()` — sender 가 `task_done()` 호출하도록 변경) `websocket.close(code=4401)` +
  감사(`rejected`, "세션 재검사 실패 — 연결 종료").
- 캐시: 접속 후 첫 cmd/teleop 은 `last_check = -inf` 라 무조건 재검사(접속 직후
  정지된 계정을 확실히 잡음). 이후 5초 이내 재검사면 캐시 사용 — 단
  `stop_all`·`clear_estop`·`set_mode` 는 `force=True` 로 항상 재검사.
- `stop_all` 은 위 강제 재검사에서 `conn.robot_farm`·`conn.scope` 를 이미 최신으로
  갱신했으므로, 접속 이후 등록된 로봇도 팬아웃에 포함된다.
- 저비용 항목도 같이 처리: `receive_json()` 을 try/except 로 감싸고
  `isinstance(msg, dict)` 가드 추가 — 비-JSON·비-dict 프레임이 트레이스백으로
  연결을 죽이지 않고 무시된다.

**테스트 (finding 3 요구 2개 + 1개 추가):**
- `test_ws_gateway.py::test_disabled_mid_session_revokes_estop_and_closes` —
  접속 유지 중 DB 에서 계정을 `disabled=True` 로 바꾸면 다음 estop 이 거부되고
  연결이 닫힘 + 감사에 남음.
- `test_ws_gateway.py::test_stop_all_includes_robot_registered_after_connect` —
  접속 후 새 로봇(r3) 등록 → `stop_all` 결과에 r3 포함.
- `test_ws_gateway.py::test_ws_survives_malformed_frame` — 깨진 JSON·비-dict
  JSON 을 보내도 연결이 죽지 않고 다음 유효 명령이 정상 처리됨.

---

## Important 4 — 임무 상태 REST/로봇보고 레이스 + 활성 임무 중복

**수정:**
- `server/fleet_server/missions.py` `apply()` — 기대 상태를 조건으로 하는
  `UPDATE missions SET ... WHERE id=:id AND state=:expected` (`synchronize_session=False`)
  로 바꿨다. `rowcount==0` 이면 `InvalidTransition` 을 내고, 호출자에게 실제
  최신 상태를 보여주려고 `db.refresh(mission)` 을 한 뒤 예외를 던진다. 성공하면
  ORM 객체 속성을 직접 갱신(추가 SELECT 없이)하고 `MissionEvent` 를 남긴다.
- `server/fleet_server/api/mission_routes.py` `create_mission` — 로봇에
  QUEUED/RUNNING/PAUSED 활성 임무가 있으면 409 + 감사(`rejected`,
  "활성 임무 이미 존재 mission=...").

**테스트:**
- `test_missions_sm.py::test_apply_optimistic_guard_stale_object` — 별도
  세션(레이스 상대)이 먼저 임무를 DONE 으로 커밋한 뒤, 원래 세션의 stale 객체로
  `apply(..., "pause")` 하면 `InvalidTransition`. (`expire_on_commit=False` 라
  같은 세션 재조회로는 재현이 안 돼, `make_session_factory(db.get_bind())` 로
  진짜 별도 세션을 만들어 실제 레이스 조건을 재현했다.)
- `test_mission_api.py::test_duplicate_active_mission_409` — 활성 임무가 있는
  로봇에 두 번째 임무 생성 시도 → 409, DB 에 활성 임무가 1개만 남음을 확인.

---

## Important 5 — SQLite 운용 프래그마 부재

**수정:** `server/fleet_server/db.py` `make_engine()` 에 `connect` 이벤트 리스너로
`PRAGMA journal_mode=WAL`(인메모리에서는 무시됨) · `busy_timeout=3000` ·
`foreign_keys=ON` 을 추가.

**FK 강제 활성화로 드러난 파급 (지시대로 확인 후 처리):**
1. `server/tests/test_service.py` 의 `test_tel_state_feed_creates_track` /
   `test_evt_feed_dedups_by_seq` 가 `robots` 테이블에 로봇을 등록하지 않고
   텔레메트리를 주입해 `IntegrityError` 로 깨짐 — 실제로는 텔레메트리는 항상
   등록된 로봇에서만 온다(register_robot 이전에 Robot 행이 먼저 생김)는 실제
   흐름에 맞게 `_seed_robot()` 헬퍼로 테스트 픽스처를 보정했다(제품 코드 결함
   아님, 테스트 픽스처가 비현실적이었던 것).
2. `create_user`/`patch_user` 의 `farm_ids` 에 존재하지 않는 농장 id 가 오면
   전에는 `UserFarm` 댕글링 행이 조용히 생겼는데, FK 강제 후엔 500 이 날 수
   있었다 — `admin_routes.py` 에 `_missing_farm_ids()` 검증을 추가해 404 로
   변환했다.
3. **구현 중 발견한 부수 버그(수정함):** `patch_user` 에서 farm_ids 검증을
   `role`/`disabled`/`password` 대입 **뒤에** 두었더니, 검증 실패로 `audit.record()`
   를 호출하는 순간 그 함수 내부의 `db.commit()` 이 앞서 대입해 둔 값까지
   같이 커밋해버려 "거부됐는데 role 변경은 반영되는" 부분 반영 버그가 생겼다.
   실제로 재현해 확인한 뒤, farm_ids 검증을 **다른 필드 대입보다 먼저**로
   옮겨 원자성을 회복했다.
4. `patch_robot` 도 `farm_id` 존재 검증을 추가(재배선 로직과 함께, Important 6 참고).

**테스트:**
- `test_admin_api.py::test_create_and_patch_user_reject_unknown_farm_ids`
- `test_admin_api.py::test_patch_user_rejects_atomically_no_partial_mutation`
  (위 3번 버그의 회귀 테스트)
- `test_admin_api.py::test_patch_robot_unknown_farm_404`
- 전체 단위 스위트(97개)가 FK 강제 아래에서 통과함을 확인.

---

## Important 6 — PATCH /robots 가 레거시 링크를 재배선하지 않음

**수정:**
- `server/fleet_server/fleet/port.py` — `FleetPort` ABC 에 `unregister_robot(robot_id)`
  추가, `InMemoryFleetPort` 에 구현(딕셔너리에서 제거).
- `server/fleet_server/fleet/legacy_ws.py` — `LegacyFleetPort.unregister_robot()` —
  링크 `stop()` + 태스크 `cancel()` + `_links`/`_tasks` 에서 제거.
- `server/fleet_server/api/admin_routes.py` `patch_robot` — `conn_kind`/
  `config_json`/`farm_id` 중 하나라도 바뀌면(대입 **전**에 원본과 비교해 판단)
  `fleet.unregister_robot(r.id)` 후 `fleet.register_robot(...)` 으로 재배선.
  `farm_id` 변경 시 대상 농장 존재 검증도 추가(404).

**테스트:**
- `test_legacy_ws.py::test_unregister_robot_stops_link_and_allows_rewire` —
  포트 레벨에서 unregister 가 링크를 실제로 멈추고(`link._stop`), 재등록 시
  새 설정(ws_url/token)을 반영한 **새 링크 객체**가 생김을 확인(기존 조기반환
  버그였다면 재등록이 no-op 이라 이 테스트가 실패했을 것).
- `test_legacy_ws.py::test_unregister_robot_missing_id_is_noop`
- `test_admin_api.py::test_patch_robot_rewires_connection_on_config_change` —
  API 레벨. `InMemoryFleetPort` 는 무조건 덮어쓰기라 재배선 여부를 구분 못 하므로
  `unregister_robot` 호출 자체를 기록하는 `_SpyFleetPort` 로 "config 변경 시엔
  unregister 호출됨 / name 만 변경 시엔 호출 안 됨" 을 검증.

---

## 함께 처리한 저비용 항목

- **CSRF 상수시간 비교** — `deps.py::csrf_protect` 를 `secrets.compare_digest` 로.
  기능적으로 관측 가능한 차이는 없어(여전히 불일치 시 403) 별도 테스트는 추가하지
  않았고, 기존 CSRF 불일치 테스트들이 계속 통과함으로 회귀 없음을 확인.
- **WS 비-dict/비-JSON 가드** — Important 3 수정에 포함(`ws.py`).
- **로그아웃 감사 기록** — `auth_routes.py::logout` 에 `audit.record(action="logout", ...)`
  추가. 테스트: `test_auth_api.py::test_logout_recorded_in_audit`.

---

## 게이트 3종 최종 결과

### 단위 테스트 (`server/.venv/bin/python -m pytest -q`)
```
........................................................................ [ 74%]
.........................                                                [100%]
97 passed, 1 warning in 9.43s
```
(경고 1건은 기존과 동일한 `httpx`/`starlette.testclient` 지원중단 경고, 무관)

### `scripts/33_verify_m1.py` (E2E)
```
29/29 통과
```
9-4~9-8 에 tz 접미사 검사 5건 추가(Critical 2 회귀). 그 외 기존 24건 전부 통과.

### `scripts/34_verify_m1_security.py` (보안 회귀)
```
29/29 통과
```
10-1~10-4 에 XSS 방어 검사 4건 추가(Critical 1 회귀). 그 외 기존 25건 전부 통과.

전문은 아래에 기록:
- `/tmp/pytest_final.txt` (세션 임시 파일 — 필요시 재실행: 위 명령 그대로)
- 본 리포트에 최종 실행 로그를 인용했으므로 임시 파일 재확인은 불필요.

---

## 미해결 항목과 사유

없음 — 브리프에 명시된 Critical 2건, Important 4건, 저비용 3건, 게이트 회귀 2건
전부 구현·테스트·검증까지 완료했다.

**범위 밖으로 남긴 것 (참고용, 브리프가 요구하지 않음):**
- `Robot.last_seen` 을 실제로 채우는 것(레거시 어댑터가 presence 를 메모리에만
  유지) — Critical 2 는 "직렬화가 tz-safe 한가"만 요구했고, "값을 채울지"는
  별개의 기능 갭이라 손대지 않았다.
- WS 세션 캐시 TTL(5초)은 브리프가 예시로 제시한 값 그대로 채택 — 별도 튜닝
  근거는 없음.

---

## 변경 파일 목록

```
server/fleet_server/timeutil.py               (신설)
server/fleet_server/api/admin_routes.py
server/fleet_server/api/auth_routes.py
server/fleet_server/api/history_routes.py
server/fleet_server/api/mission_routes.py
server/fleet_server/db.py
server/fleet_server/deps.py
server/fleet_server/fleet/legacy_ws.py
server/fleet_server/fleet/port.py
server/fleet_server/missions.py
server/fleet_server/ws.py
server/web/index.html
server/tests/test_timeutil.py                  (신설)
server/tests/test_admin_api.py
server/tests/test_auth_api.py
server/tests/test_dashboard_serving.py
server/tests/test_history_api.py
server/tests/test_legacy_ws.py
server/tests/test_mission_api.py
server/tests/test_missions_sm.py
server/tests/test_service.py
server/tests/test_ws_gateway.py
scripts/33_verify_m1.py
scripts/34_verify_m1_security.py
scripts/fake_legacy_robot.py
```
