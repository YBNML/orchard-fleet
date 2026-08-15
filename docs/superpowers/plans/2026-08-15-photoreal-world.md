# 실사 정사영상 월드 + 관제 실사 지도 구현 계획 (스펙 ④)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 실제 항공 정사영상(PNOA)을 기반으로 평탄 과수원 월드를 재구축하고, 관제 지도를 그 실사 이미지 위에 정합해 올린다 (스펙: docs/superpowers/specs/2026-08-14-photoreal-world-design.md).

**Architecture:** 정합의 단일 출처는 `maps/orchard_real/farm.json`(농장 기하 매니페스트). 이미지→매니페스트(반자동 추출·사람 승인)→월드 생성(gen_world --farm, 지면에 정사영상 텍스처)→측위 번들 재캘리브레이션→서버가 매니페스트·이미지를 서빙→대시보드가 아핀 정합으로 바탕에 렌더. 로봇 쪽은 무수정 — 미션 엔진(rows/row_spacing 파라미터)·로컬라이저(번들 geom)가 이미 데이터 주도다.

**Tech Stack:** 기존 스택 + 이미지 처리는 numpy/PIL(신규 의존성 금지 — OpenCV 불가).

## Global Constraints

- **라이선스 위생**: 커밋되는 외부 이미지는 재배포 가능 라이선스 + 출처 표기를 `sim/assets/imagery/LICENSE-DATA.md` 에 기록. "근거가 불명확하면 쓰지 않는다." 원본 타일 전체 커밋 금지 — 필요 크롭·재압축본만.
- **병존**: 기존 `sim/worlds/orchard_nav.sdf`·`maps/orchard_v1/` 는 수정·삭제 금지. 신규는 `orchard_real.sdf`·`maps/orchard_real/`. 런치 `world:=real|terraced` 선택, 기본 전환은 T7 게이트 후.
- **farm.json 이 단일 출처**: 시뮬·서버·대시보드의 농장 기하는 전부 이 파일에서 나온다. 없거나 이미지 해시 불일치면 명시적 실패(무음 기본값 금지).
- **로봇 계약 불변**: robomw 수정 금지. 기하는 에이전트/로컬라이저 **파라미터**로 전달(이미 rows·row_spacing 파라미터 존재 — mission.py:38-40, 번들 geom).
- **상부 기하(확정 수치)**: MID-360 = 롤바 아치 상단 중앙 지상 0.80m / D455 = 노즈 브래킷 지상 0.55m 하향 12°(④는 시각+프레임만, 센서 플러그인 금지 — 스펙 ⑤) / 분석 카메라 아치 다리 / 앰버 경광등 / 아치 뒤 적재 데크.
- 프로세스 관례는 스펙 ③ 계획과 동일(콜콘, 스크래치 헬퍼 파일, pgrep 자기일치 금지, git add 의도 파일만, 커밋 한국어). 스크래치: `/tmp/claude-1000/-home-myhome-YBNML/691d883b-bd7f-499c-9b36-a59b0bd14a8a/scratchpad/`.
- 실기 검증 시 서버(8000) 무중단 원칙, 재기동 필요 시 fleet.db 보존.

---

## 파일 구조 (신설·수정)

```
sim/assets/imagery/{orchard_ortho.jpg, orchard_ortho_meta.json, LICENSE-DATA.md, SOURCES.md}   # T1
scripts/51_extract_farm_geometry.py        # T2 — 열 기하 반자동 추출 → farm.json + 승인 오버레이
maps/orchard_real/farm.json                # T2 산출(스키마는 스펙 §2 그대로)
scripts/gen_world.py                       # T3 — --farm 모드(기존 모드 불변)
scripts/gen_heightmap.py                   # T3 — --flat-gentle(≤3°)
sim/models/scout_mini_mid70/model.sdf      # T3 — 상부 하이브리드(아치·D455 시각+프레임·경광등·데크)
sim/worlds/orchard_real.sdf                # T3 산출
ros2_ws/src/orchard_sim/launch/*.py        # T3/T4 — world:=real|terraced, rows·spacing 파라미터를 farm.json 에서
maps/orchard_real/{번들, anchor_walls.json} # T4 — 37·47 재실행 산출
server/fleet_server/api/farm_routes.py     # T5 신설 — GET /api/v1/farm (farm.json+terrain), 정적 이미지 서빙 배선
server/fleet_server/bt/presets.py          # T5 — N_ALLEYS 상수 → farm 기반, parity_safe 는 terrain=="terraced" 만
server/web/index.html                      # T6 — 실사 바탕 렌더·토글·폴백
docs/findings/2026-08-15-photoreal-world.md # T7
```

### Task 1: 이미지 확보·구획 선정·라이선스 문서

**Files:** Create: `sim/assets/imagery/` 4파일 위 구조대로.

**Interfaces:**
- Produces: `orchard_ortho.jpg`(선정 구획 크롭, 장변 ≤4096px, ≤2MB) · `orchard_ortho_meta.json` = `{"source":"PNOA","source_crs":"EPSG:25831","crop_origin_source_px":[..],"gsd_m":0.25,"sha256":"..","acquired":"YYYY-MM-DD"}` · LICENSE-DATA.md(라이선스 원문 링크·의무 이행 문구·표기 예시) · SOURCES.md(후보 표 — 리서치 결과가 스크래치 `imagery_candidates.md` 에 있다, 옮겨 정리).

- [ ] **Step 1**: 스크래치 `imagery_candidates.md` 를 읽고 PNOA(CNIG 다운로드센터)에서 레리다(Segrià) 사과·배 산지 타일을 내려받아 후보 구획 3개 이상 스크린캡·좌표 기록. 기준: 연속 ≥2ha, 열 간격 3~4.5m, 열 방향 정렬, 그림자·구름 최소. 다운로드가 막히면 BD ORTHO 로 폴백(계획 동일).
- [ ] **Step 2**: 후보 비교표(각 크롭 PNG + 열 간격 실측치)를 리포트에 남기고 최적 구획 1개 선정 — **컨트롤러 승인 게이트**: 선정 크롭 이미지를 리포트에 경로로 명시하고 STATUS 에 승인 요청을 포함하라(승인 후 다음 스텝).
- [ ] **Step 3**: 크롭·재압축(≤2MB)·meta·라이선스 문서 작성. sha256 기록.
- [ ] **Step 4**: Commit — `"실사 정사영상 확보 — PNOA 구획 선정·라이선스 문서"`

### Task 2: 열 기하 추출 → farm.json

**Files:** Create: `scripts/51_extract_farm_geometry.py`, `maps/orchard_real/farm.json`. Test: 합성 이미지 단위시험(스크립트 내 `--self-test`).

**Interfaces:**
- Consumes: T1 의 orchard_ortho.jpg + meta.
- Produces: `farm.json` — 스펙 §2 스키마 그대로: `image, px_per_m, origin_px[2], rotation_deg, rows, row_spacing_m, row_length_m, tree_spacing_m, row_origins[[x,y]..], headland_m, bounds_m[2][2], terrain:"flat", image_sha256`. 월드↔픽셀 아핀: `px = origin_px + R(rotation_deg)·(wx,wy)·px_per_m` (y 축 부호는 이미지 좌표계 문서화 필수). 승인 오버레이: `maps/orchard_real/geometry_overlay.png`(이미지 위 열 축·경계 표시).

- [ ] **Step 1: 실패 테스트** — `--self-test`: 합성 줄무늬 이미지(간격 14px=3.5m@4px/m, 회전 7°)를 생성해 추출기가 `row_spacing_m∈[3.43,3.57]`, `rotation_deg∈[6.5,7.5]` 를 복원하는지 단언. RED 확인.
- [ ] **Step 2**: 구현 — 식생 대비(녹색 채널 우세)→열 방향은 방향별 분산 최대화(0.5° 격자 탐색), 간격은 투영 프로파일의 자기상관 피크. 열 시작·끝은 프로파일 임계. 결과를 farm.json+오버레이로 출력. self-test GREEN.
- [ ] **Step 3**: 실제 이미지에 적용 → 오버레이 PNG 를 리포트에 명시 — **컨트롤러 승인 게이트**(스펙 §6: 사람 승인 없이 farm.json 을 쓰지 않는다).
- [ ] **Step 4**: Commit — `"농장 기하 매니페스트 — 열 추출 도구·farm.json (승인 오버레이 포함)"`

### Task 3: 월드 재구축 (지면 실사 텍스처 + 상부 하이브리드)

**Files:** Modify: `scripts/gen_world.py`(`--farm PATH` 모드 — 기존 인자 경로 불변), `scripts/gen_heightmap.py`(`--flat-gentle`: 평탄 기본 + 선택 ≤3° 단일 경사, 선회 패드 코드 미사용), `sim/models/scout_mini_mid70/model.sdf`(상부 하이브리드 — Global Constraints 수치), 런치(world 인자). Create: `sim/worlds/orchard_real.sdf`, `sim/models/orchard_terrain_real/`(하이트맵+**정사영상을 diffuse 텍스처로**).

**Interfaces:**
- Consumes: farm.json(나무 배치·경계), orchard_ortho.jpg(지면 텍스처).
- Produces: `gen_world.py --farm maps/orchard_real/farm.json --robots "scout01:<통로0 남단> scout02:<분담B 시작 남단>" --out sim/worlds/orchard_real.sdf`. 나무는 row_origins+tree_spacing 격자 스냅. 스폰 좌표는 farm.json 에서 계산해 리포트에 기록.
- 상부: 시각 지오메트리(아치·데크·경광등·D455 몸체)+`d455_link` 프레임(0.55m, pitch +12° 하향). 센서 플러그인 추가 금지. MID-360 pose 를 아치 상단 0.80m 로 이동.

- [ ] **Step 1**: gen_heightmap `--flat-gentle` + terrain_real 모델(텍스처 UV 가 farm.json 아핀과 일치 — 나무가 이미지의 열 위에 서야 한다). 검증: 지면 텍스처 위 나무 배치 top-view 스크린샷.
- [ ] **Step 2**: gen_world `--farm` 모드 + model.sdf 상부 개조 → orchard_real.sdf 생성, gz 헤드리스 스모크(모델 2대 스폰·토픽·RTF 실측). 기존 월드 재생성 바이트 동일성 확인(계단식 경로 불변 증명).
- [ ] **Step 3**: 라이다 자기반사 확인 — 아치 다리 음영 2줄 실측, 필터 파라미터 재도출(rings/거리 게이트), 잔점 0 확인. D455 프레임 TF 존재 확인.
- [ ] **Step 4**: 런치 `world:=real|terraced`(기본 terraced 유지 — 전환은 T7), rows·row_spacing 파라미터를 farm.json 값으로 전달하는 경로 추가.
- [ ] **Step 5**: Commit — `"실사 월드 — gen_world --farm·평탄 지형·지면 정사영상 텍스처·상부 하이브리드"`

### Task 4: 측위 재캘리브레이션 + 1호기 회귀

**Files:** Create: `maps/orchard_real/` 번들·anchor_walls.json (37·47 재실행 산출). Modify: 스크래치 헬퍼 WORLD 인자화(커밋 밖).

- [ ] **Step 1**: real 월드 기동 → 번들 빌드(37 — 평탄이라 사다리형 분류)·벽 캘리브레이션(47 — 새 경계 기준, 라이다 0.80m 의 벽 콘 z 대역 재확인 후 필요시 조정).
- [ ] **Step 2**: **1호기 회귀 게이트**: 정찰 3통로 무개입 완주(cmd_result completed·interventions 0) + 통로 내 횡오차가 계단식 기준선(T8 0.050~0.082m)과 동급 + self_test all_ok(라이다 sim-time 수신율 포함 — 평탄 월드 RTF 실측 기록).
- [ ] **Step 3**: Commit — `"실사 월드 측위 — 번들·벽 재캘리브레이션 (1호기 회귀 전건)"`

### Task 5: 서버 farm 소비 (매니페스트 API + 프리셋 일반화)

**Files:** Create: `server/fleet_server/api/farm_routes.py`, `server/tests/test_farm_routes.py`. Modify: `server/fleet_server/bt/presets.py`(N_ALLEYS→farm), app 배선, 정적 이미지 서빙.

**Interfaces:**
- Produces: `GET /api/v1/farm` → farm.json 내용 + `{"ortho_url":"/assets/orchard_ortho.jpg"}` (인증: 로그인 사용자 전원). 서버 설정 `FLEET_FARM_MANIFEST`(기본 `maps/orchard_real/farm.json`; 파일 없으면 기동 로그 경고 + 404 — 서버는 뜬다, 대시보드가 폴백). 엔드포인트는 서빙 이미지의 sha256 을 `image_sha256` 과 대조해 불일치면 500+로그(무음 불일치 금지 — 스펙 §6). presets: `n_alleys = rows-1` 을 farm 에서, `parity_safe` 검증은 `terrain=="terraced"` 일 때만 적용(flat 이면 순서 제약 없음).

- [ ] **Step 1: 실패 테스트**:
```python
def test_farm_endpoint_serves_manifest(client_with_farm):
    r = client_with_farm.get("/api/v1/farm")
    assert r.status_code == 200 and r.json()["rows"] == 10 and r.json()["terrain"] == "flat"

def test_presets_use_farm_alley_count():   # farm rows=8 → 통로 0..6 만 유효, 7 은 400
def test_parity_gate_disabled_on_flat():   # flat: split_k=4(구 불가 분할) 허용
def test_parity_gate_kept_on_terraced():   # terraced: 기존 400 유지
```
- [ ] **Step 2**: 구현 → GREEN → server pytest 전건.
- [ ] **Step 3**: Commit — `"서버 farm 매니페스트 — API·프리셋 일반화·terrain별 파리티"`

### Task 6: 대시보드 실사 지도

**Files:** Modify: `server/web/index.html`. Test: Playwright(webapp-testing 관례).

**Interfaces:**
- Consumes: `GET /api/v1/farm`(아핀·ortho_url). 렌더: 지도 캔버스 최하층에 이미지(월드→픽셀 아핀 역변환으로 배치·회전), 기존 오버레이(통로 밴드·로봇·궤적) 불변. 토글 버튼 "실사/벡터"(기본 실사, localStorage 기억). 이미지 로드 실패·/farm 404 → 벡터 폴백 + 콘솔 경고 1회(무음 빈 지도 금지).

- [ ] **Step 1**: 구현(이미지 프리로드·Canvas drawImage 회전 정합). 통로 클릭 판정 등 기존 기하 코드는 무수정이어야 한다(g 는 로봇 텔레메트리 geom — 검증으로 확인).
- [ ] **Step 2**: Playwright — real 월드 실기: 실사 바탕+오버레이 정합(통로 밴드가 이미지 열과 일치), 토글, 폴백(404 모의), 콘솔 오류 0, 스크린샷 2장 docs/figures/(dashboard_photoreal_map.png, dashboard_photoreal_locks.png).
- [ ] **Step 3**: Commit — `"대시보드 실사 지도 — 정사영상 바탕·토글·폴백"`

### Task 7: 최종 게이트 + 기본 전환 + 문서

**Files:** Create: `docs/findings/2026-08-15-photoreal-world.md`, `scripts/52_verify_farm_alignment.py`. Modify: 런치 기본 `world:=real`, README 현재 상태.

- [ ] **Step 1**: 정합 게이트 — 52 스크립트: 월드 나무 20그루 표본의 sdf pose ↔ farm.json 아핀 역변환 픽셀 위치 비교, 오차 ≤0.5m 전건. + 대시보드 시각 검증(열 오버레이).
- [ ] **Step 2**: **2대 동시 게이트** — real 월드에서 분담 프리셋(flat 이라 임의 분할 허용 — 기본 split) 2대 동시 무개입 완주 + RTF·수신율 실측(계단식 대비 비교표).
- [ ] **Step 3**: 병존 스모크 — `world:=terraced` 기동·프로브 응답 확인(기존 자산 무손상). server·robomw pytest 전건.
- [ ] **Step 4**: findings(게이트 표·정합 수치·RTF 비교·라이선스 요약·스펙 ⑤ 인계: d455_link 프레임 준비됨) + README 갱신 + 기본 월드 전환 → Commit — `"스펙 ④ 완료 — 실사 월드·관제 실사 지도 게이트 전 항목"`
