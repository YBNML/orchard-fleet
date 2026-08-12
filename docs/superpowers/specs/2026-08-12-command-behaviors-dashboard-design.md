# 신규 명령 동작 + 대시보드 명령 UI — 설계 (스펙 ②)

| 항목 | 내용 |
|---|---|
| 날짜 | 2026-08-12 |
| 전제 | 스펙 ①(robomw v0.1) 병합 완료 — 명령 계약 v1·SDK 5종·cmd_result 상관·라우팅(신규 4종은 UNSUPPORTED) |
| 범위 | Work(정찰)·Diag 3종 실제 동작 + 대시보드 명령 UI + 서버 배선 |
| 범위 밖 | 이기종 2호기·서버 임무 큐(스펙 ③) · 블랙박스 파일 전송(로컬 저장까지만) · spray/mow/transport 동작 |

## 0. 원칙

- 계약은 이미 있다(①) — 이 스펙은 **동작과 화면**만 붙인다. 계약 변경은 additive 텔레메트리 키 1개(`work`)뿐.
- 능력 선언이 게이트다: 로봇은 자기가 할 수 있는 작업만 `hello.capabilities.work.types` 로 선언하고, 미선언 유형은 UNSUPPORTED 로 거부한다 — "이기종 로봇마다 가능한 작업이 다르다"의 첫 실사용.
- 안전 불변: 새 동작 어느 것도 SafetyArbiter·속도 단일 경로·estop 2단계를 우회하지 않는다. self_test 는 움직임을 만들지 않는다.

## 1. Work — 정찰(scout)

### 의미론
- `mission_start` payload 에 `work:{type:"scout", params:{speed_scale?}}` 가 오면 **전 통로(0..R-2) 부스트로피돈 완주 임무**다. `alleys` 가 생략되면 전 통로로 자동 설정하고, 명시돼 있으면 그 목록을 존중한다(부분 정찰).
- `speed_scale`(0.1~1.0, 기본 1.0): 임무 주행 속도에 곱한다(`speed × scale`). 시운전 모드 0.3배와는 별개로 중첩 적용.
- 진행률 = coverage(완주 통로/전체). 텔레메트리 state 에 additive 키:
  `"work": {"type": "scout", "progress": 0.0~1.0}` (작업 없으면 키 생략).
- `work_stop`: 작업 플래그만 내린다(active=false, state 의 work 키 제거). **주행(임무)은 계속** — 임무를 세우려면 기존 mission_pause/cancel. cmd_result completed 로 응답.
- 미지원 유형(spray/mow/transport — 계약상 유효): `capabilities.work.types` 에 없으면 mission_start 를 `rejected(UNSUPPORTED, "미지원 작업 유형")` 으로 거부.

### 구현 자리
- `SimWork`(orchard_sim/adapters/sim_work.py): Work SDK 구현 — start/stop/status. 상태는 bb 를 통해 임무 엔진의 coverage 를 읽어 progress 로 노출.
- robomw.profiles.orchard.mission: mission_start 처리에서 work 수용 확장(자동 전통로·speed_scale 적용·Work.start 호출), 완료·취소 시 Work.stop.
- hello: `capabilities.work.types=["scout"]` (SimWork 주입 시).

## 2. Diag — self_test · relocalize · blackbox_dump

### self_test (operator)
- 전제: 주행 정지 상태(임무 RUNNING 이면 `rejected(BUSY, "주행 중")`).
- 항목(각각 `SelfTestItem(name, ok, detail)`):
  - `lidar`: 최근 3초 점군 수신율 ≥ 8 Hz
  - `imu`: 최근 3초 IMU 수신율 ≥ 80 Hz
  - `localizer`: pose 존재 + quality ≥ 0.3
  - `link`: 관제 링크 생존(최근 수신 < 1.5 s)
  - `drive`: Drive 어댑터 존재 + limits 유효 (**구동 테스트 없음** — 움직임 금지)
- payload `items` 로 부분 실행 가능. 결과는 `cmd_result completed, data:{items:[{name,ok,detail}...], all_ok:bool}` — 5초 내 동기 응답.
- 구현: `ScoutDiag`(orchard_sim/adapters/scout_diag.py) — RosSensors 의 수신율 카운터를 공유.

### relocalize (admin)
- payload: `{x,y,yaw}` 또는 `{alley:k, end:"north"/"south"}` (격자 계산: x=x0+(k+0.5)S, y=∓(col_l/2+1.5), yaw=북단이면 -π/2 남단이면 +π/2 — 진입 방향).
- 경로: 명령 → Localizer.reinit(pose) → RosSensors 가 map_localizer 의 신설 토픽 `~/reinit`(std_msgs/String, JSON `{"x":..,"y":..,"yaw":..}`) 발행 → map_localizer 가 수신 시 현 INIT 파라미터 절차와 동일하게 재초기화(T_mo 재설정 + 상태 리셋).
- 응답: 발행 후 2초 내 localizer diagnostics 의 quality ≥ 0.3 이면 completed, 아니면 `failed(TIMEOUT, 진단 스냅샷)`.
- 이번 세션 내내 손으로 한 운영자 복구(restart_loc)를 명령 1방으로 정식화하는 것.

### blackbox_dump (operator)
- 코어가 상시 유지하는 링버퍼: 이벤트(기존 50건) + 궤적(1 Hz pose, 최근 900초).
- 실행: `/tmp/blackbox_<robot_id>_<unix_ts>.npz` 로 저장(events json 배열·poses Nx4[t,x,y,yaw]) → `cmd_result completed, data:{path, bytes, events, poses}`.
- window_s(기본 600, 최대 900)로 궤적 범위 절단. 파일 전송은 범위 밖(경로 표시까지).

## 3. 대시보드 명령 UI (server/web/index.html)

- **전체 정찰 시작** 버튼(임무 패널 상단, operator): 클릭 → 속도 스케일 슬라이더(0.1~1.0, 기본 1.0) 확인 다이얼로그 → REST `POST /missions {robot_id, work:{type:"scout", params:{speed_scale}}}` (alleys 생략). 기존 통로 선택 임무 UI 는 불변.
- **유지보수 카드**(신설 pane, data-needs 게이팅):
  - 셀프테스트 버튼(operator) → 결과를 항목별 ✓/✗ 표로 (detail 은 hover/줄)
  - 위치 재초기화(admin): 통로(0..8)·단(북/남) 드롭다운 → confirm → 명령
  - 블랙박스 덤프 버튼(operator) → 결과 경로·크기 표시
- **cmd_result 표시**: 대시보드가 이제 명령에 `cmd_id` 를 부여(`"d"+Date.now()`), evt 채널로 돌아오는 cmd_result 를 토스트(성공 초록/거부는 code+사유)로 띄우고 사건 기록에는 요약 1줄로 정리.
- 서버 배선: REST /missions 가 work 필드 통과(검증은 로봇 몫) + alleys 생략 허용, `_ALWAYS_REVALIDATE` 에 relocalize 추가(스펙① 이월 해소). WS 액션은 ①에서 이미 뚫려 있음.

## 4. 오류 처리

- self_test 실행 중 임무 시작 요청: 임무가 우선(테스트는 5초 내 끝난다) — 라우터 직렬 처리라 자연 순차.
- relocalize 를 주행 중 호출: `rejected(BUSY, "주행 중 재초기화 불가")` (정지 상태 전제 — 안전).
- blackbox 저장 실패(디스크 등): `failed(INTERNAL)` + assistance 승격(라우터 기존 규약).
- 대시보드 cmd_result 미수신(5초): 토스트 "응답 없음 — 사건 기록 확인" (링크 상태 별도 표시 있음).

## 5. 검증 게이트

| 게이트 | 기준 |
|---|---|
| 단위 pytest | SimWork·ScoutDiag 로직(수신율 판정·전통로 자동 설정·UNSUPPORTED 거부), 기존 25+ 전건 유지 |
| self_test 실기 | 정지 상태 5항목 all_ok=true · 라이다 브리지 내림 → lidar ✗ 검출 |
| relocalize 실기 | est 를 의도적으로 2 m 오프셋 후 명령 → 2초 내 quality 복귀·오차 ≤0.3 m |
| 정찰 실기 | `work:{type:"scout", speed_scale:0.5}` 3통로 부분 정찰 — 속도 절반 확인·state.work.progress 상승·완료 보고 |
| spray 거부 | mission_start(spray) → UNSUPPORTED cmd_result |
| 대시보드 | Playwright: 정찰 버튼→미션 생성, 유지보수 카드 3기능, cmd_result 토스트, 콘솔 오류 0 |
| 회귀 | 하네스 스팟(남측 4쌍 P0 n=1)·보안 21·30·기존 대시보드 스모크 |
