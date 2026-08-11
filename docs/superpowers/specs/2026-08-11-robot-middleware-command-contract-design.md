# 로봇 미들웨어 + 명령 계약 v1 — 설계 (스펙 ①)

| 항목 | 내용 |
|---|---|
| 날짜 | 2026-08-11 |
| 목표 | 이기종 로봇이 **같은 명령을 받아, 과정은 달라도 같은 결과**를 내게 하는 로봇측 미들웨어와 명령 계약 |
| 범위 | 명령 계약 v1 정의 · `robomw` 패키지 추출 · scout_mini 재배선 · 관제 서버 프로토콜 단일화 |
| 범위 밖 | 신규 명령의 동작 구현(스펙 ②) · 이기종 2호기 실증과 서버측 임무 큐(스펙 ③) |
| 용어 | 과수원 로봇 = UGV(농업 필드 로봇). AGV 표준(VDA5050)은 채택하지 않음 — 실내 유도 기반 의미론이 야외 비정형 지형·작업 도메인과 맞지 않음 |

## 0. 배경과 원칙

현 스택은 단일 로봇(scout_mini) 기준으로 control_agent(코어) + link(프로토콜)
+ SafetyArbiter + feature registry가 한 패키지(orchard_sim)에 있다. 이 구조는
이미 미들웨어의 원형이다 — 문제는 (1) ROS·과수원·scout 전제가 코어에 섞여
있고, (2) 프로토콜 정의가 로봇과 서버에 관례로만 공유되며, (3) 명령의 "결과"
가 표준화되어 있지 않다는 것.

설계 원칙:

1. **계약이 결과를 소유한다** — 명령의 전제조건·진행 이벤트·결과 코드·완료
   보고 스키마를 계약이 정의한다. 로봇은 과정을 소유한다.
2. **층은 넷** — 공통 코어(형태 무관) / 사이트 프로파일(환경별) /
   능력군(로봇이 선언) / SDK(로봇별 구현). 미래의 제조현장·4족·휴머노이드는
   프로파일과 SDK 구현 추가로 들어오고, 코어는 불변.
3. **하위 호환은 요구사항** — 기존 관제 서버·대시보드가 무수정으로 동작해야
   한다. 봉투·명령 이름·권한표·안전 상수 불변, 확장은 additive.
4. **안전은 코어 소관** — estop 2단계·데드맨·링크두절·중재(arbitrate)는
   미들웨어 코어에 있고 SDK·프로파일이 우회할 수 없다 (현 규약 유지).

## 1. 패키지 구조

```
ros2_ws/src/robomw/                  # ament_python — colcon으로 함께 빌드
  robomw/
    core/
      host.py          # MiddlewareHost — 조립·수명주기 (링크서버+라우터+안전+펌프)
      router.py        # 명령 라우터: 입구/소비 이중 권한 판정, cmd_id 상관, 거부 코드
      safety.py        # SafetyArbiter (orchard_sim/control/safety.py 이관, 의미 불변)
      registry.py      # feature/명령 등록 (orchard_sim/control/registry 이관)
      events.py        # 이벤트 링버퍼 + 브로드캐스트 + 감사 훅 (audit.py 이관)
      telemetry.py     # state/health/map 펌프 — 스키마·주기(5Hz/1Hz/3s) 계약 소유
    link/
      protocol.py      # 봉투·topic·명령 이름·권한표·안전 상수·cmd_result 스키마
      wsserver.py      # WebSocket 서버 (이관, 전송 교체 가능 구조 유지)
    sdk/
      interfaces.py    # Drive/Localizer/Perception/Work/Diag (추상 클래스)
      types.py         # Pose, VelocityRequest, SelfTestItem, WorkStatus 등 자료형
    profiles/
      orchard/
        mission.py     # 보스트로피돈 임무 엔진 (drive_mission.py 이관)
        geometry.py    # 통로 격자 계산 (cross_y, build_waypoints 등)
    features/
      teleop.py        # drive_teleop 이관 (형태 무관 — Drive만 사용)
  package.xml / setup.py   # rclpy 의존 없음 (코어·sdk·profiles 전부)

ros2_ws/src/orchard_sim/orchard_sim/
  control_agent.py     # 얇은 ROS 노드: 어댑터 조립 → MiddlewareHost 실행
  adapters/
    ros_drive.py       # Drive → /cmd_vel 발행
    ros_localizer.py   # Localizer → map_localizer TF·진단 구독
    ros_perception.py  # Perception → 점군 원뿔 (IMU 수평화 포함)
    ros_link_extras.py # ~/local_reset ROS 토픽 등 ROS 전용 입력

server/fleet_server/
  adapters/legacy_ws.py  # 자체 봉투 조립 제거 → robomw.link.protocol import
```

- **robomw 코어에 ROS import 금지** — CI급 검사: `grep -r "rclpy\|rcl_interfaces" robomw/` 0건.
- 서버는 robomw를 경로 의존(pip install -e ros2_ws/src/robomw)으로 가져간다.
- 기존 orchard_sim의 link/·control/ 모듈은 이관 후 **호환 shim**(deprecation
  주석 + re-export)을 한 릴리스 동안 유지 — 스크립트(21·30·39 등)가 깨지지
  않게 한다.

## 2. 명령 계약 v1

### 2.1 불변 (기존 유지)

- 봉투: `{"v":1, "topic":"<site>/<robot>/<kind>", "ts":<ns>, "seq":<n>, "payload":{...}}`
- topic 접두는 **site_id** (기본 `"orchard"` — 하위 호환. 제조현장은 다른 site_id)
- 권한: observer(0) < operator(1) < admin(2), 미등록 명령 admin, 미지 역할
  observer 강등, 입구+소비 이중 판정, 거부는 denied 이벤트+감사
- 안전 상수: HEARTBEAT_MS=1000 · TELEOP_DEADMAN_MS=400 · LINK_LOSS_STOP_MS=1500
- estop 2단계: clear_estop_request(admin, 원격 승인) + local_reset(현장 확인,
  링크 수신 시 무조건 거부) · 창 600s · 해제 후 자동 재개 없음
- teleop은 별도 topic, 큐 우회, 우선순위 10

### 2.2 신규: 명령 결과 상관 (cmd_result)

모든 cmd payload에 선택 필드 `cmd_id`(문자열, 관제가 생성). 로봇은 이벤트
채널로 응답한다:

```json
{"kind":"cmd_result", "cmd_id":"c1723...", "cmd":"mission_start",
 "status":"accepted|rejected|in_progress|completed|failed",
 "code":"OK|DENIED|BAD_PARAM|BUSY|ESTOPPED|UNSUPPORTED|TIMEOUT|INTERNAL",
 "data":{...}}
```

- `accepted/rejected`는 수신 즉시(라우터가 발행), `in_progress/completed/failed`
  는 장기 명령(임무·self_test)에서 해당 기능이 발행.
- `cmd_id` 없는 명령은 종전대로 동작(결과 이벤트 생략) — 하위 호환.
- **완료 보고 스키마는 계약 소유**. 임무 완료의 data:
  `{"alleys_done":[...], "distance_m":n, "duration_s":n, "interventions":n,
    "coverage":0.0~1.0}` — 어느 로봇이든 이 스키마로 보고해야 "같은 결과"다.
- 멱등성: 같은 `cmd_id` 재수신 시 라우터가 직전 결과를 재발행하고 재실행하지
  않는다(최근 32건 캐시).

### 2.3 hello v2 (additive)

기존 키(robot_id, protocol, 기하, limits, deadman_ms, link_loss_ms, features)
는 유지하고 다음을 추가:

```json
{"site":{"type":"orchard","geometry":{...기존 과수원 기하...}},
 "capabilities":{
   "drive":{"v_max":0.7,"w_max":1.0},
   "diag":{"items":["imu","lidar","drive","localizer"]},
   "work":{"types":[]}},
 "middleware":{"name":"robomw","version":"0.1"}}
```

- 능력군 이름공간 예약(정의만): `legged`, `manipulation` — 문서·스키마에 자리만.
- 대시보드는 features 게이팅을 그대로 쓰고, capabilities는 스펙 ②에서 활용.

### 2.4 v1 명령 전체 목록

| 명령 | 역할 | 스펙 ① 동작 | 파라미터(계약) |
|---|---|---|---|
| estop | operator | 기존 그대로 | {reason?} |
| clear_estop_request / cancel | admin | 기존 그대로 | {} |
| local_reset | (링크 거부) | 기존 그대로 | — |
| set_service_mode | admin | 기존 그대로 | {mode: "" / maintenance / commissioning} |
| set_mode | admin | 기존 그대로 | {mode: idle / teleop} |
| ping | observer | 기존 그대로 | {} |
| mission_start | operator | 기존 + cmd_result·완료 보고 | {alleys:[k], **work:{type,params}?**, mission_id?, cmd_id?} |
| mission_pause / resume / cancel | operator | 기존 + cmd_result | {mission_id?, cmd_id?} |
| teleop (topic) | operator | 기존 그대로 | {v,w} |
| **self_test** | operator | 라우팅+UNSUPPORTED 거부만 (동작은 ②) | {items?:[...]} |
| **relocalize** | admin | 라우팅+UNSUPPORTED 거부만 (②) | {x,y,yaw} 또는 {alley:k, end:"north"/"south"} |
| **blackbox_dump** | operator | 라우팅+UNSUPPORTED 거부만 (②) | {window_s?:600} |
| **work_stop** | operator | 라우팅+UNSUPPORTED 거부만 (②) | {} |

- `work:{type:"scout"|"spray"|"mow"|"transport", params:{speed_scale?, ...}}` —
  작업 유형의 **이름·파라미터 스키마는 ①에서 계약으로 확정**, 실행(속도
  프로파일+상태 표시 시뮬)은 ②.
- UNSUPPORTED 거부도 cmd_result로 나간다 — 미들웨어를 단 로봇은 미구현
  명령에도 "정의된 방식으로" 반응한다.

## 3. SDK 인터페이스 (robomw.sdk)

```python
class Drive(ABC):
    def set_velocity(self, v: float, w: float) -> None: ...   # 중재 후 코어만 호출
    def stop(self) -> None: ...
    def limits(self) -> DriveLimits: ...                       # v_max, w_max

class Localizer(ABC):
    def pose(self) -> Pose | None: ...                         # x, y, yaw, quality(0~1)
    def reinit(self, pose: Pose) -> bool: ...
    def diagnostics(self) -> dict: ...                         # 슬립/상실 등 (있으면)

class Perception(ABC):
    def clearance(self) -> float: ...                          # 전방 여유거리 m (inf 가능)
    def near_frac(self) -> float: ...                          # 0.8 m 내 점 비율

class Work(ABC):                                               # ①은 인터페이스만
    def start(self, type_: str, params: dict) -> bool: ...
    def stop(self) -> None: ...
    def status(self) -> WorkStatus: ...

class Diag(ABC):                                               # ①은 인터페이스만
    def self_test(self, items: list[str]) -> list[SelfTestItem]: ...
    def blackbox_dump(self, window_s: int) -> str: ...         # 산출 경로/URI
```

- 코어·프로파일은 **이 인터페이스만** 호출한다. scout의 어댑터가 ROS를 안다.
- Work/Diag 미제공 로봇은 해당 명령에 UNSUPPORTED — hello.capabilities에서도 빠짐.
- 속도 출력 경로는 지금과 동일하게 단일 창구: 기능은 VelocityRequest 요청 →
  SafetyArbiter.arbitrate → Drive.set_velocity. SDK의 Drive를 직접 부르는
  기능 코드는 금지(리뷰 체크리스트 항목).

## 4. scout_mini 재배선

1. 이관: link/ → robomw.link, control/safety.py·registry·audit → robomw.core,
   drive_mission.py → robomw.profiles.orchard.mission, drive_teleop.py →
   robomw.features.teleop, telemetry_* → robomw.core.telemetry.
2. orchard_sim에 어댑터 4종 신설(§1), control_agent.py는 파라미터 파싱 +
   어댑터 조립 + Host 실행만 (~150줄 목표).
3. 서버 legacy_ws.py는 봉투 조립·topic 문자열을 robomw.link.protocol 함수로
   교체 (전송 로직은 그대로).
4. 호환 shim: `orchard_sim.link.protocol` 등 옛 경로는 robomw로 re-export.

### 회귀 게이트 (재배선 합격 기준 — 전부 통과해야 완료)

| 게이트 | 방법 | 기준 |
|---|---|---|
| 보안·권한 | scripts/21·30 (robomw 대상으로 이관) | 전 항목 통과 |
| 프로토콜 단위 | 신규 pytest (봉투 파싱·권한표·cmd_result·멱등 캐시) | 전 항목 통과 |
| 횡단 물리 | scripts/46 --mission-pairs --n 2 | 32/32 |
| 임무 회귀 | 3통로 미션 (alleys 0-2) | 무개입 완료 + cmd_result completed 수신 |
| 관제 호환 | 기존 대시보드 Playwright 스모크 (로그인·estop·임무 시작·해제 절차) | 콘솔 오류 0, denied 0 |
| ROS 격리 | grep rclpy in robomw/ | 0건 |

## 5. 오류 처리

- 라우터 수준 거부(BAD_PARAM/DENIED/ESTOPPED/BUSY)는 기능 코드 실행 전에
  cmd_result rejected로 통일 — 기능은 정상 경로만 구현.
- SDK 어댑터 예외는 코어가 잡아 cmd_result failed(INTERNAL) + assistance
  이벤트로 승격 — 로봇 프로세스는 죽지 않는다.
- Localizer.pose()가 None(미초기화)일 때 임무 시작은 rejected(BUSY,
  "측위 미준비") — 현재의 암묵 동작을 계약 명문화.

## 6. 스펙 ②·③ 미리보기 (이 스펙의 산출물이 만드는 자리)

- ②: Work 시뮬 구현(작업 유형별 속도 프로파일+상태 텔레메트리), Diag 구현
  (self_test = 센서 Hz·TF·측위 품질 점검, relocalize = 이번 세션의 운영자
  복구 절차 정식화, blackbox_dump = 최근 이벤트+궤적 npz), 대시보드 명령 UI.
- ③: 센서 구성이 다른 가상 2호기(라이다 없음·GPS식 Localizer 시뮬)를 SDK
  구현만으로 온보딩 → 같은 mission_start가 같은 완료 보고 스키마로 끝나는
  것을 실증. 서버측 임무 큐(QUEUED 임무의 자동 발진).
