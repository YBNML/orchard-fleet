# robomw — 로봇측 미들웨어 v0.1

이기종 로봇이 **같은 명령을 받아, 과정은 달라도 같은 결과**를 내게 하는
로봇측 미들웨어와 명령 계약. 설계 근거는
`docs/superpowers/specs/2026-08-11-robot-middleware-command-contract-design.md`
(이하 "스펙 ①")에 있다 — 이 문서는 그 설계가 실제로 어떻게 구현됐는지를
코드 기준으로 요약한다.

핵심 원칙 한 줄: **계약이 결과를 소유하고, 로봇은 과정을 소유한다.**
명령의 이름·권한·완료 보고 스키마는 `robomw`가 정하고, 그 결과를 어떻게
만들어내는지(모터를 어떻게 돌리는지, 위치를 어떻게 추정하는지)는 로봇마다
다른 SDK 구현이 정한다.

`robomw` 는 **ROS를 모른다** — 코어·SDK·프로파일 어디에도 `rclpy`/
`rcl_interfaces` import가 없다 (`tests/test_no_ros_imports.py` 가 상시
검사). ROS(또는 다른 프레임워크)를 아는 코드는 전부 로봇 패키지의
**어댑터**에 있다 (예: `orchard_sim/adapters/`).

## 1. 패키지 구조

```
robomw/
  core/        MiddlewareHost 부품 — 안전·라우팅·기능 레지스트리·감사
    base.py       Feature/Context/Blackboard/VelocityRequest
    registry.py    기능 적재·격리·describe()
    router.py      CommandRouter — 권한 재판정·cmd_id 멱등·cmd_result
    safety.py      SafetyArbiter — estop 2단계·데드맨·링크두절·중재
    audit.py        AuditLog — 명령·거부 영속 기록
  link/        전송·계약
    protocol.py    봉투·topic·명령 이름·권한표·cmd_result 스키마 (계약 원본)
    wsserver.py    WebSocket 서버 (전송 계층 — 교체 가능)
  sdk/         로봇이 구현하는 인터페이스
    interfaces.py  Drive / Localizer / Perception / Work / Diag
    types.py       Pose, DriveLimits, SelfTestItem, WorkStatus
  profiles/orchard/
    mission.py     보스트로피돈 임무 엔진 (과수원 사이트 프로파일)
  features/
    teleop.py, telemetry_state.py, telemetry_health.py, telemetry_map.py
```

`robomw` 는 코어·SDK·프로파일을 담을 뿐, 특정 로봇을 몰지 않는다. 실제로
로봇을 돌리는 조립은 로봇 패키지 쪽에 있다 — 이 워크스페이스에서는
`orchard_sim/orchard_sim/control_agent.py` (ROS 노드) 가 그 역할이다:
파라미터를 읽어 어댑터 4종(`RosDrive`, `RosSensors` = Localizer+Perception,
`RosCloudWorld`)을 만들고, `robomw.core` 부품(Blackboard·SafetyArbiter·
Registry·CommandRouter)을 조립하고, ROS 콜백에서 그 부품들을 호출한다.

## 2. 명령 계약 요약 (스펙 ① §2.4)

봉투: `{"v":1, "topic":"<site_id>/<robot_id>/<kind>", "ts":<로봇 시각 ns>,
"seq":<n>, "payload":{...}}` — `topic()` 함수 하나로 조립(`robomw.link.protocol`).

권한 3단계: `observer(0) < operator(1) < admin(2)`. 표에 없는 명령은
**admin** 이 문턱이다(fail-closed) — 등록 안 된 명령이 조용히 "아무나 쓸 수
있는 명령"이 되는 쪽이 더 위험하다는 판단.

| 명령 | 역할 | 파라미터 | 비고 |
|---|---|---|---|
| `estop` | operator | `{reason?}` | 거는 문턱은 낮게 — 위험을 본 사람이 즉시 세운다 |
| `clear_estop_request` | admin | `{}` | 관제 승인. **이것만으로는 안 풀린다** |
| `clear_estop_cancel` | admin | `{}` | 해제 절차 취소 |
| `local_reset` | — (링크 거부) | — | 현장 확인. `~/local_reset` 로컬 입력 전용, 링크로 오면 무조건 거부 |
| `set_mode` | admin | `{mode: idle\|mission\|teleop}` | |
| `set_service_mode` | admin | `{mode: ""\|maintenance\|commissioning}` | |
| `mission_start` | operator | `{alleys:[k], work:{type,params}?, mission_id?, cmd_id?}` | +cmd_result·완료 보고 |
| `mission_pause`/`resume`/`cancel` | operator | `{mission_id?, cmd_id?}` | cancel은 완료 보고 없음(중간 실적 ≠ 마감) |
| `teleop`(별도 topic) | operator | `{v,w}` | 데드맨 400ms, 큐 우회, priority 10 |
| `ping` | observer | `{}` | 관측자도 링크 확인은 해야 한다 |
| `self_test` | operator | `{items?:[...]}` | 구현됨(스펙 ②) — lidar·imu·localizer·link·drive 5항목, 움직임 없음 |
| `relocalize` | admin | `{x,y,yaw}` 또는 `{alley:k, end:...}` | 구현됨(스펙 ②) — admin 인 이유: 위치 리셋 오적용은 임무 궤적을 깬다 |
| `blackbox_dump` | operator | `{window_s?:600}` | 구현됨(스펙 ②) — 궤적·이벤트 npz 덤프(900초 상한) |
| `work_stop` | operator | `{}` | 구현됨(스펙 ②) — 작업만 중단, 주행(임무)은 계속 |

안전 상수(불변): `HEARTBEAT_MS=1000` · `TELEOP_DEADMAN_MS=400` ·
`LINK_LOSS_STOP_MS=1500`. estop 해제 후 자동 재개 없음 — 해제와 재개는
별개 결정이다.

### cmd_result — 명령 결과 상관 (신규, 하위 호환)

`payload.cmd_id`(선택, 관제가 생성)를 붙이면 이벤트 채널로 결과가 온다:

```json
{"kind":"cmd_result", "cmd_id":"c1723...", "cmd":"mission_start",
 "status":"accepted|rejected|in_progress|completed|failed",
 "code":"OK|DENIED|BAD_PARAM|BUSY|ESTOPPED|UNSUPPORTED|TIMEOUT|INTERNAL",
 "data":{...}}
```

- `cmd_id` 없으면 결과 이벤트 생략(구버전 클라이언트와 완전 호환).
- 멱등: 같은 `cmd_id` 재수신 시 직전 결과를 재발행하고 재실행하지 않는다
  (최근 32건 캐시, `CommandRouter`).
- 임무 완료 보고의 `data` 는 계약이 소유한 스키마다 — 어느 로봇이든 이
  5개 키로 보고해야 "같은 결과"다:
  `MISSION_REPORT_KEYS = ("alleys_done", "distance_m", "duration_s", "interventions", "coverage")`

### 현장 기하 `site_geom` — 통로 좌표의 단일 출처 (개정 v2, additive)

호스트가 `bb.extra["site_geom"]` 에 얹고, `hello` 의 최상위 기하와
`site.geometry` 가 **같은 객체**를 가리킨다. 이 표를 읽는 곳은 둘이고 해석
코드는 한 벌이다(`sdk/sitegeom.py`): `relocalize {alley,end}` 의 좌표 환산
(`features/maintenance.py::_grid_pose`)과 임무 웨이포인트 생성
(`profiles/orchard/mission.py::build_waypoints`).

| 키 | 필수 | 뜻 |
|---|---|---|
| `rows` / `alleys` | ✔ | 수목열 수 / 통로 수(= rows-1) |
| `row_spacing` | ✔ | 열 간격 m (균일 격자 폴백에 쓰인다) |
| `x0` | ✔ | 0번 열의 x (균일 격자 폴백) |
| `col_len` | ✔ | 대표 열 길이 m (균일 격자 폴백: 블록이 ±col_len/2) |
| `headland` | ✔ | 선회 구간 폭 m |
| **`alley_centers_x`** | ✱ | **신규(선택)** 통로별 중심 x 배열, 길이 = `alleys`. 있으면 `x0`+균일 간격 대신 이걸 쓴다 |
| **`row_span_y`** | ✱ | **신규(선택)** `[y_남단, y_북단]`(전 통로 공통) 또는 `[[y_남단, y_북단], …]`(통로별, 길이 = `alleys`). **원소 순서가 곧 남단/북단의 정의다** — 0번이 south, 1번이 north. 값은 통로의 **안쪽 끝**(이웃 두 열 중 짧은 쪽) |
| **`alley_cross_y`** | ✱ | **신규(선택, v2.1)** `row_span_y` 와 같은 두 형태. 통로를 **드나드는 횡단선** y. 없으면 `±(col_len/2 + headland·0.6667)` 폴백 |

- **하위 호환**: 세 신규 키가 없으면 종전 균일 격자 계산 그대로다(계단식
  월드 경로 불변 — 웨이포인트가 한 점도 안 바뀐다는 단위시험 있음). 있으면 우선한다.
- **`alley_cross_y` 가 `row_span_y` 로 대체되지 않는 이유**: `row_span_y` 는
  통로의 **안쪽 끝**이라 정지·재정위에는 맞지만, 통로 **사이를 가로지르는**
  선은 그 사이 열을 넘어가야 한다. 그 열의 끝 나무는 안쪽 끝보다 최대 2.7 m
  바깥에 있고(실사 농장 실측), 그 정보는 `row_span_y` 안에 없다 — 양쪽 통로
  모두에서 '바깥'이라 max/min 어느 쪽으로도 안 나온다. 안쪽 기준으로 횡단선을
  잡으면 열 끝 나무를 정면으로 들이받는다.
- **왜 필요한가**: 균일 직사각 격자 전제(`x0 + (k+0.5)·row_spacing`, `±col_len/2`)는
  실사 정사영상 농장(스펙 ④)에서 깨진다 — 열 간격 4.75~5.25 m 불균일, 열별
  길이 128~141 m, y 비대칭 블록. 그 월드에서 균일 격자로 풀면 통로 25 재정위가
  x 8.5 m·y 170 m·헤딩 180° 어긋난다.
- **남단/북단의 부호는 현장마다 다르다**: 계단식 월드는 남단이 y 최솟값,
  실사 농장은 `farm.json` `axes_note` 상 world +y 가 지리적 남이라 남단이 y
  최댓값이다. 그래서 크기 비교로 추론하지 않고 배열 위치로 못박는다.
- 정지선 `END_STANDOFF_M = 1.5 m` 는 그 단에서 **블록 바깥**으로 물러난 자리,
  요는 언제나 **블록 안쪽**을 본다.
- 호스트 구현: `orchard_sim/control_agent.py` 가 런치 파라미터
  (`alley_centers_x`·`alley_south_y`·`alley_north_y`·`alley_cross_south_y`·
  `alley_cross_north_y`)를 조립한다. 그 값은 `control.launch.py` 가
  `world:=real` 일 때 `maps/orchard_real/farm.json` 에서 계산한다(통로 중심 =
  인접 `row_origins` x 중점, y 는 `axes_note` 규약, 횡단선 = 이웃 두 열
  캐노피의 **바깥** 끝 ± `headland_m`).

### hello v2 (additive)

기존 키(`robot_id`, `protocol`, 기하, `limits`, `deadman_ms`, `link_loss_ms`,
`features`)는 그대로 두고 세 키를 얹는다:

```json
{"site":{"type":"orchard","geometry":{...}},
 "capabilities":{"drive":{"v_max":0.7,"w_max":1.0}},
 "middleware":{"name":"robomw","version":"0.1"}}
```

`capabilities` 는 로봇이 실제로 구현한 SDK만 싣는다 — scout 프로파일은
스펙 ② 부터 `drive`·`work`(`{"types":["scout"]}`)·`diag`(`{"items":[...]}`)
셋을 싣는다(아래 §4). 능력군 이름공간(`CAPABILITY_FAMILIES`)에는
`legged`·`manipulation` 도 예약돼 있다(정의만, 구현 없음).

## 3. SDK 5종 (`robomw.sdk.interfaces`)

로봇 패키지가 구현해야 하는 전부다. 코어·프로파일은 이 인터페이스만
호출한다 — 기능 코드가 SDK의 `Drive`를 직접 부르는 것은 금지(속도는 항상
`VelocityRequest` → `SafetyArbiter.arbitrate` → `Drive.set_velocity` 한
창구로만 나간다).

```python
class Drive(ABC):
    def set_velocity(self, v: float, w: float) -> None: ...  # 중재 후 코어만 호출
    def stop(self) -> None: ...
    def limits(self) -> DriveLimits: ...                      # v_max, w_max

class Localizer(ABC):
    def pose(self) -> Pose | None: ...                        # None = 미초기화/신호손실
    def reinit(self, pose: Pose) -> bool: ...                  # True = 재초기화 + 측위 회복 확인
    def diagnostics(self) -> dict: ...                         # 'bias_x' 등, 실수 또는 상태 문자열

class Perception(ABC):
    def clearance(self) -> float: ...                          # 전방 개활거리 m (inf 가능)
    def near_frac(self) -> float: ...                          # 0~1, 근처 점 비율

class Work(ABC):                                               # 구현됨(스펙 ②) — scout: SimWork
    def start(self, type_: str, params: dict) -> None: ...
    def stop(self) -> None: ...
    def status(self) -> WorkStatus: ...

class Diag(ABC):
    def self_test(self, items: list[str] | None = None) -> list[SelfTestItem]: ...  # None=전체 항목
    def blackbox_dump(self, window_s: float) -> dict: ...        # 구현됨(스펙 ②) — scout: ScoutDiag
```

자료형(`robomw.sdk.types`): `Pose(x, y, yaw, quality=1.0)` ·
`DriveLimits(v_max, w_max)` · `SelfTestItem(name, ok, detail)` ·
`WorkStatus(active=False, type="", progress=0.0, detail="")`.

**scout_mini(과수원 프로파일)의 현재 구현 상태** — 참고용, 새 로봇이
"5종 다 채워야 하나"를 가늠하는 기준:

| SDK | 구현 | 위치 |
|---|---|---|
| `Drive` | 구현됨 | `orchard_sim/adapters/ros_drive.py::RosDrive` |
| `Localizer` | 구현됨 | `orchard_sim/adapters/ros_sensors.py::RosSensors` (Localizer+Perception 겸함) |
| `Perception` | 구현됨 | 위와 동일 클래스 |
| `Work` | **구현됨** | `orchard_sim/adapters/sim_work.py::SimWork` — `mission_start.work:{type:"scout"}` 로 부스트로피돈 정찰(전 통로 자동·부분 정찰·speed_scale), hello.capabilities.work.types=["scout"] |
| `Diag` | **구현됨** | `orchard_sim/adapters/scout_diag.py::ScoutDiag` — `self_test`(lidar·imu·localizer·link·drive 5항목) · `relocalize`(격자/좌표 재정위, admin) · `blackbox_dump`(궤적·이벤트 npz) 전부 스펙 ② 에서 채워짐 |

즉 SDK 5종 인터페이스는 계약으로 확정돼 있고, scout 프로파일은 **5종
전부를 실제 로봇으로 구현**한 상태다(스펙 ①로 Drive/Localizer/Perception,
스펙 ②로 Work/Diag).

## 4. 새 로봇 온보딩 = SDK 5종 구현 + hello 선언

새 로봇(다른 센서 구성·다른 현장)을 붙이는 절차는 이 넷뿐이다. **`robomw`
코어 파일은 한 줄도 고치지 않는다.**

1. **어댑터 패키지를 만든다** (로봇 저장소 쪽, `robomw` 밖). `robomw.sdk.
   interfaces` 의 `Drive`/`Localizer`/`Perception`(필수) 을 상속해 구현한다.
   `Work`/`Diag` 는 그 로봇이 실제로 지원하는 만큼만 — 구현 안 하면 그
   명령은 자동으로 UNSUPPORTED 거부가 된다(라우터가 `_supported()` 로
   판정, 코어를 고칠 필요 없음).
2. **로봇 프로세스에서 코어를 조립한다.** `robomw.core.base.Blackboard` +
   `robomw.core.safety.SafetyArbiter` + `robomw.core.registry.Registry` +
   `robomw.core.router.CommandRouter` + `robomw.link.wsserver.ControlServer`
   를 만들고, 위 어댑터 인스턴스를 자기 프레임워크(ROS 콜백이든 다른
   무엇이든)의 진입점에서 호출한다. `orchard_sim/control_agent.py` 가
   ROS 기준의 참조 구현이다 — 다른 프레임워크라면 이 파일을 참고해 자기
   프레임워크의 진입점에 맞게 새로 짜면 된다(ROS를 안 쓰는 로봇이면
   `robomw`에는 그 사실이 전혀 드러나지 않는다).
3. **필요하면 사이트 프로파일을 고른다.** 과수원이면
   `robomw.profiles.orchard.mission`(보스트로피돈)을 기능으로 얹는다.
   새 현장(제조현장 등)이면 `robomw/profiles/<현장>/` 에 같은 모양으로
   새로 만든다 — mission 엔진이 SDK만 호출하므로 로봇이 바뀌어도
   프로파일은 그대로 재사용된다.
4. **hello 를 선언한다.** 접속 직후 1회 보내는 `hello` 봉투에 자기
   `capabilities`(구현한 SDK만), `site.type`, `middleware.name/version`
   을 싣는다 (`_on_ws_open` 참조 — `orchard_sim/control_agent.py:562`).
   관제 대시보드는 `hello.features`(기능 목록)로 패널을 켜고 끄고,
   신형 관제는 `hello.capabilities` 로 무엇을 물어봐도 되는지 안다.
   이 선언이 "이 로봇이 미들웨어를 달았다"는 유일한 증거다 — 나머지는
   전부 `robomw` 가 계약대로 강제한다(권한표·estop 2단계·데드맨·
   cmd_result 형식은 로봇이 아무것도 안 해도 이미 지켜진다).

기능(플러그인) 하나를 더 붙이는 절차는 더 가볍다 —
`robomw.core.base.Feature` 를 상속하고 `features` 파라미터 목록에
모듈명만 넣으면 된다(코어도 대시보드도 안 고친다, `robomw/robomw/core/
base.py` 상단 docstring 참조).

### 4.1 같은 로봇을 여러 대 — 네임스페이스·포트·서버 등록

**`robomw` 는 다중 로봇을 모른다.** 코어에는 로봇이 몇 대인지가 드러나지
않는다 — 인스턴스마다 자기 `robot_id` 로 `ControlServer` 를 하나씩 띄우면
그게 곧 다중 로봇이다. 실제로 2대 동시 운용을 실증하며 고친 코어 파일은
0개다(스펙 ③ / `docs/findings/2026-08-13-multirobot-bt.md`).

붙이는 데 필요한 것은 셋뿐이다.

1. **네임스페이스** — 로봇별 토픽·TF 를 `scout0N/` 아래로 민다. 어댑터·
   로컬라이저·브리지를 `__ns:=/scout0N` 로 기동하면 되고, **코드 안에서는
   상대 토픽만 쓴다**(절대경로를 박으면 그 순간 다중화가 깨진다).
   TF 프레임은 `scout0N/odom`·`scout0N/base_link` 로 나누고 `map` 만
   공유한다. `tf`/`tf_static` 토픽 자체는 전역으로 두고 프레임 이름으로
   구분한다(ROS 관례).

   ```bash
   ros2 launch orchard_sim control.launch.py \
     robot_id:=scout02 ns:=scout02 port:=8081 clock:=false   # /clock 브리지는 월드당 하나
   ```

2. **포트** — 관제 링크(WebSocket)는 로봇마다 따로 연다. 이 저장소 관례는
   scout01 → 8080, scout02 → 8081. `control_agent` 의 `port` 파라미터다.

3. **서버 등록** — 관제 DB 에 로봇 행을 하나 더 넣는다. **기존
   `register_robot` 경로를 그대로 쓰며 서버를 멈추지 않아도 된다** —
   접속 주소는 `config_json.ws_url` 로 준다.

   ```json
   {"id": "scout02", "farm_id": 1, "name": "스카우트 2호", "kind": "orchard",
    "conn_kind": "legacy_ws",
    "config_json": {"ws_url": "ws://127.0.0.1:8081/ws", "token": ""}}
   ```

검증 도구는 전부 `--robot`(기본 `scout01`)을 받는다 — 옛 무인자 호출은 1호기
동작 그대로다: `42_probe_robot_state.py --robot scout02 --port 8081`,
`39_verify_localization_live.py --robot scout02`, `46_climb_harness.py --robot`.

**한 대를 붙일 때마다 같이 확인할 것**

- **점군 대역은 시스템 전역 천장을 나눠 쓴다** — 벽시계 약 2.7 프레임/초가
  로봇 수로 갈린다. 2대에서 이미 한 로봇이 `self_test` 의 8 Hz 문턱 아래로
  내려간다(실측 3.9~8.1 Hz). 3대째부터는 라이다 표본 감축이 사실상 전제다.
- **통로 잠금(AlleyLock)은 임무 단위 공간 분리**라, 두 로봇의 통로 집합이
  겹치거나 사이 헤드랜드 패드를 공유하면 뒤 임무가 `QUEUED_LOCK` 으로 선다.
- **선회 평지 패드 파리티** — 통로 목록은 오름차순이면 첫 통로가 짝수,
  내림차순이면 홀수여야 물리적으로 완주 가능하다(`presets.parity_safe`).

## 5. 회귀 게이트 (재배선 합격 기준)

전 항목 통과 수치는 `docs/findings/2026-08-12-robomw-extraction.md` 참조.
게이트 정의는 스펙 ① §4:

| 게이트 | 방법 | 기준 |
|---|---|---|
| ROS 격리 | `tests/test_no_ros_imports.py` | `rclpy`/`rcl_interfaces` import 0건 |
| 프로토콜 단위 | `pytest robomw/tests/` | 전 항목 통과 |
| 보안·권한 | `scripts/21_verify_security.py` · `scripts/30_verify_audit_roles.py` | 전 항목 통과 |
| 진단 자동정지 | `scripts/49_verify_diag_stop.py` | 전 항목 통과 |
| 지도 공급 격리 | `scripts/50_verify_cloud_isolation.py` | 전 항목 통과 |
| 횡단 물리 | `scripts/46_climb_harness.py --mission-pairs --policies P0,P9 --n 2` | 32/32 |
| 임무 회귀 | 3통로 미션(alleys 0-2) | 무개입 완료 + cmd_result `completed` 수신 |
| 관제 호환 | 대시보드 Playwright 스모크(로그인·estop·명령 중계) | 콘솔 오류 0, denied 0 |
