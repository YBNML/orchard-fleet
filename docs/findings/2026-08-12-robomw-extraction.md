# robomw 추출·scout 재배선 — 최종 회귀 게이트 결과

2026-08-12 · 관련 코드: `ros2_ws/src/robomw/`, `ros2_ws/src/orchard_sim/`,
`server/fleet_server/` · 스펙: `docs/superpowers/specs/2026-08-11-robot-middleware-command-contract-design.md`
· 계획: `docs/superpowers/plans/2026-08-11-robomw-extraction.md`

## 배경

T1~T8(코어·SDK·과수원 프로파일 추출, scout_mini 재배선, 관제 서버 프로토콜
단일화)이 각각 리뷰를 통과한 뒤, 이 태스크(T9)에서 스펙 ① §4 회귀 게이트
표 전 항목을 **한 세션에서 일괄 재실행**해 최종 수치를 남긴다. 개별 게이트는
이전에도(T1·T2·T4·T6·T7·T8) 통과했지만, 이관이 끝난 최종 상태에서 전부를
다시 돌린 적은 없었다 — 이 노트가 그 결과다.

## 게이트 결과

| 게이트 | 방법 | 기준 | 결과 |
|---|---|---|---|
| ROS 격리 | `pytest robomw/tests/test_no_ros_imports.py` | import 0건 | **PASS** (2/2, 아래 pytest 합산에 포함) |
| 프로토콜·라우터·SDK 단위 | `pytest ros2_ws/src/robomw/tests/ -v` | 전 항목 통과 | **PASS — 25/25** |
| 보안(TLS+토큰) | `scripts/21_verify_security.py` (격리 인스턴스, port 8444) | 전 항목 통과 | **PASS — 9/9** |
| 감사·역할 | `scripts/30_verify_audit_roles.py` (격리 인스턴스, port 8081) | 전 항목 통과 | **PASS — 10/10** |
| 진단 자동정지 | `scripts/49_verify_diag_stop.py` (메인 에이전트) | 전 항목 통과 | **PASS — 13/13** |
| 지도 공급 격리 | `scripts/50_verify_cloud_isolation.py` (독립 실행) | 전 항목 통과 | **PASS — 18/18** |
| 횡단 물리 | `scripts/46_climb_harness.py --mission-pairs --policies P0,P9 --n 2` | 32/32 | **PASS — 32/32** |
| 임무 회귀 | 3통로 미션(alleys 0-2), cmd_id 상관 | 무개입 완료 + cmd_result `completed` | **PASS — 8/8** (완주, 무개입) |
| 관제 호환 | 대시보드 Playwright 스모크(로그인·estop·명령 중계) | 콘솔 오류 0·denied 0 | **PASS — 10/10** |

**7개 게이트 전 항목 통과.** 실패·BLOCKED 항목 없음.

## 실행 상세

### pytest (전건)

```
$ python3 -m pytest ros2_ws/src/robomw/tests/ -v
25 passed in 0.03s
```

5개 파일: `test_no_ros_imports.py`(2) · `test_protocol_contract.py`(5) ·
`test_router.py`(9) · `test_mission_report.py`(6) · `test_sdk_types.py`(3).

### 보안·감사 — 격리 인스턴스 방법 (task-2-report.md 절차 재사용)

메인 에이전트(port 8080, 실주행 중)는 손대지 않고, 매번 스크래치 파라미터
파일로 별도 `control_agent` 인스턴스를 띄워 검증한 뒤 실제 실행 PID를
`ps aux` 로 특정해 개별 종료했다(패턴 매칭 kill 없음).

- `scripts/21_verify_security.py --port 8444 --token <sim/certs/control.token> --cert sim/certs/control.crt`
  → robot_id=`robomw_t9_sectest` · **9/9 통과**
- `scripts/30_verify_audit_roles.py --port 8081 --audit <scratch>/audit_t9.jsonl --token-observer OBS --token-admin ADM`
  → robot_id=`robomw_t9_audittest` · **10/10 통과**

두 인스턴스 종료 후 메인 에이전트 PID(90922, port 8080)는 시작부터 끝까지
무변화 확인.

### 진단 자동정지 (scripts/49) — 메인 에이전트 대상

`/map_localizer/diagnostics` 에 critical 진단(LOST_LONG)과 TRACTION_LOSS를
직접 주입해 `control_agent._on_loc_diag` 의 자동정지 분기를 실기로 밟았다.
에이전트 생존·자동 일시정지·개입 큐 발행·해소 이벤트·검증 후 자동 복귀까지
**13/13 통과**. 로봇은 idle·무임무 상태였으므로 안전하게 실행.

### 지도 공급 격리 (scripts/50) — 독립 실행 (ROS 기동 불필요)

`ControlAgent._on_cloud` 를 함수 단위로 불러 지도 공급 고장이 밀착 정지
판단을 막지 못함을 확인. **18/18 통과** (5개 절: 고장 격리·반복고장 로그
상한·정상 회귀·솎인 프레임·어댑터 결손 메시지).

### 횡단 물리 하네스 (scripts/46)

```
$ scripts/46_climb_harness.py --mission-pairs --policies P0,P9 --n 2
```

메인 에이전트를 내리고(하네스가 `/cmd_vel` 을 직접 잡음) 8개 통로쌍 ×
2개 정책(P0 기본직진·P9 폐루프조준) × 2회 = **32/32 통과**, 평균 소요
10.7~10.8초/회, 최대 정체 0.5초 이내(모든 쌍에서 일관).

하네스 종료 후 복구: `recover_robot.py`(통로 0 남단 텔레포트) →
`restart_loc.sh`(INIT_X=-14.0 INIT_Y=-31.5 INIT_YAW=1.5708) →
`restart_agent.sh` → `restart8000.sh`(관제 서버, 스테일 링크 해소) →
`42_probe_robot_state.py` 로 idle·paused False·estop clear 확인.

### 임무 회귀 (3통로, cmd_id 상관) — T7 게이트 재실행

절차는 task-7-report.md 를 그대로 재사용했다: `cmd_id=t9run1` 로
`mission_start{alleys:[0,1,2], work:{type:"scout"}}` 을 보내고, 링크를 붙인
채 30초 간격으로 상태를 폴링하며 `cmd_result` 완료 보고를 기다렸다. 총
소요 18분 47초(00:46:29 시작 → 01:05:36 완료), 통로 0→1→2 전 구간을
개입 없이 통과.

완료 보고(계약 그대로):

```json
{"kind":"cmd_result","cmd_id":"t9run1","cmd":"mission_start",
 "status":"completed","code":"OK",
 "data":{"alleys_done":[0,1,2],"distance_m":220.2,"duration_s":1128.4,
         "interventions":0,"coverage":1.0}}
```

8개 판정 항목 전부 통과: `status==completed` · `cmd==mission_start` ·
보고 키 5종이 계약(`MISSION_REPORT_KEYS`)과 정확히 일치 · `alleys_done ==
[0,1,2]` · `coverage==1.0` · `distance_m(220.2) > 150` · `duration_s(1128.4)
> 600` · `interventions == 0`(무개입).

로컬라이저 경보(`LOST_LONG`, 8초째 위치 보정 실패)가 두 번의 선회 구간
(통로 1 진입부·통로 2 진입부)에서 그대로 떴고 1분 안에 스스로 `resolved` 로
해소됐다 — T7 리포트가 이미 밝힌 대로 로봇이 서지 않은 사건은 개입으로
세지 않는다(판정: `control_agent._is_intervention`). 화면에는 그대로
보이고, 완료 보고의 숫자만 사실(정지 여부)을 반영한다. distance_m 220.2m
는 T7 1·2차 결과(226.3m·227.3m)와 같은 자릿수 — 재현성 확인.

게이트 종료 후: estop 없음·paused 없음, `42_probe_robot_state.py` 로
idle·통로 2 북단(-6.9, 30.8) 확인.

### 관제 호환 (대시보드 Playwright 스모크) — T8 게이트 재실행

`server/.venv/bin/python` 으로 T8 이 남긴 `t8_smoke.py` 를 그대로 재실행
(로그인 admin/123 → 텔레메트리 수신 → estop → 2단계 해제 카드 → self_test
신규 명령 중계 → denied/콘솔 오류 집계). **10/10 통과** — denied 0건,
콘솔 오류 0건, `self_test` 가 서버를 거쳐 `cmd_result{status:rejected,
code:UNSUPPORTED}` 로 왕복.

스모크가 남긴 estop 래치는 절차대로 2단계 해제(현장 확인 `~/local_reset`
ROS 토픽 + 관제 승인 `clear_estop_request`)로 정리했다. **여기서 재현
가능한 함정 하나를 발견했다**: 신규 rclpy 노드가 퍼블리셔를 만들자마자
바로 발행하면(디스커버리 이전) 구독자에게 도달하지 않는다 — 첫 시도에서
`clear_estop_request`(관제 승인)만 반영되고 `local_reset`(현장 확인)은
유실돼 `estop_stage=awaiting_local` 로 멈췄다. `pub.
get_subscription_count() > 0` 을 기다린 뒤 재발행하니 정상 반영됐다(단발성
스크립트 작성 시 주의할 점으로 기록 — robomw 계약이나 코드의 결함이
아니라 ROS2 디스커버리 레이턴시). 이후 `mission_resume` 으로 일시정지도
해제해 로봇을 완전한 idle 로 되돌렸다(estop clear · paused False).

## 결론

스펙 ① §4 회귀 게이트 표의 **7개 항목 전부 통과**했다 — ROS 격리,
프로토콜·라우터·SDK 단위, 보안(TLS+토큰), 감사·역할, 횡단 물리(32/32),
임무 회귀(무개입 완주 + 계약 스키마 그대로의 완료 보고), 관제 호환(콘솔
오류 0·denied 0). 추가로 이번 태스크 범위에서 명시적으로 포함하라고 한
진단 자동정지(scripts/49)·지도 공급 격리(scripts/50)도 전 항목 통과했다.

BLOCKED 로 보고할 실패 항목 없음. 이관(T1~T8)이 만든 코드가 최종 상태에서
스펙이 정의한 회귀 기준을 전부 충족한다.

이월 사항(과거 태스크 리포트에 이미 기록됐고 이번 게이트로 재확인만 한
것 — 이 태스크가 새로 발견한 결함은 아니다):
- `robomw` 는 numpy 에 의존한다(지도 격자 기능) — "ROS 를 모른다"로 좁혀
  읽어야 한다는 T7 의 지적 그대로.
- `docs/findings/2026-07-30-control-architecture.md` 의 경로 설명이
  `control/features/` 기준으로 낡아 있다 — 이번 태스크 범위 밖.
- Work/Diag SDK 는 v0.1 에서 인터페이스만 있고 미구현(스펙 ② 범위) —
  README §3 에 명시.
