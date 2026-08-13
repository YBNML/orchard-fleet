# 신규 명령 동작 + 대시보드 명령 UI — 최종 게이트 결과

2026-08-13 · 관련 코드: `ros2_ws/src/robomw/`, `ros2_ws/src/orchard_sim/`,
`server/fleet_server/`, `server/web/index.html` · 스펙:
`docs/superpowers/specs/2026-08-12-command-behaviors-dashboard-design.md`
(이하 "스펙 ②") · 계획: `docs/superpowers/plans/2026-08-12-command-behaviors.md`

## 배경

T1~T6(Work 정찰·Diag 3종·서버 배선·대시보드)이 각각 리뷰를 통과한 뒤, 이
태스크(T7)에서 스펙 ② §5 게이트 표 전 항목을 한 세션에서 일괄 재실행해
최종 수치를 남긴다. 게이트 실행 도중 별도 디버그 태스크가 관제 링크
플랩 결함(3라운드)을 찾아 고쳤고, 그 결함이 정찰 게이트 자체를 세 번
막았다 — §3 에 그 경위를 전부 남긴다.

## 게이트 결과

| 게이트 | 방법 | 기준 | 결과 |
|---|---|---|---|
| 단위 pytest | `pytest ros2_ws/src/robomw/tests/` | 전 항목 통과, 기존 25+ 유지 | **PASS — 50/50** |
| self_test 실기 | 정지 상태 5항목 · 라이다 브리지 다운/복구 | all_ok 정상/검출/복구 | **PASS** |
| relocalize 실기 | est 2 m 오프셋 주입 → 명령 | completed·오차 ≤0.3 m | **PASS — 0.187 m** |
| 정찰 실기 | `work:{type:scout,speed_scale:0.5}` + alleys[0,1,2] | 반속·progress 상승·완료 보고 | **PASS(4차 시도)** — 아래 §3 |
| spray 거부 | `mission_start(spray)` | UNSUPPORTED | **PASS** |
| blackbox_dump 실기 | 정찰 직후 1회 | npz 에 방금 궤적 | **PASS** |
| 대시보드 Playwright | 로그인·정비카드·정찰버튼·토스트·콘솔오류 | 콘솔오류 0 | **PASS — 0/0** |
| 회귀: 하네스 남측 4쌍 | `46_climb_harness.py` P0 n=1 ×4지점 | 4/4 통과 | **PASS — 4/4** |
| 회귀: 보안 | `scripts/21_verify_security.py`(격리 8444) | 전 항목 통과 | **PASS — 9/9** |
| 회귀: 감사·역할 | `scripts/30_verify_audit_roles.py`(격리 8081) | 전 항목 통과 | **PASS — 10/10** |
| 회귀: 진단 자동정지 | `scripts/49_verify_diag_stop.py`(메인 에이전트) | 전 항목 통과 | **PASS — 13/13** |
| 회귀: 지도 공급 격리 | `scripts/50_verify_cloud_isolation.py`(독립 실행) | 전 항목 통과 | **PASS — 18/18** |

**전 항목 통과.** 실패·BLOCKED 항목 없음. (참고: 링크 플랩 수정 태스크가
남긴 `robomw/tests/test_wsserver_link.py`(5) · `orchard_sim/test/test_ping_flood.py`(25)
도 이번 세션에서 재확인 — robomw 50/50 에 포함, orchard_sim 25/25 별도 통과.)

## 1. 단위 pytest

```
$ python3 -m pytest ros2_ws/src/robomw/tests/ -q
50 passed in 23.92s
$ python3 -m pytest ros2_ws/src/orchard_sim/test/ -q
25 passed in 1.37s
$ cd server && .venv/bin/python -m pytest tests/ -q
114 passed, 1 warning in 11.24s
```

robomw 는 T1~T6 종료 시점 45(스펙②) + 링크 플랩 수정이 남긴 회귀 시험
5(`test_wsserver_link.py`)로 50. server 는 T5 이후 무변화(114).

## 2. self_test · relocalize · spray — 실기

### self_test (정상 → 라이다 다운 → 복구)

```json
정상   {"items":[{"name":"lidar","ok":true,"detail":"13.7 Hz"},
               {"name":"imu","ok":true,"detail":"316.6 Hz"},
               {"name":"localizer","ok":true,"detail":"quality=1.00"},
               {"name":"link","ok":true,"detail":"정상"},
               {"name":"drive","ok":true,"detail":"v_max=0.80 w_max=1.20"}],
        "all_ok":true}
라이다다운 {"lidar":{"ok":false,"detail":"0.0 Hz"}, 나머지 4항목 ok=true, "all_ok":false}
복구   all_ok=true (lidar 12.9 Hz 재확인)
```

절차: `bridge_down.sh`(livox_sim_bridge 종료 미니 헬퍼) → 4초 대기(3초
수신율 창) → self_test → `bridge_up.sh`(`apply_fov_mask:=false` 유지) →
3초 대기 → self_test. agent.log 오류 0.

### relocalize (2 m 오프셋 주입 → 복구)

`/map_localizer/reinit` 로 est 를 참값 +2 m 로 주입(물리 로봇은 안 건드림,
T3 실기 절차 재사용) → `relocalize{alley:0,end:south}`:

```
주입 후 오차       2.000 m
relocalize 응답    completed, quality=0.507, 0.20 s
복구 후 오차        0.187 m   (≤0.3 m 기준 통과)
```

### spray 거부

```json
{"cmd":"mission_start","status":"rejected","code":"UNSUPPORTED",
 "data":{"reason":"미지원 작업 유형"}}
```

## 3. 정찰 실기 — 4차 시도 (관제 링크 플랩 3라운드가 앞선 3회를 막았다)

### 3.1 무슨 일이 있었나

`work:{type:"scout",params:{speed_scale:0.5}}` + `alleys:[0,1,2]` 로 부분
정찰을 걸고 30초 간격으로 `state`(progress·pose)를 폴링하며 완료 보고
(`cmd_result`)를 기다리는 방식은 처음부터 맞았다. 문제는 로봇이 아니라
**관제 링크가 임무 중간에 끊기는 것**이었다 — 세 번 연속, 매번 다른
결함이었다.

| 시도 | cmd_id | 결과 | 정지 지점 | 비고 |
|---|---|---|---|---|
| 1 | `t7-scout-patrol` | 정체 → 3900s 하드타임아웃 | 통로1 idx4/9, pose(-10.49,-2.05) | 링크 플랩 자체를 처음 노출 |
| 2 | `t7-scout-patrol-v2` | 정체 → 4200s 하드타임아웃 | 통로0 idx0/9, pose(-14.02,3.64) | 라운드1 수정(`8ced3dd`) 후에도 재현 |
| 3 | `t7-scout-patrol-v3` | 정체(2200s+) → 수동 종료 | 통로0 idx0/9, pose(-14.00,18.29) | 라운드2 수정(`173e052`) 후에도 재현 |
| 4 | `t7-scout-patrol-v4` | **completed** | 통로2 idx8/9 완주 | 라운드3 수정(`2e652d3`) 후 클린 완주 |

매 시도 사이에 표준 절차로 스택을 원복했다: `restart_agent.sh` →
`restart8000.sh`(수정 반영) → `recover_robot.py`(RX=-14.0 RY=-31.5
RYAW=1.5708, 통로 0 남단 텔레포트) → `restart_loc.sh`(동일 INIT).

### 3.2 세 라운드 원인 (요약 — 상세는 `debug-link-flap-report.md`)

**라운드 1 (`8ced3dd`)** — `robomw/link/wsserver.py`의 송신 락과 수신
루프가 얽혀 있었다. `Conn.send_text()`가 락을 쥔 채 시한 없는
`sock.sendall()`을 부르고, 수신 루프의 `OP_PING` 처리가 PONG 을 보내려면
같은 락이 필요했다. 관제가 잠깐이라도 소켓을 안 읽으면 송신이 무기한
막히고, 그동안 수신도 같이 멈춰 하트비트를 못 읽는다 → 로봇이 **살아
있는 링크**를 두절로 오판. 수정: 수신 루프는 락을 최대 0.2초만 기다리고
(`PONG_LOCK_WAIT_S`), 못 보낸 PONG 은 버리지 않고 다음 송신에 얹으며
(`_pending_pong`), 송신 자체에 `SO_SNDTIMEO=2.0초` 시한을 건다.

**라운드 2 (`173e052`)** — 라운드 1이 두 구멍을 남겼다. (a) 락을 잡은
뒤의 PONG 송신 자체는 기본 `SEND_TIMEOUT_S=2.0초`를 그대로 썼는데, 이는
`P.LINK_LOSS_STOP_MS=1.5초`보다 길어 여전히 오판을 낼 수 있었다 → 전용
`PONG_SEND_TIMEOUT_S=0.2초`로 분리(최대 정지 0.2+0.2=0.4초). (b)
`broadcast`가 죽은 연결을 목록에서만 빼고 **소켓을 안 닫아**, 어댑터는
연결이 살아 있다고 믿는데 로봇 장부에서만 사라지는 반쪽짜리 연결이
생겼다 → 걷어낸 연결은 `shutdown(SHUT_RDWR)` 후 `close()`로 확실히 닫음.

**라운드 3 (`2e652d3`, 이번 정찰 게이트의 실제 진범)** — `wsserver.py`가
아니라 **진단 도구 자신**이 원인이었다. `scripts/42_probe_robot_state.py`
가 프레임을 받을 때마다(즉 루프를 돌 때마다) `ping`을 다시 보내는
패턴이었는데, 로봇은 `ping` 한 건마다 `pong` **이벤트**를 전 관제에
브로드캐스트한다 → 그 이벤트 프레임이 probe 에 도착 → probe 가 또
`ping` → **되먹임 고리가 닫힌다**. 실측 DB 타임스탬프로 발산이
지문처럼 남았다(1,056 → 2,241 → 3,317 → **6,673 pong/s**, 1초 간격
거의 배증). 관제(fleet_server)가 이벤트를 건건이 동기 SQLite INSERT 로
적다 보니 초당 수천 건에 이벤트 루프가 굶고, 같은 루프의 1 Hz
하트비트가 2.1~3.8초씩 밀렸다 — 그 침묵을 로봇이 1.5초 링크두절로
읽어 `link_lost` → 임무 자동 일시정지. 서버 1011 이 0건이고 어댑터
접속이 안 끊긴 채로 로봇만 두절을 찍은 이번 지문이 정확히 이 경로에서
나온다. 이중 방어로 고쳤다: (a) 도구 — `42_probe_robot_state.py`의
`ping`을 1 Hz 로만 보내게 제한(`PING_PERIOD_S`), (b) 로봇 —
`control_agent`가 `pong` **이벤트** 발행 자체에 간격 상한을 건다
(`PONG_EVENT_MIN_GAP_S=0.5초`) — 도구 하나를 고치는 것으로 끝내지
않고, 어떤 클라이언트가 비슷한 순진한 킵얼라이브 패턴을 다시 들고
와도 로봇 쪽에서 증폭 자체를 막는다.

### 3.3 배운 것

**진단 도구가 관측 대상을 무너뜨릴 수 있다.** 이번 사고의 진범은
로봇도 관제 서버도 아니라, 상태를 "그냥 조회만" 하려던 폴링 스크립트
였다. 겉보기에 부작용이 없어 보이는 킵얼라이브 패턴(프레임 수신마다
ping 재발행)이, 시스템 자신의 이벤트 에코(ping → pong 이벤트
브로드캐스트) 와 만나 공진하는 되먹임 고리를 만들었고, 그 발산이
정작 지키려던 제어 링크를 굶겼다. 방어는 도구 하나를 고치는 것으로
끝내지 않고 **소스(도구)와 수신측(로봇) 양쪽에 상한을 거는 이중
방어**로 마무리했다 — 잘 작성된 도구 하나에 기대는 것은 다음에 올
비슷한 도구까지 막아주지 못한다.

### 3.4 4차(최종) 완료 보고

```json
{"kind":"cmd_result","cmd_id":"t7-scout-patrol-v4","cmd":"mission_start",
 "status":"completed","code":"OK",
 "data":{"alleys_done":[0,1,2],"distance_m":234.5,"duration_s":2237.6,
         "interventions":0,"coverage":1.0}}
```

판정: `status==completed` · 보고 키 5종이 계약(`MISSION_REPORT_KEYS`)과
일치 · `alleys_done==[0,1,2]` · `coverage==1.0` · `interventions==0`(무개입,
로컬라이저 8초 경보 2건은 1분 내 스스로 `resolved` 로 해소돼 개입으로
안 셈 — T9 로봇 이관 게이트와 동일 판정) · **반속 확인**:
`duration_s(2237.6) / 정상속도 3통로 기준선(1128.4초, 2026-08-12
robomw-extraction 게이트) = 1.98배` ≈ 2배(스펙이 요구하는 "구간
소요시간 ~2배" 그대로).
`distance_m(234.5)`도 정상속도 기준선(220.2 m)과 같은 자릿수.

state.work.progress 상승은 통로 완주 단위로 0.0 → 0.333(통로0 완주) →
0.667(통로1 완주) → 1.0(완료 직전) 으로 30초 간격 폴링에서 그대로
관측됨(§3.4 로그 전문은 `t7_patrol.log`, 스크래치 보관).

완료 후 로봇은 통로 2 북단 부근(-7.02, 31.07)에 idle 로 섰다(gate 파악용
`42_probe_robot_state.py` 및 T7 자체 WS 폴링으로 확인).

## 4. blackbox_dump 실기 (정찰 직후 1회)

```json
{"status":"completed","code":"OK",
 "data":{"path":"/tmp/blackbox_scout01_1786623525.npz","bytes":14510,
         "events":2,"poses":848}}
```

npz 검증: `poses.shape=(848,4)` float32(900초 링 상한에 걸려 901개 미만
— 설계대로), 마지막 포즈 시각이 호출 시점으로부터 **54초 전**(방금 주행
궤적), 첫 포즈는 950초 전(900초 창의 경계와 일치), y 축 궤적 범위
66.15 m(통로 하나를 왕복한 크기와 부합). 이벤트 2건 —
`resolved`(LOCALIZATION_LOST 해소)·`mission_done` — 방금 정찰의 실제
사건과 일치.

## 5. 대시보드 Playwright 스모크

T6 리포트 스모크(로그인 → 정비 탭 → self_test → relocalize → 스크린샷 →
임무 탭 → 슬라이더 → 정찰 시작 → 즉시 취소)를 서버 재기동(`restart8000.sh`,
링크 사건 대응으로 수 차례 재기동됨) 이후 다시 실행:

- self_test 5/5 ✓, 성공 토스트
- relocalize(통로 0·남단, harness 이후 복구된 실제 위치) → **completed**,
  카드 "완료 · 품질 0.51", 토스트 "scout01 · 명령 완료"
- 정찰 버튼: 슬라이더 기본 1.0 확인 → 0.5 조정 → `POST /missions` 200,
  `spec=={"work":{"type":"scout","params":{"speed_scale":0.5}}}`(alleys
  키 없음) → 즉시 취소 200
- 콘솔 오류 0 · 페이지 오류 0

스크린샷: `docs/figures/dashboard_maint_card_t7.png`,
`docs/figures/dashboard_scout_button_t7.png`.

## 6. 회귀

### 하네스 남측 4쌍 (P0, n=1)

```
(-10.5,-34.0)→x≥-7.5   P0 1/1 통과 11.1초 최대정체0.4초
(-3.5,-34.0)→x≥-0.5    P0 1/1 통과 10.9초 최대정체0.4초
(3.5,-34.0)→x≥6.5      P0 1/1 통과 11.0초 최대정체0.5초
(10.5,-34.0)→x≥13.5    P0 1/1 통과 11.1초 최대정체0.5초
```

4/4 통과, 소요·정체 모두 남측 기존 실측(2026-08-11 자, 10.6~11.0초)과
같은 자릿수 — 재현성 확인. 하네스 종료 후 `recover_robot.py` → 통로 0
남단 텔레포트 → `restart_loc.sh` → `restart_agent.sh` → `restart8000.sh`
로 전 스택 원복.

### 보안·감사 (격리 인스턴스, 메인 에이전트 8080 무관)

`scripts/21_verify_security.py --port 8444 --token <sim/certs/control.token>
--cert sim/certs/control.crt` → **9/9**. `scripts/30_verify_audit_roles.py
--port 8081 --audit <scratch>/audit_t7.jsonl --token-observer OBS
--token-admin ADM` → **10/10**. 두 인스턴스 모두 실제 PID 를 특정해
개별 종료, 메인 에이전트 무영향.

### 진단 자동정지 (scripts/49, 메인 에이전트 대상)

로봇 idle·무임무 상태에서 실행(전제 충족). critical(LOST_LONG)·
TRACTION_LOSS 진단 주입 → 자동 일시정지·개입 큐 발행·해소 이벤트·검증
후 자동 원복까지 **13/13 통과**. 실행 후 `paused=False` 자동 확인.

### 지도 공급 격리 (scripts/50, 독립 실행)

ROS 기동 불필요 — `ControlAgent._on_cloud` 함수 단위 호출.
**18/18 통과**(고장 격리·반복고장 로그 상한·정상 회귀·솎인 프레임·
어댑터 결손 메시지 5개 절).

## 7. 최종 상태 확인

게이트 종료 시점: `mode=idle · estop=False · paused=False · gate=idle ·
mission=None`, pose (-14.20, -31.41, 90°) — 통로 0 남단, 이 태스크
시작 시점과 동일한 좌표. 활성 임무 없음(정찰 완료 임무는 DB 에
COMPLETED, 대시보드 스모크가 만든 미션은 즉시 CANCELED).

## 결론

스펙 ② §5 게이트 표 **전 항목 통과**. 정찰 게이트는 로봇/명령 계약
자체의 결함이 아니라 관제 링크 인프라(3라운드 결함, §3)가 네 번째
시도까지 막았고, 그 결함들은 이 태스크와 병행된 디버그 태스크가 전부
찾아 고쳤다(`8ced3dd`·`173e052`·`2e652d3`) — 세 커밋 모두 이번 세션에서
로봇/서버 재기동으로 실기에 반영·검증됐다.

BLOCKED 로 보고할 항목 없음.

이월 사항(이번 태스크 범위 밖, 과거·병행 리포트에 이미 기록된 것):
- `pong` **이벤트**를 서버가 영구 DB 기록으로 남기는 설계 자체는 이번
  되먹임 사고의 피해를 키운 별건이다(디버그 리포트 §6-B/부록2 §6) —
  `ingest.event` 가 링크 확인 잡음(`pong`)을 거르는 편이 옳으나 스펙 ②
  범위 밖.
- relocalize 확인 대기(최대 0.x초, 정상 성공 경로는 0.2초 내)가 여전히
  동기라 그 창에서 `control_tick`이 멎는다 — T3 리포트가 이미 남긴
  이월(비동기 relocalize는 계약 변경, 별도 스펙 후보).
