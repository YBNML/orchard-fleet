# orchard-fleet 🍎

**계단식 과수원 무인이동체(UGV) 자율주행 + 통합관제 플랫폼** — 시뮬레이션·로봇·통신 미들웨어·관제 웹을 하나의 스택으로.

한국 구릉지의 계단식(단구식) 사과 과수원을 Gazebo에 물리 수준으로 재현하고, 그 위에서 AgileX Scout Mini(+Livox MID-360)가 사전 맵 기반 측위로 **9개 통로를 무개입 완주**하며, FastAPI 관제 서버와 웹 대시보드로 원격 감시·명령한다. 이기종 로봇 온보딩을 위한 로봇측 미들웨어(`robomw`)가 명령 계약을 소유한다.

## 구조

```
orchard-fleet/
├── ros2_ws/src/orchard_sim/   # 무인이동체 + 시뮬레이션
│   │                          #   Gazebo 월드(계단식 지형·나무 410그루·환경 구조물),
│   │                          #   사전 맵 측위(map_localizer), 제어 에이전트, ROS 어댑터
│   └── ...
├── ros2_ws/src/robomw/        # 통신 미들웨어 (ROS 비의존 코어)
│   │                          #   명령 계약 v1(권한·cmd_result·hello)·SafetyArbiter·
│   │                          #   SDK 5종 인터페이스·과수원 임무 프로파일 → README 참조
│   └── README.md
├── server/                    # 통합관제 플랫폼
│   │                          #   FastAPI + SQLite, WebSocket 텔레메트리 허브,
│   │                          #   웹 대시보드(단일 파일, 로그인·개입 큐·2단계 비상정지 해제)
│   └── web/index.html
├── scripts/                   # 검증·생성 도구 50종 (지형/월드 생성, 보안·측위·횡단 하네스)
├── maps/                      # 맵 번들 (주행가능 격자·통로 그래프·벽 교정 테이블)
└── docs/                      # 설계서·실험 보고서·findings (전 과정 기록)
```

## 핵심 특징

- **지형이 곧 난이도** — 통로 간 26~50 cm 단차의 계단식 지형, 선회 평지 패드, 남북 비대칭 램프. 횡단 통과율을 정책별로 직접 재는 클라임 하네스 포함
- **사전 맵 측위** — 열 주기 구조(3.5×1.5 m)의 위상 보정 + 통로×단별 실측 벽 앵커 + AHRS 요 융합. 통로 안 RMS 0.09 m
- **안전 우선 설계** — 비상정지 2단계 해제(원격 승인+현장 확인), 텔레옵 데드맨 400 ms, 링크두절 1.5 s 자율 정지, 속도 출력 단일 창구(SafetyArbiter)
- **명령 계약 미들웨어(robomw)** — 이기종 로봇이 같은 명령을 받아 과정은 달라도 같은 결과 보고를 내도록: cmd_result 상관(멱등)·표준 완료 보고·능력군 선언(hello). 새 로봇 온보딩 = SDK 5종(Drive·Localizer·Perception·Work·Diag) 구현
- **실측 문화** — 모든 결론은 시뮬 실주행·하네스 수치로 검증하고 docs/findings·실험 보고서에 기록

## 빠른 시작 (시뮬)

```bash
# 의존: ROS2 Jazzy, Gazebo(gz sim), Python 3.12
cd ros2_ws && colcon build --packages-select robomw orchard_sim && source install/setup.bash && cd ..

# 지형·월드 생성 (커밋된 월드를 재생성할 때)
python3 scripts/gen_heightmap.py --rows 10 --trees-per-row 41
# --robots "이름:x,y,yaw도 ..." — 로봇 인스턴스 이름이 곧 토픽·TF 접두다 (여러 대면 공백으로 나열)
python3 scripts/gen_world.py --rows 10 --trees-per-row 41 \
  --robots "scout01:-14.0,-33.0,90 scout02:7.0,-33.0,90" \
  --environment --detail 2 --instrumented-rows 0 \
  --out sim/worlds/orchard_nav.sdf

# 시뮬레이터 (월드 하나에 로봇 모두)
gz sim -s -r -v2 sim/worlds/orchard_nav.sdf

# 로봇 스택은 대수만큼 — /clock 브리지는 월드당 하나뿐이라 첫 대만 clock:=true
ros2 launch orchard_sim control.launch.py robot_id:=scout01 ns:=scout01 port:=8080 clock:=true
ros2 launch orchard_sim control.launch.py robot_id:=scout02 ns:=scout02 port:=8081 clock:=false

# 관제 서버 (별도 터미널)
cd server && python -m uvicorn fleet_server.app:create_app --factory --host 0.0.0.0 --port 8000
```

다중 로봇 온보딩(네임스페이스·포트·서버 등록)의 전체 절차는 `ros2_ws/src/robomw/README.md` §4.1 에 있다. 계정 설정은 `server/` 참조 — 저장소의 admin/123 등은 로컬 시뮬 전용 예시 계정이다.

## 문서 지도

| 문서 | 내용 |
|---|---|
| `docs/design/M3_localization_report.md` | 측위 실험 전 기록 — 실패 아홉 번의 해부, MID-360 전환, 선회 패드, 무개입 완주까지 |
| `docs/superpowers/specs/` | 설계 스펙 (3계층 플랫폼, robomw 명령 계약 v1) |
| `docs/findings/` | 날짜별 발견 노트 (하이트맵 y-플립, robomw 추출 게이트 수치 등) |
| `ros2_ws/src/robomw/README.md` | 명령 계약 요약·SDK 시그니처·새 로봇 온보딩 절차 |

## 현재 상태

- ✅ M3 자율주행: 9통로 무개입 완주 재현(2연속) · 통로 안 측위 RMS 0.09 m
- ✅ robomw v0.1: 명령 계약 + scout 재배선 + 서버 프로토콜 단일화 (회귀 게이트 전 항목 녹색)
- ✅ 신규 명령 동작: 작업 유형(정찰 work)·진단 3종(self_test·relocalize·blackbox_dump) + 대시보드 명령 UI
- ✅ **2대 동시 운용**: scout01·scout02 를 네임스페이스로 다중화하고 통로 잠금(AlleyLock)으로 공간 분리 — 분담 정찰 프리셋으로 통로 [0..4]/[6,7,8] 동시 주행 실증(69.8분), 상호 라이다 오염 0점/프레임 실측
- ✅ **서버측 Behavior Tree 임무 큐**: 노드 5종(Sequence·Selector·Retry·Condition·Action) + 프리셋 3종 + 서버 재기동 복원, 대시보드에 트리 상태·통로 점유 오버레이
- ⏳ 다음: 선회 구역 보정 자기잠금 해소(횡단 중 자동 정지의 원인), 측위 종방향 1.5 m 위상 고착 종결, 3대 이상 운용을 위한 점군 대역 확보(DDS 전송 설정 또는 표본 감축)

최신 게이트 수치는 `docs/findings/2026-08-13-multirobot-bt.md` 참조.

## 라이선스

[Apache License 2.0](LICENSE) — ⓒ 2026 YBNML
