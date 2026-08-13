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
# --robots "이름:x,y,yaw도 ..." — 로봇 인스턴스 이름이 곧 토픽·TF 접두다
python3 scripts/gen_world.py --rows 10 --trees-per-row 41 \
  --robots "scout01:-14.0,-33.0,90" --environment --detail 2 --instrumented-rows 0 \
  --out sim/worlds/orchard_nav.sdf

# 시뮬 + 로봇 스택 + 관제 (관제 접속 주소를 안내해준다)
bash scripts/run_control.sh
```

관제 서버(8000)는 `server/`에서 uvicorn으로 별도 기동한다 — 자세한 절차와 계정 설정은 `docs/` 및 `server/` 참조. 저장소의 admin/123 등은 로컬 시뮬 전용 예시 계정이다.

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
- ⏳ 다음: 신규 명령 동작 구현(작업 유형·진단), 이기종 2호기 온보딩 실증, 측위 위상 고착 종결(후방 산포 게이트)

## 라이선스

[Apache License 2.0](LICENSE) — ⓒ 2026 YBNML
