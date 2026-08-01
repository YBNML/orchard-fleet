#!/usr/bin/env python3
"""설계서 구조도 생성 — docs/figures/design/*.png

graphviz(dot)로 구조도·플로차트를, matplotlib로 로드맵을 그린다.
설계서(docs/superpowers/specs/2026-08-01-*, docs/design/*.docx)에 삽입된다.
"""
import subprocess
import sys
from pathlib import Path

OUT = Path("docs/figures/design")
OUT.mkdir(parents=True, exist_ok=True)

F = "Noto Sans CJK KR"
HDR = f'''
  graph [fontname="{F}", fontsize=11, pad=0.25];
  node  [fontname="{F}", fontsize=11, style="filled,rounded", shape=box,
         penwidth=1.2, margin="0.18,0.09"];
  edge  [fontname="{F}", fontsize=10, penwidth=1.1];
'''
# 계층 팔레트
C_ROBOT = "#e8f1e4"   # 현장(로봇) 연녹
C_SRV   = "#e4ecf7"   # 서버 연청
C_WEB   = "#fdf3dc"   # 사용자 연황
C_FUT   = "#f3e9f5"   # 미래 확장 연보라
C_WARN  = "#fbe3e0"   # 안전/경고 연적
E_ROBOT = "#4c7d43"
E_SRV   = "#3a6ea5"
E_WEB   = "#b8860b"
E_FUT   = "#8e5ba6"
E_WARN  = "#b0483d"

FIGS: dict[str, str] = {}

# ── 그림 1. 전체 아키텍처 ──────────────────────────────────────────────────
FIGS["fig1_arch"] = f'''
digraph arch {{
{HDR}
  rankdir=LR; ranksep=0.9; nodesep=0.45;
  compound=true; newrank=true;

  subgraph cluster_field {{
    label="노지 과수원 (현장)"; labeljust=l; fontsize=13;
    style="rounded,filled"; fillcolor="#f7faf5"; color="{E_ROBOT}";
    subgraph cluster_robot {{
      label="무인이동체 (Scout Mini + MID-70)"; fillcolor="{C_ROBOT}"; color="{E_ROBOT}";
      nav   [label="자율주행 스택\\l  사전맵 로컬리제이션\\l  경로 생성·추종\\l", fillcolor="white", color="{E_ROBOT}"];
      safety[label="SafetyArbiter\\l  비상정지 래치·데드맨\\l  장애물·주행경계·기울기\\l  (온보드 완결)\\l", fillcolor="{C_WARN}", color="{E_WARN}"];
      agent [label="로봇 에이전트\\l  Feature 플러그인\\l  store-and-forward 버퍼\\l", fillcolor="white", color="{E_ROBOT}"];
      rec   [label="로컬 기록\\l  점군·주행 로그 청크\\l", fillcolor="white", color="{E_ROBOT}"];
      nav -> safety [label="속도 요청"];
      agent -> safety [label="원격 명령"];
    }}
  }}

  subgraph cluster_srv {{
    label="회사 서버 (컨테이너, 이전 가능)"; labeljust=l; fontsize=13;
    style="rounded,filled"; fillcolor="#f4f8fc"; color="{E_SRV}";
    zenohd [label="zenohd 라우터 ≥1.8\\l  TLS + 로봇별 mTLS + ACL\\l  MQTT 플러그인 (이종 로봇)\\l", fillcolor="{C_SRV}", color="{E_SRV}"];
    core   [label="관제 코어\\l  Zenoh↔내부 브리지\\l  권한 1차 판정·감사\\l  FleetPort (교체 가능)\\l", fillcolor="{C_SRV}", color="{E_SRV}"];
    api    [label="FastAPI\\l  계정·로그인 / 농장·로봇\\l  임무·이력 API\\l  WebSocket 게이트웨이\\l  배치 업로드 수신\\l", fillcolor="{C_SRV}", color="{E_SRV}"];
    db     [label="SQLite\\l  (SQLAlchemy — PG 이전 가능)\\l", shape=cylinder, fillcolor="white", color="{E_SRV}"];
    oro    [label="외부 플랫폼 어댑터 포트\\l  OpenRobOps·Open-RMF (향후)\\l", style="filled,rounded,dashed", fillcolor="{C_FUT}", color="{E_FUT}"];
    zenohd -> core; core -> api [dir=both]; api -> db; core -> oro [style=dashed];
  }}

  browser [label="농민·관리자 브라우저\\l  로그인 → 농장 선택 → 관제\\l", fillcolor="{C_WEB}", color="{E_WEB}"];
  future  [label="향후 추가 로봇\\l  실내 AMR (VDA 5050/MQTT)\\l  ORO agent 로봇\\l", style="filled,rounded,dashed", fillcolor="{C_FUT}", color="{E_FUT}"];

  agent -> zenohd [label="LTE · Zenoh client\\n아웃바운드 접속", color="{E_ROBOT}", fontcolor="{E_ROBOT}", ltail=cluster_robot];
  rec   -> api    [label="HTTP 배치 업로드\\n(1분 청크·이어올리기)", style=dashed, color="{E_ROBOT}", fontcolor="{E_ROBOT}"];
  future -> zenohd [label="MQTT(S)", style=dashed, color="{E_FUT}", fontcolor="{E_FUT}"];
  api -> browser  [label="HTTPS + WebSocket", dir=both, color="{E_WEB}", fontcolor="{E_WEB}"];
}}
'''

# ── 그림 2. 통신 계층 — 채널 이원화 ────────────────────────────────────────
FIGS["fig2_comm"] = f'''
digraph comm {{
{HDR}
  rankdir=LR; ranksep=1.0; nodesep=0.5;

  subgraph cluster_r {{
    label="무인이동체"; style="rounded,filled"; fillcolor="{C_ROBOT}"; color="{E_ROBOT}";
    rt  [label="실시간 채널 (Zenoh)\\l  tel/state ~2 Hz (최신값)\\l  tel/health 1 Hz\\l  tel/map 델타\\l  evt·ack (신뢰 전달)\\l  cmd·teleop 수신\\l", fillcolor="white"];
    sf  [label="store-and-forward\\l  SQLite 큐 (evt·ack·상태 1 Hz)\\l  단절 시 적재 → 재연결 시\\l  seq 순 재전송\\l", fillcolor="white"];
    blk [label="벌크 경로 (Zenoh 밖)\\l  점군·주행 로그\\l  1분 청크 파일 기록\\l", fillcolor="white"];
  }}

  subgraph cluster_s {{
    label="회사 서버"; style="rounded,filled"; fillcolor="{C_SRV}"; color="{E_SRV}";
    zd  [label="zenohd ≥1.8\\l  TLS 7447 · mTLS 로봇 인증\\l  ACL: 로봇은 자기 key만\\l  liveliness → 연결 감시\\l", fillcolor="white"];
    mq  [label="MQTT 플러그인\\l  1883/8883\\l  토픽↔key 자동 매핑\\l", style="filled,rounded,dashed", fillcolor="{C_FUT}", color="{E_FUT}"];
    up  [label="업로드 수신기\\l  PUT /api/v1/upload\\l  이어올리기·무결성 검사\\l", fillcolor="white"];
    dedup [label="수집기\\l  (robot, seq) 중복 제거\\l  DB 기록·WS 팬아웃\\l", fillcolor="white"];
    zd -> dedup; up -> dedup [style=invis];
  }}

  amr [label="실내 AMR·ORO agent (향후)", style="filled,rounded,dashed", fillcolor="{C_FUT}", color="{E_FUT}"];

  rt -> zd  [label="Zenoh/TLS (LTE)\\n아웃바운드 · 자동 재연결", dir=both, color="{E_ROBOT}", fontcolor="{E_ROBOT}"];
  sf -> rt  [label="재연결 시 재생"];
  blk -> up [label="HTTP PUT (기회적 업로드)", style=dashed, color="{E_ROBOT}", fontcolor="{E_ROBOT}"];
  amr -> mq [label="MQTT(S)", style=dashed, color="{E_FUT}", fontcolor="{E_FUT}"];
  mq -> zd [label="key 공간 합류", color="{E_FUT}", fontcolor="{E_FUT}"];
}}
'''

# ── 그림 3. 명령 처리 흐름 (권한 2중 판정) ─────────────────────────────────
FIGS["fig3_cmd"] = f'''
digraph cmd {{
{HDR}
  rankdir=TB; ranksep=0.5;

  u   [label="사용자 (브라우저)\\n임무 시작 / 비상정지 / 텔레옵", fillcolor="{C_WEB}", color="{E_WEB}"];
  ws  [label="WebSocket 게이트웨이\\n세션 쿠키 → 사용자·역할 식별", fillcolor="{C_SRV}", color="{E_SRV}"];
  a1  [label="서버 1차 권한 판정\\n역할×명령 매트릭스 (fail-closed)\\n+ 농장 스코프 확인", shape=diamond, fillcolor="{C_SRV}", color="{E_SRV}"];
  aud [label="감사 기록 (DB)\\n수락·거부 전부", shape=note, fillcolor="white", color="{E_SRV}"];
  pub [label="Zenoh 발행\\nfleet/v1/{{농장}}/{{로봇}}/cmd\\ncmd_id·발행자·역할 포함", fillcolor="{C_SRV}", color="{E_SRV}"];
  a2  [label="로봇 최종 판정\\n동일 매트릭스 재검증 (fail-closed)\\n+ SafetyArbiter 게이트", shape=diamond, fillcolor="{C_ROBOT}", color="{E_ROBOT}"];
  exe [label="실행\\n(임무 시작 / 정지 래치 / 속도 중재)", fillcolor="{C_ROBOT}", color="{E_ROBOT}"];
  ack [label="ack 발행 (cmd_id 상관)\\n→ 서버 기록 → 사용자 화면 반영", fillcolor="{C_ROBOT}", color="{E_ROBOT}"];
  rej1[label="거부 응답 + 감사\\n(이벤트 억제: 주소·역할 키)", fillcolor="{C_WARN}", color="{E_WARN}"];
  rej2[label="거부 ack + 이벤트", fillcolor="{C_WARN}", color="{E_WARN}"];

  u -> ws -> a1;
  a1 -> rej1 [label="권한 없음", color="{E_WARN}", fontcolor="{E_WARN}"];
  a1 -> pub  [label="허용"];
  a1 -> aud [style=dashed];
  pub -> a2 [label="Zenoh (LTE)"];
  a2 -> rej2 [label="권한 없음/안전 저지", color="{E_WARN}", fontcolor="{E_WARN}"];
  a2 -> exe [label="허용"];
  exe -> ack; rej2 -> ack [style=invis];
}}
'''

# ── 그림 4. 링크 단절 대응 ─────────────────────────────────────────────────
FIGS["fig4_linkloss"] = f'''
digraph ll {{
{HDR}
  rankdir=TB; ranksep=0.45;

  det [label="링크 단절 감지\\n(Zenoh 세션 상실 / liveliness 소멸)", fillcolor="{C_WARN}", color="{E_WARN}"];
  mode[label="현재 모드?", shape=diamond, fillcolor="white", color="{E_ROBOT}"];
  tel [label="텔레옵 중\\n데드맨 400 ms → 즉시 정지\\n(기존 불변식 유지)", fillcolor="{C_ROBOT}", color="{E_ROBOT}"];
  mis [label="임무 중\\n계속 주행 (온보드 안전 게이트 전제)\\n단절 30분 초과: 통로 끝 일시정지\\n(농장 설정으로 변경 가능)", fillcolor="{C_ROBOT}", color="{E_ROBOT}"];
  idle[label="대기 중\\n정지 상태 유지", fillcolor="{C_ROBOT}", color="{E_ROBOT}"];
  buf [label="store-and-forward 적재\\nevt·ack·상태 1 Hz → SQLite 큐\\n(상한 도달 시 오래된 tel부터 순환)", fillcolor="white", color="{E_ROBOT}"];
  rec [label="재연결 (자동, 지수 백오프 1→30초)\\n큐 재생 → 서버 (robot, seq) 중복 제거\\n비상정지 래치 상태 보고", fillcolor="{C_ROBOT}", color="{E_ROBOT}"];
  srv [label="서버 측\\nliveliness 소멸 → 대시보드 '연결 끊김'\\n마지막 수신 시각 표시 (2차: 알림)", fillcolor="{C_SRV}", color="{E_SRV}"];

  det -> mode;
  mode -> tel [label="텔레옵"]; mode -> mis [label="임무"]; mode -> idle [label="대기"];
  tel -> buf; mis -> buf; idle -> buf;
  buf -> rec;
  det -> srv [style=dashed];
}}
'''

# ── 그림 5. 관제 서버 내부 구조 ────────────────────────────────────────────
FIGS["fig5_server"] = f'''
digraph srv {{
{HDR}
  rankdir=LR; ranksep=0.8; nodesep=0.4;

  subgraph cluster_edge {{
    label="입출력"; style="rounded,filled"; fillcolor="#f4f8fc"; color="{E_SRV}";
    zd [label="zenohd\\l  + MQTT 플러그인\\l", fillcolor="{C_SRV}"];
    wsg[label="WebSocket\\l게이트웨이\\l", fillcolor="{C_SRV}"];
    rest[label="REST /api/v1\\l  auth·farms·robots\\l  missions·history·users\\l", fillcolor="{C_SRV}"];
    upl [label="업로드 수신\\l", fillcolor="{C_SRV}"];
  }}

  subgraph cluster_core {{
    label="관제 코어 (도메인)"; style="rounded,filled"; fillcolor="#f4f8fc"; color="{E_SRV}";
    fp  [label="FleetPort (인터페이스)\\l  로봇 등록·텔레메트리 수집\\l  명령 라우팅·연결 상태\\l  ← ORO 공개 시 대체 평가 지점\\l", fillcolor="{C_FUT}", color="{E_FUT}"];
    auth[label="AuthService\\l  Argon2 해시·세션\\l  역할×명령 (fail-closed)\\l  농장 스코프\\l", fillcolor="{C_SRV}"];
    ms  [label="MissionService\\l  RMF 호환 태스크 모델\\l  task_id·phases·상태 전이\\l", fillcolor="{C_SRV}"];
    hs  [label="HistoryService\\l  궤적 1 Hz·이벤트·감사\\l", fillcolor="{C_SRV}"];
  }}

  db [label="SQLite\\l users·farms·robots\\l missions·tracks·events·audit\\l", shape=cylinder, fillcolor="white", color="{E_SRV}"];

  zd -> fp [dir=both]; wsg -> auth; rest -> auth;
  auth -> ms; auth -> hs; ms -> fp [dir=both]; upl -> hs;
  fp -> hs [label="기록"]; ms -> db; hs -> db; auth -> db;
  fp -> wsg [label="실시간 팬아웃"];
}}
'''

# ── 그림 6. 현장 자율주행 스택 (사전 맵 기반) ──────────────────────────────
FIGS["fig6_field"] = f'''
digraph field {{
{HDR}
  rankdir=TB; ranksep=0.5; nodesep=0.4;

  subgraph cluster_off {{
    label="오프라인 (맵핑 세션 후 1회)"; style="rounded,filled"; fillcolor="#fbf8f2"; color="{E_WEB}";
    cloud [label="사전 맵 점군\\n(맵핑 세션 산출물 — 별도 트랙)", fillcolor="white", color="{E_WEB}"];
    trav  [label="주행가능도 분석\\n(traversability — 검증 완료 모듈)", fillcolor="white", color="{E_WEB}"];
    corr  [label="통로 추출·경로 그래프\\n(row_structure + corridor — 115 mm 검증)", fillcolor="white", color="{E_WEB}"];
    bundle[label="맵 번들 v1\\ncloud.pcd · trav.npz · graph.json · meta.yaml", shape=folder, fillcolor="{C_WEB}", color="{E_WEB}"];
    cloud -> trav -> corr -> bundle;
  }}

  subgraph cluster_on {{
    label="온라인 (주행 중, 온보드)"; style="rounded,filled"; fillcolor="#f7faf5"; color="{E_ROBOT}";
    sens [label="센서: MID-70 점군 · IMU · 휠 엔코더", fillcolor="white", color="{E_ROBOT}"];
    ekf  [label="EKF 오도메트리 (고빈도)\\n휠(회전 과대 보정) + IMU 자이로", fillcolor="{C_ROBOT}", color="{E_ROBOT}"];
    reg  [label="스캔↔사전맵 정합 (NDT, 1~2 Hz)\\n절대 위치 보정 — 오차 비누적", fillcolor="{C_ROBOT}", color="{E_ROBOT}"];
    gate [label="퇴화 게이트\\n구조점 비율·측방 분산 검사\\n(MID-70 전방시야 대응 — 진단 모듈 재사용)", shape=diamond, fillcolor="white", color="{E_ROBOT}"];
    fuse [label="융합 포즈 (map→odom TF)\\n보정 스킵 구간은 오도메트리 관성 항법\\n+ 공분산 팽창", fillcolor="{C_ROBOT}", color="{E_ROBOT}"];
    plan [label="경로 생성\\n임무(통로 목록) → 그래프 탐색 → 경로점", fillcolor="{C_ROBOT}", color="{E_ROBOT}"];
    ctrl [label="경로 추종\\npure pursuit + 곡률 감속", fillcolor="{C_ROBOT}", color="{E_ROBOT}"];
    sa   [label="SafetyArbiter (최종 게이트)\\n비상정지·데드맨·장애물\\n주행경계·기울기 25°", fillcolor="{C_WARN}", color="{E_WARN}"];
    out  [label="cmd_vel", shape=oval, fillcolor="white", color="{E_ROBOT}"];

    sens -> ekf; sens -> reg;
    reg -> gate; gate -> fuse [label="정합 신뢰"];
    gate -> fuse [label="퇴화 → 스킵", style=dashed, color="{E_WARN}", fontcolor="{E_WARN}"];
    ekf -> fuse;
    fuse -> ctrl; plan -> ctrl; ctrl -> sa -> out;
  }}

  bundle -> reg [label="cloud.pcd"];
  bundle -> plan [label="graph.json"];
}}
'''

DOT_ORDER = ["fig1_arch", "fig2_comm", "fig3_cmd", "fig4_linkloss",
             "fig5_server", "fig6_field"]

for name in DOT_ORDER:
    dot = FIGS[name]
    src = OUT / f"{name}.dot"
    png = OUT / f"{name}.png"
    src.write_text(dot, encoding="utf-8")
    r = subprocess.run(["dot", "-Tpng", "-Gdpi=130", str(src), "-o", str(png)],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print(f"!! {name}: {r.stderr.strip()[:300]}", file=sys.stderr)
        sys.exit(1)
    print(f"{png}  ({png.stat().st_size//1024} KB)")

# ── 그림 7. 구현 로드맵 (matplotlib) ───────────────────────────────────────
import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

# .ttc 컬렉션은 matplotlib 이 못 읽으므로 KR 폰트를 추출해 등록한다
_KR_OTF = Path("/tmp") / "NotoSansCJKkr-Regular.otf"
if not any(F in f.name for f in font_manager.fontManager.ttflist):
    if not _KR_OTF.exists():
        from fontTools.ttLib import TTCollection
        tc = TTCollection("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
        for _f in tc.fonts:
            if "KR" in (_f["name"].getDebugName(4) or ""):
                _f.save(str(_KR_OTF))
                break
    font_manager.fontManager.addfont(str(_KR_OTF))

plt.rcParams["font.family"] = F
plt.rcParams["axes.unicode_minus"] = False

MS = [
    ("M1  관제 서버 코어",
     "계정·로그인 (Argon2)\n농장·로봇·사용자 모델\n임무·궤적·이벤트 이력 DB\n대시보드 로그인·이력 화면\n(로봇 링크는 기존\n WebSocket 직결 유지)",
     "#e4ecf7", "#3a6ea5"),
    ("M2  통신 전환 (Zenoh)",
     "zenohd ≥1.8 + mTLS + ACL\n+ MQTT 플러그인\n로봇 에이전트 Zenoh 전환\nstore-and-forward 버퍼\n단절 주입·재연결 스톰 검증\n(필수 통과 게이트)",
     "#e8f1e4", "#4c7d43"),
    ("M3  현장 자율주행",
     "맵 번들 파이프라인 승격\nEKF+NDT 로컬리제이션\n퇴화 게이트 (전방시야 대응)\n경로 생성·추종 E2E\n완주 기준: 9개 통로\n위치 오차 RMS < 0.3 m",
     "#fdf3dc", "#b8860b"),
    ("M4  통합 검증·확장 준비",
     "전체 회귀 (23+9+10 이식)\n문서·매뉴얼 갱신\nORO 공개 시 FleetPort\n 대체 평가\n실내 로봇: RMF fleet\n adapter (교통 협상 시점)",
     "#f3e9f5", "#8e5ba6"),
]

fig, ax = plt.subplots(figsize=(11.5, 4.6))
ax.set_xlim(0, 10); ax.set_ylim(0, 10); ax.axis("off")
w, gap = 2.28, 0.14
for i, (title, body, fc, ec) in enumerate(MS):
    x = 0.12 + i * (w + gap)
    ax.add_patch(FancyBboxPatch((x, 2.2), w, 6.4,
                 boxstyle="round,pad=0.06,rounding_size=0.14",
                 fc=fc, ec=ec, lw=1.6))
    ax.text(x + w/2, 7.9, title, ha="center", va="center",
            fontsize=12.5, fontweight="bold", color=ec)
    ax.text(x + 0.11, 7.0, body, ha="left", va="top", fontsize=8.6, linespacing=1.75)
    if i < len(MS) - 1:
        ax.annotate("", xy=(x + w + gap - 0.02, 5.4), xytext=(x + w + 0.02, 5.4),
                    arrowprops=dict(arrowstyle="-|>", lw=2.0, color="#666666"))
ax.text(0.12, 1.2, "각 마일스톤은 독립 검증 게이트를 통과해야 다음으로 진행 — 게이트 실패 시 범위 조정 후 재시도",
        fontsize=9.5, color="#555555")
ax.text(0.12, 9.6, "구현 로드맵 — 관제 우선 (사용자 지정 순서)", fontsize=14, fontweight="bold")
fig.tight_layout()
p = OUT / "fig7_roadmap.png"
fig.savefig(p, dpi=150, bbox_inches="tight", facecolor="white")
print(f"{p}  ({p.stat().st_size//1024} KB)")
print("완료")
