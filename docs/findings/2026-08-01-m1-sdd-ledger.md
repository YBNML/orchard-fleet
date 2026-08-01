# SDD ledger — plan: docs/superpowers/plans/2026-08-01-m1-fleet-server.md
Task 1: fix round 1/5 (2 findings dispatched — pytest ROS 플러그인 차단, 잔재 정리)
Task 1: fix round 1/5 (2 addressed, 0 open — pytest addopts ROS 플러그인 차단, 잔재 삭제·egg-info ignore; commits 3e36a32..1c14ed0)
Task 1: minor (deferred): test_config.py 불용 import os (브리프 원문 그대로)
Task 1: minor (deferred): offline_after_s 환경변수 오버라이드 없음 — 컨트롤러 판정: 의도됨 (스펙 §3.1 고정 상수)
Task 1: minor (deferred): env 오버라이드 테스트 커버리지 얇음 (브리프 명세 그대로)
Task 1: complete (commits ca7f9d5..1c14ed0, review clean)
Task 2: minor (deferred): FK 에 relationship/cascade 미정의 (인터페이스 요구 밖 — 후속 태스크 시 필요하면 추가)
Task 2: minor (deferred): Farm.name unique 미테스트 (계획 명세 밖)
Task 2: complete (commits 1c14ed0..8fcff33, review clean)
Task 3: minor (deferred): verify_password 가 TypeError(비문자열 해시) 미포착 — 단 models.pw_hash 는 NOT NULL 이라 실위험 낮음
Task 3: complete (commits 8fcff33..6ab748d, review clean)
Task 4: fix round 1/5 dispatched (1 open — 만료 세션 tz 경로 회귀 테스트 없음)
Task 4: minor (deferred): 로그인 실패 지연이 미존재 계정에서 더 빠름 (타이밍 부채널, 브리프 원문 패턴)
Task 4: minor (deferred): CSRF 비교가 상수시간 아님 (secrets.compare_digest 미사용)
Task 4: minor (deferred): 세션 토큰 DB 평문 저장 (해시 저장 고려 — 스펙 요구 밖)
Task 4: minor (deferred): auth.py 중간 import (브리프 원문 그대로, E402류)
Task 4: minor (deferred): require_min_role 403 분기 이 태스크 미실행 (Task 5 에서 실행됨)
Task 4: fix round 1/5 (1 addressed, 0 open — 만료 세션 회귀 테스트; commits 9cd14fe..f1a3af8)
Task 4: complete (commits 6ab748d..f1a3af8, review clean)
Task 5: minor (deferred): patch_robot 이 farm_id 재배정 시 농장 존재 미확인
Task 5: minor (deferred): create/patch_user 의 farm_ids 존재 미검증 (댕글링 스코프 행 허용)
Task 5: minor (deferred): 중복 Farm.name/Robot.id 생성 시 IntegrityError → 500 (4xx 미변환)
Task 5: minor (deferred): list_robots 의 admin 재계산 (scope is None 으로 단순화 가능)
Task 5: complete (commits f1a3af8..bfec522, review clean)
Task 6: fix round 1/5 dispatched (4 open — patch 2곳 감사 누락(Critical), 거부 경로 미기록, 행위자 신원 누락, 마스킹 다단어 유출)
Task 6: minor (deferred): 행위 커밋과 감사 커밋 비원자적 (브리프 패턴 유래 — 크래시 시 감사 공백 가능)
Task 6: minor (deferred): audit.record 호출부 들여쓰기 비정렬 (외관)
Task 6: fix round 1/5 (4 addressed, 0 open — patch 감사·거부 경로·행위자 신원·마스킹 강화; commits f33587c..f8f85e6)
Task 6: minor (deferred): patch_robot/patch_user 감사 detail 이 적용 키가 아닌 요청 키 전체 기록 (patch_farm 과 비대칭)
Task 6: complete (commits bfec522..f8f85e6, review clean)
Task 7: minor (deferred): PAUSED 에서 estop 재발동 부정 테스트 없음 (표 구성상 안전하나 명시 테스트 부재)
Task 7: minor (deferred): 다운스트림 주의 — 비 RUNNING 임무에 estop 이벤트 적용 시 InvalidTransition (Task 10 의 _sync_mission 이 catch, Task 9 의 verb 는 409 — 커버됨)
Task 7: complete (commits f8f85e6..b656ff2, review clean)
Task 8: fix round 1/5 dispatched (1 open — 상태 라우트 403/404 자동화 테스트 부재)
Task 8: minor (deferred): status 라우트 404가 403보다 먼저 — 로봇 ID 존재 스캔 가능 (계획 의사코드 그대로; 정보 은닉 필요 시 스코프 밖도 404 통일 고려)
Task 8: fix round 1/5 (1 addressed, 0 open — 403/404 회귀 테스트; commits 710e513..d2317a6)
Task 8: complete (commits b656ff2..d2317a6, review clean)
Task 9: fix round 1/5 dispatched (2 open — [Critical] 전달 실패에도 상태 커밋(계획 코드 결함, 전역 제약 우선 판정), [Important] 임무 거부 경로 감사 누락)
Task 9: minor (deferred): ingest._last_track_ts 프로세스 로컬 — 멀티워커/재시작 시 다운샘플 리셋 (M1 허용 한계)
Task 9: 설계 노트 (deferred): 영구 오프라인 로봇의 QUEUED 임무 정리 수단 없음 — M2 에서 서버 로컬 취소 정책 검토
Task 9: fix round 1/5 (2 addressed, 0 open — 전달 실패 상태 불변+409, 거부 경로 감사; commits fd264cc..cb35c53)
Task 9: minor (deferred): audit action 컬럼(String 64) 에 미검증 verb 삽입 — SQLite 는 무해, 엄격 DB 전환 시 절단 필요
Task 9: minor (deferred): 전이 사전검사가 TRANSITIONS 를 손동기화 (규칙이 dict 밖으로 자라면 헬퍼 통합 필요)
Task 9: minor (deferred): 거부 감사 '행' 내용 자체를 단언하는 테스트 없음 (상태코드·DB 상태로만 검증)
Task 9: complete (commits d2317a6..cb35c53, review clean)
Task 10: fix round 1/5 dispatched (4 open — [Critical] teleop payload cmd_id 주입(계획 코드 유래, 제약 우선), FleetService 무테스트, 무로깅 예외 삼킴, 레거시 등록 경로 CI 미실행)
Task 10: minor (deferred): shutdown 이 태스크 cancel 후 await 안 함 (teardown 노이즈 가능)
Task 10: minor (deferred): 토큰 URL 인코딩 없음 (&·= 포함 토큰 깨짐)
Task 10: minor (deferred): 토픽 파싱이 orchard 접두사·robot_id 일치 미검증
Task 10: fix round 1/5 (4 addressed, 0 open — teleop 순수성+실소켓 테스트, test_service.py 6종, 링크 로깅·예외 격리, 러닝루프 등록 테스트; commits b21c678..d8585cb)
Task 10: minor (deferred): on_message 예외 격리 동작 자체의 테스트 없음 (검사로만 확인)
Task 10: minor (deferred): stop() 이 소켓 강제 종료 안 함; 재연결 반복 시 warning 로그 소음
Task 10: complete (commits cb35c53..d8585cb, review clean)
Task 11: fix round 1/5 dispatched (4 open — [Critical] 임무 WS 거부 무감사, 스냅샷 삭제→무경쟁 재구현+단일라이터, 텔레옵 감사 시작시로, Origin fail-closed)
Task 11: minor (deferred): SESSION_COOKIE 불용 import, _Req 덕타이핑, 큐 포화 시 QueueFull 이 루프 로거로만, 접속 시 robot_farm/scope 스냅샷 고정(재접속 전 갱신 안 됨), send_task cancel 미await
Task 11: fix round 1/5 (4 addressed, 0 open — 임무거부 감사, 스냅샷 복원+단일라이터, 텔레옵 시작 감사, Origin fail-closed; commits ee2cb64..f120f63)
Task 11: minor (deferred): mission_* 거부 감사에 target=robot 누락; 스냅샷 루프 QueueFull 미처리(>1000 항목 시); GIL 프리엠션 이론상 스냅샷-구독 창 (실배치 스레드 유입 시)
Task 11: complete (commits d8585cb..f120f63, review clean)
Task 14 체크리스트 메모: FLEET_ALLOWED_ORIGINS 미설정 시 WS 전면 차단(fail-closed) — compose 기본값 필수
Task 12: fix round 1/5 dispatched (4 open — [Critical] applyFeatures 가 역할게이트 덮어씀, cmd_result 미처리, readyState 가드 없음, missionId 새로고침 소실)
Task 12: fix round 1/5 (4 addressed, 0 open — elementAllowed 통합게이트, cmd_result 표시, srvReady 가드, restoreMissions; commits 9f880ec..3a54a74)
Task 12: minor (deferred): GET /missions limit 200 — 200건 밖 활성 PAUSED 임무는 복원 실패 가능; restoreMissions 가 id desc 정렬에 암묵 결합
Task 12: complete (commits f120f63..3a54a74, review clean)
Task 13: minor (deferred): refreshHistory/refreshUsers 미await — 보고서의 try/catch 설명 부정확 (동작 무해)
Task 13: minor (deferred): hist-clear 가 이벤트 타임라인은 안 지움 (브리프 원문 그대로)
Task 13: complete (commits 3a54a74..3527114, review clean)
Task 14: fix round 1/5 dispatched (1 open — [Critical] 비-editable 설치에서 web_dir 부재 → 컨테이너 대시보드 서빙 불가; FLEET_WEB_DIR 명시로 수정)
Task 14: minor (deferred): 33/34 스크립트 보일러플레이트 ~90줄 중복 (기존 17/21/30 관례와 일치 — 공용 헬퍼는 후속)
Task 14: minor (deferred): Dockerfile root 실행·HEALTHCHECK 없음
Task 14: 판정 기록: uvicorn 이 pre-accept close 코드를 HTTP 403 으로 뭉갬(구조적) / 감사 무결성 4개 경로 스코프 축소 / docker 미설치 SKIP — 3건 모두 검토자 독립 검증으로 타당 확인
Task 14: fix round 1/5 (1 addressed, 0 open — FLEET_WEB_DIR 명시 + 비-editable 재현·검증; commits f08d4cb..5c18668)
Task 14: complete (commits 3527114..5c18668, review clean) — M1 게이트 3종 통과 (단위 79 / E2E 24 / 보안 25, 각 2회)
== 전 태스크 14/14 complete — 최종 전체 브랜치 리뷰 진행 ==

== 최종 전체 브랜치 리뷰 (fable) ==
최종리뷰: Critical 2 (대시보드 XSS 체인, tz-naive 직렬화로 KST 이력재생 실패) + Important 4 발견
최종수정 웨이브: 1회 (commits 5c18668..ba0fe5b, 4커밋) — 전 항목 ADDRESSED (재검토 fable)
최종수정 부가: patch_user 검증 순서 재배치 (audit.record 내부 커밋이 거부 요청의 변경을 유출하던 원자성 결함) — 재검토 VALID 판정
parked: ws.py _deny_and_close 의 queue.join() — 거부와 급작 종료가 겹치면 sender 가 task_done 없이 죽어 영구 대기·구독 누수 가능 (Minor, 2차 수정 웨이브 없음 정책)
  ruling: 실재하나 좁은 레이스이고 M2 에서 asyncio.wait_for 로 1줄 수정 예정 — M2 백로그 1순위
게이트 최종 (컨트롤러 직접 실행): 단위 97 passed / E2E 29/29 / 보안 29/29
