def test_index_served_with_login(client):
    html = client.get("/").text
    assert "로그인" in html            # 로그인 오버레이 존재
    assert "관제 v2" in html           # v2 마커
    assert "?token=" not in html       # 로봇 직결 토큰 방식 제거됨
    assert "elementAllowed" in html    # 역할 게이트 통합 (applyFeatures 가 덮어쓰지 않음) 회귀 마커
    assert "cmd_result" in html        # WS cmd_result 처리 회귀 마커


def test_history_and_user_admin_present(client):
    html = client.get("/").text
    assert "이력" in html and "사용자 관리" in html
    assert 'data-min-role="admin"' in html


def test_xss_escape_helper_applied_at_robot_derived_sinks(client):
    """로봇 유래 문자열(이벤트 msg·기능 이름·헬스 경고·모드·로봇 id)을 innerHTML 로
    렌더하는 자리마다 esc() 이스케이프가 적용돼 있는지 정적으로 확인한다
    (Critical: 대시보드 XSS 체인 회귀)."""
    html = client.get("/").text
    assert "function esc(" in html
    for marker in ("esc(r.id)", "esc(e.msg)", "esc(f.name)", "esc(f.summary)",
                  "esc(w)", "esc(v)", "esc(modeName(r.state.mode))"):
        assert marker in html, f"이스케이프 누락 의심: {marker}"
