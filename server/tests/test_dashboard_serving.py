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
