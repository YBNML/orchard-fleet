def test_index_served_with_login(client):
    html = client.get("/").text
    assert "로그인" in html            # 로그인 오버레이 존재
    assert "관제 v2" in html           # v2 마커
    assert "?token=" not in html       # 로봇 직결 토큰 방식 제거됨
