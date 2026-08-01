"""시각 직렬화 — SQLite 왕복 후 tz 정보가 사라지는 문제를 서버측에서 통일 보정한다.

`DateTime(timezone=True)` 컬럼이라도 SQLite 는 tzinfo 를 저장하지 않는다. 같은
세션에서 방금 만든 객체는(expire_on_commit=False) 파이썬이 넣은 tzinfo 를 그대로
들고 있지만, 다른 요청(다른 세션)이 다시 조회하면 naive(tz 없는) datetime 이
돌아온다. naive 에 그대로 `.isoformat()` 을 쓰면 접미사 없는 문자열이 나가고,
대시보드의 `Date.parse()` 가 이를 로컬시간(KST)으로 오해석해 실제 UTC 값과
9시간 어긋난다(이력 재생용 from_ts/to_ts 계산이 빈 결과를 냄). 저장은 always UTC
이므로, naive 는 UTC 로 간주하고 접미사를 붙여 돌려준다.
"""
from __future__ import annotations

import datetime as dt


def iso_utc(dt_: dt.datetime | None) -> str | None:
    """datetime → ISO8601 문자열. naive 는 UTC 로 간주해 접미사를 붙인다."""
    if dt_ is None:
        return None
    return (dt_ if dt_.tzinfo else dt_.replace(tzinfo=dt.UTC)).isoformat()
