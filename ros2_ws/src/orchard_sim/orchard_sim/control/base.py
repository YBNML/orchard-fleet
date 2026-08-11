"""이관됨 → robomw.core.base (스펙 ①, 2026-08-11). 이 shim 은 기존
스크립트·서버 호환용 — 새 코드는 robomw 를 직접 import 할 것."""
from robomw.core.base import *                  # noqa: F401,F403
from robomw.core.base import Blackboard, Context, VelocityRequest  # noqa: F401  (안전핀 — 별표가 놓칠 이름 없음을 재확인)
import robomw.core.base as _p


def __getattr__(name):                          # 별표에 안 잡히는 밑줄 이름 위임
    return getattr(_p, name)
