"""이관됨 → robomw.profiles.orchard.mission (스펙 ①, 2026-08-11). 이 shim 은
기존 스크립트 호환용 — 새 코드는 robomw 를 직접 import 할 것.

**레지스트리 적재는 이 경로로 하지 않는다.** 레지스트리는 모듈 안에서 정의된
Feature 하위 클래스만 찾으므로(재수출은 __module__ 이 다르다) 여기서는 적재가
안 된다. 옛 이름('drive_mission')으로 적힌 기능 목록은 control_agent 가 새
경로로 바꿔 준다.
"""
from robomw.profiles.orchard.mission import *            # noqa: F401,F403
from robomw.profiles.orchard.mission import DriveMission  # noqa: F401
