"""SDK 기본 자료형."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose:
    """로봇 포즈.

    Attributes:
        x: 지도 프레임 x (m)
        y: 지도 프레임 y (m)
        yaw: 헤딩 (rad)
        quality: 측위 신뢰도 (0.0 ~ 1.0, 기본 1.0)
    """
    x: float
    y: float
    yaw: float
    quality: float = 1.0


@dataclass(frozen=True)
class DriveLimits:
    """구동 속도 한계.

    Attributes:
        v_max: 선속도 최대 (m/s)
        w_max: 각속도 최대 (rad/s)
    """
    v_max: float
    w_max: float


@dataclass(frozen=True)
class SelfTestItem:
    """자진단 항목.

    Attributes:
        name: 항목명
        ok: 통과 여부
        detail: 상세 메시지
    """
    name: str
    ok: bool
    detail: str


@dataclass(frozen=True)
class WorkStatus:
    """작업 상태.

    Attributes:
        active: 진행 중 여부 (기본 False)
        type: 작업 종류 (기본 "")
        progress: 진행률 (0.0 ~ 1.0, 기본 0.0)
        detail: 상세 메시지 (기본 "")
    """
    active: bool = False
    type: str = ""
    progress: float = 0.0
    detail: str = ""
