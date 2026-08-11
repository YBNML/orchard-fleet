"""SDK 추상 인터페이스."""
import abc
from robomw.sdk.types import DriveLimits, Pose, SelfTestItem, WorkStatus


class Drive(abc.ABC):
    """구동 제어 인터페이스."""

    @abc.abstractmethod
    def set_velocity(self, v: float, w: float) -> None:
        """속도 명령.

        Args:
            v: 선속도 (m/s)
            w: 각속도 (rad/s)
        """
        pass

    @abc.abstractmethod
    def stop(self) -> None:
        """즉시 정지."""
        pass

    @abc.abstractmethod
    def limits(self) -> DriveLimits:
        """구동 한계 조회.

        Returns:
            DriveLimits: v_max, w_max
        """
        pass


class Localizer(abc.ABC):
    """측위 인터페이스."""

    @abc.abstractmethod
    def pose(self) -> Pose | None:
        """현재 포즈 조회.

        Returns:
            Pose: 측정 포즈 (품질 정보 포함)
            None: 측위 미초기화 또는 신호 손실
        """
        pass

    @abc.abstractmethod
    def reinit(self, pose: Pose) -> None:
        """포즈 재초기화.

        Args:
            pose: 새로운 포즈 (지도 프레임)
        """
        pass

    @abc.abstractmethod
    def diagnostics(self) -> dict:
        """진단 정보 조회.

        Returns:
            dict: 키는 'bias_x', 'bias_y' 등 — 실수 또는 상태 문자열
        """
        pass


class Perception(abc.ABC):
    """지각(센서 관찰) 인터페이스."""

    @abc.abstractmethod
    def clearance(self) -> float:
        """전방 개활 거리.

        Returns:
            float: 미터 단위 거리. float("inf")는 개활 (장애 없음).
        """
        pass

    @abc.abstractmethod
    def near_frac(self) -> float:
        """근처 점 비율.

        Returns:
            float: 0.0 ~ 1.0 사이 점유율
        """
        pass


class Work(abc.ABC):
    """작업 관리 인터페이스."""

    @abc.abstractmethod
    def start(self, type_: str, params: dict) -> None:
        """작업 시작.

        Args:
            type_: 작업 종류 ("survey", "harvest", ...)
            params: 작업 파라미터
        """
        pass

    @abc.abstractmethod
    def stop(self) -> None:
        """작업 중단."""
        pass

    @abc.abstractmethod
    def status(self) -> WorkStatus:
        """작업 상태 조회.

        Returns:
            WorkStatus: 진행률, 상태 등
        """
        pass


class Diag(abc.ABC):
    """진단 인터페이스."""

    @abc.abstractmethod
    def self_test(self, items: list[SelfTestItem]) -> None:
        """자진단 항목 기록.

        Args:
            items: SelfTestItem 목록
        """
        pass

    @abc.abstractmethod
    def blackbox_dump(self, window_s: float) -> dict:
        """블랙박스 덤프.

        Args:
            window_s: 과거 몇 초치 기록을 뽑을지

        Returns:
            dict: 기록 데이터
        """
        pass
