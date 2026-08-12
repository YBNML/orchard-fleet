"""Diag SDK 구현 — 자가진단(self_test) 5항목.

판정 기준 (스펙 §2)
    lidar       수신율 ≥ 8 Hz  (RosSensors.rates()["lidar"])
    imu         수신율 ≥ 80 Hz (RosSensors.rates()["imu"])
    localizer   포즈가 있고 quality ≥ 0.3
    link        관제 연결이 최근(<1.5 s)인가 — SafetyArbiter.snapshot()["link_ok"]
                (update_link 이 이미 P.LINK_LOSS_STOP_MS=1500 ms 기준으로 재 놓은
                값을 그대로 읽는다. 여기서 시각을 다시 재지 않는 이유: 판정
                기준이 안전 게이트와 진단 두 곳에서 따로 계산되면 언젠가
                갈라진다 — 안전 조정자가 **실제로 링크 정지를 거는 데 쓰는**
                값을 그대로 읽어야 self_test 의 link 항목이 항상 안전 정지와
                같은 답을 낸다)
    drive       구동 한계가 유효(v_max > 0) — SDK Drive.limits()

**움직임을 만들지 않는다.** 이 파일에 속도를 발행하는 코드가 없다 — drive
항목은 limits() 조회뿐이고, 다른 네 항목도 전부 읽기만 한다.

blackbox_dump 는 스펙 ② T4 전까지 NotImplementedError — 조용히 빈 사전을
돌려주면 관제가 "덤프가 됐는데 비었다"로 오인한다(reinit 의 관례와 같다).
"""
from __future__ import annotations

from robomw.sdk.interfaces import Diag
from robomw.sdk.types import SelfTestItem

LIDAR_MIN_HZ = 8.0
IMU_MIN_HZ = 80.0
LOCALIZER_MIN_QUALITY = 0.3


class ScoutDiag(Diag):
    """시뮬레이터(scout_mini) 용 Diag 구현.

    sensors(RosSensors)·safety(SafetyArbiter)·drive(RosDrive) 는 control_agent
    가 이미 만든 인스턴스를 그대로 건넨다 — 여기서 새로 만들지 않는다(상태를
    두 곳에 중복해서 갖지 않기 위해서다. cloud_world 전례와 같은 배선 방향).
    """

    ITEMS = ("lidar", "imu", "localizer", "link", "drive")

    def __init__(self, sensors, safety, drive, robot_id=""):
        self._sensors = sensors
        self._safety = safety
        self._drive = drive
        self.robot_id = robot_id
        self._checks = {
            "lidar": self._check_lidar,
            "imu": self._check_imu,
            "localizer": self._check_localizer,
            "link": self._check_link,
            "drive": self._check_drive,
        }

    def self_test(self, items=None):
        """자가진단 실행. items 가 없으면(None/빈 값) 지원 전 항목을 검사한다."""
        names = list(items) if items else list(self.ITEMS)
        out = []
        for name in names:
            fn = self._checks.get(name)
            out.append(fn() if fn is not None
                       else SelfTestItem(name, False, "알 수 없는 항목"))
        return out

    def blackbox_dump(self, window_s: float) -> dict:
        raise NotImplementedError("blackbox_dump 미구현 — 스펙 ② T4")

    # ── 항목별 판정 ─────────────────────────────────────────────────────────
    def _check_lidar(self):
        hz = self._sensors.rates().get("lidar", 0.0)
        return SelfTestItem("lidar", hz >= LIDAR_MIN_HZ, f"{hz:.1f} Hz")

    def _check_imu(self):
        hz = self._sensors.rates().get("imu", 0.0)
        return SelfTestItem("imu", hz >= IMU_MIN_HZ, f"{hz:.1f} Hz")

    def _check_localizer(self):
        pose = self._sensors.pose()
        if pose is None:
            return SelfTestItem("localizer", False, "포즈 없음")
        ok = pose.quality >= LOCALIZER_MIN_QUALITY
        return SelfTestItem("localizer", ok, f"quality={pose.quality:.2f}")

    def _check_link(self):
        ok = bool(self._safety.snapshot().get("link_ok"))
        return SelfTestItem("link", ok, "정상" if ok else "끊김(1.5s 초과)")

    def _check_drive(self):
        lim = self._drive.limits()
        ok = lim.v_max > 0.0
        return SelfTestItem("drive", ok, f"v_max={lim.v_max:.2f} w_max={lim.w_max:.2f}")
