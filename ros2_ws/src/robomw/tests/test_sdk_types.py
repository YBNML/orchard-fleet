import math
import pytest
from robomw.sdk.types import DriveLimits, Pose, SelfTestItem, WorkStatus
from robomw.sdk.interfaces import Diag, Drive, Localizer, Perception, Work


def test_pose_fields():
    p = Pose(1.0, -2.0, math.pi / 2, 0.8)
    assert (p.x, p.y) == (1.0, -2.0) and 0.0 <= p.quality <= 1.0


def test_interfaces_are_abstract():
    for cls in (Drive, Localizer, Perception, Work, Diag):
        with pytest.raises(TypeError):
            cls()


def test_minimal_drive_impl():
    class D(Drive):
        def __init__(self): self.last = None
        def set_velocity(self, v, w): self.last = (v, w)
        def stop(self): self.last = (0.0, 0.0)
        def limits(self): return DriveLimits(0.7, 1.0)
    d = D(); d.set_velocity(0.5, 0.1)
    assert d.last == (0.5, 0.1) and d.limits().v_max == 0.7
