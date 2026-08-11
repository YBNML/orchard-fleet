"""robomw 는 ROS 를 모른다 — 코어 격리 상시 검사 (스펙 §1)."""
import pathlib
import re

PKG = pathlib.Path(__file__).resolve().parent.parent / "robomw"
BANNED = re.compile(r"^\s*(import|from)\s+(rclpy|rcl_interfaces|std_msgs|geometry_msgs|sensor_msgs)")


def test_no_ros_imports():
    hits = []
    for p in PKG.rglob("*.py"):
        for i, line in enumerate(p.read_text(encoding="utf-8").splitlines(), 1):
            if BANNED.match(line):
                hits.append(f"{p}:{i}: {line.strip()}")
    assert not hits, "robomw 안에 ROS import:\n" + "\n".join(hits)


def test_protocol_importable_without_ros():
    import robomw.link.protocol as P
    assert P.PROTOCOL_VERSION == 1
