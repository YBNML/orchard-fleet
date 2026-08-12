from setuptools import find_packages, setup

setup(
    name="robomw",
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/robomw"]),
        ("share/robomw", ["package.xml"]),
    ],
    zip_safe=True,
    description="로봇측 미들웨어 — 명령 계약·안전·링크 (ROS 비의존 코어)",
    license="Apache-2.0",
)
