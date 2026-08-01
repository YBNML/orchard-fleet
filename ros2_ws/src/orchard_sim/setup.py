from setuptools import find_packages, setup

package_name = "orchard_sim"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch",
         ["launch/livox_bridge.launch.py", "launch/stage0.launch.py",
          "launch/stage7.launch.py", "launch/control.launch.py"]),
        ("share/" + package_name + "/web", ["web/index.html"]),
        ("share/" + package_name + "/config", ["config/livox_bridge.yaml", "config/bridge_core.yaml",
          "config/bridge_cameras.yaml", "config/robot_check.rviz",
          "config/fastlio_mid70.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="myhome",
    maintainer_email="rla1231013@gmail.com",
    description="과수원 시뮬레이션 — Livox 인터페이스 브리지",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "livox_sim_bridge = orchard_sim.livox_sim_bridge:main",
            "sdf_static_tf = orchard_sim.sdf_static_tf:main",
            "gt_localizer = orchard_sim.gt_localizer:main",
            "map_localizer = orchard_sim.map_localizer:main",
            "mapping_run = orchard_sim.mapping_run:main",
            "lio_recorder = orchard_sim.lio_recorder:main",
            "control_agent = orchard_sim.control_agent:main",
        ],
    },
)
