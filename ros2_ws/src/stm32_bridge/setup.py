"""ament_python packaging script.

★ Note: ``libomninav_core.so`` is **not installed by this package** — per contract §5.3 it has three
   lookup paths, and its production location on the Raspberry Pi is ``/opt/omninav/lib/libomninav_core.so``,
   installed by ``pi_setup/install_core.sh`` (it must be built natively on the Pi as aarch64). This package
   only ships the Python code.
"""

from setuptools import setup

package_name = "stm32_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OmniNav",
    maintainer_email="omninav@example.invalid",
    description="OmniNav STM32 <-> Raspberry Pi 5 serial bridge (ROS2 Jazzy)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stm32_bridge = stm32_bridge.bridge_node:main",
            "wheel_joint_states = stm32_bridge.wheel_joint_states:main",
        ],
    },
)
