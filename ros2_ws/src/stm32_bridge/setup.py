"""ament_python 打包脚本。

★ 注意：``libomninav_core.so`` **不由本包安装** —— 按契约 §5.3 它有三条查找路径，
   树莓派上的正式位置是 ``/opt/omninav/lib/libomninav_core.so``，由 ``pi_setup/install_core.sh``
   安装（架构必须是树莓派本地编出来的 aarch64）。本包只负责 Python 代码。
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
    description="OmniNav STM32 <-> 树莓派 5 串口桥（ROS2 Jazzy）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stm32_bridge = stm32_bridge.bridge_node:main",
        ],
    },
)
