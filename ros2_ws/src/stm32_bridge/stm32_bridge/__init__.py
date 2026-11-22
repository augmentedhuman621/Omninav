"""stm32_bridge —— STM32F103（USART2）<-> 树莓派 5（ROS2 Jazzy）的串口桥。

包内三个模块的分工（照 T2 任务卡）：

==================  ==========================================================
``protocol``        纯编码 / 解析。**不 import rclpy**，所以在没装 ROS2 的
                    Windows / WSL 上也能直接跑单测。
``kinematics``      ``ctypes`` 加载 ``libomninav_core.so``，把 ``firmware_core``
                    的 C 符号包成 Python 函数。**不做任何数学**。
``bridge_node``     ``rclpy`` 节点（唯一 import ROS2 的地方）。
==================  ==========================================================

★ 本 ``__init__.py`` **故意什么都不 import**：一旦在这里 import ``bridge_node``，
``import stm32_bridge.protocol`` 就会连带拉进 ``rclpy``，
"Windows 上能跑单测" 这条验收标准立刻失效。
"""

__version__ = "0.1.0"

__all__ = ["protocol", "kinematics", "bridge_node"]
