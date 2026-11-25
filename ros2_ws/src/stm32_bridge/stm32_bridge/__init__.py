"""stm32_bridge — serial bridge between STM32F103 (USART2) and Raspberry Pi 5 (ROS2 Jazzy).

Division of work between the three modules in this package (per the T2 task card):

==================  ==========================================================
``protocol``        Pure encoding / parsing. **Does not import rclpy**, so its
                    unit tests also run on Windows / WSL without ROS2 installed.
``kinematics``      Loads ``libomninav_core.so`` via ``ctypes`` and wraps the C
                    symbols of ``firmware_core`` as Python functions. **No math here**.
``bridge_node``     The ``rclpy`` node (the only place that imports ROS2).
==================  ==========================================================

★ This ``__init__.py`` **deliberately imports nothing**: if ``bridge_node`` were imported here,
``import stm32_bridge.protocol`` would drag in ``rclpy`` as well, and the acceptance criterion
"unit tests run on Windows" would immediately break.
"""

__version__ = "0.1.0"

__all__ = ["protocol", "kinematics", "bridge_node"]
