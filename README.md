# OmniNav · 智能仓储麦轮导航机器人（代码仓库）

面向 20 m² 室内仓储的麦轮自主导航小车：STM32F103ZET6 硬实时底盘（整数定点、无 FPU）+ 树莓派 5（Ubuntu 24.04 + ROS2 Jazzy，2 GB）。

> 本仓库**只放代码**。项目文档、报告、笔记、实测日志暂不纳入版本库（保留在本地工作区）。

## 目录

| 目录 | 内容 |
|---|---|
| `firmware_core/` | 纯逻辑算法层：定点运算、麦轮运动学、里程计、PID、协议（C99，无 HAL，PC 可编可测） |
| `stm32_fw/` | STM32F103 固件：Keil MDK 工程、HAL 层、自检、bench、USART2、单轮安全固件 |
| `ros2_ws/` | ROS2 Jazzy 包 `stm32_bridge`（`/cmd_vel` ↔ 串口 ↔ `/odom` + TF）及零硬件工具链（假 STM32 / e2e / 跨语言对拍） |
| `pi_setup/` | 树莓派联调与环境自检脚本（udev、端口识别、`.so` 安装、preflight、雷达检查） |
| `qa/` | 可复跑的门禁脚本 |

## 复现门禁（WSL Ubuntu 22.04 + ROS2 Humble）

| 检查 | 命令 |
|---|---|
| 算法层四门禁 | `cd firmware_core && make check-purity && make lib && make test` |
| 协议层回归 | `python3 ros2_ws/src/stm32_bridge/test/test_protocol.py` |
| C↔Python 跨语言对拍 | `python3 ros2_ws/tools/check_core.py` |
| 零硬件端到端 | `python3 ros2_ws/tools/run_e2e.py` |
| 一键复跑 | `bash qa/run_core_tests.sh`、`bash qa/run_py_tests.sh` |

## 约定

- 串口整数文本协议：线速度 mm/s、角速度 mrad/s；`firmware_core/src` 与 `include` 禁止引入 HAL/ROS2/第三方头。
- 目标平台：ROS2 Jazzy，`ROS_DOMAIN_ID=42`，`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`。
- 私有工程仓库：含硬件细节，请勿公开。
