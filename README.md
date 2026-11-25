# OmniNav · 小车运行代码主目录

卡内基 20 m² 室内仓储麦轮自主导航小车：**STM32F103ZET6 硬实时底盘**（C99 整数定点、无 FPU、无 HAL）＋ **树莓派 5**（Ubuntu 24.04 + ROS2 Jazzy，2 GB）。

> **这是小车代码的唯一主目录（2026-11-25 起）。**
> 以后所有代码改动都在 `E:\工程设计小车代码` 里做；`D:\dsh\工程设计` 里的代码副本是历史快照（冻结，不再同步），那里只保留文档、报告、实测证据。
>
> 目录名保留英文，是为了 Keil MDK 和 colcon 的路径兼容（Keil 对中文路径支持不好，改中文目录名会导致工程打不开/编译报错）。每个目录的中文含义、职责、入口、编译方式**写在各目录的 `_模块说明.md`**，见下表。

## 模块地图（按「跑在哪块板子」分）

| 目录 | 中文名 | 跑在哪 | 入口 / 关键文件 | 改了会影响谁 |
|---|---|---|---|---|
| `firmware_core/` | **算法核心**：定点运算、麦轮运动学、里程计、PID、串口协议 | 两端共用：编进 STM32 固件；PC/树莓派编成 `libomninav_core.so` 供对拍 | `include/*.h`、`src/*.c`、`Makefile` | 固件 + Python 对拍；7 个导出符号是**冻结接口** |
| `stm32_fw/` | **STM32 底盘固件**：HAL 适配、自检、bench、USART2 链路、单轮安全调试 | STM32F103ZET6（Keil MDK 5.34 + ST-Link 烧录） | `MDK-ARM/OmniNav_STM32.uvprojx` → `Src/main.c` | 上电行为、串口帧、电机安全 |
| `ros2_ws/src/stm32_bridge/` | **树莓派 ROS2 桥**：`/cmd_vel` ↔ 串口帧 ↔ `/odom` + TF | 树莓派 5（ROS2 Jazzy） | `stm32_bridge/bridge_node.py`（`bridge_node:main`） | 上位机导航唯一入口 |
| `ros2_ws/src/omninav_sim/` | **Gazebo 仿真包**（新增，未提交 git）：URDF + world + launch | 虚拟机 / PC（Gazebo Classic 11） | `ros2 launch omninav_sim gazebo.launch.py` | 只影响仿真，不动真机协议 |
| `ros2_ws/tools/` | **零硬件开发工具**：假 STM32、端到端、跨语言对拍 | PC（WSL / Windows） | `run_e2e.py`、`fake_stm32.py`、`check_core.py` | 只是测试工具，不参与整车运行 |
| `pi_setup/` | **树莓派上机前脚本**：udev 规则、串口识别、`.so` 安装、preflight、雷达检查 | 树莓派 5 | `preflight.sh`、`install_core.sh`、`check_lidar.sh` | 上机环境，不改算法 |
| `qa/` | **测试门禁脚本**：算法四门禁、从零构建、Python 回归 | PC（WSL Ubuntu 22.04） | `run_core_tests.sh`、`run_py_tests.sh` | 只读校验，不改产品代码 |

## 每次改完必须过的门禁（WSL 里跑）

| 检查 | 命令（在仓库根目录） |
|---|---|
| 算法层四门禁 | `cd firmware_core && make check-purity && make lib && make test` |
| 协议层回归 | `python3 ros2_ws/src/stm32_bridge/test/test_protocol.py` |
| C↔Python 跨语言对拍 | `python3 ros2_ws/tools/check_core.py` |
| 零硬件端到端 | `python3 ros2_ws/tools/run_e2e.py` |
| 一键全跑 | `bash qa/run_core_tests.sh && bash qa/run_py_tests.sh` |

## 改动规则（硬性）

1. **只在本目录改代码**。`D:\dsh\工程设计` 下的同名目录已冻结，不要两边都改（会造成版本分叉）。
2. `firmware_core` 的 7 个导出符号（`mecanum_forward`/`mecanum_inverse`/`mecanum_l_mm`/`mecanum_default_geom`/`mecanum_geom_is_valid`/`odom_update`/`odom_reset`）是冻结接口，**不得改名、改签名、改单位**；要改先改接口契约。
3. `firmware_core/src`、`firmware_core/include` **禁止** include HAL / ROS2 / 第三方头；算法层**禁止浮点**。
4. 串口协议：整数文本，线速度 mm/s、角速度 mrad/s，帧格式按《11_接口契约》§四，不要新增帧。
5. 目录按模块分开，一个目录只放一类代码；新增模块要建新目录并补 `_模块说明.md`。
6. STM32 改动必须在 Keil 里编译通过才算完成；烧录只走 ST-Link（Keil Flash Download 不认 CH340）。
7. 提交信息写清「改了什么、为什么」；提交前门禁必须全绿。

## 仓库信息

| 项 | 值 |
|---|---|
| 远端 | `git@github.com:augmentedhuman621/Omninav.git`（私有） |
| 分支 / HEAD | `master` / `b89b22e` |
| 代码文件数 | 69（git 跟踪）＋ 仿真包 7 个文件待提交 |
| 配套文档 | `D:\dsh\工程设计\`：`00`–`21` 设计文档、`reports/`、`qa/evidence/`、`CURRENT_STATE.md` |
| 模型与网格 | `ros2_ws/src/omninav_sim/meshes/`；SolidWorks 母版 `E:\(D305)酷点机器人-4WD四驱底盘\...\总体工程.SLDASM` |
