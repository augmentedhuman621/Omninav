#!/usr/bin/env bash
# Q-01：在本机 WSL（Ubuntu-22.04 + ROS2 Humble）尝试复跑 ros2_ws 的三类检查
#   1) test_protocol.py —— 纯 Python，零 ROS2 依赖
#   2) check_core.py    —— 需要 firmware_core/build/libomninav_core.so（x86-64）
#   3) run_e2e.py       —— 需要 rclpy（WSL 里是 Humble，不是树莓派的 Jazzy）
# 只读源码；不改任何实现文件；日志写到 qa/evidence/。
set -o pipefail
WS=/mnt/d/dsh/工程设计
RWS=$WS/ros2_ws
EVID=$WS/qa/evidence
mkdir -p "$EVID"
LOG=$EVID/q01_ros2_ws_rerun.log
{
  echo "### date(UTC+8): $(TZ=Asia/Shanghai date -Is)"
  echo "### python3: $(python3 --version 2>&1)"
  echo "### ROS_DISTRO before source: ${ROS_DISTRO:-<unset>}"
  if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi
  echo "### ROS_DISTRO after source : ${ROS_DISTRO:-<unset>}"
  echo "### ros2: $(command -v ros2 || echo MISSING)"
  echo "### rclpy: $(python3 -c 'import rclpy; print(rclpy.__file__)' 2>&1 | head -1)"
  echo "### pyserial: $(python3 -c 'import serial; print(serial.__version__)' 2>&1 | head -1)"
  echo
  echo "=========== 1) py_compile 全部 .py ==========="
  ( cd "$RWS" && python3 -m py_compile \
      src/stm32_bridge/stm32_bridge/protocol.py \
      src/stm32_bridge/stm32_bridge/kinematics.py \
      src/stm32_bridge/stm32_bridge/bridge_node.py \
      src/stm32_bridge/stm32_bridge/__init__.py \
      src/stm32_bridge/test/test_protocol.py \
      tools/check_core.py tools/fake_stm32.py tools/run_e2e.py \
      ../pi_setup/test_stm32_link.py ../pi_setup/shadow_probe.py \
    && echo PY_COMPILE_OK ); echo "PY_COMPILE_EXIT=$?"
  echo
  echo "=========== 2) test_protocol.py（纯 Python 单测） ==========="
  ( cd "$RWS/src/stm32_bridge/test" && python3 test_protocol.py ); echo "TEST_PROTOCOL_EXIT=$?"
  echo
  echo "=========== 3) check_core.py（.so 交接验收） ==========="
  ( cd "$RWS" && OMNINAV_CORE_SO=$WS/firmware_core/build/libomninav_core.so python3 tools/check_core.py ); echo "CHECK_CORE_EXIT=$?"
  echo
  echo "=========== 4) run_e2e.py（pty 假 STM32 + 真 rclpy，需 ROS2） ==========="
  ( cd "$RWS" && OMNINAV_CORE_SO=$WS/firmware_core/build/libomninav_core.so timeout 180 python3 tools/run_e2e.py ); echo "RUN_E2E_EXIT=$?"
} 2>&1 | tee "$LOG"
echo "LOGPATH=$LOG"
