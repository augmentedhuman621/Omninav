#!/usr/bin/env bash
# Q-01: on this WSL box (Ubuntu-22.04 + ROS2 Humble), try to re-run the three checks of ros2_ws
#   1) test_protocol.py - pure Python, zero ROS2 dependencies
#   2) check_core.py    - needs firmware_core/build/libomninav_core.so (x86-64)
#   3) run_e2e.py       - needs rclpy (Humble in WSL, not Jazzy as on the Pi)
# Read-only on source; does not modify any implementation file; logs are written to qa/evidence/.
set -o pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
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
  echo "=========== 1) py_compile all .py ==========="
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
  echo "=========== 2) test_protocol.py (pure Python unit tests) ==========="
  ( cd "$RWS/src/stm32_bridge/test" && python3 test_protocol.py ); echo "TEST_PROTOCOL_EXIT=$?"
  echo
  echo "=========== 3) check_core.py (.so handoff acceptance) ==========="
  ( cd "$RWS" && OMNINAV_CORE_SO=$WS/firmware_core/build/libomninav_core.so python3 tools/check_core.py ); echo "CHECK_CORE_EXIT=$?"
  echo
  echo "=========== 4) run_e2e.py (pty fake STM32 + real rclpy, needs ROS2) ==========="
  ( cd "$RWS" && OMNINAV_CORE_SO=$WS/firmware_core/build/libomninav_core.so timeout 180 python3 tools/run_e2e.py ); echo "RUN_E2E_EXIT=$?"
} 2>&1 | tee "$LOG"
echo "LOGPATH=$LOG"
