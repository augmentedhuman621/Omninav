#!/usr/bin/env bash
# Q-03 的 B-02 部分：复核 fake_stm32.py 修复后的 e2e 可复现性（只读，不改任何工作区源码）
# 用法：在 WSL Ubuntu-22.04 下执行 bash qa/run_q03_b02.sh
# 输出：qa/evidence/q03_b02_rerun.log（tee）
# 注意：ROS2 Humble 的 setup.bash 引用未定义变量，不能开 -u
set -o pipefail

ROOT="/mnt/d/dsh/工程设计"
WS="$ROOT/ros2_ws"
LOG="$ROOT/qa/evidence/q03_b02_rerun.log"

mkdir -p "$ROOT/qa/evidence"
cd "$WS" || exit 1
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
export OMNINAV_CORE_SO="$ROOT/firmware_core/build/libomninav_core.so"

{
  echo "# 时间: $(date -Iseconds)"
  echo "# 环境: $(uname -srm) bash $BASH_VERSION"
  echo "# ROS2: ${ROS_DISTRO:-unset}  CORE_SO=$OMNINAV_CORE_SO"
  echo
  echo "## T1 e2e 第一次（B-02 修复后）"
  python3 tools/run_e2e.py
  echo "E2E_RUN1_EXIT=$?"
  echo
  echo "## T2 e2e 第二次（复现性）"
  python3 tools/run_e2e.py
  echo "E2E_RUN2_EXIT=$?"
  echo
  echo "## T3 test_protocol 回归（期望 46 用例 / 153 断言 / 0 失败）"
  python3 src/stm32_bridge/test/test_protocol.py
  echo "TEST_PROTOCOL_EXIT=$?"
  echo
  echo "## T4 B-02 改动文件时间戳（确认只有 fake_stm32.py 与日志被改）"
  ls -l --time-style=long-iso tools/fake_stm32.py tools/e2e_fixed_run1.log tools/e2e_fixed_run2.log tools/b02_test_protocol.log src/stm32_bridge/stm32_bridge/*.py 2>/dev/null
} 2>&1 | tee "$LOG"

echo "日志已写入: $LOG"
