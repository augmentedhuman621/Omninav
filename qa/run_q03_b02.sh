#!/usr/bin/env bash
# Q-03 B-02 part: re-verify e2e reproducibility after the fake_stm32.py fix (read-only, touches no workspace source)
# Usage: run bash qa/run_q03_b02.sh under WSL Ubuntu-22.04
# Output: qa/evidence/q03_b02_rerun.log (tee)
# Note: ROS2 Humble's setup.bash references undefined variables, so -u cannot be enabled
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/ros2_ws"
LOG="$ROOT/qa/evidence/q03_b02_rerun.log"

mkdir -p "$ROOT/qa/evidence"
cd "$WS" || exit 1
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
export OMNINAV_CORE_SO="$ROOT/firmware_core/build/libomninav_core.so"

{
  echo "# time: $(date -Iseconds)"
  echo "# environment: $(uname -srm) bash $BASH_VERSION"
  echo "# ROS2: ${ROS_DISTRO:-unset}  CORE_SO=$OMNINAV_CORE_SO"
  echo
  echo "## T1 e2e first run (after the B-02 fix)"
  python3 tools/run_e2e.py
  echo "E2E_RUN1_EXIT=$?"
  echo
  echo "## T2 e2e second run (reproducibility)"
  python3 tools/run_e2e.py
  echo "E2E_RUN2_EXIT=$?"
  echo
  echo "## T3 test_protocol regression (expect 46 cases / 153 assertions / 0 failures)"
  python3 src/stm32_bridge/test/test_protocol.py
  echo "TEST_PROTOCOL_EXIT=$?"
  echo
  echo "## T4 B-02 changed-file timestamps (confirm only fake_stm32.py and the logs changed)"
  ls -l --time-style=long-iso tools/fake_stm32.py tools/e2e_fixed_run1.log tools/e2e_fixed_run2.log tools/b02_test_protocol.log src/stm32_bridge/stm32_bridge/*.py 2>/dev/null
} 2>&1 | tee "$LOG"

echo "Log written to: $LOG"
