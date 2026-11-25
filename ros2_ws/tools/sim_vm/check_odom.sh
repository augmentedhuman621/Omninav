#!/usr/bin/env bash
# Diagnose the bridge-sim ROS graph without relying on the ros2 CLI daemon.
#
# Run it in a THIRD terminal while run_bridge_sim.sh is still running:
#   bash /media/sf_vmshare/omninav_bridge_sim/check_odom.sh
#
# It prints: the ROS middleware environment (a ROS_DOMAIN_ID / RMW mismatch
# between terminals is the #1 reason for "topic does not appear to be
# published"), the discovered nodes and topics, and the /odom message count.
set -euo pipefail

KIT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${OMNINAV_WS:-$HOME/omninav_ws}"

echo "== environment =="
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset, default 0>}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset, rmw_fastrtps_cpp>}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<unset>}"
echo "OMNINAV_WS=$WS"

# ROS 2 Humble's setup.bash reads AMENT_TRACE_SETUP_FILES without a default and
# aborts under 'set -u', so relax nounset just around sourcing.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
fi
set -u

echo
echo "== rclpy graph probe =="
# 'set -e' would abort before the hints below, so capture the status explicitly.
if python3 "$KIT_DIR/check_odom.py"; then
  rc=0
else
  rc=$?
fi
if [ "$rc" -ne 0 ]; then
  echo
  echo "!! no /odom received. Check, in this order:"
  echo "   1) is the terminal running run_bridge_sim.sh still alive (it owns the bridge)?"
  echo "   2) does its log show 'Serial bridge ready' and 'Sent reset (3/3)'?"
  echo "   3) if this environment line above differs from that terminal (ROS_DOMAIN_ID /"
  echo "      RMW_IMPLEMENTATION), export the same values here and re-run."
  exit 1
fi
