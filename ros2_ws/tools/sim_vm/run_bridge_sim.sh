#!/usr/bin/env bash
# OmniNav bridge simulation: fake STM32 on a pty + stm32_bridge as the single /odom owner.
#
# Prerequisite (once):
#   bash setup_bridge_env.sh
# And Gazebo must already be running with the plug-in odom switched off:
#   ros2 launch omninav_sim gazebo.launch.py gui:=false gazebo_odom:=false
#
# Then in this terminal:  bash run_bridge_sim.sh
# Check in another terminal:  ros2 topic hz /odom   -> ~49-50 Hz, published by stm32_bridge
set -euo pipefail

KIT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${OMNINAV_WS:-$HOME/omninav_ws}"
FAKE="$WS/tools/fake_stm32.py"
CORE_SO="$WS/lib/libomninav_core.so"
LOG="${TMPDIR:-/tmp}/omninav_fake_stm32.log"

[ -f "$FAKE" ]    || { echo "!! $FAKE missing; run setup_bridge_env.sh first" >&2; exit 1; }
[ -f "$CORE_SO" ] || { echo "!! $CORE_SO missing; run setup_bridge_env.sh first" >&2; exit 1; }
[ -f "$WS/install/setup.bash" ] || { echo "!! $WS/install/setup.bash missing; run setup_bridge_env.sh first" >&2; exit 1; }

# The bridge and the fake target both load the C core library from here.
export OMNINAV_CORE_SO="$CORE_SO"

# ROS 2 Humble's setup.bash reads AMENT_TRACE_SETUP_FILES without a default and
# aborts under 'set -u', so relax nounset just around sourcing.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

# If setup_bridge_env.sh had to vendor pyserial (apt mirror blocked), make it
# importable for the bridge node started below.
PY_VENDOR="$WS/lib/pyserial_vendor"
if [ -d "$PY_VENDOR" ]; then
  export PYTHONPATH="$PY_VENDOR${PYTHONPATH:+:$PYTHONPATH}"
  echo "-- pyserial: $PY_VENDOR"
fi
python3 -c 'import serial; print("-- pyserial", serial.__version__, "from", serial.__file__)'

echo "-- start fake STM32 (log: $LOG)"
rm -f "$LOG"
python3 -u "$FAKE" --pty --hz 50 >"$LOG" 2>&1 &
FAKE_PID=$!
cleanup() {
  kill "$FAKE_PID" 2>/dev/null || true
  wait "$FAKE_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

PTY=""
for _ in $(seq 1 100); do
  PTY="$(sed -n 's/^PTY_SLAVE=//p' "$LOG" 2>/dev/null | head -n 1)"
  [ -n "$PTY" ] && break
  if ! kill -0 "$FAKE_PID" 2>/dev/null; then
    echo "!! fake STM32 exited early; log:" >&2
    cat "$LOG" >&2
    exit 1
  fi
  sleep 0.1
done
if [ -z "$PTY" ]; then
  echo "!! fake STM32 did not report a pty; log:" >&2
  cat "$LOG" >&2
  exit 1
fi

echo "-- pty slave: $PTY"
echo "-- start stm32_bridge (owner of /odom + TF odom->base_link); Ctrl+C stops both"
ros2 run stm32_bridge stm32_bridge --ros-args \
  -p serial_port:="$PTY" \
  -p baudrate:=115200 \
  -p publish_tf:=true \
  -p odom_topic:=odom \
  -p cmd_vel_topic:=cmd_vel
