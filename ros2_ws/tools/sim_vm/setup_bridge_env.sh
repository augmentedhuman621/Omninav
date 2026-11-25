#!/usr/bin/env bash
# OmniNav VM bridge-simulation installer (Ubuntu 22.04 + ROS2 Humble).
#
# Copies the stm32_bridge package, fake_stm32.py and the x86-64 core library into
# ~/omninav_ws, makes sure pyserial exists, then builds the package.
# Safe to re-run. No GUI / no Gazebo knowledge needed here.
set -euo pipefail

KIT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${OMNINAV_WS:-$HOME/omninav_ws}"

echo "== OmniNav bridge-sim setup =="
echo "   kit : $KIT_DIR"
echo "   ws  : $WS"

need_file() { [ -f "$1" ] || { echo "!! missing file: $1" >&2; exit 1; }; }
need_dir()  { [ -d "$1" ] || { echo "!! missing dir : $1" >&2; exit 1; }; }

need_dir  "$KIT_DIR/stm32_bridge"
need_file "$KIT_DIR/stm32_bridge/package.xml"
need_file "$KIT_DIR/stm32_bridge/setup.py"
need_file "$KIT_DIR/fake_stm32.py"
need_file "$KIT_DIR/libomninav_core.so"

mkdir -p "$WS/src" "$WS/tools" "$WS/lib"

echo "-- copy ROS2 package stm32_bridge -> $WS/src/stm32_bridge"
rm -rf "$WS/src/stm32_bridge"
cp -r "$KIT_DIR/stm32_bridge" "$WS/src/stm32_bridge"

# fake_stm32.py resolves "stm32_bridge" through <script>/../src/stm32_bridge,
# so it has to sit in <ws>/tools next to <ws>/src.
echo "-- copy fake_stm32.py -> $WS/tools/fake_stm32.py"
cp -f "$KIT_DIR/fake_stm32.py" "$WS/tools/fake_stm32.py"

echo "-- copy core library -> $WS/lib/libomninav_core.so"
cp -f "$KIT_DIR/libomninav_core.so" "$WS/lib/libomninav_core.so"
file "$WS/lib/libomninav_core.so" | sed 's/^/   /'

echo "-- check pyserial (bridge imports 'serial')"
if python3 -c 'import serial' >/dev/null 2>&1; then
  echo "   pyserial already present"
elif command -v pip3 >/dev/null 2>&1 && pip3 install --user pyserial >/dev/null 2>&1; then
  echo "   installed pyserial with 'pip3 install --user'"
else
  echo "   falling back to 'sudo apt-get install -y python3-serial'"
  if command -v sudo >/dev/null 2>&1 && sudo apt-get install -y python3-serial; then
    echo "   installed python3-serial"
  else
    echo "!! pyserial still missing. Install it, then re-run this script:" >&2
    echo "     sudo apt-get install -y python3-serial" >&2
    echo "     python3 -m pip install --user pyserial" >&2
    exit 1
  fi
fi

echo "-- colcon build stm32_bridge"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
cd "$WS"

# If the kit sits next to a fresh copy of omninav_sim (the usual layout on the
# shared folder), refresh that package too: the bridge mode needs the
# "gazebo_odom" launch argument added in that package.
SIM_SRC="$(cd "$KIT_DIR/.." && pwd)/omninav_sim"
BUILD_PKGS="stm32_bridge"
if [ -d "$SIM_SRC" ] && [ -f "$SIM_SRC/package.xml" ]; then
  echo "-- refresh omninav_sim from $SIM_SRC"
  rm -rf "$WS/src/omninav_sim"
  cp -r "$SIM_SRC" "$WS/src/omninav_sim"
  BUILD_PKGS="omninav_sim stm32_bridge"
fi

colcon build --packages-select $BUILD_PKGS --symlink-install

echo
echo "== done =="
echo "Next step:"
echo "  bash $KIT_DIR/run_bridge_sim.sh"
