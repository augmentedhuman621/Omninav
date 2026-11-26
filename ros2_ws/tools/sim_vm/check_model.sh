#!/usr/bin/env bash
# OmniNav model diagnostic for the VM.
#
# Prints, in one go, everything needed to tell whether the workspace really
# contains the current robot model: mesh files and sizes, the URDF markers we
# expect (continuous wheel joints with limits, no gzclient material overrides),
# the installed share copies, and - if Gazebo is running - the link poses that
# gzserver actually built.
#
# Usage:  bash check_model.sh          (works without ROS being sourced)
set -uo pipefail

WS="${OMNINAV_WS:-$HOME/omninav_ws}"
SIM="$WS/src/omninav_sim"
INST="$WS/install/omninav_sim/share/omninav_sim"
URDF="$SIM/urdf/omninav_sim.urdf.xacro"

echo "== OmniNav model check =="
echo "   ws   : $WS"
if [ ! -d "$SIM" ]; then
  echo "!! $SIM not found: run setup_bridge_env.sh first" >&2
  exit 1
fi

echo
echo "-- mesh files in $SIM/meshes"
ls -l "$SIM/meshes"/wheel_*.stl 2>/dev/null || echo "!! wheel meshes missing"
find "$SIM/meshes" -maxdepth 1 -name '*.stl' | wc -l | sed 's/^/   stl count: /'
du -sb "$SIM/meshes" | sed 's/^/   meshes total bytes: /'
echo "   md5:"
md5sum "$SIM/meshes"/wheel_*.stl 2>/dev/null | sed 's/^/     /'

echo
echo "-- URDF markers in $URDF"
if [ -f "$URDF" ]; then
  echo "   continuous wheel joints : $(grep -c 'type="continuous"' "$URDF")  (expect 4)"
  echo "   <limit lower=...        : $(grep -c '<limit lower=' "$URDF")  (expect 1: it lives in the wheel macro)"
  echo "   gzclient <material>     : $(grep -c '<material>Gazebo' "$URDF")  (expect 0)"
  echo "   mesh file references    : $(grep -c '\.stl"' "$URDF")"
else
  echo "!! $URDF missing"
fi

echo
echo "-- installed share"
if [ -d "$INST" ]; then
  ls -l "$INST/urdf/omninav_sim.urdf.xacro" 2>/dev/null | sed 's/^/   /'
  md5sum "$INST/meshes"/wheel_*.stl 2>/dev/null | sed 's/^/   /'
  if cmp -s "$URDF" "$INST/urdf/omninav_sim.urdf.xacro"; then
    echo "   installed URDF matches the source copy: yes"
  else
    echo "   installed URDF matches the source copy: NO (rebuild needed)"
  fi
else
  echo "!! $INST not found (colcon build never ran?)"
fi

echo
echo "-- gzserver model info"
if command -v gz >/dev/null 2>&1; then
  gz model -m omninav_sim -i 2>&1 | sed 's/^/   /' || echo "   (gz returned an error - is the simulation running?)"
else
  echo "   (no 'gz' command on PATH)"
fi

echo
echo "== done =="
