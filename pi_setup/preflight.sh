#!/usr/bin/env bash
# =============================================================================
# OmniNav · preflight.sh -- Raspberry Pi on-target self-check
# -----------------------------------------------------------------------------
# What   : print PASS / FAIL / WARN item by item, spelling out "what is missing on
#          the Pi right now" in one run.
#          **Every FAIL prints its matching one-line fix command.**
# Where  : Raspberry Pi 5 (aarch64). **Must run on the Pi.**
# Usage  :
#            ./preflight.sh              # normal self-check
#            ./preflight.sh --scan       # additionally wait 12 s for /scan data (lidar driver must already run)
# Exit   : 0 = no FAIL; 1 = some FAIL; 2 = not on the Pi (or --help used)
# Basis  : contract doc 11 (interface contract) §2/§5.3, AGENTS.md §7 (version-alignment triple),
#          handover doc 09 §2.3, the Pi 5 host flashing and configuration checklist
# =============================================================================
set -euo pipefail

DO_SCAN=0
for arg in "$@"; do
  case $arg in
    --scan) DO_SCAN=1 ;;
    -h | --help)
      printf 'Usage: %s [--scan]\n' "${0##*/}"
      printf '  --scan   additionally check whether /scan has a data stream (skipped by default; needs the lidar driver up)\n'
      exit 2
      ;;
    *)
      printf '✖ Unknown option: %s (try --help)\n' "$arg" >&2
      exit 2
      ;;
  esac
done

PASS=0
FAIL=0
WARN=0

ok() {
  PASS=$((PASS + 1))
  printf '[PASS] %s\n' "$1"
  if [[ -n ${2:-} ]]; then printf '       %s\n' "$2"; fi
  return 0
}
bad() {
  FAIL=$((FAIL + 1))
  printf '[FAIL] %s\n' "$1"
  if [[ -n ${2:-} ]]; then printf '       → Fix: %s\n' "$2"; fi
  return 0
}
warn() {
  WARN=$((WARN + 1))
  printf '[WARN] %s\n' "$1"
  if [[ -n ${2:-} ]]; then printf '       → %s\n' "$2"; fi
  return 0
}
info() { printf '[INFO] %s\n' "$*"; }
sec() { printf '\n=== %s ===\n' "$*"; }

need_pi() {
  if [[ ${OMNINAV_SKIP_ARCH_CHECK:-0} == 1 ]]; then
    printf '⚠️  OMNINAV_SKIP_ARCH_CHECK=1: aarch64 check skipped (logic smoke test only).\n' >&2
    return 0
  fi
  local m
  m=$(uname -m)
  if [[ $m != aarch64 ]]; then
    printf '✖ preflight.sh must run on a Raspberry Pi 5 (uname -m should be aarch64), current is %s.\n' "$m" >&2
    printf '  It checks the Pi network, device nodes and ROS2 package list.\n' >&2
    printf '  To validate script logic on a laptop only: OMNINAV_SKIP_ARCH_CHECK=1 %s\n' "$0" >&2
    return 2
  fi
  return 0
}

# Is the ROS2 package present: check /opt/ros/jazzy first, then the $HOME/ros2_ws overlay
pkg_present() {
  local p=$1 d
  if [[ -d /opt/ros/jazzy/share/$p ]]; then return 0; fi
  if [[ -d $HOME/ros2_ws/install/$p/share/$p ]]; then return 0; fi
  for d in "$HOME"/ros2_ws/install/*/share/"$p"; do
    if [[ -d $d ]]; then return 0; fi
  done
  return 1
}

bashrc_exports() {
  local var=$1 val=$2
  grep -qE "^[[:space:]]*export[[:space:]]+${var}=${val}([[:space:]]|\$)" "$HOME/.bashrc" 2>/dev/null
}

main() {
  printf '================================================================\n'
  printf ' OmniNav · Raspberry Pi on-target self-check preflight.sh\n'
  printf '================================================================\n'

  # ---------------------------------------------------------------- 0 basic info
  sec "0. Basic info"
  info "uname -m   = $(uname -m)"
  info "hostname   = $(uname -n)"
  info "user       = $(id -un) (uid=$(id -u))"
  info "kernel     = $(uname -r)"
  info "os         = $(. /etc/os-release 2>/dev/null && printf '%s %s' "$NAME" "$VERSION_ID" || printf '(cannot read /etc/os-release)')"
  info "memory     = $(free -h 2>/dev/null | awk 'NR==2{print $2" total / "$7" available"}' || printf '(no free)')"
  info "root fs    = $(df -h / 2>/dev/null | awk 'NR==2{print $2" total / "$4" available ("$5" used)"}' || printf '(no df)')"

  sec "1. Network (★ current top blocker)"
  local defroute v4 gw
  defroute=$(ip route show default 2>/dev/null || true)
  if [[ -n $defroute ]]; then
    ok "default route present" "$defroute"
  else
    bad "no default route (ip route show default is empty) => no internet => apt cannot install anything" \
      'See pi_setup/README.md section 1. Pick one of three routes: 1) on Windows untick ICS and tick it back; 2) give the Pi a static 192.168.137.50/24 + gateway 192.168.137.1; 3) connect the Pi to a phone hotspot. After fixing, `ip route` must show `default via ...`'
  fi

  v4=$(ip -4 -o addr show scope global 2>/dev/null || true)
  if [[ -n $v4 ]]; then
    ok "global IPv4 address present" "$(printf '%s' "$v4" | awk '{print $2" = "$4}')"
  else
    bad "no global IPv4 address (IPv6 link-local only) => cross-machine DDS / foxglove cannot connect" \
      'Same as above, see README section 1'
  fi

  gw=$(ip route show default 2>/dev/null | awk '/default/{print $3; exit}')
  if [[ -n $gw ]]; then
    if ping -c 1 -W 2 -- "$gw" >/dev/null 2>&1; then
      ok "default gateway $gw is pingable"
    else
      warn "default gateway $gw does not answer ping" "a firewall may block ICMP; if `ip route` and DNS are fine, ignore this"
    fi
  fi

  if command -v getent >/dev/null 2>&1; then
    if getent hosts archive.ubuntu.com >/dev/null 2>&1; then
      ok "DNS resolves archive.ubuntu.com"
    else
      bad "DNS resolution failed (getent hosts archive.ubuntu.com returned nothing)" \
        'DNS normally comes with the default route. If the default route is fixed but DNS still fails, add nameservers: addresses: [8.8.8.8, 1.1.1.1] in netplan, then sudo netplan apply'
    fi
  else
    warn "no getent command, skipping the DNS check"
  fi

  if timeout 5 bash -c 'exec 3<>/dev/tcp/1.1.1.1/443' >/dev/null 2>&1; then
    ok "can open an internet TCP connection (1.1.1.1:443)"
  else
    bad "cannot reach internet TCP (1.1.1.1:443, 5 s timeout)" 'default route + NAT is not working, see README section 1'
  fi

  # ---------------------------------------------------------------- 2 serial permissions
  sec "2. Serial permissions (dialout group)"
  local me dialout_line in_session=0 in_file=0
  me=$(id -un)
  if id -nG 2>/dev/null | tr ' ' '\n' | grep -qx dialout; then in_session=1; fi
  dialout_line=$(getent group dialout 2>/dev/null || true)
  if printf '%s' "$dialout_line" | grep -qE "(^|[ ,:])${me}(,|\$)"; then in_file=1; fi

  if [[ $in_session -eq 1 ]]; then
    ok "current user $me is already in the dialout group" "$dialout_line"
  elif [[ $in_file -eq 1 ]]; then
    bad "listed in /etc/group's dialout, but **this session has not picked it up yet**" \
      "★ This is the most common misdiagnosis. You must **log in again**: exit SSH and reconnect (not just open another terminal window), or temporarily use `newgrp dialout`. Then confirm with `id -nG | tr ' ' '\\n' | grep dialout`"
  else
    bad "current user $me is not in the dialout group => reading /dev/ttyUSB* fails with Permission denied" \
      "sudo usermod -aG dialout $me   then ★**log in again** (exit SSH and reconnect) for it to take effect"
  fi

  # ---------------------------------------------------------------- 3 device nodes
  sec "3. Frozen device nodes (contract §2)"
  local d pair
  for pair in "ttyLIDAR:lidar (plugged in)" "ttyVISION:vision comm board (plugged in)"; do
    d=/dev/${pair%%:*}
    if [[ -e $d ]]; then
      if [[ -r $d && -w $d ]]; then
        ok "$d exists and is readable/writable" "$(ls -l -- "$d" 2>/dev/null || true)"
      else
        bad "$d exists but the current user cannot read/write it" "1) fix the dialout group first (previous section); 2) confirm the udev rules have GROUP=\"dialout\" MODE=\"0660\""
      fi
    else
      bad "$d does not exist (${pair##*:})" \
        '1) run pi_setup/identify_ports.sh first to determine the port; 2) cp the generated rules to /etc/udev/rules.d/; 3) sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty'
    fi
  done

  d=/dev/ttySTM32
  if [[ -e $d ]]; then
    ok "$d exists" "$(ls -l -- "$d" 2>/dev/null || true)"
  else
    warn "$d does not exist" '★ **ignore for now**: the STM32 independent USB-TTL is not connected yet (add the commented-out rule line once it is)'
  fi

  local raw
  raw=$(ls -1 /dev/ttyUSB* 2>/dev/null || true)
  if [[ -n $raw ]]; then
    info "raw device nodes: $(printf '%s' "$raw" | tr '\n' ' ')"
    if [[ ! -e /dev/ttyLIDAR || ! -e /dev/ttyVISION ]]; then
      info "  ^ the devices are **plugged in**, they just are not named ttyLIDAR / ttyVISION yet => the udev rules are the problem, not the hardware."
    fi
  else
    info "raw device nodes: none of /dev/ttyUSB* exists"
  fi

  if [[ -f /etc/udev/rules.d/99-omninav-serial.rules ]]; then
    ok "udev rules installed: /etc/udev/rules.d/99-omninav-serial.rules"
  else
    bad "udev rules not installed (/etc/udev/rules.d/99-omninav-serial.rules missing)" \
      'See README section 3: generate with identify_ports.sh -> sudo cp -> udevadm reload + trigger'
  fi

  # ---------------------------------------------------------------- 4 shared library
  sec "4. firmware_core shared library (contract §5.3)"
  local so=/opt/omninav/lib/libomninav_core.so
  if [[ -n ${OMNINAV_CORE_SO:-} ]]; then
    info "environment variable OMNINAV_CORE_SO=$OMNINAV_CORE_SO detected (contract §5.3 says it takes priority over this path)"
  fi
  if [[ -f $so ]]; then
    ok "$so exists" "$(ls -l -- "$so" 2>/dev/null || true)"
    if command -v nm >/dev/null 2>&1; then
      local defined missing='' sym
      defined=$(nm -D --defined-only -- "$so" 2>/dev/null || true)
      for sym in mecanum_forward mecanum_inverse mecanum_l_mm mecanum_default_geom mecanum_geom_is_valid odom_update odom_reset; do
        if ! printf '%s\n' "$defined" | grep -qw -- "$sym"; then missing="$missing $sym"; fi
      done
      if [[ -z $missing ]]; then
        ok "all seven required symbols present (contract §5.2)"
      else
        bad "missing exported symbols: $missing" 'this .so build is incomplete, report it to core-algo; the bridge fails loudly and never falls back to Python on its own'
      fi
    else
      warn "no nm, skipping symbol verification" 'sudo apt install -y binutils'
    fi
  else
    bad "$so does not exist => the bridge (stm32_bridge) cannot start" \
      'See README section 4: scp firmware_core over, then run sudo ./install_core.sh'
  fi

  # ---------------------------------------------------------------- 5 ROS2
  sec "5. ROS2 Jazzy environment"
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    ok "/opt/ros/jazzy/setup.bash exists"
  else
    bad "/opt/ros/jazzy/setup.bash does not exist => ROS2 Jazzy is not installed or is elsewhere" 'reinstall ROS2 Jazzy per the Pi 5 host flashing and configuration checklist'
  fi

  local p
  for p in slam_toolbox nav2_bringup nav2_amcl nav2_controller nav2_planner nav2_bt_navigator teleop_twist_keyboard; do
    if pkg_present "$p"; then
      ok "required package installed: $p"
    else
      bad "required package missing: $p" "with internet: sudo apt install -y ros-jazzy-${p//_/-}   without internet see the offline routes in README section 6"
    fi
  done

  for p in foxglove_bridge robot_localization micro_ros_agent twist_mux; do
    if pkg_present "$p"; then
      ok "optional package installed: $p"
    else
      warn "optional package not installed: $p" "$(case $p in
        foxglove_bridge) printf 'mapping/navigation must be observable -> README section 7 (apt install ros-jazzy-foxglove-bridge; Tsinghua tuna mirror has 3.5.0)' ;;
        robot_localization) printf '★ **not needed for now** in this project: the IMU is cut (BOM doc 03:162); do not write an EKF for it' ;;
        micro_ros_agent) printf 'only needed for the "flash the vision module with ROS firmware" route (an architecture branch outside the contract)' ;;
        twist_mux) printf 'optional: velocity arbitration between teleop and navigation' ;;
      esac)"
    fi
  done

  if [[ -f /opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so ]]; then
    ok "RMW implementation library present: librmw_cyclonedds_cpp.so"
  else
    bad "librmw_cyclonedds_cpp.so missing, but the environment requires RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
      'sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp (needs internet). ★ Without it ROS2 either fails to start or silently switches implementation, killing cross-machine communication'
  fi

  info "current environment: ROS_DOMAIN_ID='${ROS_DOMAIN_ID:-(not set)}'  RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION:-(not set)}'"
  local dom_ok=1 rmw_ok=1
  if bashrc_exports ROS_DOMAIN_ID 42; then
    ok "~/.bashrc has export ROS_DOMAIN_ID=42"
  else
    dom_ok=0
    bad "~/.bashrc has no export ROS_DOMAIN_ID=42" \
      'printf "\nexport ROS_DOMAIN_ID=42\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n" >> ~/.bashrc && source ~/.bashrc'
  fi
  if bashrc_exports RMW_IMPLEMENTATION rmw_cyclonedds_cpp; then
    ok "~/.bashrc has export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
  else
    rmw_ok=0
    bad "~/.bashrc has no export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
      'printf "\nexport ROS_DOMAIN_ID=42\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n" >> ~/.bashrc && source ~/.bashrc'
  fi
  if [[ $dom_ok -eq 1 && $rmw_ok -eq 1 ]]; then
    info "  ★ this pair must match the laptop/VM **exactly**, otherwise cross-machine DDS fails silently with no error (AGENTS.md §7)."
  fi

  if grep -q '/opt/ros/jazzy/setup.bash' "$HOME/.bashrc" 2>/dev/null; then
    ok "~/.bashrc sources /opt/ros/jazzy/setup.bash"
  else
    warn "~/.bashrc does not source /opt/ros/jazzy/setup.bash" 'echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc'
  fi

  local sw
  sw=$(swapon --show 2>/dev/null || true)
  if [[ -n $sw ]]; then
    info "swap: $(printf '%s' "$sw" | tr '\n' ' ')"
  else
    warn "no swap (building C++ on a 2GB machine OOMs easily)" 'for big packages use colcon build --parallel-workers 1 and consider enabling zram'
  fi

  # ---------------------------------------------------------------- 6 optional /scan
  if [[ $DO_SCAN -eq 1 ]]; then
    sec "6. Optional: /scan data stream (--scan)"
    info "waiting up to 12 s... (precondition: the lidar driver is already running in another terminal)"
    local out
    out=$(timeout 12 bash -lc 'source /opt/ros/jazzy/setup.bash 2>/dev/null; ros2 topic hz /scan' 2>&1 || true)
    if printf '%s' "$out" | grep -q 'average rate'; then
      ok "/scan has a data stream" "$(printf '%s' "$out" | grep 'average rate' | head -1)"
    else
      bad "/scan has no data stream (no average rate within 12 s)" \
        '1) is the lidar driver up (does ros2 topic list show /scan); 2) does the driver use /dev/ttyLIDAR; 3) do lidar model and driver match (rplidar_ros <-> A1M8, ldlidar_stl_ros2 <-> LD19)'
    fi
  fi

  # ---------------------------------------------------------------- summary
  sec "Summary"
  printf 'PASS = %d    FAIL = %d    WARN = %d\n' "$PASS" "$FAIL" "$WARN"
  if [[ $FAIL -gt 0 ]]; then
    printf '\n%d item(s) must be fixed. Fix in the order given in pi_setup/README.md (network in section 1 first -- it unlocks every later install), then rerun this script.\n' "$FAIL"
    printf 'Paste each [FAIL] line together with its "Fix" text and it maps directly onto a README section.\n'
    exit 1
  fi
  printf '\nNo FAIL. Handle WARN items as needed (those marked "ignore for now" can wait).\n'
  return 0
}

if ! need_pi; then
  exit 2
fi

main
