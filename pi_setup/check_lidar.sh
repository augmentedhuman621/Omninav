#!/usr/bin/env bash
# =============================================================================
# OmniNav · check_lidar.sh -- read-only diagnosis: "does the lidar produce data"
# on the Raspberry Pi
# -----------------------------------------------------------------------------
# What it does (prints section by section, verdict at the end):
#   0 basic info / 1 permissions & dialout / 2 USB enumeration / 3 device node & udev /
#   4 CH340 & model clues / 5 driver packages installed? / 6 launch command reference
#   (print only, never run) / 7 passive data-stream probe (--probe) / 8 ROS2 graph,
#   /scan and TF status / 9 verdict and next step
#
# Safety (hard constraint, same as the work order):
#   * **read-only** by default: installs nothing, does not modify /etc, writes no
#     files, restarts no services.
#   * The only thing that touches device runtime parameters is `stty` (baud rate)
#     in the --probe section: it only changes that device's current termios,
#     which reverts on unplug/reboot and is never persisted. --request-rplidar
#     writes a 2-byte request command to the device; it is off by default and
#     must be enabled explicitly.
#   * Sections that need root **never sudo themselves**: the script only prints
#     the commands you should run by hand.
#
# Where to run: Raspberry Pi 5 (aarch64, Ubuntu 24.04 + ROS2 Jazzy).
#         On a Windows laptop use the Get-PnpDevice section of check_lidar.md;
#         this script exits immediately on non-Linux.
#
# Usage:
#   bash check_lidar.sh
#   bash check_lidar.sh --probe
#   bash check_lidar.sh --probe --device /dev/ttyUSB0     # device already confirmed (space form)
#   bash check_lidar.sh --probe --device=/dev/ttyUSB0     # equals form is equivalent, both accepted
#   sudo bash check_lidar.sh --probe                       # temporary workaround without dialout
#   bash check_lidar.sh --probe --request-rplidar          # retry an RPLIDAR request when idle
#
# Exit codes: 0 = no FAIL; 1 = some FAIL; 2 = wrong environment / bad argument / --help
# Basis: contract doc 11 (interface contract) §2/§4, pi_setup/README.md section 6, BOM doc 03 §C,
#        purchase ledger doc 04. Sibling script: pi_setup/preflight.sh (same conventions).
# =============================================================================
set -uo pipefail

# ---- Frozen facts (changing these changes the contract, ask the Lead first) ---
LIDAR_NODE=/dev/ttyLIDAR
VISION_NODE=/dev/ttyVISION
STM32_NODE=/dev/ttySTM32
RULES_FILE=/etc/udev/rules.d/99-omninav-serial.rules
#: 99-omninav-serial.rules now distinguishes two same-model CH340s (1a86:7522) by ID_PATH.
#: The values below are historical measurements from the real device; they change
#: when the USB port changes, and a change is a WARN, not a FAIL.
EXPECTED_LIDAR_ID_PATH="platform-xhci-hcd.0-usb-0:2:1.0"
EXPECTED_VISION_ID_PATH="platform-xhci-hcd.1-usb-0:1:1.0"
#: Candidate driver packages (whether they are really installed is decided by
#: /opt/ros/jazzy/share and the overlay)
DRIVER_CANDIDATES="rplidar_ros ldlidar_stl_ros2 ldlidar_ros2 ydlidar_ros2_driver ydlidar_ros2 urg_node"
#: Candidate baud rates for the passive probe (LD19 usually 230400; RPLIDAR A1 115200; some lidars use 256000)
PROBE_BAUDS="115200 230400 256000 460800"

DO_PROBE=0
DO_REQUEST=0
PROBE_DEV=""

while [[ $# -gt 0 ]]; do
  case $1 in
    --probe)
      DO_PROBE=1
      shift
      ;;
    --request-rplidar)
      DO_REQUEST=1
      shift
      ;;
    --device=*)
      PROBE_DEV=${1#--device=}
      if [[ -z $PROBE_DEV ]]; then
        printf '✖ --device= must not be empty: --device=/dev/ttyUSB0\n' >&2
        exit 2
      fi
      shift
      ;;
    --device)
      if [[ $# -lt 2 || -z ${2:-} || ${2:0:1} == - ]]; then
        printf '✖ --device requires an argument: --device /dev/ttyUSB0 (also accepts --device=/dev/ttyUSB0)\n' >&2
        exit 2
      fi
      PROBE_DEV=$2
      shift 2
      ;;
    -h | --help)
      printf 'Usage: %s [--probe] [--device DEV | --device=DEV] [--request-rplidar]\n' "${0##*/}"
      printf '  --probe            also run a passive data-stream probe (3 s per baud rate, read-only)\n'
      printf '  --device DEV       probe device (space form; default /dev/ttyLIDAR; skipped if the node is absent and no --device is given)\n'
      printf '  --device=DEV       same as above, equals form; both forms are equivalent\n'
      printf '  --request-rplidar  when idle, write RPLIDAR request A5 20 to the device (writes the device, off by default; requires --probe)\n'
      exit 2
      ;;
    *)
      printf '✖ Unknown option: %s (try --help)\n' "$1" >&2
      exit 2
      ;;
  esac
done

PASS=0
FAIL=0
WARN=0
N7522=0
N7523=0
NCP210=0
NTOTAL=0
DRIVER_FOUND=""
DRIVER_LIST=""
LIDAR_NODE_OK=0
PROBE_RUN=0
PROBE_BYTES=0
PROBE_STTY_FAIL=0
PROBE_ENTRIES=()

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
  if [[ $(uname -m) != aarch64 ]]; then
    printf '✖ check_lidar.sh must run on a Raspberry Pi 5 (uname -m should be aarch64), current is %s.\n' "$(uname -m)" >&2
    printf '  On a Windows laptop use the Get-PnpDevice section of check_lidar.md (proves enumeration only, not scanning).\n' >&2
    printf '  To validate script logic only: OMNINAV_SKIP_ARCH_CHECK=1 %s\n' "$0" >&2
    return 2
  fi
  return 0
}

# Is the package installed: check /opt/ros/* first, then the $HOME/ros2_ws overlay
pkg_present() {
  local p=$1 d
  for d in /opt/ros/*/share/"$p"; do [[ -d $d ]] && return 0; done
  for d in "$HOME"/ros2_ws/install/*/share/"$p"; do [[ -d $d ]] && return 0; done
  return 1
}

hex_of() { # stdin -> one line of lowercase hex
  od -An -v -tx1 2>/dev/null | tr -d ' \n'
}

# Passive byte read: the caller sets stty first (so "cannot set termios" and
# "0 bytes" are reported separately); here we only read for ${2} seconds
probe_read() {
  local dev=$1 secs=$2
  timeout "$secs" cat "$dev" 2>/dev/null | hex_of
  return 0
}

# Print the key udev fields (one ttyUSB per line)
tty_usb_info() {
  local t=$1
  if command -v udevadm >/dev/null 2>&1 && [[ -e /dev/$t ]]; then
    udevadm info -q property -n "/dev/$t" 2>/dev/null |
      awk -F= '/^ID_VENDOR_ID=/{v=$2} /^ID_MODEL_ID=/{m=$2} /^ID_MODEL=/{n=$2} /^ID_PATH=/{p=$2}
                END{printf "vendor=%s model_id=%s model=%s id_path=%s\n", v, m, n, p}'
  else
    printf '(udevadm unavailable or /dev/%s missing)\n' "$t"
  fi
}

main() {
  printf '================================================================\n'
  printf ' OmniNav · lidar read-only diagnosis check_lidar.sh\n'
  printf ' Read-only by default: no installs, no /etc changes, no files; --probe only sets runtime baud via stty\n'
  printf '================================================================\n'

  # ---------------------------------------------------------- 0 basic info
  sec "0. Basic info"
  local m
  m=$(uname -m)
  info "uname      = $(uname -s) / $m / $(uname -r)"
  info "hostname   = $(uname -n)"
  info "user       = $(id -un) (uid=$(id -u))"
  info "os         = $(. /etc/os-release 2>/dev/null && printf '%s %s' "$NAME" "$VERSION_ID" || printf '(cannot read os-release)')"
  info "memory     = $(free -h 2>/dev/null | awk 'NR==2{print $2" total / "$7" available"}' || printf '(no free)')"
  info "python3    = $(command -v python3 >/dev/null 2>&1 && python3 --version 2>&1 || printf '(none)')"
  info "ROS_DISTRO = ${ROS_DISTRO:-(empty -- source /opt/ros/jazzy/setup.bash first)}"
  info "ROS_DOMAIN_ID = ${ROS_DOMAIN_ID:-(not set; expected 42)}"
  info "ros2 CLI   = $(command -v ros2 >/dev/null 2>&1 && printf 'yes' || printf 'no')"
  info "lsusb      = $(command -v lsusb >/dev/null 2>&1 && printf 'yes' || printf 'no (does not matter, /sys fallback is used)')"
  info "timeout    = $(command -v timeout >/dev/null 2>&1 && printf 'yes' || printf 'no (--probe will be skipped)')"
  info "stty       = $(command -v stty >/dev/null 2>&1 && printf 'yes' || printf 'no')"

  # ---------------------------------------------------------- 1 permissions / dialout
  sec "1. Permissions and dialout (★ first step, read this first)"
  local target_user=${SUDO_USER:-$(id -un)} cur_groups dialout_members
  cur_groups=$(id -nG "$target_user" 2>/dev/null || id -nG)
  info "target user = $target_user (the lidar driver ends up running as this user)"
  info "current groups = $cur_groups"
  dialout_members=$(getent group dialout 2>/dev/null | awk -F: '{print $4}' || true)
  info "dialout group = ${dialout_members:-(cannot read /etc/group)}"
  if printf '%s\n' "$cur_groups" | tr ' ' '\n' | grep -qx dialout; then
    ok "$target_user is in the dialout group" "Reading /dev/ttyLIDAR needs no sudo."
  else
    bad "$target_user is not in the dialout group => reading /dev/ttyLIDAR and /dev/ttySTM32 will fail with Permission denied" \
      "sudo usermod -aG dialout $target_user ; then **log out and back in** (or reconnect SSH). usermod does not update groups of an already logged-in session; newgrp dialout works for a temporary check"
  fi
  # Effective groups of this process may differ from the login-time groups -- check both
  info "effective groups of this process = $(id -nG) (this line shows the script process's real privileges; a mismatch with the target user's groups means you have not logged in again)"

  # ---------------------------------------------------------- 2 USB enumeration
  sec "2. USB enumeration (is the lidar even plugged in)"
  if command -v lsusb >/dev/null 2>&1; then
    local usb_dump
    usb_dump=$(lsusb 2>/dev/null || true)
    N7522=$(printf '%s\n' "$usb_dump" | grep -c '1a86:7522' || true)
    N7523=$(printf '%s\n' "$usb_dump" | grep -c '1a86:7523' || true)
    NCP210=$(printf '%s\n' "$usb_dump" | grep -ciE '10c4:ea60|10c4:ea70' || true)
    info "CH340 1a86:7522 = ${N7522} (recorded value for lidar/vision)"
    info "CH340 1a86:7523 = ${N7523} (the STM32's USB-TTL)"
    info "CP210x 10c4:ea6x = ${NCP210} (common chip on RPLIDAR vendor adapter boards)"
    info "all serial-class USB devices:"
    printf '%s\n' "$usb_dump" | grep -iE '1a86|10c4|0403|serial|uart' | sed 's/^/       /' || info "       (no device matching 1a86 / 10c4 / 0403)"
  else
    warn "lsusb unavailable, falling back to /sys (may be inaccurate)" "sudo apt install usbutils (ignore if offline; this script can continue)"
    N7522=$(ls -d /dev/ttyUSB* 2>/dev/null | wc -l)
    N7523=0
    NCP210=0
    info "fallback: /dev/ttyUSB* count = ${N7522} (cannot distinguish 7522/7523/CP210x)"
  fi

  local ttys=() t
  for t in /dev/ttyUSB* /dev/ttyACM*; do [[ -e $t ]] && ttys+=("$t"); done
  NTOTAL=${#ttys[@]}
  info "serial device node count = ${NTOTAL} (including ttyACM)"
  if [[ $NTOTAL -gt 0 ]]; then
    for t in "${ttys[@]}"; do
      info "$t  $(tty_usb_info "${t##*/}")"
    done
  fi
  info "by-path links:"
  local bp
  for bp in /dev/serial/by-path/*; do
    [[ -e $bp ]] && info "  ${bp##*/} -> $(readlink -f "$bp")"
  done
  info "by-id links (★ same-model CH340s have no unique serial number and collide; code must never rely on them):"
  for bp in /dev/serial/by-id/*; do
    [[ -e $bp ]] && info "  ${bp##*/} -> $(readlink -f "$bp")"
  done
  local klog
  klog=$( { dmesg 2>/dev/null || journalctl -k --no-pager 2>/dev/null; } | grep -iE 'ch34|cp210|ftdi|ttyUSB|usb-serial' | tail -n 12 || true)
  if [[ -n $klog ]]; then
    info "recent kernel log (ch34/cp210/ttyUSB):"
    printf '%s\n' "$klog" | sed 's/^/       /'
  else
    info "cannot see the kernel log (dmesg needs root or is disabled) -- does not affect the verdict"
  fi
  if [[ ${N7522:-0} -eq 0 && ${NCP210:-0} -eq 0 && $NTOTAL -eq 0 ]]; then
    bad "no serial device on USB (neither 1a86:7522 nor CP210x, and /dev/ttyUSB* is empty)" \
      "1) replug the lidar with a **data-capable** USB cable; 2) try another Pi USB port; 3) check adapter board power (a Pi 5 on non-PD power limits total USB current to about 600 mA); 4) plug into the laptop and use the Windows section of check_lidar.md to see whether it enumerates"
  fi

  # ---------------------------------------------------------- 3 device node / udev
  sec "3. Device nodes and udev rules (is /dev/ttyLIDAR present)"
  local n
  for n in "$LIDAR_NODE" "$VISION_NODE" "$STM32_NODE"; do
    if [[ -e $n ]]; then
      info "$n exists -> $(readlink -f "$n"), mode $(stat -c '%A %U:%G' "$n" 2>/dev/null || printf '(stat failed)')"
      if [[ -r $n ]]; then
        ok "$n is readable by the current user"
      else
        bad "$n exists but is not readable by the current user" "add the dialout group first (section 1); sudo is for temporary troubleshooting only -- never put 0666 into the rules"
      fi
      if [[ ! -w $n ]]; then
        warn "$n is not writable by the current user (drivers usually open with O_RDWR and may fail)" "normal mode is root:dialout 0660; first confirm section 1's dialout membership is effective"
      fi
      if [[ $n == "$LIDAR_NODE" ]]; then LIDAR_NODE_OK=1; fi
    else
      bad "$n does not exist" "see the udev check below in this section; device names are frozen by contract -- do not substitute /dev/ttyUSB*"
    fi
  done
  if [[ -f $RULES_FILE ]]; then
    ok "udev rules file installed: $RULES_FILE" "$(wc -l <"$RULES_FILE" 2>/dev/null || true) lines"
    if grep -q 'ttyLIDAR' "$RULES_FILE"; then
      info "rules contain a ttyLIDAR entry: $(grep -n 'ttyLIDAR' "$RULES_FILE" | head -n 3 | tr '\n' ' ')"
    else
      bad "rules file has no ttyLIDAR entry" "compare with pi_setup/99-omninav-serial.rules; do not hand-edit /etc without syncing the repo"
    fi
    if grep -q "$EXPECTED_LIDAR_ID_PATH" "$RULES_FILE"; then
      info "the lidar ID_PATH in the rules is still the last measured $EXPECTED_LIDAR_ID_PATH"
    else
      warn "the lidar ID_PATH in the rules differs from the historical value recorded here (changed USB port?)" "the rules file is authoritative; regenerate it under pi_setup for the new ID_PATH, do not change it inside this script"
    fi
  else
    bad "udev rules file not installed: $RULES_FILE does not exist => /dev/ttyLIDAR can never appear automatically" \
      "sudo cp pi_setup/99-omninav-serial.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty ; replug the lidar afterwards. ★ This modifies the system and is a user-side action; this script never executes it"
  fi
  # 7522 enumerated but node missing: distinguish "rules not effective" from "ID_PATH changed"
  if [[ $LIDAR_NODE_OK -eq 0 && ${N7522:-0} -gt 0 ]]; then
    local match=0 tj
    for t in "${ttys[@]}"; do
      [[ $t == *ttyUSB* ]] || continue
      tj=$(tty_usb_info "${t##*/}")
      if printf '%s' "$tj" | grep -q "$EXPECTED_LIDAR_ID_PATH"; then match=1; fi
    done
    if [[ $match -eq 1 ]]; then
      bad "a 7522 device's ID_PATH exactly matches the lidar path in the rules, yet /dev/ttyLIDAR still does not exist => rules not effective" \
        "reinstall the rules and reload+trigger (see above), then replug the lidar; if that fails try \`udevadm test --action=add /sys/class/tty/ttyUSB0\`"
    else
      warn "7522 devices enumerated, but none is on the lidar ID_PATH recorded in the rules => port changed, rules cannot match" \
        "re-read the current ID_PATH with pi_setup/identify_ports.sh and report it to the Lead so pi_setup updates the rules; do not temporarily change the contract device name in this round"
    fi
  fi

  # ---------------------------------------------------------- 4 model clues
  sec "4. CH340 and model clues (how far we can infer)"
  if [[ $NTOTAL -gt 0 ]]; then
    for t in "${ttys[@]}"; do
      [[ $t == *ttyUSB* ]] || continue
      local info_line vid pid product
      info_line=$(tty_usb_info "${t##*/}")
      vid=$(printf '%s' "$info_line" | sed -n 's/.*vendor=\([^ ]*\).*/\1/p')
      pid=$(printf '%s' "$info_line" | sed -n 's/.*model_id=\([^ ]*\).*/\1/p')
      product=$(printf '%s' "$info_line" | sed -n 's/.*model=\(.*\) id_path=.*/\1/p')
      info "$t  vendor=$vid pid=$pid product=${product:-(empty)}"
      if [[ $pid == 7522 ]]; then
        info "  -> 7522 = CH340 family (lidar and vision are the same model; USB descriptors carry no distinguishing model)"
      elif [[ $pid == 7523 ]]; then
        info "  -> 7523 = classic CH340 USB-TTL (recorded value: STM32 link, not the lidar)"
      elif [[ $pid == ea60 || $pid == ea70 ]]; then
        info "  -> CP210x (common chip on RPLIDAR vendor adapter boards) => strong hint towards RPLIDAR A1, but the nameplate wins"
      fi
    done
  fi
  if [[ ${N7522:-0} -gt 1 ]]; then
    info "there are ${N7522} 7522 devices (lidar + vision): USB descriptors cannot tell them apart, only the physical port ID_PATH can; **do not guess ttyUSB0/1**"
  fi
  info "model priority: nameplate/silkscreen photo > adapter chip (CP2102 => RPLIDAR hint) > data-stream signature (54 2c => LD19 hint)"
  info "★ do not guess the model this round; without a nameplate use the photo checklist in check_lidar.md and ask the user for photos"

  # ---------------------------------------------------------- 5 driver packages
  sec "5. Are driver packages installed (Pi has no internet => only preinstalled packages count)"
  local pkg
  for pkg in $DRIVER_CANDIDATES; do
    if pkg_present "$pkg"; then
      DRIVER_FOUND+=" $pkg"
      DRIVER_LIST+="$pkg "
      ok "driver package installed: $pkg"
    fi
  done
  if command -v ros2 >/dev/null 2>&1; then
    local ros_pkgs
    ros_pkgs=$(timeout 30 ros2 pkg list 2>/dev/null | grep -iE 'rplidar|ldlidar|ydlidar|urg_node|lidar_driver|lslidar|hokuyo' | head -n 20 || true)
    if [[ -n $ros_pkgs ]]; then
      info "lidar-related packages in ros2 pkg list:"
      printf '%s\n' "$ros_pkgs" | sed 's/^/       /'
      while IFS= read -r pkg; do
        [[ -n $pkg ]] || continue
        case " $DRIVER_LIST " in *" $pkg "*) ;; *) DRIVER_LIST+="$pkg " ;; esac
      done <<<"$ros_pkgs"
      if [[ -z $DRIVER_FOUND ]]; then
        DRIVER_FOUND="(outside candidate list) $DRIVER_LIST"
      fi
    else
      info "ros2 pkg list has no rplidar/ldlidar/ydlidar/urg related package"
    fi
  else
    warn "ros2 CLI unavailable, driver check only looked at /opt/ros/*/share" "source /opt/ros/jazzy/setup.bash and rerun (does not fix the offline problem)"
  fi
  if [[ -z $DRIVER_FOUND ]]; then
    bad "no lidar driver package installed (candidates: $DRIVER_CANDIDATES)" \
      "The Pi has no internet => cannot install. Three offline routes in pi_setup/README.md section 6: 1) tether to a phone hotspot; 2) git clone the source on a networked machine, scp it over and colcon build --packages-select <pkg>; 3) on another Ubuntu box apt-get download then scp + dpkg -i. ★ Photograph the model first, then decide which one to install -- do not install both"
  fi

  # ---------------------------------------------------------- 6 launch command reference
  sec "6. Launch command reference (print only, never executed)"
  info "four general checks: 1) the serial port must be $LIDAR_NODE (drivers default to ttyUSB0, contract §2 forbids that); 2) the baud rate comes from the model/datasheet; 3) some packages need product_name/model arguments; 4) the /scan frame_id must be laser"
  info "look up the real parameter names first (do not guess): ros2 launch <pkg> <launchfile> --show-args  or  ls \$(ros2 pkg prefix <pkg>)/share/<pkg>/"
  if printf '%s' "$DRIVER_LIST" | grep -q 'rplidar_ros'; then
    info "-> rplidar_ros branch: ros2 launch rplidar_ros rplidar_a1_launch.py (the actual launch filename is whatever is under \$(ros2 pkg prefix rplidar_ros)/share/rplidar_ros/launch)"
    info "  change the serial argument to $LIDAR_NODE; RPLIDAR A1 commonly uses 115200; after launch run ros2 topic hz /scan"
  fi
  if printf '%s' "$DRIVER_LIST" | grep -qE 'ldlidar'; then
    info "-> ldlidar branch: launch files and yaml differ between ldlidar_stl_ros2 / ldlidar_ros2 versions; first ls share/launch and share/config"
    info "  LD19 commonly uses 230400 (not 115200); change the port to $LIDAR_NODE; after launch run ros2 topic hz /scan (LD19 10 Hz)"
  fi
  if printf '%s' "$DRIVER_LIST" | grep -qE 'ydlidar'; then
    info "-> ydlidar branch: confirm the model parameter and baud rate first, then change the port to $LIDAR_NODE"
  fi
  if [[ -z $DRIVER_FOUND ]]; then
    info "(no driver package at all => section 6 cannot give launch commands; resolve the section 5 blocker first)"
  elif [[ $DRIVER_FOUND == "(outside candidate list)"* ]]; then
    info "lidar-related packages outside the candidate list: $DRIVER_LIST -> manually confirm whether one of them is this lidar's driver before using it"
  fi
  info "★ do not run the probe section of check_lidar.sh while a driver launch is running (only one process may read a serial port)"

  # ---------------------------------------------------------- 7 data-stream probe
  sec "7. Passive data-stream probe (only with --probe; read-only, runtime baud only)"
  if [[ $DO_REQUEST -eq 1 && $DO_PROBE -eq 0 ]]; then
    info "--request-rplidar writes the device only after --probe received 0 bytes; --probe was not given, so the request section is skipped (avoids a pointless device write)."
  fi
  if [[ $DO_PROBE -eq 0 ]]; then
    info "not enabled. To find out whether any bytes come out, rerun with --probe (about 12 s)."
    info "If you would rather not run the script: sudo stty -F $LIDAR_NODE 230400 raw -echo && sudo timeout 3 cat $LIDAR_NODE | od -An -tx1 | head"
  else
    if [[ -z $PROBE_DEV ]]; then
      if [[ $LIDAR_NODE_OK -eq 1 ]]; then
        PROBE_DEV=$LIDAR_NODE
      else
        warn "no probeable device: $LIDAR_NODE does not exist and --device was not given" "fix section 3 first; once the user confirms which /dev/ttyUSBn is the lidar, run --probe --device /dev/ttyUSBn"
      fi
    fi
    if [[ -n $PROBE_DEV ]]; then
      if [[ ! -e $PROBE_DEV ]]; then
        bad "probe device does not exist: $PROBE_DEV"
      elif [[ ! -r $PROBE_DEV || ! -w $PROBE_DEV ]]; then
        bad "current user has no read/write permission on $PROBE_DEV (stty needs it)" \
          "run manually: sudo bash $0 --probe  (temporary); the long-term fix is the dialout group, see section 1"
      elif ! command -v timeout >/dev/null 2>&1; then
        warn "no timeout command on the system, skipping probe (no package installation)" "sudo apt install coreutils needs internet; you can also probe manually with stty+cat"
      else
        PROBE_RUN=1
        info "Probe device = $PROBE_DEV; candidate baud rates = $PROBE_BAUDS; 3 s per rate"
        local baud out bytes first hint stty_ok=1
        # stty pre-check: cannot set termios => device cannot be opened / is not a tty / no permission;
        # that is not the "0 bytes" case, so report it separately
        if ! stty -F "$PROBE_DEV" 115200 cs8 -parenb -cstopb raw -echo -iexten -crtscts 2>/dev/null; then
          PROBE_STTY_FAIL=1
          stty_ok=0
          warn "stty failed on $PROBE_DEV (cannot open / not a tty / no permission) -- this is not the \"0 bytes\" case, probe not executed this round" \
            "confirm manually: ls -l $PROBE_DEV; stty -F $PROBE_DEV 115200 raw -echo; for permissions see section 1"
        fi
        if [[ $stty_ok -eq 1 ]]; then
          for baud in $PROBE_BAUDS; do
            if ! stty -F "$PROBE_DEV" "$baud" cs8 -parenb -cstopb raw -echo -iexten -crtscts 2>/dev/null; then
              PROBE_ENTRIES+=("$baud  stty setup failed (this rate not probed)")
              info "$baud  -> stty setup failed (baud not accepted by device/driver; != 0 bytes)"
              continue
            fi
            out=$(probe_read "$PROBE_DEV" 3) || true
            if [[ -z $out ]]; then
              PROBE_ENTRIES+=("$baud  0 B")
              info "$baud  -> 0 bytes"
              continue
            fi
            bytes=$(( ${#out} / 2 ))
            PROBE_BYTES=$((PROBE_BYTES + bytes))
            first=${out:0:64}
            hint=""
            if [[ $out == *542c* ]]; then
              hint="contains 54 2c (LD19 header signature)"
            elif [[ $out == *a55a* ]]; then
              hint="contains A5 5A (RPLIDAR signature, points towards A1)"
            fi
            PROBE_ENTRIES+=("$baud  ${bytes} B  ${hint:-no known header signature}")
            info "$baud  -> ${bytes} bytes${hint:+, }$hint; first 32 bytes: $first"
          done
        fi
        if [[ ${#PROBE_ENTRIES[@]} -gt 0 ]]; then
          info "per-rate results:"
          for out in "${PROBE_ENTRIES[@]}"; do info "  $out"; done
        fi
        if [[ $PROBE_BYTES -gt 0 ]]; then
          ok "passively received $PROBE_BYTES bytes => the device is actively streaming data (not a dead device)"
          info "★ bytes present != /scan ready: you still need a driver package and correct parameters (sections 5, 6). Stream signatures are directional hints only -- the nameplate wins"
        elif [[ $stty_ok -eq 0 ]]; then
          warn "probe not executed (stty cannot configure the device, != 0 bytes)" \
            "fix the cannot-open / not-a-tty / permission issue first, then rerun --probe; do not treat this as \"the lidar has no data\""
        else
          warn "All candidate baud rates produced 0 bytes within 3 s" \
            "possible causes: 1) wrong baud rate (try more rates); 2) lidar unpowered / wrong wire order; 3) this model needs a request first (RPLIDAR family). Replug, check the adapter board LED, then retry with --request-rplidar (it writes A5 20 to the device, off by default)"
        fi
        if [[ $DO_REQUEST -eq 1 && $PROBE_BYTES -eq 0 && $stty_ok -eq 1 ]]; then
          info "trying RPLIDAR request: writing A5 20 to $PROBE_DEV (writes the device; still nothing on disk)"
          if printf '\245\040' >"$PROBE_DEV" 2>/dev/null; then
            local req_stty=1
            if stty -F "$PROBE_DEV" 115200 cs8 -parenb -cstopb raw -echo -iexten -crtscts 2>/dev/null; then
              out=$(probe_read "$PROBE_DEV" 3) || true
            else
              req_stty=0
              out=""
              info "stty 115200 failed after the request: cannot read (distinct from \"0 bytes\" -- this is a device setup failure)"
            fi
            if [[ -n $out ]]; then
              bytes=$(( ${#out} / 2 ))
              PROBE_BYTES=$((PROBE_BYTES + bytes))
              PROBE_ENTRIES+=("115200(after request)  ${bytes} B")
              info "after the request received ${bytes} bytes; first 32 bytes: ${out:0:64}"
              ok "response after the request => more like the RPLIDAR request/response protocol (the nameplate still wins)"
            elif [[ $req_stty -eq 0 ]]; then
              warn "cannot read after the request (stty setup failed) -- not \"0 bytes\"; check device/permissions first" \
                "confirm manually: ls -l $PROBE_DEV; stty -F $PROBE_DEV 115200 raw -echo"
            else
              warn "still no bytes after the request" "check TX/RX crossover, power and baud rate; also photograph the adapter board chip"
            fi
          else
            warn "writing the device failed (permissions?)" "use sudo bash $0 --probe --request-rplidar --device $PROBE_DEV"
          fi
        elif [[ $DO_REQUEST -eq 1 && $stty_ok -eq 0 ]]; then
          info "stty cannot configure $PROBE_DEV, skipping the --request-rplidar write (writing the device would be meaningless)"
        fi
      fi
    fi
  fi

  # ---------------------------------------------------------- 8 ROS2 status
  sec "8. ROS2 graph, /scan and TF status (reads current state, starts no node)"
  if ! command -v ros2 >/dev/null 2>&1; then
    warn "ros2 CLI unavailable, skipping" "source /opt/ros/jazzy/setup.bash and rerun"
  else
    local topics
    topics=$(timeout 20 ros2 topic list 2>/dev/null || true)
    if [[ -z $topics ]]; then
      warn "ros2 topic list gave no output (ROS2 daemon cannot start, or environment not sourced)" "source /opt/ros/jazzy/setup.bash first; on 2GB RAM avoid running many nodes at once"
    else
      for n in /scan /tf /odom /cmd_vel /tf_static; do
        if printf '%s\n' "$topics" | grep -qx "$n"; then info "topic exists: $n"; else info "topic absent (normal if no node is running): $n"; fi
      done
      if printf '%s\n' "$topics" | grep -qx /scan; then
        ok "/scan topic exists (some node declared it)" "confirm the stream manually: timeout 20 ros2 topic hz /scan; confirm frame_id: ros2 topic echo /scan --field header.frame_id --once"
      else
        info "no /scan right now => the lidar driver is not up yet (or is up but does not declare the topic)"
      fi
    fi
    info "manually verify /scan and TF (run while the nodes are up):"
    info "  timeout 20 ros2 topic hz /scan"
    info "  ros2 topic echo /scan --field header.frame_id --once    # must be laser"
    info "  ros2 run tf2_ros tf2_echo odom base_link                # the bridge should publish this"
    info "  ros2 run tf2_ros tf2_echo base_link laser              # needs URDF/static transform (not added in this project yet, see reports)"
    info "  ros2 run tf2_ros tf2_echo map odom                     # published by slam_toolbox while mapping, by AMCL while localizing -- never both at once"
  fi

  # ---------------------------------------------------------- 9 verdict
  sec "9. Verdict and next step (paste this whole section back to the Lead)"
  local which
  which=""
  if [[ ${N7522:-0} -eq 0 && ${NCP210:-0} -eq 0 ]]; then
    which="A not enumerated"
  elif [[ $LIDAR_NODE_OK -eq 0 ]]; then
    which="B enumerated but no $LIDAR_NODE"
  elif [[ -z $DRIVER_FOUND ]]; then
    which="C node present but driver package not installed"
  elif [[ $PROBE_RUN -eq 1 && $PROBE_STTY_FAIL -eq 1 ]]; then
    which="D2 probe preconditions not met (stty cannot configure the device, != 0 bytes)"
  elif [[ $PROBE_RUN -eq 1 && $PROBE_BYTES -eq 0 ]]; then
    which="D driver may be installed but no serial data (probe got 0 bytes)"
  else
    which="E device/driver/data look complete -> next verify /scan and TF"
  fi
  printf '\n>>> Verdict: %s\n' "$which"
  printf '    (basis: 7522=%d 7523=%d CP210x=%d, ttyLIDAR=%s, driver=[%s], probe=%s)\n' \
    "${N7522:-0}" "${N7523:-0}" "${NCP210:-0}" \
    "$([[ $LIDAR_NODE_OK -eq 1 ]] && printf 'present' || printf 'absent')" \
    "${DRIVER_FOUND:-none}" \
    "$([[ $PROBE_RUN -eq 1 ]] && printf '%d bytes' "$PROBE_BYTES" || printf 'not probed')"
  if [[ $PROBE_STTY_FAIL -eq 1 ]]; then
    printf '    (probe not executed: stty cannot configure the device -- reported separately from \"0 bytes\", see failure tree D2)\n'
  fi

  printf '\nFailure tree quick reference (symptom -> what to check -> minimal next experiment):\n'
  printf '  A not enumerated      -> lsusb/dmesg shows no 1a86:7522 -> swap cable and port, check power; the laptop Get-PnpDevice only proves enumeration\n'
  printf '  B enumerated, no node -> udev rules not installed / ID_PATH changed -> install rules and reload+trigger; or re-read ID_PATH with identify_ports.sh and report to the Lead\n'
  printf '  C node present, no driver -> ros2 pkg list has no lidar package -> photograph the model -> install offline (Pi has no internet; this is a blocker)\n'
  printf '  D driver present, no data -> --probe shows 0 bytes everywhere -> try other baud rates / check wire order / --request-rplidar; inspect the adapter board chip\n'
  printf '  D2 probe preconditions not met -> stty cannot configure the device (cannot open / not a tty / no permission) -> check ls -l and permissions first; this is not 0 bytes\n'
  printf '  E data present, no /scan -> launch fails -> is the serial port %s, the baud rate, product_name; ros2 node list / topic list\n' "$LIDAR_NODE"
  printf '  F /scan present, no TF -> is frame_id laser -> this project still has no base_link->laser URDF/static transform (known gap, see reports/B_ros2.md §5.2 C1 / §6.3, to be added before M4)\n'

  printf '\nSaving evidence (send back the log file plus photos):\n'
  printf '  bash %s %s 2>&1 | tee ~/check_lidar_$(date +%%F_%%H%%M).log\n' \
    "${0##*/}" "$([[ $DO_PROBE -eq 1 ]] && printf -- '--probe' || printf '')"
  printf '  then: cat ~/check_lidar_*.log  (or just copy the file away)\n'

  printf '\n================================================================\n'
  printf ' Summary: PASS=%d  FAIL=%d  WARN=%d\n' "$PASS" "$FAIL" "$WARN"
  printf ' This script installed no packages, changed no /etc, wrote no files; --probe only changes the device runtime baud rate.\n'
  printf '================================================================\n'
  if [[ $FAIL -gt 0 ]]; then return 1; fi
  return 0
}

if ! need_pi >/dev/null 2>&1; then
  need_pi
  exit 2
fi

main
rc=$?
exit $rc
