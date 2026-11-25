#!/usr/bin/env bash
# =============================================================================
# OmniNav · identify_ports.sh
# -----------------------------------------------------------------------------
# Purpose : turn "which physical USB port is the lidar and which is the vision
#           module's communication board" from an **unknown into a known** within
#           two minutes, and print udev rule text you can copy into
#           /etc/udev/rules.d/.
# Where   : Raspberry Pi 5 (aarch64). Must run on the Pi -- it reads
#           /dev/serial/by-path.
# Depends : **shell builtins + ls only** (acceptance criterion 2). No sudo, no root.
# Output  : • all interactive prompts and tables -> **stderr** (so they stay
#             visible even if stdout is redirected)
#           • the generated udev rules -> **stdout**
#           usage:  ./identify_ports.sh > /tmp/99-omninav-serial.rules
# Why unplugging identifies it: the two CH340s are the same model with no unique
#           serial number; the only distinguishing clue is "which physical port
#           it is plugged into". Unplugging one cable and seeing which by-path
#           entry disappears gives exactly that mapping, without reading data.
# ★ usbv2: new udev builds create **two** by-path names (`usb-` and `usbv2-`) for
#           the same physical port (same chip). udev's ID_PATH reports the `usb-`
#           one, so this script **lists only the non-usbv2 entry**, avoiding
#           "2 devices plugged in but 4 lines listed" and copy mistakes.
# =============================================================================
set -euo pipefail

readonly BY_PATH_DIR=/dev/serial/by-path

# --- output channels ---------------------------------------------------------
UI()  { printf '%s\n' "$*" >&2; }   # human-readable
OUT() { printf '%s\n' "$*"; }       # generated artifact

hr() { UI "----------------------------------------------------------------"; }

need_pi() {
  if [[ ${OMNINAV_SKIP_ARCH_CHECK:-0} == 1 ]]; then
    UI "⚠️  OMNINAV_SKIP_ARCH_CHECK=1: aarch64 check skipped (logic smoke test on a laptop only)."
    return 0
  fi
  local m
  m=$(uname -m)
  if [[ $m != aarch64 ]]; then
    UI "✖ This script must run on a Raspberry Pi 5 (uname -m should be aarch64), current is $m."
    UI "  It reads the Pi's /dev/serial/by-path, which is meaningless on other machines."
    UI "  For a logic smoke test on a laptop only: OMNINAV_SKIP_ARCH_CHECK=1 ./identify_ports.sh"
    return 2
  fi
  return 0
}

# --- count the duplicate usbv2 entries (used only to print one hint in main) ---
# Use glob rather than ls / grep, keeping the "shell builtins + ls only" constraint.
# With no match the glob keeps the pattern literally; then -e and -L are both false
# => skipped automatically.
count_usbv2() {
  local f n=0
  for f in "$BY_PATH_DIR"/*-usbv2-*; do
    if [[ ! -e $f && ! -L $f ]]; then
      continue
    fi
    n=$((n + 1))
  done
  printf '%s' "$n"
}

# --- by-path link name -> udev ID_PATH value ----------------------------------
# ★★★ This is the easiest and **completely silent** trap in this project:
#   a link name under /dev/serial/by-path/ is assembled by udev from
#       $ID_PATH + "-port" + $ID_USB_INTERFACE_NUM
#   (see /usr/lib/udev/rules.d/60-serial.rules). So the link name has an extra
#   `-port0` tail that ENV{ID_PATH} itself does not have:
#
#       link name  platform-xhci-hcd.0-usb-0:2:1.0-port0
#       ID_PATH    platform-xhci-hcd.0-usb-0:2:1.0          <- must go into the rules
#
#   Measured on the real device (2026-10, Raspberry Pi 5):
#       /dev/ttyUSB0 -> ID_PATH=platform-xhci-hcd.0-usb-0:2:1.0
#       /dev/ttyUSB1 -> ID_PATH=platform-xhci-hcd.1-usb-0:1:1.0
#
#   So the generated rules **must strip `-port0`** -- with that tail they never
#   match, and udev reports no error at all; the symptom is just "rules installed,
#   reloaded, but the symlink never appears".
#   When no -port<N> tail is present, return the argument unchanged.
idpath_of() {
  printf '%s' "${1%-port[0-9]*}"
}

# --- snapshot: resolve /dev/serial/by-path into "physical port<TAB>device name" ---
# Shell builtins + ls only:
#   ls -l /dev/serial/by-path/xxx  ->  "lrwxrwxrwx ... xxx -> ../../ttyUSB0"
#   take the part after " -> " and strip the directory prefix to get ttyUSB0.
#
# ★ Entries containing "-usbv2-" are **silently** dropped here: they point to the
#   same ttyUSBn as the entry without it, and udev's ENV{ID_PATH} uses the
#   non-usbv2 one. The hint text is printed once by main (it cannot be printed
#   here -- snapshot runs in a process-substitution subshell, where a de-dup flag
#   does not survive, and the same sentence would appear three times).
snapshot() {
  local -a entries=() keep=()
  local f base lnk tgt t
  if [[ ! -d $BY_PATH_DIR ]]; then
    return 1
  fi
  for f in "$BY_PATH_DIR"/*; do
    if [[ ! -e $f && ! -L $f ]]; then
      continue                      # empty dir: the glob keeps the literal "*"
    fi
    entries+=("$f")
  done
  if [[ ${#entries[@]} -eq 0 ]]; then
    return 0
  fi

  for f in "${entries[@]}"; do
    base=${f##*/}
    if [[ $base != *-usbv2-* ]]; then
      keep+=("$f")
    fi
  done
  if [[ ${#keep[@]} -gt 0 ]]; then
    entries=("${keep[@]}")
  fi
  # Fallback: if the whole by-path set only offers usbv2 names, never drop all
  # devices -- fall back to the unfiltered list (README section 3 explains how to
  # adjust the udev match key for that case).

  for f in "${entries[@]}"; do
    base=${f##*/}
    lnk=$(ls -l -- "$f" 2>/dev/null || true)
    tgt=${lnk##* -> }
    if [[ $tgt == "$lnk" ]]; then
      t=$'\t(cannot parse)'             # not a symlink, or unexpected ls output
    else
      t=${tgt##*/}
    fi
    printf '%s\t%s\n' "$base" "$t"
  done
  return 0
}

# Load the snapshot into an array (skip empty lines)
load_snapshot() {
  local -n _out=$1                 # bash 4.3+ nameref; supported by bash 5.x on Ubuntu 24.04
  _out=()
  local line
  while IFS= read -r line; do
    if [[ -n $line ]]; then
      _out+=("$line")
    fi
  done < <(snapshot || true)
}

# Does the array contain a given physical-port name?
has_path() {
  local needle=$1; shift
  local line
  for line in "$@"; do
    if [[ ${line%%$'\t'*} == "$needle" ]]; then
      return 0
    fi
  done
  return 1
}

print_table() {
  local line i=0 name dev
  for line in "$@"; do
    i=$((i + 1))
    name=${line%%$'\t'*}
    dev=${line##*$'\t'}
    UI "$(printf '  %-3s %-46s %s' "$i" "$name" "$dev")"
  done
}

# --- main flow ---------------------------------------------------------------
main() {
  UI "================================================================"
  UI " OmniNav · USB serial port identification (identify_ports.sh)"
  UI "================================================================"
  UI ""
  UI "It asks you to unplug one cable and plug it back in; about a minute, changing no system setting."
  UI ""

  if [[ ! -d $BY_PATH_DIR ]]; then
    UI "✖ $BY_PATH_DIR does not exist."
    UI "  Possible causes: 1) no USB serial device is plugged in; 2) the kernel has not loaded usbserial modules."
    UI "  Confirm both the lidar and the vision module USB cables are plugged into the Pi, then rerun."
    exit 1
  fi

  local -a before=() after=() gone=() remain=()
  load_snapshot before

  if [[ ${#before[@]} -eq 0 ]]; then
    UI "✖ $BY_PATH_DIR is empty -- no USB serial device has been enumerated."
    UI "  Plug the devices in first (the kernel needs 1-2 s to create nodes), then rerun this script."
    exit 1
  fi

  hr
  local usbv2_n
  usbv2_n=$(count_usbv2)
  if [[ $usbv2_n -gt 0 ]]; then
    UI "ℹ️  /dev/serial/by-path/ also contains $usbv2_n \"usbv2\" duplicate entries -- that is another"
    UI "    naming of the **same physical port** (same CH340); udev's ID_PATH uses the non-usbv2 one."
    UI "    The table below already drops them, so the count equals the real number of devices."
    UI ""
  fi
  UI "Step 0 · USB serial devices currently enumerated: ${#before[@]}"
  UI ""
  UI "  #   physical port (= /dev/serial/by-path/ name without usbv2 duplicates, i.e. ID_PATH)   current device name"
  print_table "${before[@]}"
  UI ""

  if [[ ${#before[@]} -lt 2 ]]; then
    UI "⚠️  Only ${#before[@]} seen. A normal setup for this project should have **at least 2** (lidar + vision module)."
    UI "   You can continue, but the conclusion can only pin down this one device."
    UI ""
  fi

  hr
  UI "Step 1 · Unplug"
  UI ""
  UI "  Please unplug **exactly one** USB cable. The vision module cable is suggested (the Type-C / CH340"
  UI "  cable on the communication board) because it is the easiest to unplug. ★ Remember which one you unplugged; the next step asks."
  UI ""
  UI "  Press Enter after unplugging..."
  read -r _ || true

  load_snapshot after

  local line
  for line in "${before[@]}"; do
    if ! has_path "${line%%$'\t'*}" "${after[@]}"; then
      gone+=("$line")
    fi
  done

  if [[ ${#gone[@]} -eq 0 ]]; then
    UI ""
    UI "✖ No device disappeared. Possible causes: 1) you did not unplug anything; 2) you unplugged something else (network cable, power);"
    UI "  3) the kernel has not reacted yet -- wait 2 s and rerun this script."
    exit 1
  fi
  if [[ ${#gone[@]} -gt 1 ]]; then
    UI ""
    UI "⚠️  ${#gone[@]} devices disappeared at the same time:"
    print_table "${gone[@]}"
    UI "  You probably unplugged a multi-port cable or a USB hub. Unplug exactly one device and rerun."
    exit 1
  fi

  local unplugged_path unplugged_dev
  unplugged_path=${gone[0]%%$'\t'*}
  unplugged_dev=${gone[0]##*$'\t'}

  hr
  UI "Step 2 · Confirm which one you unplugged"
  UI ""
  UI "  The unplugged physical port is: $unplugged_path"
  UI "  (the device name it had was $unplugged_dev; it should be gone now)"
  UI ""
  UI "  What was it? Enter 1 = vision module communication board (ESP32-S3), 2 = lidar, then press Enter:"

  local which=''
  while true; do
    # ★ Do not use `read -r which || true`: when stdin is /dev/null or EOF, read
    #   fails immediately, which stays empty forever, and this while loops
    #   **forever** (printing "enter 1 or 2" endlessly).
    if ! read -r which; then
      UI ""
      UI "✖ Standard input is closed (not running in an interactive terminal) -- step 2 cannot be answered, exiting."
      UI "  Run it directly in the Pi's terminal: ./pi_setup/identify_ports.sh"
      exit 1
    fi
    case $which in
      1 | 2) break ;;
      *) UI "  Enter 1 or 2:" ;;
    esac
  done

  local vision_path='' vision_dev='' lidar_path='' lidar_dev=''
  if [[ $which == 1 ]]; then
    vision_path=$unplugged_path; vision_dev=$unplugged_dev
  else
    lidar_path=$unplugged_path;  lidar_dev=$unplugged_dev
  fi

  hr
  UI "Step 3 · Plug it back into **the same port** (a different port changes the physical path and voids the result), then press Enter"
  read -r _ || true

  load_snapshot after

  if ! has_path "$unplugged_path" "${after[@]}"; then
    UI ""
    UI "✖ That port did not come back. Please confirm: 1) the cable is seated; 2) it is the original port; 3) wait 2 s and rerun."
    exit 1
  fi
  UI "  ✅ Back: $unplugged_path"

  # Remaining devices: if exactly one is left, it plays the other role
  local l
  for l in "${after[@]}"; do
    if [[ ${l%%$'\t'*} != "$unplugged_path" ]]; then
      remain+=("$l")
    fi
  done

  if [[ ${#remain[@]} -eq 1 ]]; then
    if [[ $which == 1 ]]; then
      lidar_path=${remain[0]%%$'\t'*};  lidar_dev=${remain[0]##*$'\t'}
    else
      vision_path=${remain[0]%%$'\t'*}; vision_dev=${remain[0]##*$'\t'}
    fi
  fi

  hr
  local lidar_idpath='' vision_idpath=''
  if [[ -n $lidar_path ]];  then lidar_idpath=$(idpath_of "$lidar_path");   fi
  if [[ -n $vision_path ]]; then vision_idpath=$(idpath_of "$vision_path"); fi

  UI "Conclusion"
  UI ""
  UI "  Lidar               : ${lidar_idpath:-(not determined)}"
  if [[ -n $lidar_path ]]; then
    UI "                    (device ${lidar_dev:-?}; by-path link name $lidar_path)"
  fi
  UI "  Vision comm board   : ${vision_idpath:-(not determined)}"
  if [[ -n $vision_path ]]; then
    UI "                    (device ${vision_dev:-?}; by-path link name $vision_path)"
  fi
  UI ""
  UI "  ★ The two strings after \"lidar / vision comm board\" above are the ENV{ID_PATH} values."
  UI "    Note they have **no -port0 tail** -- by-path link names have it, ID_PATH does not;"
  UI "    writing that tail into the rules fails silently (see \"Trap 1\" at the top of the rules file)."
  UI ""
  if [[ ${#remain[@]} -gt 1 ]]; then
    UI "  ⚠️  ${#remain[@]} devices are still undetermined (you may still have the STM32 USB-TTL plugged in):"
    print_table "${remain[@]}"
    UI "      -> Run this script again and unplug the other one; that separates them."
    UI ""
  fi
  UI "  Self-check: compare the physical ports in the two conclusions above with what you observed,"
  UI "        if they are swapped, step 2 was answered wrongly -- just rerun (no side effects)."
  UI ""

  # ---------------- generated artifact -> stdout ----------------
  OUT "# ============================================================================="
  OUT "# OmniNav · fixed naming for USB serial devices (measured and generated by pi_setup/identify_ports.sh)"
  OUT "# Install to: /etc/udev/rules.d/99-omninav-serial.rules"
  OUT "# Conclusion of the plug/unplug run that generated it (in parentheses is the by-path link name; the rules use it without -port0):"
  OUT "#   Lidar           = ${lidar_idpath:-undetermined}   ${lidar_path:+($lidar_path)}"
  OUT "#   Vision comm board = ${vision_idpath:-undetermined}   ${vision_path:+($vision_path)}"
  OUT "# ============================================================================="
  OUT ""
  if [[ -n $lidar_idpath ]]; then
    OUT "SUBSYSTEM==\"tty\", KERNEL==\"ttyUSB*\", ENV{ID_PATH}==\"$lidar_idpath\", SYMLINK+=\"ttyLIDAR\", GROUP=\"dialout\", MODE=\"0660\""
  else
    OUT "# [undetermined] the lidar line was not generated -- rerun identify_ports.sh"
  fi
  OUT ""
  if [[ -n $vision_idpath ]]; then
    OUT "SUBSYSTEM==\"tty\", KERNEL==\"ttyUSB*\", ENV{ID_PATH}==\"$vision_idpath\", SYMLINK+=\"ttyVISION\", GROUP=\"dialout\", MODE=\"0660\""
  else
    OUT "# [undetermined] the vision module line was not generated -- rerun identify_ports.sh"
  fi
  OUT ""
  OUT "# Independent USB-TTL module for the STM32 serial bridge (not connected yet, so its port is unknown in advance):"
  OUT "#   plug it in, see which /dev/ttyUSB* is new (say ttyUSB2), then read its ID_PATH:"
  OUT "#     udevadm info -q property -n /dev/ttyUSB2 | grep '^ID_PATH='"
  OUT "#   paste the whole string after = into the line below and remove the leading #. ★ Do not copy the by-path link name (it has an extra -port0)."
  OUT "# SUBSYSTEM==\"tty\", KERNEL==\"ttyUSB*\", ENV{ID_PATH}==\"<fill here>\", SYMLINK+=\"ttySTM32\", GROUP=\"dialout\", MODE=\"0660\""

  UI "=================================================================="
  UI "The rule text has been printed to **stdout**. Save it and install:"
  UI ""
  UI "  ./identify_ports.sh > /tmp/99-omninav-serial.rules   # skip if already redirected"
  UI "  ★ double-check: udevadm info -q property -n /dev/ttyUSB0 | grep '^ID_PATH='"
  UI "  sudo cp /tmp/99-omninav-serial.rules /etc/udev/rules.d/"
  UI "  sudo udevadm control --reload-rules"
  UI "  sudo udevadm trigger --subsystem-match=tty"
  UI "  ls -l /dev/ttyLIDAR /dev/ttyVISION"
  UI ""
  UI "The last command must print two symlinks pointing at ttyUSBn (see pi_setup/README.md section 3)."
  UI "=================================================================="
}

if ! need_pi; then
  exit 2
fi

main "$@"
