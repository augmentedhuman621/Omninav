#!/usr/bin/env bash
# =============================================================================
# Q-03 B-01 review: check_lidar.sh static/logic verification (read-only, no hardware)
# Output: qa/evidence/q03_check_lidar_review.log
# Run by Q (acceptance) in WSL Ubuntu-22.04; no test modifies source, installs packages,
# or writes system files.
# =============================================================================
set -uo pipefail
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)" || exit 1
mkdir -p qa/evidence
LOG=qa/evidence/q03_check_lidar_review.log

{
  echo "# Q-03 B-01 check_lidar review log"
  echo "# time: $(date -Is)"
  echo "# environment: $(uname -srm) / bash $BASH_VERSION"
  echo

  echo "## T0 line counts"
  wc -l pi_setup/check_lidar.sh pi_setup/check_lidar.md reports/B_ros2.md
  echo

  echo "## T1 syntax check bash -n"
  bash -n pi_setup/check_lidar.sh && echo "SYNTAX_OK"
  echo

  echo "## T2 logic smoke test (OMNINAV_SKIP_ARCH_CHECK=1, no hardware)"
  OMNINAV_SKIP_ARCH_CHECK=1 bash pi_setup/check_lidar.sh
  echo "SMOKE_EXIT=$?"
  echo

  echo "## T3 space form as documented: --device /dev/ttyUSB0 (expected: parse failure)"
  OMNINAV_SKIP_ARCH_CHECK=1 bash pi_setup/check_lidar.sh --probe --device /dev/ttyUSB0 2>&1 | head -8
  echo "SPACE_FORM_EXIT=${PIPESTATUS[0]}"
  echo

  echo "## T4 equals form: --device=/dev/ttyUSB0 (expected: enters the probe section and reports the device missing)"
  OMNINAV_SKIP_ARCH_CHECK=1 bash pi_setup/check_lidar.sh --probe --device=/dev/ttyUSB0 2>&1 | tail -12
  echo "EQUALS_FORM_EXIT=${PIPESTATUS[0]}"
  echo

  echo "## T5 probe_read timeout semantics reproduction (key: does pipefail + '|| out=\"\"' discard already-received bytes?)"
  echo "#    actual implementation: check_lidar.sh:39 'set -uo pipefail'; :166+ probe_read uses timeout cat | hex_of; :453 'out=\$(probe_read ...) || out=\"\"'"
  hex_of() { od -An -v -tx1 2>/dev/null | tr -d ' \n'; }
  probe_like() { local s=$1; timeout "$s" bash -c 'printf "54 2c 11 22"; sleep 5' 2>/dev/null | hex_of; }
  out=$(probe_like 1) || out=""
  echo "# current form out=\$(probe_like 1) || out=\"\"  -> bytes_kept=${#out} (0 = data discarded = bug)"
  out2=$(probe_like 1) || true
  echo "# without clearing out2=\$(probe_like 1) || true      -> bytes_kept=${#out2} (non-zero = original bytes still there)"
  echo

  echo "## T6 destructive command scan (verify the script's safety claim)"
  if grep -nE '(^|[;&|] *)sudo |apt(-get)? |rm -[rf]|mv |dd if=|mkfs|chmod |chown |tee |>\s*/(etc|dev|usr)' pi_setup/check_lidar.sh; then
    echo "GREP_HITS_ABOVE"
  else
    echo "NO_DESTRUCTIVE_COMMANDS"
  fi
  echo "# note: the --request-rplidar branch has printf '\\245\\040' >\"\$PROBE_DEV\" (writes 2 bytes to the device, off by default, see :488)"
} 2>&1 | tee "$LOG"
