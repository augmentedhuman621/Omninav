#!/usr/bin/env bash
# Q-05 B-03 re-review harness (read-only: does not modify pi_setup/, the fixture runs on a /tmp copy)
# Usage (WSL): bash qa/run_q05_b03.sh 2>&1 | tee qa/evidence/q05_b03_rerun.log
set -uo pipefail

ROOT=${ROOT:-"$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"}
cd "$ROOT" || { echo "ROOT_NOT_FOUND=$ROOT"; exit 9; }
mkdir -p qa/evidence

echo "=== T0 environment ==="
date '+%Y-%m-%dT%H:%M:%S%z'
echo "PWD=$(pwd)"

echo ""
echo "=== T1 script integrity ==="
SCRIPT=pi_setup/check_lidar.sh
echo "LINES=$(wc -l < "$SCRIPT")"
echo "SHA256=$(sha256sum "$SCRIPT" | cut -d' ' -f1)"
echo "FIXTURE_LINES=$(wc -l < pi_setup/check_lidar_fix_pty_test.py)"
echo "MD_LINES=$(wc -l < pi_setup/check_lidar.md)"

echo ""
echo "=== T2 bash -n ==="
bash -n "$SCRIPT"
echo "SYNTAX_EXIT=$?"

echo ""
echo "=== T3 run B's fixture on the /tmp copy (does not modify pi_setup) ==="
TMP=/tmp/q05_b03_copy_$$
rm -rf "$TMP"
mkdir -p "$TMP"
cp -a pi_setup "$TMP/pi_setup"
echo "COPY_SHA256=$(sha256sum "$TMP/pi_setup/check_lidar.sh" | cut -d' ' -f1)"
( cd "$TMP" && python3 pi_setup/check_lidar_fix_pty_test.py ) > qa/evidence/q05_b03_pty_fixture.log 2>&1
echo "FIXTURE_EXIT=$?"
grep -E 'Assertion summary|\[FAIL\]' qa/evidence/q05_b03_pty_fixture.log | tail -6
grep -E 'passively received [0-9]+ bytes' qa/evidence/q05_b03_pty_fixture.log | tail -2
grep -E '115200  -> [0-9]+ bytes|230400  -> [0-9]+ bytes|256000|460800  -> [0-9]+ bytes' qa/evidence/q05_b03_pty_fixture.log | tail -6
rm -rf "$TMP"

echo ""
echo "=== T4 argument boundaries (OMNINAV_SKIP_ARCH_CHECK=1) ==="
export OMNINAV_SKIP_ARCH_CHECK=1
run_case() {
  local name=$1
  shift
  local out=/tmp/q05_case_$$.log
  bash "$SCRIPT" "$@" > "$out" 2>&1
  local rc=$?
  echo "CASE $name EXIT=$rc"
  grep -E '✖|Usage:|Probe device =|Verdict:|probe not executed|All candidate baud rates' "$out" | head -5 | sed 's/^/    /'
  rm -f "$out"
  return 0
}
run_case HELP --help
run_case MISSING --device
run_case EMPTY --device=
run_case VALUELIKE --device --probe
run_case UNKNOWN --bogus
run_case EQ_NULL --probe --device=/dev/null
run_case SP_NULL --probe --device /dev/null
run_case SP_NONEXIST --probe --device /dev/ttyUSB0

echo ""
echo "=== T5 read-only boundary scan ==="
echo "WRITE_PROBE_DEV_COUNT=$(grep -c '>"$PROBE_DEV"' "$SCRIPT" || true)"
echo "WRITE_DEV_VAR_COUNT=$(grep -F -c '>"$dev"' "$SCRIPT" || true)"
echo "APT_EXEC_COUNT=$(grep -cE '^[[:space:]]*(sudo[[:space:]]+)?(apt|apt-get|pip3?|dnf|yum|snap)[[:space:]]+(install|remove)' "$SCRIPT" || true)"
echo "SYS_WRITE_COUNT=$(grep -cE '^[[:space:]]*(sudo[[:space:]]+)?(tee|cp|mv|rm|chmod|chown|usermod|udevadm[[:space:]]+control)[^#]*/(etc|usr|opt)' "$SCRIPT" || true)"
echo "SUDO_EXEC_COUNT=$(grep -cE '^[[:space:]]*sudo[[:space:]]' "$SCRIPT" || true)"
grep -nE '^[[:space:]]*sudo[[:space:]]|^[[:space:]]*(apt|apt-get)[[:space:]]+install' "$SCRIPT" | head -5 | sed 's/^/    /'

echo ""
echo "=== T6 two usage forms (script vs doc) ==="
grep -n 'device DEV | --device=DEV' "$SCRIPT" | sed 's/^/    SCRIPT /'
grep -n '\-\-device DEV\|--device=DEV' pi_setup/check_lidar.md | head -6 | sed 's/^/    MD /'

echo ""
echo "=== T7 doc references / status words ==="
grep -n 'documented only' pi_setup/check_lidar.md | head -6 | sed 's/^/    /'
grep -n 'D2\|§5.2 C1\|§6.3' pi_setup/check_lidar.md | head -8 | sed 's/^/    /'

echo ""
echo "=== T8 B_ros2 §14.3 report numbers vs B's own log ==="
grep -nE '7440|21744|7488|21792' reports/B_ros2.md | sed 's/^/    REPORT /'
grep -oE '115200  -> [0-9]+ bytes|passively received [0-9]+ bytes' pi_setup/check_lidar_fix_pty.log | tail -4 | sed 's/^/    BLOG /'
grep -n '24/24' pi_setup/check_lidar_fix_pty.log | tail -2 | sed 's/^/    BLOG /'

echo ""
echo "HARNESS_DONE"
exit 0
