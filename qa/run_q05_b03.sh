#!/usr/bin/env bash
# Q-05 B-03 复核 harness（只读：不修改 pi_setup/，夹具在 /tmp 副本上跑）
# 用法（WSL）: bash qa/run_q05_b03.sh 2>&1 | tee qa/evidence/q05_b03_rerun.log
set -uo pipefail

ROOT=${ROOT:-/mnt/d/dsh/工程设计}
cd "$ROOT" || { echo "ROOT_NOT_FOUND=$ROOT"; exit 9; }
mkdir -p qa/evidence

echo "=== T0 环境 ==="
date '+%Y-%m-%dT%H:%M:%S%z'
echo "PWD=$(pwd)"

echo ""
echo "=== T1 脚本完整性 ==="
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
echo "=== T3 /tmp 副本跑 B 的夹具（不改 pi_setup） ==="
TMP=/tmp/q05_b03_copy_$$
rm -rf "$TMP"
mkdir -p "$TMP"
cp -a pi_setup "$TMP/pi_setup"
echo "COPY_SHA256=$(sha256sum "$TMP/pi_setup/check_lidar.sh" | cut -d' ' -f1)"
( cd "$TMP" && python3 pi_setup/check_lidar_fix_pty_test.py ) > qa/evidence/q05_b03_pty_fixture.log 2>&1
echo "FIXTURE_EXIT=$?"
grep -E '断言汇总|\[FAIL\]' qa/evidence/q05_b03_pty_fixture.log | tail -6
grep -E '被动收到 [0-9]+ 字节' qa/evidence/q05_b03_pty_fixture.log | tail -2
grep -E '115200  → [0-9]+ 字节|230400  → [0-9]+ 字节|256000|460800  → [0-9]+ 字节' qa/evidence/q05_b03_pty_fixture.log | tail -6
rm -rf "$TMP"

echo ""
echo "=== T4 参数边界（OMNINAV_SKIP_ARCH_CHECK=1） ==="
export OMNINAV_SKIP_ARCH_CHECK=1
run_case() {
  local name=$1
  shift
  local out=/tmp/q05_case_$$.log
  bash "$SCRIPT" "$@" > "$out" 2>&1
  local rc=$?
  echo "CASE $name EXIT=$rc"
  grep -E '✖|用法：|探测设备 =|判定：|探测未执行|所有候选波特率' "$out" | head -5 | sed 's/^/    /'
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
echo "=== T5 只读边界扫描 ==="
echo "WRITE_PROBE_DEV_COUNT=$(grep -c '>"$PROBE_DEV"' "$SCRIPT" || true)"
echo "WRITE_DEV_VAR_COUNT=$(grep -F -c '>"$dev"' "$SCRIPT" || true)"
echo "APT_EXEC_COUNT=$(grep -cE '^[[:space:]]*(sudo[[:space:]]+)?(apt|apt-get|pip3?|dnf|yum|snap)[[:space:]]+(install|remove)' "$SCRIPT" || true)"
echo "SYS_WRITE_COUNT=$(grep -cE '^[[:space:]]*(sudo[[:space:]]+)?(tee|cp|mv|rm|chmod|chown|usermod|udevadm[[:space:]]+control)[^#]*/(etc|usr|opt)' "$SCRIPT" || true)"
echo "SUDO_EXEC_COUNT=$(grep -cE '^[[:space:]]*sudo[[:space:]]' "$SCRIPT" || true)"
grep -nE '^[[:space:]]*sudo[[:space:]]|^[[:space:]]*(apt|apt-get)[[:space:]]+install' "$SCRIPT" | head -5 | sed 's/^/    /'

echo ""
echo "=== T6 usage 两形式（脚本 vs 文档） ==="
grep -n 'device DEV | --device=DEV' "$SCRIPT" | sed 's/^/    SCRIPT /'
grep -n '\-\-device DEV\|--device=DEV' pi_setup/check_lidar.md | head -6 | sed 's/^/    MD /'

echo ""
echo "=== T7 文档引用/状态词 ==="
grep -n '仅文档声称' pi_setup/check_lidar.md | head -6 | sed 's/^/    /'
grep -n 'D2\|§5.2 C1\|§6.3' pi_setup/check_lidar.md | head -8 | sed 's/^/    /'

echo ""
echo "=== T8 B_ros2 §14.3 报告数字 vs B 自己的日志 ==="
grep -nE '7440|21744|7488|21792' reports/B_ros2.md | sed 's/^/    REPORT /'
grep -oE '115200  → [0-9]+ 字节|被动收到 [0-9]+ 字节' pi_setup/check_lidar_fix_pty.log | tail -4 | sed 's/^/    BLOG /'
grep -n '24/24' pi_setup/check_lidar_fix_pty.log | tail -2 | sed 's/^/    BLOG /'

echo ""
echo "HARNESS_DONE"
exit 0
