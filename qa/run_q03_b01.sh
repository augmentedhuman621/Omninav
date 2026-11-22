#!/usr/bin/env bash
# =============================================================================
# Q-03 B-01 审查：check_lidar.sh 静态/逻辑验证（只读，不碰硬件）
# 产出：qa/evidence/q03_check_lidar_review.log
# 由 Q（验收）在 WSL Ubuntu-22.04 执行；所有测试不改源码、不装包、不写系统文件。
# =============================================================================
set -uo pipefail
cd /mnt/d/dsh/工程设计 || exit 1
mkdir -p qa/evidence
LOG=qa/evidence/q03_check_lidar_review.log

{
  echo "# Q-03 B-01 check_lidar 审查日志"
  echo "# 时间: $(date -Is)"
  echo "# 环境: $(uname -srm) / bash $BASH_VERSION"
  echo

  echo "## T0 行数"
  wc -l pi_setup/check_lidar.sh pi_setup/check_lidar.md reports/B_ros2.md
  echo

  echo "## T1 语法检查 bash -n"
  bash -n pi_setup/check_lidar.sh && echo "SYNTAX_OK"
  echo

  echo "## T2 逻辑冒烟（OMNINAV_SKIP_ARCH_CHECK=1，无硬件）"
  OMNINAV_SKIP_ARCH_CHECK=1 bash pi_setup/check_lidar.sh
  echo "SMOKE_EXIT=$?"
  echo

  echo "## T3 文档写的空格形式 --device /dev/ttyUSB0（预期: 解析失败）"
  OMNINAV_SKIP_ARCH_CHECK=1 bash pi_setup/check_lidar.sh --probe --device /dev/ttyUSB0 2>&1 | head -8
  echo "SPACE_FORM_EXIT=${PIPESTATUS[0]}"
  echo

  echo "## T4 等号形式 --device=/dev/ttyUSB0（预期: 进入探测段并报设备不存在）"
  OMNINAV_SKIP_ARCH_CHECK=1 bash pi_setup/check_lidar.sh --probe --device=/dev/ttyUSB0 2>&1 | tail -12
  echo "EQUALS_FORM_EXIT=${PIPESTATUS[0]}"
  echo

  echo "## T5 probe_read 超时语义复现（关键：pipefail + '|| out=\"\"' 是否丢弃已收字节）"
  echo "#    真实现: check_lidar.sh:31 'set -uo pipefail'; :137-144 probe_read 用 timeout cat | hex_of; :410 'out=\$(probe_read ...) || out=\"\"'"
  hex_of() { od -An -v -tx1 2>/dev/null | tr -d ' \n'; }
  probe_like() { local s=$1; timeout "$s" bash -c 'printf "54 2c 11 22"; sleep 5' 2>/dev/null | hex_of; }
  out=$(probe_like 1) || out=""
  echo "# 当前写法 out=\$(probe_like 1) || out=\"\"  -> bytes_kept=${#out} (0 = 数据被丢弃 = bug)"
  out2=$(probe_like 1) || true
  echo "# 去掉清空 out2=\$(probe_like 1) || true      -> bytes_kept=${#out2} (非 0 = 原有字节还在)"
  echo

  echo "## T6 破坏性命令扫描（脚本安全性声明核对）"
  if grep -nE '(^|[;&|] *)sudo |apt(-get)? |rm -[rf]|mv |dd if=|mkfs|chmod |chown |tee |>\s*/(etc|dev|usr)' pi_setup/check_lidar.sh; then
    echo "GREP_HITS_ABOVE"
  else
    echo "NO_DESTRUCTIVE_COMMANDS"
  fi
  echo "# 注: --request-rplidar 分支有 printf '\\245\\040' >\"\$PROBE_DEV\"（写设备 2 字节，默认关闭，见 :439-441）"
} 2>&1 | tee "$LOG"
