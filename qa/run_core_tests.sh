#!/usr/bin/env bash
# Q-01 审计脚本：在 WSL Ubuntu-22.04 里真实执行 firmware_core 的 check-purity / lib / test / test-asan
# 只读工作区源码与既有 build/ 产物；不改实现代码、不删除既有产物。
# 用法：wsl -d Ubuntu-22.04 -- bash /mnt/d/dsh/工程设计/qa/run_core_tests.sh
set -o pipefail
WS=/mnt/d/dsh/工程设计
FD=$WS/firmware_core
EVID=$WS/qa/evidence
mkdir -p "$EVID"
LOG=$EVID/q01_firmware_core.log
{
  echo "### date(UTC+8): $(TZ=Asia/Shanghai date -Is)"
  echo "### uname: $(uname -a)"
  echo "### gcc: $(gcc --version | head -1)"
  echo "### make: $(make --version | head -1)"
  echo
  echo "### build/ BEFORE"
  ls -la "$FD/build"
  md5sum "$FD/build/libomninav_core.so" 2>/dev/null || echo "(no .so before)"
  echo
  echo "=========== make check-purity ==========="
  ( cd "$FD" && make check-purity ); echo "CHECK_PURITY_EXIT=$?"
  echo
  echo "=========== make lib ==========="
  ( cd "$FD" && make lib ); echo "LIB_EXIT=$?"
  echo
  echo "=========== make test ==========="
  ( cd "$FD" && make test ); echo "TEST_EXIT=$?"
  echo
  echo "=========== make test-asan ==========="
  ( cd "$FD" && make test-asan ); echo "TEST_ASAN_EXIT=$?"
  echo
  echo "### build/ AFTER"
  ls -la "$FD/build"
  md5sum "$FD/build/libomninav_core.so" 2>/dev/null || echo "(no .so after)"
} 2>&1 | tee "$LOG"
echo "LOGPATH=$LOG"
