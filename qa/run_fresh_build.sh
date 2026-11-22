#!/usr/bin/env bash
# Q-01：从零复制一份 firmware_core 到 /tmp，再从源码全新编译 + 跑全部测试
# 目的：证明"今天这份源码真的能从零编译通过"，且不碰工作区既有 build/ 产物。
set -o pipefail
WS=/mnt/d/dsh/工程设计
SRC=$WS/firmware_core
WORK=/tmp/q01_firmware_core_fresh
EVID=$WS/qa/evidence
mkdir -p "$EVID"
LOG=$EVID/q01_firmware_core_fresh_build.log
{
  echo "### date(UTC+8): $(TZ=Asia/Shanghai date -Is)"
  echo "### fresh copy: $WORK"
  echo "### 源文件 SHA256（工作区）:"
  ( cd "$SRC" && sha256sum src/*.c include/*.h test/*.c Makefile )
  rm -rf "$WORK"
  mkdir -p "$WORK"
  cp -a "$SRC/src" "$SRC/include" "$SRC/test" "$SRC/Makefile" "$WORK"/
  echo
  echo "### 复制后（无 build/）："; ls -la "$WORK"
  echo
  echo "=========== fresh: make all（check + test + lib） ==========="
  ( cd "$WORK" && make all ); echo "FRESH_ALL_EXIT=$?"
  echo
  echo "=========== fresh: make test-asan ==========="
  ( cd "$WORK" && make test-asan ); echo "FRESH_ASAN_EXIT=$?"
  echo
  echo "### fresh build/ 产物:"
  ls -la "$WORK/build"
  file "$WORK/build/libomninav_core.so"
} 2>&1 | tee "$LOG"
echo "LOGPATH=$LOG"
