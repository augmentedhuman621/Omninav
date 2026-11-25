#!/usr/bin/env bash
# Q-01: copy firmware_core from scratch into /tmp, then do a fully fresh build from source + run all tests
# Purpose: prove this exact source tree really builds from scratch today, without touching the workspace's existing build/ artifacts.
set -o pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC=$WS/firmware_core
WORK=/tmp/q01_firmware_core_fresh
EVID=$WS/qa/evidence
mkdir -p "$EVID"
LOG=$EVID/q01_firmware_core_fresh_build.log
{
  echo "### date(UTC+8): $(TZ=Asia/Shanghai date -Is)"
  echo "### fresh copy: $WORK"
  echo "### source file SHA256 (workspace):"
  ( cd "$SRC" && sha256sum src/*.c include/*.h test/*.c Makefile )
  rm -rf "$WORK"
  mkdir -p "$WORK"
  cp -a "$SRC/src" "$SRC/include" "$SRC/test" "$SRC/Makefile" "$WORK"/
  echo
  echo "### after copy (no build/):"; ls -la "$WORK"
  echo
  echo "=========== fresh: make all (check + test + lib) ==========="
  ( cd "$WORK" && make all ); echo "FRESH_ALL_EXIT=$?"
  echo
  echo "=========== fresh: make test-asan ==========="
  ( cd "$WORK" && make test-asan ); echo "FRESH_ASAN_EXIT=$?"
  echo
  echo "### fresh build/ artifacts:"
  ls -la "$WORK/build"
  file "$WORK/build/libomninav_core.so"
} 2>&1 | tee "$LOG"
echo "LOGPATH=$LOG"
