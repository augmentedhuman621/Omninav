#!/usr/bin/env bash
# Q-04 / A-02 L1 re-review (read-only): WSL gcc -fsyntax-only, macro=0/1 x app_single_wheel.c/main.c
# Environment: WSL Ubuntu-22.04; Cube headers come from the C:\Users\ASUS\STM32Cube\... path in uvprojx IncludePath
# Produces no .o/.so and modifies no workspace file.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CUBE="/mnt/c/Users/ASUS/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.7"
INC="-I$ROOT/stm32_fw/Inc -I$ROOT/firmware_core/include -I$CUBE/Drivers/CMSIS/Device/ST/STM32F1xx/Include -I$CUBE/Drivers/CMSIS/Include -I$CUBE/Drivers/STM32F1xx_HAL_Driver/Inc"
CFLAGS="-fsyntax-only -std=c99 -Wall -Wextra -DUSE_HAL_DRIVER -DSTM32F103xE"

echo "=== T0 environment ==="
date '+%Y-%m-%dT%H:%M:%S%z'
gcc --version | head -1
echo "ROOT=$ROOT"
echo "CUBE=$CUBE"
if [ -f "$CUBE/Drivers/CMSIS/Include/core_cm3.h" ] && [ -f "$CUBE/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h" ]; then
  echo "CUBE_HEADERS=FOUND"
else
  echo "CUBE_HEADERS=MISSING"; echo "RESULT=BLOCKED_ENV"; exit 3
fi
cd "$ROOT" || { echo "RESULT=BLOCKED_CWD"; exit 3; }

total_warn_from_src=0
for macro in 0 1; do
  for src in stm32_fw/Src/app_single_wheel.c stm32_fw/Src/main.c; do
    echo "=== T: OMNINAV_APP_SINGLE_WHEEL=$macro  $src ==="
    out=$(gcc $CFLAGS -DOMNINAV_APP_SINGLE_WHEEL=$macro $INC "$src" 2>&1)
    rc=$?
    printf '%s\n' "$out"
    src_hits=$(printf '%s\n' "$out" | grep -cE "^(\.\./)?(stm32_fw/)?(Src|Inc)/[^:]+:[0-9]+:(warning|error)" || true)
    all_hits=$(printf '%s\n' "$out" | grep -cE '(warning|error):' || true)
    echo "FROM_SRC_DIR_HITS=$src_hits  TOTAL_WARN_ERR_LINES=$all_hits  EXIT=$rc"
    total_warn_from_src=$((total_warn_from_src + src_hits))
  done
done
echo "RESULT=FROM_SRC_TOTAL_HITS=$total_warn_from_src"
echo DONE_EXIT=0
