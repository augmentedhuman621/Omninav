#!/usr/bin/env bash
# =============================================================================
# OmniNav · install_core.sh
# -----------------------------------------------------------------------------
# What  : on the Raspberry Pi, build the firmware_core algorithm layer and install
#         the artifact to the contract-frozen path
#           /opt/omninav/lib/libomninav_core.so
#         Basis: contract doc 11 (interface contract) §5.2 (make lib target and artifact) and §5.3
#         (item 3 of the search order).
# Where : Raspberry Pi 5 (aarch64). **Must run on the Pi** -- it installs into /opt.
# Idempotent: safe to rerun; the artifact is not recopied when unchanged.
# Usage :
#            ./install_core.sh                      # auto-find the source, run tests, install
#            ./install_core.sh ~/omninav/firmware_core
#            ./install_core.sh --skip-tests         # build + install only (when in a hurry)
#            OMNINAV_PREFIX=/opt/omninav ./install_core.sh
# Where the source tree comes from (this script does not copy it):
#   The single source of truth lives on the laptop at <repo>/firmware_core ;
#   send it over with scp:
#     (on the laptop) scp -r "<repo>/firmware_core" yjc@omninav-pi.local:~/omninav/
# ★★ But scp -r also copies build/, and the laptop's build/ may contain an x86-64
#    libomninav_core.so (built in WSL). So delete build/ on the Pi after scp:
#     (on the Pi) rm -rf ~/omninav/firmware_core/build
#    Step 1 of this script double-checks the architecture, but do not rely on it.
# ★★ Iron rule: **never scp a .so built on the laptop onto the Pi** (different
#    architecture; loading fails with Exec format error). Always build from source
#    on the Pi.
# =============================================================================
set -euo pipefail

SKIP_TESTS=0
SRC_ARG=''
for arg in "$@"; do
  case $arg in
    --skip-tests) SKIP_TESTS=1 ;;
    -h | --help)
      printf 'Usage: %s [source dir] [--skip-tests]\n' "${0##*/}"
      exit 0
      ;;
    -*)
      printf '✖ Unknown option: %s (try --help)\n' "$arg" >&2
      exit 2
      ;;
    *) SRC_ARG=$arg ;;
  esac
done

PREFIX=${OMNINAV_PREFIX:-/opt/omninav}
LIBDIR=${OMNINAV_LIB_DIR:-$PREFIX/lib}
SO_NAME=libomninav_core.so
DEST=$LIBDIR/$SO_NAME

# "Must-export symbols" from contract §5.2
readonly NEED_SYMS='mecanum_forward mecanum_inverse mecanum_l_mm mecanum_default_geom mecanum_geom_is_valid odom_update odom_reset'

say()  { printf '%s\n' "$*"; }
ok()   { printf '  [OK]   %s\n' "$*"; }
bad()  { printf '  [FAIL] %s\n' "$*"; }
step() { printf '\n── %s ────────────────────────────────────────\n' "$*"; }

need_pi() {
  if [[ ${OMNINAV_SKIP_ARCH_CHECK:-0} == 1 ]]; then
    printf '⚠️  OMNINAV_SKIP_ARCH_CHECK=1: aarch64 check skipped (logic smoke test only).\n' >&2
    return 0
  fi
  local m
  m=$(uname -m)
  if [[ $m != aarch64 ]]; then
    printf '✖ This script must run on a Raspberry Pi 5 (uname -m should be aarch64), current is %s.\n' "$m" >&2
    printf '  It builds an aarch64 .so and installs it into %s.\n' "$PREFIX" >&2
    printf '  For a logic smoke test on a laptop only: OMNINAV_SKIP_ARCH_CHECK=1 %s --skip-tests\n' "$0" >&2
    return 2
  fi
  return 0
}

# --- read the ELF e_machine (to tell the artifact architecture) ---------------
# ★★ Why this check is mandatory: the laptop/WSL is x86-64, the Pi is aarch64.
#    Moving an x86-64 .so onto the Pi fails at load time with Exec format error;
#    worse, when build/ holds a wrong-architecture .so, make may consider it
#    up to date, so "the build succeeds" yet produces the wrong architecture.
#    Hence the independent verification here.
#    Prefer readelf (binutils, needed for nm in step 4 anyway); fall back to od
#    reading e_machine at ELF header offset 18 (little-endian two bytes) -- od is
#    coreutils and always present.
so_arch() {
  local f=$1 out
  if command -v readelf >/dev/null 2>&1; then
    out=$(readelf -h "$f" 2>/dev/null | sed -n 's/^ *Machine: *//p' | head -n1 || true)
    if [[ -n $out ]]; then
      printf '%s\n' "$out"
      return 0
    fi
  fi
  if command -v od >/dev/null 2>&1; then
    out=$(od -An -tx1 -j18 -N2 "$f" 2>/dev/null | tr -d ' \n' || true)
    case $out in
      b700) printf 'AArch64\n' ;;
      3e00) printf 'Advanced Micro Devices X86-64\n' ;;
      *)    printf 'e_machine=0x%s\n' "${out:-??}" ;;
    esac
    return 0
  fi
  printf 'unknown\n'
}

# --- find the source directory -----------------------------------------------
find_src() {
  local -a cands=()
  if [[ -n $SRC_ARG ]]; then cands+=("$SRC_ARG"); fi
  if [[ -n ${FIRMWARE_CORE_DIR:-} ]]; then cands+=("$FIRMWARE_CORE_DIR"); fi
  cands+=(
    "$HOME/omninav/firmware_core"
    "$PREFIX/src/firmware_core"
    "$HOME/firmware_core"
    "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/firmware_core"
    "./firmware_core"
  )
  local c
  for c in "${cands[@]}"; do
    if [[ -f $c/Makefile ]]; then
      printf '%s\n' "$c"
      return 0
    fi
  done
  return 1
}

main() {
  say "================================================================"
  say " OmniNav · install_core.sh"
  say "================================================================"

  local src
  if ! src=$(find_src); then
    say ""
    say "✖ firmware_core source directory not found (needs a Makefile). Looked in:"
    say "    ${SRC_ARG:-(no argument given)}"
    say "    ${FIRMWARE_CORE_DIR:-(FIRMWARE_CORE_DIR not set)}"
    say "    $HOME/omninav/firmware_core"
    say "    $PREFIX/src/firmware_core"
    say "    $HOME/firmware_core"
    say "    <repo>/firmware_core  (the firmware_core/ directory next to pi_setup/)"
    say ""
    say "  Copy it over from the laptop (single source of truth at <repo>/firmware_core):"
    say "    scp -r <repo>/firmware_core yjc@omninav-pi.local:~/omninav/"
    say "    (after scp, delete any build/ that came along on the Pi: rm -rf ~/omninav/firmware_core/build)"
    say "  Or point at it directly:  $0 ~/omninav/firmware_core"
    exit 1
  fi
  say ""
  say "  source dir : $src"
  say "  install to : $DEST"

  # ★ Report the three preconditions separately: "no make", "no C compiler",
  #   "Makefile has no lib target" must be three distinct errors, not one
  #   blanket "cannot install the shared library".
  step "Prerequisites · build toolchain"
  local miss_tool=0 cc_bin=''
  if command -v make >/dev/null 2>&1; then
    ok "make found: $(command -v make)"
  else
    bad "no make"
    say "       → Fix: sudo apt install -y build-essential"
    miss_tool=1
  fi
  if [[ -n ${CC:-} ]] && command -v "$CC" >/dev/null 2>&1; then
    cc_bin=$CC
  elif command -v cc >/dev/null 2>&1; then
    cc_bin=cc
  elif command -v gcc >/dev/null 2>&1; then
    cc_bin=gcc
  fi
  if [[ -n $cc_bin ]]; then
    ok "C compiler found: $(command -v "$cc_bin") -- $(command "$cc_bin" --version 2>/dev/null | head -n1 || true)"
  else
    bad "no C compiler (looked for \$CC -> cc -> gcc, none present)"
    say "       → Fix: sudo apt install -y build-essential"
    miss_tool=1
  fi
  if [[ $miss_tool -ne 0 ]]; then
    say ""
    say "  ★ Both come from build-essential, and **installing it needs internet** --"
    say "    this Pi currently has no default route (README section 5 has the measured FAIL), so it cannot be installed now."
    say "    Fix IPv4 per README section 1 first, then rerun this script."
    say "    If they are already installed: command -v make cc should each print a path."
    exit 1
  fi

  # ★ The lib target from contract §5.2 is delivered by teammate core-algo. It may not exist yet.
  step "Step 0 · confirm the Makefile has a lib target"
  if make -C "$src" -n lib >/dev/null 2>&1; then
    ok "Makefile has a lib target"
  else
    bad "Makefile has no lib target"
    say ""
    say "  This is not a problem with this script. Contract contract doc 11 (interface contract) §5.2 defines make lib and"
    say "  build/libomninav_core.so as teammate core-algo's deliverable,"
    say "  and it may not be written into the Makefile yet. **Rerun this script once it lands.**"
    say ""
    say "  For now you can list the existing targets (you should see test / check-purity etc.):"
    say "    make -C \"$src\" -pn | grep -E '^[a-zA-Z0-9_-]+:' | head"
    exit 3
  fi

  step "Step 1 · make lib (★ build from source on the Pi, never scp a prebuilt .so)"
  local so="$src/build/$SO_NAME"

  # See the comment above so_arch(): a wrong-architecture .so sitting in build/ may stop make from rebuilding.
  if [[ -f $so ]]; then
    local pre_arch
    pre_arch=$(so_arch "$so")
    if [[ $pre_arch == AArch64 ]]; then
      ok "build/ already contains an aarch64 artifact (make decides whether to rebuild)"
    else
      say "  ⚠️  build/ already contains a **non-aarch64** $SO_NAME (read: $pre_arch)"
      say "      It most likely came from the laptop / WSL. It is not for the Pi => make clean and rebuild."
      make -C "$src" clean
      ok "ran make clean"
    fi
  fi

  make -C "$src" lib
  if [[ ! -f $so ]]; then
    bad "make lib finished but $so does not exist"
    say "  Per contract §5.2 the artifact must land at build/$SO_NAME. Report this to core-algo."
    exit 1
  fi
  ok "artifact present: $so"

  # ★★ Hard gate: the artifact must be aarch64.
  local arch
  arch=$(so_arch "$so")
  if [[ $arch == AArch64 ]]; then
    ok "correct architecture: AArch64 (Raspberry Pi 5)"
  else
    bad "artifact architecture is not aarch64 but: $arch"
    say ""
    say "  **Never install it.** On the Pi, ctypes will fail to load it with errors like"
    say "  \"cannot open shared object file\" / \"Exec format error\"."
    say "  The usual cause is that build/$SO_NAME was copied from the laptop / WSL and make thinks it is up to date."
    say "  Fix:"
    say "    make -C \"$src\" clean"
    say "    $0 ${SRC_ARG:-$src}"
    say "  Rationale: laptop and WSL are x86-64, the Pi is aarch64 -- **shared libraries must be built on the Pi itself**."
    exit 1
  fi

  if [[ $SKIP_TESTS -eq 0 ]]; then
    step "Step 2 · regression tests (contract §5.2 requires make test / check-purity to stay green)"
    if make -C "$src" test; then
      ok "make test passed"
    else
      bad "make test failed -- **do not install**, this build computes wrong results"
      exit 1
    fi
    if make -C "$src" check-purity; then
      ok "make check-purity passed"
    else
      bad "make check-purity failed -- third-party headers / floats / library calls leaked into the algorithm layer"
      exit 1
    fi
  else
    step "Step 2 · regression tests (--skip-tests, skipped)"
    say "  ⚠️  skipping make test / check-purity. Only do this when you know exactly what you are doing."
  fi

  step "Step 3 · install to $DEST"
  local sudo_cmd=()
  if [[ ${EUID} -ne 0 ]]; then
    if command -v sudo >/dev/null 2>&1; then
      sudo_cmd=(sudo)
    else
      bad "not root and no sudo, cannot write $LIBDIR"
      exit 1
    fi
  fi

  "${sudo_cmd[@]}" install -d -m 0755 -- "$LIBDIR"

  local new_sum old_sum='(not present before)'
  new_sum=$(sha256sum -- "$so" | cut -d' ' -f1)
  if [[ -f $DEST ]]; then
    old_sum=$(sha256sum -- "$DEST" | cut -d' ' -f1)
  fi
  say "  new artifact sha256 : $new_sum"
  say "  installed sha256    : $old_sum"

  if [[ $new_sum == "$old_sum" ]]; then
    ok "contents identical -- copying once anyway (idempotent), result unchanged"
  else
    say "  contents changed, copying..."
  fi
  "${sudo_cmd[@]}" install -m 0644 -- "$so" "$DEST"
  "${sudo_cmd[@]}" chown root:root -- "$DEST" 2>/dev/null || true
  ok "installed: $DEST"

  step "Step 4 · verify exported symbols (the seven from contract §5.2)"
  if ! command -v nm >/dev/null 2>&1; then
    say "  ⚠️  no nm (binutils not installed), skipping symbol verification. Install: sudo apt install -y binutils"
  else
    local defined missing='' sym
    defined=$(nm -D --defined-only -- "$DEST" 2>/dev/null || true)
    for sym in $NEED_SYMS; do
      if ! printf '%s\n' "$defined" | grep -qw -- "$sym"; then
        missing="$missing $sym"
      fi
    done
    if [[ -z $missing ]]; then
      ok "all seven symbols present"
    else
      bad "missing exported symbols: $missing"
      say "  The bridge (stm32_bridge) will exit with an error because these symbols are missing --"
      say "  per contract §5.3 it **fails loudly** and never silently degrades to computing in Python."
      say "  Report this to core-algo (the symbol table in contract doc 11 (interface contract) §5.2)."
      exit 1
    fi
  fi

  step "Done"
  say "  You should see: $DEST exists, with a size matching the make lib artifact above."
  say ""
  say "  Verify (either works, neither needs sudo):"
  say "    ls -l $DEST"
  say "    nm -D --defined-only $DEST | grep -E 'mecanum_forward|odom_update'"
  say ""
  say "  Note: **no ldconfig needed**. Contract §5.3 specifies that the bridge loads it"
  say "        via ctypes by absolute path, bypassing the dynamic loader search path, so"
  say "        /etc/ld.so.cache is irrelevant here."
}

if ! need_pi; then
  exit 2
fi

main
