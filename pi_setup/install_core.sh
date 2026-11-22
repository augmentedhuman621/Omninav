#!/usr/bin/env bash
# =============================================================================
# OmniNav · install_core.sh
# -----------------------------------------------------------------------------
# 做什么 ：在树莓派上编译 firmware_core 的算法层，把产物装到契约冻结路径
#            /opt/omninav/lib/libomninav_core.so
#          依据：11_接口契约.md §5.2（make lib 目标与产物）与 §5.3（查找顺序第 3 条）。
# 在哪跑 ：树莓派 5（aarch64）。**必须在树莓派上跑** —— 要装进 /opt。
# 幂等   ：可以反复跑；产物相同就不重复复制。
# 用法   ：
#            ./install_core.sh                      # 自动找源码目录，跑测试后安装
#            ./install_core.sh ~/omninav/firmware_core
#            ./install_core.sh --skip-tests         # 只编译+安装（赶时间时用）
#            OMNINAV_PREFIX=/opt/omninav ./install_core.sh
# 需要的源码目录从哪来（本脚本不负责搬运）：
#   源码唯一真源在笔记本的 D:\dsh\工程设计\firmware_core\ ，用 scp 送过去：
#     （在笔记本上） scp -r "D:\dsh\工程设计\firmware_core" yjc@omninav-pi.local:~/omninav/
# ★★ 但 scp -r 会**连 build/ 一起拷**，而笔记本上的 build/ 里可能有 x86-64 的
#     libomninav_core.so（WSL 里编出来的）。所以 scp 完在树莓派上删掉 build/：
#     （在树莓派上） rm -rf ~/omninav/firmware_core/build
#     本脚本第 1 步会再查一次架构兜底，但别依赖兜底。
# ★★ 铁律：**绝不要把笔记本上编好的 .so scp 到树莓派上用**（架构不同，加载会
#     Exec format error）。必须在树莓派上从源码编。
# =============================================================================
set -euo pipefail

SKIP_TESTS=0
SRC_ARG=''
for arg in "$@"; do
  case $arg in
    --skip-tests) SKIP_TESTS=1 ;;
    -h | --help)
      printf '用法：%s [源码目录] [--skip-tests]\n' "${0##*/}"
      exit 0
      ;;
    -*)
      printf '✖ 不认识的选项：%s（试 --help）\n' "$arg" >&2
      exit 2
      ;;
    *) SRC_ARG=$arg ;;
  esac
done

PREFIX=${OMNINAV_PREFIX:-/opt/omninav}
LIBDIR=${OMNINAV_LIB_DIR:-$PREFIX/lib}
SO_NAME=libomninav_core.so
DEST=$LIBDIR/$SO_NAME

# 契约 §5.2 里"必须导出的符号"
readonly NEED_SYMS='mecanum_forward mecanum_inverse mecanum_l_mm mecanum_default_geom mecanum_geom_is_valid odom_update odom_reset'

say()  { printf '%s\n' "$*"; }
ok()   { printf '  [OK]   %s\n' "$*"; }
bad()  { printf '  [FAIL] %s\n' "$*"; }
step() { printf '\n── %s ────────────────────────────────────────\n' "$*"; }

need_pi() {
  if [[ ${OMNINAV_SKIP_ARCH_CHECK:-0} == 1 ]]; then
    printf '⚠️  OMNINAV_SKIP_ARCH_CHECK=1：已跳过 aarch64 检查（仅用于逻辑冒烟测试）。\n' >&2
    return 0
  fi
  local m
  m=$(uname -m)
  if [[ $m != aarch64 ]]; then
    printf '✖ 这个脚本要在树莓派 5 上跑（uname -m 应为 aarch64），当前是 %s。\n' "$m" >&2
    printf '  它要编译 aarch64 的 .so 并装进 %s。\n' "$PREFIX" >&2
    printf '  若只想在笔记本上做逻辑冒烟测试：OMNINAV_SKIP_ARCH_CHECK=1 %s --skip-tests\n' "$0" >&2
    return 2
  fi
  return 0
}

# --- 读 ELF 的 e_machine（判断产物架构）-------------------------------------
# ★★ 为什么必须查：笔记本/WSL 是 x86-64，树莓派是 aarch64。
#    把 x86-64 的 .so 弄到树莓派上，装载时会报 Exec format error；
#    比这更阴的是——build/ 里躺着一个错架构的 .so 时，make 可能认为它是最新的，
#    于是"编译成功"却产出错架构的东西。所以这里要独立验一遍。
#    优先用 readelf（binutils，第 4 步反正也要 nm）；退化到 od 读 ELF 头
#    偏移 18 的 e_machine（little-endian 两字节）——od 是 coreutils，必有。
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

# --- 找源码目录 -------------------------------------------------------------
find_src() {
  local -a cands=()
  if [[ -n $SRC_ARG ]]; then cands+=("$SRC_ARG"); fi
  if [[ -n ${FIRMWARE_CORE_DIR:-} ]]; then cands+=("$FIRMWARE_CORE_DIR"); fi
  cands+=(
    "$HOME/omninav/firmware_core"
    "$PREFIX/src/firmware_core"
    "$HOME/firmware_core"
    "/mnt/d/dsh/工程设计/firmware_core"
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
    say "✖ 没找到 firmware_core 源码目录（要有 Makefile）。找过这些位置："
    say "    ${SRC_ARG:-（未给参数）}"
    say "    ${FIRMWARE_CORE_DIR:-（未设 FIRMWARE_CORE_DIR）}"
    say "    $HOME/omninav/firmware_core"
    say "    $PREFIX/src/firmware_core"
    say "    $HOME/firmware_core"
    say "    /mnt/d/dsh/工程设计/firmware_core"
    say ""
    say "  从笔记本搬过去（源码唯一真源在 D:\\dsh\\工程设计\\firmware_core\\ ）："
    say "    scp -r \"D:\\dsh\\工程设计\\firmware_core\" yjc@omninav-pi.local:~/omninav/"
    say "    （scp 完在树莓派上删掉可能一起拷过来的 build/：rm -rf ~/omninav/firmware_core/build）"
    say "  也可以直接指定：  $0 ~/omninav/firmware_core"
    exit 1
  fi
  say ""
  say "  源码目录 ： $src"
  say "  安装目标 ： $DEST"

  # ★ 三项前置分开报错：「没有 make」「没有 C 编译器」「Makefile 没有 lib 目标」
  #   必须是三条分得开的报错，不能都归成一句"共享库装不了"。
  step "前置 · 构建工具链"
  local miss_tool=0 cc_bin=''
  if command -v make >/dev/null 2>&1; then
    ok "make 在：$(command -v make)"
  else
    bad "没有 make"
    say "       → 修法：sudo apt install -y build-essential"
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
    ok "C 编译器在：$(command -v "$cc_bin") —— $(command "$cc_bin" --version 2>/dev/null | head -n1 || true)"
  else
    bad "没有 C 编译器（按 \$CC → cc → gcc 找过，都没有）"
    say "       → 修法：sudo apt install -y build-essential"
    miss_tool=1
  fi
  if [[ $miss_tool -ne 0 ]]; then
    say ""
    say "  ★ 这两项都在 build-essential 里，而**装它需要外网** ——"
    say "    这台树莓派当前没有默认路由（README 第 5 节有实测 FAIL），所以现在装不了。"
    say "    先按 README 第 1 节修好 IPv4，再回来跑本脚本。"
    say "    已经装过的话：command -v make cc 应当各打印一行路径。"
    exit 1
  fi

  # ★ 契约 §5.2 的 lib 目标由 teammate core-algo 提供。它可能还没落地。
  step "第 0 步 · 确认 Makefile 里有 lib 目标"
  if make -C "$src" -n lib >/dev/null 2>&1; then
    ok "Makefile 有 lib 目标"
  else
    bad "Makefile 里没有 lib 目标"
    say ""
    say "  这不是本脚本的问题。契约 11_接口契约.md §5.2 把 make lib 与"
    say "  build/libomninav_core.so 规定为 teammate core-algo 的交付物，"
    say "  它可能还没写进 Makefile。**等它落地后再跑本脚本。**"
    say ""
    say "  现在可以先看 Makefile 里已有哪些目标（应当看到 test / check-purity 等）："
    say "    make -C \"$src\" -pn | grep -E '^[a-zA-Z0-9_-]+:' | head"
    exit 3
  fi

  step "第 1 步 · make lib（★ 在树莓派上从源码编，绝不 scp 现成的 .so）"
  local so="$src/build/$SO_NAME"

  # 见 so_arch() 上面的注释：build/ 里若躺着错架构的 .so，make 可能不重编。
  if [[ -f $so ]]; then
    local pre_arch
    pre_arch=$(so_arch "$so")
    if [[ $pre_arch == AArch64 ]]; then
      ok "build/ 里已有 aarch64 产物（make 自行判断要不要重编）"
    else
      say "  ⚠️  build/ 里已有一个**非 aarch64** 的 $SO_NAME（读到：$pre_arch）"
      say "      很可能是从笔记本 / WSL 复制过来的。它不是给树莓派用的 ⇒ make clean 清掉重编。"
      make -C "$src" clean
      ok "已 make clean"
    fi
  fi

  make -C "$src" lib
  if [[ ! -f $so ]]; then
    bad "make lib 跑完了，但 $so 不存在"
    say "  以契约 §5.2 为准，产物必须落在 build/$SO_NAME。请把这条报给 core-algo。"
    exit 1
  fi
  ok "产物存在：$so"

  # ★★ 硬闸门：产物必须是 aarch64。
  local arch
  arch=$(so_arch "$so")
  if [[ $arch == AArch64 ]]; then
    ok "架构正确：AArch64（树莓派 5）"
  else
    bad "产物架构不是 aarch64，而是：$arch"
    say ""
    say "  **绝对不要安装它。** 装到树莓派上会在 ctypes 加载时报"
    say "  「cannot open shared object file」/「Exec format error」之类的错。"
    say "  原因通常是 build/$SO_NAME 是从笔记本 / WSL 复制过来的，而 make 认为它是最新的。"
    say "  修法："
    say "    make -C \"$src\" clean"
    say "    $0 ${SRC_ARG:-$src}"
    say "  原理：笔记本与 WSL 都是 x86-64，树莓派是 aarch64 —— **共享库必须在树莓派上本机编译**。"
    exit 1
  fi

  if [[ $SKIP_TESTS -eq 0 ]]; then
    step "第 2 步 · 回归测试（契约 §5.2 要求 make test / check-purity 继续全绿）"
    if make -C "$src" test; then
      ok "make test 通过"
    else
      bad "make test 失败 —— **不要安装**，这版算错了"
      exit 1
    fi
    if make -C "$src" check-purity; then
      ok "make check-purity 通过"
    else
      bad "make check-purity 失败 —— 算法层混进了第三方头/浮点/库函数"
      exit 1
    fi
  else
    step "第 2 步 · 回归测试（--skip-tests，已跳过）"
    say "  ⚠️  跳过 make test / check-purity。只在明确知道自己在干什么时这样用。"
  fi

  step "第 3 步 · 安装到 $DEST"
  local sudo_cmd=()
  if [[ ${EUID} -ne 0 ]]; then
    if command -v sudo >/dev/null 2>&1; then
      sudo_cmd=(sudo)
    else
      bad "当前不是 root 且没有 sudo，无法写 $LIBDIR"
      exit 1
    fi
  fi

  "${sudo_cmd[@]}" install -d -m 0755 -- "$LIBDIR"

  local new_sum old_sum='(原来不存在)'
  new_sum=$(sha256sum -- "$so" | cut -d' ' -f1)
  if [[ -f $DEST ]]; then
    old_sum=$(sha256sum -- "$DEST" | cut -d' ' -f1)
  fi
  say "  新产物 sha256 ： $new_sum"
  say "  已装机 sha256 ： $old_sum"

  if [[ $new_sum == "$old_sum" ]]; then
    ok "内容完全相同 —— 仍然复制一次（幂等），结果不变"
  else
    say "  内容有变化，开始复制……"
  fi
  "${sudo_cmd[@]}" install -m 0644 -- "$so" "$DEST"
  "${sudo_cmd[@]}" chown root:root -- "$DEST" 2>/dev/null || true
  ok "已安装：$DEST"

  step "第 4 步 · 校验导出的符号（契约 §5.2 的七个）"
  if ! command -v nm >/dev/null 2>&1; then
    say "  ⚠️  没有 nm（binutils 未装），跳过符号校验。装：sudo apt install -y binutils"
  else
    local defined missing='' sym
    defined=$(nm -D --defined-only -- "$DEST" 2>/dev/null || true)
    for sym in $NEED_SYMS; do
      if ! printf '%s\n' "$defined" | grep -qw -- "$sym"; then
        missing="$missing $sym"
      fi
    done
    if [[ -z $missing ]]; then
      ok "七个符号全在"
    else
      bad "缺少导出符号：$missing"
      say "  桥（stm32_bridge）会因为这些符号找不到而报错退出 ——"
      say "  按契约 §5.3，它是**明确报错**，不会静默降级成 Python 自己算。"
      say "  请把这条报给 core-algo（11_接口契约.md §5.2 的符号表）。"
      exit 1
    fi
  fi

  step "完成"
  say "  你应该看到：$DEST 存在，且 size 与上面 make lib 的产物一致。"
  say ""
  say "  验证（任选，都不需要 sudo）："
  say "    ls -l $DEST"
  say "    nm -D --defined-only $DEST | grep -E 'mecanum_forward|odom_update'"
  say ""
  say "  说明：**不需要跑 ldconfig**。契约 §5.3 规定桥是按绝对路径 ctypes 加载的，"
  say "        不走动态链接器的搜索路径，所以 /etc/ld.so.cache 与本文件无关。"
}

if ! need_pi; then
  exit 2
fi

main
