#!/usr/bin/env bash
# =============================================================================
# OmniNav · identify_ports.sh
# -----------------------------------------------------------------------------
# 目的   ：两分钟之内把「哪个 USB 物理口是雷达、哪个是视觉模块通信板」这个
#          **未知量变成已知量**，并直接生成能拷进 /etc/udev/rules.d/ 的规则文本。
# 在哪跑 ：树莓派 5（aarch64）。必须在树莓派上跑 —— 它读的是 /dev/serial/by-path。
# 依赖   ：**只用 shell 内建 + ls**（验收标准 2）。不需要 sudo，不需要 root。
# 输出   ：• 所有交互提示、表格 → **stderr**（所以重定向 stdout 也看得见）
#          • 生成的 udev 规则   → **stdout**
#          用法：  ./identify_ports.sh > /tmp/99-omninav-serial.rules
# 为什么拔插就能判定：两颗 CH340 型号完全相同、都没有唯一序列号，
#          唯一能区分的线索就是"它插在哪个物理口"。而"拔掉哪一根、
#          哪个 by-path 条目消失"正好给出这个映射，且不需要读任何数据。
# ★ usbv2：新版 udev 会为同一个物理口同时建 `usb-` 和 `usbv2-` **两条** by-path
#          名字（指向同一颗芯片）。udev 的 ID_PATH 报的是 `usb-` 那条，所以本脚本
#          **只列不带 usbv2 的那一条**，避免"插了 2 个设备却列出 4 条"以及抄错。
# =============================================================================
set -euo pipefail

readonly BY_PATH_DIR=/dev/serial/by-path

# --- 输出通道 ---------------------------------------------------------------
UI()  { printf '%s\n' "$*" >&2; }   # 给人看
OUT() { printf '%s\n' "$*"; }       # 生成物

hr() { UI "----------------------------------------------------------------"; }

need_pi() {
  if [[ ${OMNINAV_SKIP_ARCH_CHECK:-0} == 1 ]]; then
    UI "⚠️  OMNINAV_SKIP_ARCH_CHECK=1：已跳过 aarch64 检查（仅用于在笔记本上做逻辑冒烟测试）。"
    return 0
  fi
  local m
  m=$(uname -m)
  if [[ $m != aarch64 ]]; then
    UI "✖ 这个脚本要在树莓派 5 上跑（uname -m 应为 aarch64），当前是 $m。"
    UI "  它读的是树莓派的 /dev/serial/by-path，在别的机器上跑没有意义。"
    UI "  若只想在笔记本上做逻辑冒烟测试：OMNINAV_SKIP_ARCH_CHECK=1 ./identify_ports.sh"
    return 2
  fi
  return 0
}

# --- 数一数有几条 usbv2 重复项（只用于在 main 里提示一次） --------------------
# 用 glob 而不是 ls / grep，保持"只用 shell 内建 + ls"的约束。
# 没有任何匹配时 glob 会原样保留模式串，此时 -e 与 -L 都为假 ⇒ 自动跳过。
count_usbv2() {
  local f n=0
  for f in "$BY_PATH_DIR"/*-usbv2-*; do
    if [[ ! -e $f && ! -L $f ]]; then
      continue
    fi
    n=$((n + 1))
  done
  printf '%s' "$n"
}

# --- by-path 链接名 → udev 的 ID_PATH 值 -------------------------------------
# ★★★ 这是本项目最容易踩、而且**完全静默**的一个坑：
#   /dev/serial/by-path/ 里的链接名，是 udev 用
#       $ID_PATH + "-port" + $ID_USB_INTERFACE_NUM
#   拼出来的（见 /usr/lib/udev/rules.d/60-serial.rules）。也就是链接名**多一个
#   `-port0` 尾巴**，而 ENV{ID_PATH} 本身没有：
#
#       链接名  platform-xhci-hcd.0-usb-0:2:1.0-port0
#       ID_PATH platform-xhci-hcd.0-usb-0:2:1.0          ← 规则里必须写这个
#
#   真机实测（2026-10，树莓派 5）：
#       /dev/ttyUSB0 → ID_PATH=platform-xhci-hcd.0-usb-0:2:1.0
#       /dev/ttyUSB1 → ID_PATH=platform-xhci-hcd.1-usb-0:1:1.0
#
#   所以生成的规则里**必须把 `-port0` 去掉** —— 多这一个尾巴就永远匹配不上，
#   而 udev 不会报任何错，症状只有"规则装了、reload 了，软链接就是不出来"。
#   末尾没有 -port<N> 时原样返回。
idpath_of() {
  printf '%s' "${1%-port[0-9]*}"
}

# --- 快照：把 /dev/serial/by-path 解析成「物理口<TAB>设备名」 -----------------
# 只用 shell 内建 + ls：
#   ls -l /dev/serial/by-path/xxx  →  "lrwxrwxrwx ... xxx -> ../../ttyUSB0"
#   取 " -> " 之后的部分，再去掉目录前缀，就得到 ttyUSB0。
#
# ★ 这里**静默**丢掉带 "-usbv2-" 的条目：它与不带的那条指向同一个 ttyUSBn，
#   而 udev 的 ENV{ID_PATH} 用的是不带 usbv2 的那条。提示文字由 main 打印一次
#   （不能在这里打印 —— snapshot 是在进程替换的子 shell 里跑的，去重标志存不住，
#    会把同一句话重复三遍）。
snapshot() {
  local -a entries=() keep=()
  local f base lnk tgt t
  if [[ ! -d $BY_PATH_DIR ]]; then
    return 1
  fi
  for f in "$BY_PATH_DIR"/*; do
    if [[ ! -e $f && ! -L $f ]]; then
      continue                      # 目录为空时 glob 会原样留下 "*"
    fi
    entries+=("$f")
  done
  if [[ ${#entries[@]} -eq 0 ]]; then
    return 0
  fi

  for f in "${entries[@]}"; do
    base=${f##*/}
    if [[ $base != *-usbv2-* ]]; then
      keep+=("$f")
    fi
  done
  if [[ ${#keep[@]} -gt 0 ]]; then
    entries=("${keep[@]}")
  fi
  # 兜底：万一整套 by-path 只提供 usbv2 形式，绝不能把设备全丢掉 ——
  # 此时退回未过滤的完整列表（README 第 3 节末尾写了这种情况怎么改 udev 匹配键）。

  for f in "${entries[@]}"; do
    base=${f##*/}
    lnk=$(ls -l -- "$f" 2>/dev/null || true)
    tgt=${lnk##* -> }
    if [[ $tgt == "$lnk" ]]; then
      t=$'\t(无法解析)'             # 不是软链接，或 ls 输出格式意外
    else
      t=${tgt##*/}
    fi
    printf '%s\t%s\n' "$base" "$t"
  done
  return 0
}

# 把快照读进数组（跳过空行）
load_snapshot() {
  local -n _out=$1                 # bash 4.3+ 的 nameref；Ubuntu 24.04 的 bash 5.x 支持
  _out=()
  local line
  while IFS= read -r line; do
    if [[ -n $line ]]; then
      _out+=("$line")
    fi
  done < <(snapshot || true)
}

# 数组里是否含有某个物理口名
has_path() {
  local needle=$1; shift
  local line
  for line in "$@"; do
    if [[ ${line%%$'\t'*} == "$needle" ]]; then
      return 0
    fi
  done
  return 1
}

print_table() {
  local line i=0 name dev
  for line in "$@"; do
    i=$((i + 1))
    name=${line%%$'\t'*}
    dev=${line##*$'\t'}
    UI "$(printf '  %-3s %-46s %s' "$i" "$name" "$dev")"
  done
}

# --- 主流程 -----------------------------------------------------------------
main() {
  UI "================================================================"
  UI " OmniNav · USB 串口口位判定（identify_ports.sh）"
  UI "================================================================"
  UI ""
  UI "它会让你拔一根线、再插回去，全程一分钟左右，不改动系统任何设置。"
  UI ""

  if [[ ! -d $BY_PATH_DIR ]]; then
    UI "✖ $BY_PATH_DIR 不存在。"
    UI "  可能原因：① 一个 USB 串口设备都没插；② 内核没加载 usbserial 相关模块。"
    UI "  先确认雷达和视觉模块的 USB 线都插在树莓派上，再重跑。"
    exit 1
  fi

  local -a before=() after=() gone=() remain=()
  load_snapshot before

  if [[ ${#before[@]} -eq 0 ]]; then
    UI "✖ $BY_PATH_DIR 是空的 —— 没有任何 USB 串口设备被枚举出来。"
    UI "  先插好设备（插上后 1–2 秒内核才建节点），再重跑本脚本。"
    exit 1
  fi

  hr
  local usbv2_n
  usbv2_n=$(count_usbv2)
  if [[ $usbv2_n -gt 0 ]]; then
    UI "ℹ️  /dev/serial/by-path/ 里另有 $usbv2_n 条「usbv2」重复项 —— 那是**同一个物理口**的"
    UI "    另一种命名（指向同一颗 CH340），udev 的 ID_PATH 用的是不带 usbv2 的那条。"
    UI "    下面这张表已经把它们去掉了，所以条数 = 真实设备数。"
    UI ""
  fi
  UI "第 0 步 · 现在枚举到的 USB 串口设备共 ${#before[@]} 个："
  UI ""
  UI "  #   物理口（= /dev/serial/by-path/ 的名字去掉 usbv2 重复项，也就是 ID_PATH）   当前设备名"
  print_table "${before[@]}"
  UI ""

  if [[ ${#before[@]} -lt 2 ]]; then
    UI "⚠️  只看到 ${#before[@]} 个。本项目的正常情况应该**至少 2 个**（雷达 + 视觉模块）。"
    UI "   继续也可以，但结论只能定出这一个设备是什么。"
    UI ""
  fi

  hr
  UI "第 1 步 · 拔线"
  UI ""
  UI "  请**只拔掉一根** USB 线。建议拔视觉模块那根（通信板上的 Type-C / CH340 线），"
  UI "  因为它最好拔。★ 记住你拔的是哪一个，下一步要回答。"
  UI ""
  UI "  拔掉后按回车继续……"
  read -r _ || true

  load_snapshot after

  local line
  for line in "${before[@]}"; do
    if ! has_path "${line%%$'\t'*}" "${after[@]}"; then
      gone+=("$line")
    fi
  done

  if [[ ${#gone[@]} -eq 0 ]]; then
    UI ""
    UI "✖ 没有任何设备消失。可能是：① 你没拔；② 拔的是别的东西（比如网线、电源）；"
    UI "  ③ 拔出后内核还没反应过来 —— 请等 2 秒重跑一次本脚本。"
    exit 1
  fi
  if [[ ${#gone[@]} -gt 1 ]]; then
    UI ""
    UI "⚠️  同时消失了 ${#gone[@]} 个设备："
    print_table "${gone[@]}"
    UI "  你多半拔的是一根一拖多的线或者 USB 集线器。请改成都只拔一根，重跑本脚本。"
    exit 1
  fi

  local unplugged_path unplugged_dev
  unplugged_path=${gone[0]%%$'\t'*}
  unplugged_dev=${gone[0]##*$'\t'}

  hr
  UI "第 2 步 · 确认你拔的是哪一个"
  UI ""
  UI "  被拔掉的物理口是： $unplugged_path"
  UI "  （它刚才对应的设备名是 $unplugged_dev，现在应该已经不见了）"
  UI ""
  UI "  它是什么？ 输入 1 = 视觉模块通信板（ESP32-S3），2 = 激光雷达，然后回车："

  local which=''
  while true; do
    # ★ 不能用 `read -r which || true`：stdin 是 /dev/null 或 EOF 时 read 立刻失败，
    #   which 永远是空串，这个 while 会**死循环**（打印无数遍"请输入 1 或 2"）。
    if ! read -r which; then
      UI ""
      UI "✖ 标准输入已结束（不像是在交互终端里跑的）—— 第 2 步没法回答，退出。"
      UI "  请在树莓派的终端里直接跑：./pi_setup/identify_ports.sh"
      exit 1
    fi
    case $which in
      1 | 2) break ;;
      *) UI "  请输入 1 或 2：" ;;
    esac
  done

  local vision_path='' vision_dev='' lidar_path='' lidar_dev=''
  if [[ $which == 1 ]]; then
    vision_path=$unplugged_path; vision_dev=$unplugged_dev
  else
    lidar_path=$unplugged_path;  lidar_dev=$unplugged_dev
  fi

  hr
  UI "第 3 步 · 把它插回**原来那个口**（换口的话物理路径就变了，结论作废），按回车"
  read -r _ || true

  load_snapshot after

  if ! has_path "$unplugged_path" "${after[@]}"; then
    UI ""
    UI "✖ 那个口还是没有回来。请确认：① 线插紧了；② 插的是原来那个口；③ 等 2 秒再重跑。"
    exit 1
  fi
  UI "  ✅ 回来了：$unplugged_path"

  # 剩下的设备：如果正好剩一个，那个就是另一个角色
  local l
  for l in "${after[@]}"; do
    if [[ ${l%%$'\t'*} != "$unplugged_path" ]]; then
      remain+=("$l")
    fi
  done

  if [[ ${#remain[@]} -eq 1 ]]; then
    if [[ $which == 1 ]]; then
      lidar_path=${remain[0]%%$'\t'*};  lidar_dev=${remain[0]##*$'\t'}
    else
      vision_path=${remain[0]%%$'\t'*}; vision_dev=${remain[0]##*$'\t'}
    fi
  fi

  hr
  local lidar_idpath='' vision_idpath=''
  if [[ -n $lidar_path ]];  then lidar_idpath=$(idpath_of "$lidar_path");   fi
  if [[ -n $vision_path ]]; then vision_idpath=$(idpath_of "$vision_path"); fi

  UI "结论"
  UI ""
  UI "  激光雷达      ： ${lidar_idpath:-（未判定）}"
  if [[ -n $lidar_path ]]; then
    UI "                  （设备 ${lidar_dev:-?}；by-path 链接名 $lidar_path）"
  fi
  UI "  视觉模块通信板： ${vision_idpath:-（未判定）}"
  if [[ -n $vision_path ]]; then
    UI "                  （设备 ${vision_dev:-?}；by-path 链接名 $vision_path）"
  fi
  UI ""
  UI "  ★ 上面「激光雷达 / 视觉模块通信板」后面那两串就是 ENV{ID_PATH} 的值。"
  UI "    注意它**不带 -port0 尾巴** —— by-path 的链接名有、ID_PATH 没有，"
  UI "    规则里写错这个尾巴会静默失效（详见规则文件开头「陷阱一」）。"
  UI ""
  if [[ ${#remain[@]} -gt 1 ]]; then
    UI "  ⚠️  还剩 ${#remain[@]} 个设备没能一次判定完（你可能还插着 STM32 的 USB-TTL）："
    print_table "${remain[@]}"
    UI "      → 再跑一次本脚本、这次拔掉另一个，就能把它们分开。"
    UI ""
  fi
  UI "  自检：把上面两条结论里的物理口与你自己拔插的观察对一下，"
  UI "        如果相反，说明第 2 步选错了，重跑一次即可（不会有任何副作用）。"
  UI ""

  # ---------------- 生成物 → stdout ----------------
  OUT "# ============================================================================="
  OUT "# OmniNav · USB 串口设备固定命名（由 pi_setup/identify_ports.sh 实测生成）"
  OUT "# 装到： /etc/udev/rules.d/99-omninav-serial.rules"
  OUT "# 生成它的那次拔插结论（括号里是 by-path 链接名，规则里用的是它去掉 -port0 的茎）："
  OUT "#   激光雷达       = ${lidar_idpath:-未判定}   ${lidar_path:+($lidar_path)}"
  OUT "#   视觉模块通信板 = ${vision_idpath:-未判定}   ${vision_path:+($vision_path)}"
  OUT "# ============================================================================="
  OUT ""
  if [[ -n $lidar_idpath ]]; then
    OUT "SUBSYSTEM==\"tty\", KERNEL==\"ttyUSB*\", ENV{ID_PATH}==\"$lidar_idpath\", SYMLINK+=\"ttyLIDAR\", GROUP=\"dialout\", MODE=\"0660\""
  else
    OUT "# 【未判定】激光雷达那一行没生成出来 —— 重跑 identify_ports.sh"
  fi
  OUT ""
  if [[ -n $vision_idpath ]]; then
    OUT "SUBSYSTEM==\"tty\", KERNEL==\"ttyUSB*\", ENV{ID_PATH}==\"$vision_idpath\", SYMLINK+=\"ttyVISION\", GROUP=\"dialout\", MODE=\"0660\""
  else
    OUT "# 【未判定】视觉模块那一行没生成出来 —— 重跑 identify_ports.sh"
  fi
  OUT ""
  OUT "# STM32 串口桥用的独立 USB-TTL 模块（现在还没接上，所以无法预先知道它的口）："
  OUT "#   插上它，看 /dev/ttyUSB* 新增的是哪个（假设 ttyUSB2），然后取它的 ID_PATH："
  OUT "#     udevadm info -q property -n /dev/ttyUSB2 | grep '^ID_PATH='"
  OUT "#   把 = 后面那一整串填进下面、去掉行首 #。★ 不要抄 by-path 链接名（会多个 -port0）。"
  OUT "# SUBSYSTEM==\"tty\", KERNEL==\"ttyUSB*\", ENV{ID_PATH}==\"<填这里>\", SYMLINK+=\"ttySTM32\", GROUP=\"dialout\", MODE=\"0660\""

  UI "=================================================================="
  UI "已经把规则文本打印到 **stdout**。存下来并安装："
  UI ""
  UI "  ./identify_ports.sh > /tmp/99-omninav-serial.rules   # 若已重定向则跳过"
  UI "  ★ 核对一遍： udevadm info -q property -n /dev/ttyUSB0 | grep '^ID_PATH='"
  UI "  sudo cp /tmp/99-omninav-serial.rules /etc/udev/rules.d/"
  UI "  sudo udevadm control --reload-rules"
  UI "  sudo udevadm trigger --subsystem-match=tty"
  UI "  ls -l /dev/ttyLIDAR /dev/ttyVISION"
  UI ""
  UI "最后一条必须打印出两个指向 ttyUSBn 的软链接（详见 pi_setup/README.md 第 3 节）。"
  UI "=================================================================="
}

if ! need_pi; then
  exit 2
fi

main "$@"
