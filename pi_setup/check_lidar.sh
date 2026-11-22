#!/usr/bin/env bash
# =============================================================================
# OmniNav · check_lidar.sh —— 树莓派上"激光雷达能不能出数据"的只读诊断
# -----------------------------------------------------------------------------
# 做什么（逐段打印，最后给结论）：
#   0 基础信息 / 1 权限与 dialout / 2 USB 枚举 / 3 设备节点与 udev /
#   4 CH340 与型号线索 / 5 驱动包是否已装 / 6 启动命令参考（只打印不执行）/
#   7 被动数据流探测（--probe）/ 8 ROS2 图与 /scan、TF 现状 / 9 判定与下一步
#
# 安全性（硬约束，和任务单一致）：
#   * 默认**只读**：不安装任何包、不改 /etc、不写文件、不重启服务。
#   * 唯一会碰设备运行时参数的是 --probe 段里的 `stty`（设波特率）：它只改该设备
#     当前的 termios，拔插/重启即恢复，不落盘。--request-rplidar 会向设备写 2 字节
#     请求命令，默认关闭，必须显式打开。
#   * 需要 root 的段**不会自己 sudo**：脚本只打印你要手动执行的命令。
#
# 在哪跑：树莓派 5（aarch64，Ubuntu 24.04 + ROS2 Jazzy）。
#         Windows 笔记本请用 check_lidar.md 的 Get-PnpDevice 段；本脚本非 Linux 直接退出。
#
# 用法：
#   bash check_lidar.sh
#   bash check_lidar.sh --probe
#   bash check_lidar.sh --probe --device /dev/ttyUSB0     # 用户已确认设备时（空格形式）
#   bash check_lidar.sh --probe --device=/dev/ttyUSB0     # 等号形式等价，两种都收
#   sudo bash check_lidar.sh --probe                       # 未加 dialout 组时的临时办法
#   bash check_lidar.sh --probe --request-rplidar          # 无数据时再试 RPLIDAR 请求
#
# 退出码：0 = 无 FAIL；1 = 有 FAIL；2 = 环境不对 / 参数错 / --help
# 依据：11_接口契约.md §2/§4、pi_setup/README.md 第 6 节、03_底盘物料清单.md §C、
#       04_底盘采购记账.md。姊妹脚本：pi_setup/preflight.sh（用法一致）。
# =============================================================================
set -uo pipefail

# ---- 冻结事实（改这里等于改契约，先问 Lead）--------------------------------
LIDAR_NODE=/dev/ttyLIDAR
VISION_NODE=/dev/ttyVISION
STM32_NODE=/dev/ttySTM32
RULES_FILE=/etc/udev/rules.d/99-omninav-serial.rules
#: 99-omninav-serial.rules 现在按 ID_PATH 区分两个同型号 CH340（1a86:7522）。
#: 下面是真机实测的历史值；换 USB 口后会变，变了就是 WARN 不是 FAIL。
EXPECTED_LIDAR_ID_PATH="platform-xhci-hcd.0-usb-0:2:1.0"
EXPECTED_VISION_ID_PATH="platform-xhci-hcd.1-usb-0:1:1.0"
#: 候选驱动包（实际装没装，以 /opt/ros/jazzy/share 与 overlay 为准）
DRIVER_CANDIDATES="rplidar_ros ldlidar_stl_ros2 ldlidar_ros2 ydlidar_ros2_driver ydlidar_ros2 urg_node"
#: 被动探测的候选波特率（LD19 常见 230400；RPLIDAR A1 115200；别的雷达也有 256000）
PROBE_BAUDS="115200 230400 256000 460800"

DO_PROBE=0
DO_REQUEST=0
PROBE_DEV=""

while [[ $# -gt 0 ]]; do
  case $1 in
    --probe)
      DO_PROBE=1
      shift
      ;;
    --request-rplidar)
      DO_REQUEST=1
      shift
      ;;
    --device=*)
      PROBE_DEV=${1#--device=}
      if [[ -z $PROBE_DEV ]]; then
        printf '✖ --device= 后面不能为空：--device=/dev/ttyUSB0\n' >&2
        exit 2
      fi
      shift
      ;;
    --device)
      if [[ $# -lt 2 || -z ${2:-} || ${2:0:1} == - ]]; then
        printf '✖ --device 需要带参数：--device /dev/ttyUSB0（也接受 --device=/dev/ttyUSB0）\n' >&2
        exit 2
      fi
      PROBE_DEV=$2
      shift 2
      ;;
    -h | --help)
      printf '用法：%s [--probe] [--device DEV | --device=DEV] [--request-rplidar]\n' "${0##*/}"
      printf '  --probe            额外做被动数据流探测（每波特率 3 秒，只读）\n'
      printf '  --device DEV       指定探测设备（空格形式；默认 /dev/ttyLIDAR；不给而节点不存在则跳过探测）\n'
      printf '  --device=DEV       同上，等号形式，两种写法等价\n'
      printf '  --request-rplidar  无数据时向设备写 RPLIDAR 请求 A5 20（会写设备，默认关；需同时给 --probe）\n'
      exit 2
      ;;
    *)
      printf '✖ 不认识的选项：%s（试 --help）\n' "$1" >&2
      exit 2
      ;;
  esac
done

PASS=0
FAIL=0
WARN=0
N7522=0
N7523=0
NCP210=0
NTOTAL=0
DRIVER_FOUND=""
DRIVER_LIST=""
LIDAR_NODE_OK=0
PROBE_RUN=0
PROBE_BYTES=0
PROBE_STTY_FAIL=0
PROBE_ENTRIES=()

ok() {
  PASS=$((PASS + 1))
  printf '[PASS] %s\n' "$1"
  if [[ -n ${2:-} ]]; then printf '       %s\n' "$2"; fi
  return 0
}
bad() {
  FAIL=$((FAIL + 1))
  printf '[FAIL] %s\n' "$1"
  if [[ -n ${2:-} ]]; then printf '       → 修法：%s\n' "$2"; fi
  return 0
}
warn() {
  WARN=$((WARN + 1))
  printf '[WARN] %s\n' "$1"
  if [[ -n ${2:-} ]]; then printf '       → %s\n' "$2"; fi
  return 0
}
info() { printf '[INFO] %s\n' "$*"; }
sec() { printf '\n=== %s ===\n' "$*"; }

need_pi() {
  if [[ ${OMNINAV_SKIP_ARCH_CHECK:-0} == 1 ]]; then
    printf '⚠️  OMNINAV_SKIP_ARCH_CHECK=1：已跳过 aarch64 检查（仅用于逻辑冒烟测试）。\n' >&2
    return 0
  fi
  if [[ $(uname -m) != aarch64 ]]; then
    printf '✖ check_lidar.sh 要在树莓派 5 上跑（uname -m 应为 aarch64），当前是 %s。\n' "$(uname -m)" >&2
    printf '  Windows 笔记本请用 check_lidar.md 的 Get-PnpDevice 段（只能证明枚举，不能证明扫描）。\n' >&2
    printf '  只验证脚本逻辑：OMNINAV_SKIP_ARCH_CHECK=1 %s\n' "$0" >&2
    return 2
  fi
  return 0
}

# 包是否已装：先看 /opt/ros/*，再看 $HOME/ros2_ws 的 overlay
pkg_present() {
  local p=$1 d
  for d in /opt/ros/*/share/"$p"; do [[ -d $d ]] && return 0; done
  for d in "$HOME"/ros2_ws/install/*/share/"$p"; do [[ -d $d ]] && return 0; done
  return 1
}

hex_of() { # stdin -> 单行小写十六进制
  od -An -v -tx1 2>/dev/null | tr -d ' \n'
}

# 被动收字节：stty 由调用方先做（好把"设不了 termios"与"0 字节"分开报），这里只读 ${2} 秒
probe_read() {
  local dev=$1 secs=$2
  timeout "$secs" cat "$dev" 2>/dev/null | hex_of
  return 0
}

# 打印 udev 关键字段（一个 ttyUSB 一行）
tty_usb_info() {
  local t=$1
  if command -v udevadm >/dev/null 2>&1 && [[ -e /dev/$t ]]; then
    udevadm info -q property -n "/dev/$t" 2>/dev/null |
      awk -F= '/^ID_VENDOR_ID=/{v=$2} /^ID_MODEL_ID=/{m=$2} /^ID_MODEL=/{n=$2} /^ID_PATH=/{p=$2}
                END{printf "vendor=%s model_id=%s model=%s id_path=%s\n", v, m, n, p}'
  else
    printf '（udevadm 不可用或 /dev/%s 不存在）\n' "$t"
  fi
}

main() {
  printf '================================================================\n'
  printf ' OmniNav · 激光雷达只读诊断 check_lidar.sh\n'
  printf ' 默认不安装、不改 /etc、不写文件；--probe 只用 stty 设运行时波特率\n'
  printf '================================================================\n'

  # ---------------------------------------------------------- 0 基础信息
  sec "0. 基础信息"
  local m
  m=$(uname -m)
  info "uname      = $(uname -s) / $m / $(uname -r)"
  info "主机名     = $(uname -n)"
  info "用户       = $(id -un) (uid=$(id -u))"
  info "系统       = $(. /etc/os-release 2>/dev/null && printf '%s %s' "$NAME" "$VERSION_ID" || printf '(读不到 os-release)')"
  info "内存       = $(free -h 2>/dev/null | awk 'NR==2{print $2" 总 / "$7" 可用"}' || printf '(无 free)')"
  info "python3    = $(command -v python3 >/dev/null 2>&1 && python3 --version 2>&1 || printf '(无)')"
  info "ROS_DISTRO = ${ROS_DISTRO:-(空 —— 先 source /opt/ros/jazzy/setup.bash)}"
  info "ROS_DOMAIN_ID = ${ROS_DOMAIN_ID:-(未设置；预期 42)}"
  info "ros2 CLI   = $(command -v ros2 >/dev/null 2>&1 && printf '有' || printf '没有')"
  info "lsusb      = $(command -v lsusb >/dev/null 2>&1 && printf '有' || printf '没有（不影响，脚本有 /sys 兜底）')"
  info "timeout    = $(command -v timeout >/dev/null 2>&1 && printf '有' || printf '没有（--probe 会跳过）')"
  info "stty       = $(command -v stty >/dev/null 2>&1 && printf '有' || printf '没有')"

  # ---------------------------------------------------------- 1 权限 / dialout
  sec "1. 权限与 dialout（★ 第一步，先看这里）"
  local target_user=${SUDO_USER:-$(id -un)} cur_groups dialout_members
  cur_groups=$(id -nG "$target_user" 2>/dev/null || id -nG)
  info "目标用户   = $target_user（雷达驱动最终以这个用户跑）"
  info "当前组     = $cur_groups"
  dialout_members=$(getent group dialout 2>/dev/null | awk -F: '{print $4}' || true)
  info "dialout 组 = ${dialout_members:-(读不到 /etc/group)}"
  if printf '%s\n' "$cur_groups" | tr ' ' '\n' | grep -qx dialout; then
    ok "$target_user 在 dialout 组里" "读 /dev/ttyLIDAR 不需要 sudo。"
  else
    bad "$target_user 不在 dialout 组 ⇒ 读 /dev/ttyLIDAR、/dev/ttySTM32 会 Permission denied" \
      "sudo usermod -aG dialout $target_user ；然后**注销重新登录**（或 SSH 重连）。usermod 不会更新已登录会话的组；临时验证可用 newgrp dialout"
  fi
  # 当前进程实际生效的组跟 "登录时的组" 可能不同，两个都看
  info "当前进程生效组 = $(id -nG)（本行是脚本进程的真实权限；与目标用户组不同说明还没重登）"

  # ---------------------------------------------------------- 2 USB 枚举
  sec "2. USB 枚举（雷达到底插上了没有）"
  if command -v lsusb >/dev/null 2>&1; then
    local usb_dump
    usb_dump=$(lsusb 2>/dev/null || true)
    N7522=$(printf '%s\n' "$usb_dump" | grep -c '1a86:7522' || true)
    N7523=$(printf '%s\n' "$usb_dump" | grep -c '1a86:7523' || true)
    NCP210=$(printf '%s\n' "$usb_dump" | grep -ciE '10c4:ea60|10c4:ea70' || true)
    info "CH340 1a86:7522 = ${N7522} 个（雷达/视觉记录值）"
    info "CH340 1a86:7523 = ${N7523} 个（STM32 的 USB-TTL）"
    info "CP210x 10c4:ea6x = ${NCP210} 个（RPLIDAR 原厂转接板常见芯片）"
    info "所有串口类 USB 设备："
    printf '%s\n' "$usb_dump" | grep -iE '1a86|10c4|0403|serial|uart' | sed 's/^/       /' || info "       （没有匹配 1a86 / 10c4 / 0403 的设备）"
  else
    warn "lsusb 不可用，改用 /sys 判断（可能不准）" "sudo apt install usbutils（无外网则忽略，本脚本仍可继续）"
    N7522=$(ls -d /dev/ttyUSB* 2>/dev/null | wc -l)
    N7523=0
    NCP210=0
    info "兜底：/dev/ttyUSB* 数量 = ${N7522}（无法区分 7522/7523/CP210x）"
  fi

  local ttys=() t
  for t in /dev/ttyUSB* /dev/ttyACM*; do [[ -e $t ]] && ttys+=("$t"); done
  NTOTAL=${#ttys[@]}
  info "串口设备节点数 = ${NTOTAL}（含 ttyACM）"
  if [[ $NTOTAL -gt 0 ]]; then
    for t in "${ttys[@]}"; do
      info "$t  $(tty_usb_info "${t##*/}")"
    done
  fi
  info "by-path 链接："
  local bp
  for bp in /dev/serial/by-path/*; do
    [[ -e $bp ]] && info "  ${bp##*/} -> $(readlink -f "$bp")"
  done
  info "by-id 链接（★ 同型号 CH340 没唯一序列号，会撞名，禁止代码依赖）："
  for bp in /dev/serial/by-id/*; do
    [[ -e $bp ]] && info "  ${bp##*/} -> $(readlink -f "$bp")"
  done
  local klog
  klog=$( { dmesg 2>/dev/null || journalctl -k --no-pager 2>/dev/null; } | grep -iE 'ch34|cp210|ftdi|ttyUSB|usb-serial' | tail -n 12 || true)
  if [[ -n $klog ]]; then
    info "最近内核日志（ch34/cp210/ttyUSB）："
    printf '%s\n' "$klog" | sed 's/^/       /'
  else
    info "看不到内核日志（dmesg 需要 root 或被禁用）—— 不影响判定"
  fi
  if [[ ${N7522:-0} -eq 0 && ${NCP210:-0} -eq 0 && $NTOTAL -eq 0 ]]; then
    bad "USB 上没有任何串口设备（既没有 1a86:7522，也没有 CP210x，/dev/ttyUSB* 为空）" \
      "① 拔插雷达并换一根**能传数据**的 USB 线；② 换树莓派 USB 口；③ 检查雷达转接板供电（Pi 5 非 PD 供电时 USB 总限流约 600 mA）；④ 插到笔记本上用 check_lidar.md 的 Windows 段看是否枚举"
  fi

  # ---------------------------------------------------------- 3 设备节点 / udev
  sec "3. 设备节点与 udev 规则（/dev/ttyLIDAR 在不在）"
  local n
  for n in "$LIDAR_NODE" "$VISION_NODE" "$STM32_NODE"; do
    if [[ -e $n ]]; then
      info "$n 存在 → $(readlink -f "$n")，权限 $(stat -c '%A %U:%G' "$n" 2>/dev/null || printf '(stat 失败)')"
      if [[ -r $n ]]; then
        ok "$n 当前用户可读"
      else
        bad "$n 存在但当前用户不可读" "先加 dialout 组（见第 1 段）；临时排障才用 sudo，不要把 0666 写进规则"
      fi
      if [[ ! -w $n ]]; then
        warn "$n 当前用户不可写（驱动多以 O_RDWR 打开，可能失败）" "正常权限是 root:dialout 0660；先确认第 1 段 dialout 是否生效"
      fi
      if [[ $n == "$LIDAR_NODE" ]]; then LIDAR_NODE_OK=1; fi
    else
      bad "$n 不存在" "见本段下面的 udev 检查结论；设备名是契约冻结的，不要用 /dev/ttyUSB* 替代"
    fi
  done
  if [[ -f $RULES_FILE ]]; then
    ok "udev 规则文件已安装：$RULES_FILE" "$(wc -l <"$RULES_FILE" 2>/dev/null || true) 行"
    if grep -q 'ttyLIDAR' "$RULES_FILE"; then
      info "规则里有 ttyLIDAR 条目：$(grep -n 'ttyLIDAR' "$RULES_FILE" | head -n 3 | tr '\n' ' ')"
    else
      bad "规则文件里没有 ttyLIDAR 条目" "对比 pi_setup/99-omninav-serial.rules；不要手改 /etc 后再不同步仓库"
    fi
    if grep -q "$EXPECTED_LIDAR_ID_PATH" "$RULES_FILE"; then
      info "规则里的雷达 ID_PATH 仍是你上次实测的 $EXPECTED_LIDAR_ID_PATH"
    else
      warn "规则里的雷达 ID_PATH 与本脚本记录的历史值不一致（换过 USB 口？）" "以规则文件为准；让 pi_setup 按新 ID_PATH 重生成，不要在本脚本里改"
    fi
  else
    bad "udev 规则文件未安装：$RULES_FILE 不存在 ⇒ /dev/ttyLIDAR 不可能自动出现" \
      "sudo cp pi_setup/99-omninav-serial.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty ；装完重新拔插雷达。★ 这一步会改系统，属用户现场操作，本脚本不执行"
  fi
  # 已枚举到 7522 但节点不存在时，给出“规则没生效 / ID_PATH 变了”的区分
  if [[ $LIDAR_NODE_OK -eq 0 && ${N7522:-0} -gt 0 ]]; then
    local match=0 tj
    for t in "${ttys[@]}"; do
      [[ $t == *ttyUSB* ]] || continue
      tj=$(tty_usb_info "${t##*/}")
      if printf '%s' "$tj" | grep -q "$EXPECTED_LIDAR_ID_PATH"; then match=1; fi
    done
    if [[ $match -eq 1 ]]; then
      bad "有一个 7522 设备的 ID_PATH 恰好等于规则里的雷达路径，但 /dev/ttyLIDAR 仍不存在 ⇒ 规则没生效" \
        "重新装规则并 reload+trigger（见上），然后拔插雷达；再不行 `udevadm test --action=add /sys/class/tty/ttyUSB0`"
    else
      warn "枚举到 7522 设备，但它们都不在规则里记录的雷达 ID_PATH 上 ⇒ 插口变了，规则匹配不到" \
        "用 pi_setup/identify_ports.sh 重新取当前 ID_PATH，把结果报给 Lead 由 pi_setup 更新规则；不要在本轮临时改契约设备名"
    fi
  fi

  # ---------------------------------------------------------- 4 型号线索
  sec "4. CH340 与型号线索（能推断到哪一步）"
  if [[ $NTOTAL -gt 0 ]]; then
    for t in "${ttys[@]}"; do
      [[ $t == *ttyUSB* ]] || continue
      local info_line vid pid product
      info_line=$(tty_usb_info "${t##*/}")
      vid=$(printf '%s' "$info_line" | sed -n 's/.*vendor=\([^ ]*\).*/\1/p')
      pid=$(printf '%s' "$info_line" | sed -n 's/.*model_id=\([^ ]*\).*/\1/p')
      product=$(printf '%s' "$info_line" | sed -n 's/.*model=\(.*\) id_path=.*/\1/p')
      info "$t  vendor=$vid pid=$pid product=${product:-（空）}"
      if [[ $pid == 7522 ]]; then
        info "  → 7522 = CH340 系列（雷达与视觉同型，USB 描述符里没有可区分的型号）"
      elif [[ $pid == 7523 ]]; then
        info "  → 7523 = CH340 经典 USB-TTL（记录值是 STM32 链路，不是雷达）"
      elif [[ $pid == ea60 || $pid == ea70 ]]; then
        info "  → CP210x（RPLIDAR 原厂转接板常见芯片）⇒ 强线索偏 RPLIDAR A1，但以铭牌为准"
      fi
    done
  fi
  if [[ ${N7522:-0} -gt 1 ]]; then
    info "有 ${N7522} 个 7522 设备（雷达 + 视觉）：无法从 USB 描述符区分，只能靠物理插口 ID_PATH；**不要猜 ttyUSB0/1**"
  fi
  info "型号判定优先级：铭牌/丝印照片 > 转接板芯片（CP2102⇒RPLIDAR 线索）> 数据流特征（54 2c⇒LD19 线索）"
  info "★ 本轮不猜型号；拿不到铭牌就按 check_lidar.md 的拍照清单让用户拍回来"

  # ---------------------------------------------------------- 5 驱动包
  sec "5. 驱动包是否已装（Pi 无外网 ⇒ 只能靠已装包）"
  local pkg
  for pkg in $DRIVER_CANDIDATES; do
    if pkg_present "$pkg"; then
      DRIVER_FOUND+=" $pkg"
      DRIVER_LIST+="$pkg "
      ok "驱动包已装：$pkg"
    fi
  done
  if command -v ros2 >/dev/null 2>&1; then
    local ros_pkgs
    ros_pkgs=$(timeout 30 ros2 pkg list 2>/dev/null | grep -iE 'rplidar|ldlidar|ydlidar|urg_node|lidar_driver|lslidar|hokuyo' | head -n 20 || true)
    if [[ -n $ros_pkgs ]]; then
      info "ros2 pkg list 里与雷达相关的包："
      printf '%s\n' "$ros_pkgs" | sed 's/^/       /'
      while IFS= read -r pkg; do
        [[ -n $pkg ]] || continue
        case " $DRIVER_LIST " in *" $pkg "*) ;; *) DRIVER_LIST+="$pkg " ;; esac
      done <<<"$ros_pkgs"
      if [[ -z $DRIVER_FOUND ]]; then
        DRIVER_FOUND="(候选清单外) $DRIVER_LIST"
      fi
    else
      info "ros2 pkg list 里没有 rplidar/ldlidar/ydlidar/urg 相关包"
    fi
  else
    warn "ros2 CLI 不可用，驱动检查只看了 /opt/ros/*/share 目录" "source /opt/ros/jazzy/setup.bash 后重跑（不解决无外网问题）"
  fi
  if [[ -z $DRIVER_FOUND ]]; then
    bad "没有装任何雷达驱动包（候选：$DRIVER_CANDIDATES）" \
      "Pi 无外网 ⇒ 装不了。三条离线路线见 pi_setup/README.md 第 6 节：① 连手机热点借网；② 有网机器 git clone 源码 scp 后 colcon build --packages-select <包>；③ 另台 Ubuntu apt-get download 后 scp+dpkg -i。★ 先拍照确认型号再决定装哪个，不要两个都装"
  fi

  # ---------------------------------------------------------- 6 启动命令参考
  sec "6. 启动命令参考（只打印，不执行）"
  info "通用四查：① 串口必须改成 $LIDAR_NODE（驱动默认写 ttyUSB0，契约 §2 禁止）；② 波特率来自型号/说明书；③ 部分包要 product_name/model 参数；④ /scan 的 frame_id 必须是 laser"
  info "先查真实参数名（不要猜）：ros2 launch <包名> <launch文件> --show-args  或  ls \$(ros2 pkg prefix <包名>)/share/<包名>/"
  if printf '%s' "$DRIVER_LIST" | grep -q 'rplidar_ros'; then
    info "→ rplidar_ros 分支：ros2 launch rplidar_ros rplidar_a1_launch.py（实际 launch 文件名以 \$(ros2 pkg prefix rplidar_ros)/share/rplidar_ros/launch 为准）"
    info "  把串口参数改为 $LIDAR_NODE；波特率 RPLIDAR A1 常见 115200；启动后 ros2 topic hz /scan"
  fi
  if printf '%s' "$DRIVER_LIST" | grep -qE 'ldlidar'; then
    info "→ ldlidar 分支：ldlidar_stl_ros2 / ldlidar_ros2 的 launch 与 yaml 因版本而异，先 ls share/launch 与 share/config"
    info "  LD19 常见波特率 230400（不是 115200）；把端口改 $LIDAR_NODE；启动后 ros2 topic hz /scan（LD19 10 Hz）"
  fi
  if printf '%s' "$DRIVER_LIST" | grep -qE 'ydlidar'; then
    info "→ ydlidar 分支：先确认型号参数与波特率，再把端口改为 $LIDAR_NODE"
  fi
  if [[ -z $DRIVER_FOUND ]]; then
    info "（没有任何驱动包 ⇒ 第 6 段无启动命令可给，先解决第 5 段的阻塞）"
  elif [[ $DRIVER_FOUND == "(候选清单外)"* ]]; then
    info "候选清单外的雷达相关包：$DRIVER_LIST → 人工确认它是否就是本雷达的驱动，再决定是否用"
  fi
  info "★ 驱动 launch 起来后不要同时再跑 check_lidar.sh 的探测段（同一串口只能一个进程读）"

  # ---------------------------------------------------------- 7 数据流探测
  sec "7. 被动数据流探测（--probe 才执行；只读，仅设运行时波特率）"
  if [[ $DO_REQUEST -eq 1 && $DO_PROBE -eq 0 ]]; then
    info "--request-rplidar 只在 --probe 收到 0 字节后才写设备；本次没给 --probe，跳过请求段（避免无意义写设备）。"
  fi
  if [[ $DO_PROBE -eq 0 ]]; then
    info "未启用。要判断“有没有字节出来”，加 --probe 重跑（约 12 秒）。"
    info "若不想跑脚本：sudo stty -F $LIDAR_NODE 230400 raw -echo && sudo timeout 3 cat $LIDAR_NODE | od -An -tx1 | head"
  else
    if [[ -z $PROBE_DEV ]]; then
      if [[ $LIDAR_NODE_OK -eq 1 ]]; then
        PROBE_DEV=$LIDAR_NODE
      else
        warn "没有可探测的设备：$LIDAR_NODE 不存在且未给 --device" "先解决第 3 段；用户确认某个 /dev/ttyUSBn 是雷达时再跑 --probe --device /dev/ttyUSBn"
      fi
    fi
    if [[ -n $PROBE_DEV ]]; then
      if [[ ! -e $PROBE_DEV ]]; then
        bad "探测设备不存在：$PROBE_DEV"
      elif [[ ! -r $PROBE_DEV || ! -w $PROBE_DEV ]]; then
        bad "当前用户对 $PROBE_DEV 无读写权限（stty 需要）" \
          "手动执行：sudo bash $0 --probe  （临时）；长期修法是加 dialout 组，见第 1 段"
      elif ! command -v timeout >/dev/null 2>&1; then
        warn "系统没有 timeout 命令，跳过探测（不装包）" "sudo apt install coreutils 需外网；也可以手动用 stty+cat 探测"
      else
        PROBE_RUN=1
        info "探测设备 = $PROBE_DEV；候选波特率 = $PROBE_BAUDS；每档收 3 秒"
        local baud out bytes first hint stty_ok=1
        # stty 预检：设不了 termios ⇒ 设备打不开 / 不是 tty / 无权限，与"0 字节"是两回事，分开报
        if ! stty -F "$PROBE_DEV" 115200 cs8 -parenb -cstopb raw -echo -iexten -crtscts 2>/dev/null; then
          PROBE_STTY_FAIL=1
          stty_ok=0
          warn "stty 设 $PROBE_DEV 失败（设备打不开 / 不是 tty / 无权限）——这不是「0 字节」问题，本轮探测未执行" \
            "手动确认：ls -l $PROBE_DEV；stty -F $PROBE_DEV 115200 raw -echo；无权限见第 1 段"
        fi
        if [[ $stty_ok -eq 1 ]]; then
          for baud in $PROBE_BAUDS; do
            if ! stty -F "$PROBE_DEV" "$baud" cs8 -parenb -cstopb raw -echo -iexten -crtscts 2>/dev/null; then
              PROBE_ENTRIES+=("$baud  stty 设置失败（该档未探测）")
              info "$baud  → stty 设置失败（该波特率不被设备/驱动接受；≠ 0 字节）"
              continue
            fi
            out=$(probe_read "$PROBE_DEV" 3) || true
            if [[ -z $out ]]; then
              PROBE_ENTRIES+=("$baud  0 B")
              info "$baud  → 0 字节"
              continue
            fi
            bytes=$(( ${#out} / 2 ))
            PROBE_BYTES=$((PROBE_BYTES + bytes))
            first=${out:0:64}
            hint=""
            if [[ $out == *542c* ]]; then
              hint="含 54 2c（LD19 包头特征）"
            elif [[ $out == *a55a* ]]; then
              hint="含 A5 5A（RPLIDAR 特征，偏 A1 方向）"
            fi
            PROBE_ENTRIES+=("$baud  ${bytes} B  ${hint:-无已知包头特征}")
            info "$baud  → ${bytes} 字节${hint:+，}$hint；前 32 字节：$first"
          done
        fi
        if [[ ${#PROBE_ENTRIES[@]} -gt 0 ]]; then
          info "逐档结果："
          for out in "${PROBE_ENTRIES[@]}"; do info "  $out"; done
        fi
        if [[ $PROBE_BYTES -gt 0 ]]; then
          ok "被动收到 $PROBE_BYTES 字节 ⇒ 设备在主动吐数据（不是哑设备）"
          info "★ 有字节 ≠ 能出 /scan：还要驱动包 + 正确参数（第 5、6 段）。数据流特征只做方向性线索，型号以铭牌为准"
        elif [[ $stty_ok -eq 0 ]]; then
          warn "探测未执行（stty 设不了设备，≠ 0 字节）" \
            "先解决设备打不开 / 不是 tty / 权限问题，再重跑 --probe；不要把这条当成「雷达没数据」"
        else
          warn "所有候选波特率下 3 秒内都是 0 字节" \
            "可能：① 波特率不对（试更多档位）；② 雷达没供电/线序错；③ 该型号要先收请求（RPLIDAR 类）。先拔插、看转接板指示灯，再用 --request-rplidar 重试（会向设备写 A5 20，默认关闭）"
        fi
        if [[ $DO_REQUEST -eq 1 && $PROBE_BYTES -eq 0 && $stty_ok -eq 1 ]]; then
          info "尝试 RPLIDAR 请求：向 $PROBE_DEV 写 A5 20（写设备；仍不落盘）"
          if printf '\245\040' >"$PROBE_DEV" 2>/dev/null; then
            local req_stty=1
            if stty -F "$PROBE_DEV" 115200 cs8 -parenb -cstopb raw -echo -iexten -crtscts 2>/dev/null; then
              out=$(probe_read "$PROBE_DEV" 3) || true
            else
              req_stty=0
              out=""
              info "请求后 stty 115200 设置失败：无法读取（与「0 字节」区分，这是设备设置失败）"
            fi
            if [[ -n $out ]]; then
              bytes=$(( ${#out} / 2 ))
              PROBE_BYTES=$((PROBE_BYTES + bytes))
              PROBE_ENTRIES+=("115200(请求后)  ${bytes} B")
              info "请求后收到 ${bytes} 字节；前 32 字节：${out:0:64}"
              ok "请求后有响应 ⇒ 更像 RPLIDAR 请求-应答协议（仍以铭牌为准）"
            elif [[ $req_stty -eq 0 ]]; then
              warn "请求后无法读取（stty 设置失败）——不是「0 字节」，先查设备/权限" \
                "手动确认：ls -l $PROBE_DEV；stty -F $PROBE_DEV 115200 raw -echo"
            else
              warn "请求后仍无字节" "查 TX/RX 交叉、供电、波特率；也拍转接板芯片照片回来"
            fi
          else
            warn "写设备失败（权限？）" "用 sudo bash $0 --probe --request-rplidar --device $PROBE_DEV"
          fi
        elif [[ $DO_REQUEST -eq 1 && $stty_ok -eq 0 ]]; then
          info "stty 设不了 $PROBE_DEV，跳过 --request-rplidar 请求段（写设备无意义）"
        fi
      fi
    fi
  fi

  # ---------------------------------------------------------- 8 ROS2 现状
  sec "8. ROS2 图与 /scan、TF 现状（读当前系统状态，不启动任何节点）"
  if ! command -v ros2 >/dev/null 2>&1; then
    warn "ros2 CLI 不可用，跳过" "source /opt/ros/jazzy/setup.bash 后重跑"
  else
    local topics
    topics=$(timeout 20 ros2 topic list 2>/dev/null || true)
    if [[ -z $topics ]]; then
      warn "ros2 topic list 无输出（ROS2 守护进程起不来或没 source 环境）" "先 source /opt/ros/jazzy/setup.bash；2GB 内存下避免同时开很多节点"
    else
      for n in /scan /tf /odom /cmd_vel /tf_static; do
        if printf '%s\n' "$topics" | grep -qx "$n"; then info "话题存在：$n"; else info "话题不存在（可能没节点在跑，属正常）：$n"; fi
      done
      if printf '%s\n' "$topics" | grep -qx /scan; then
        ok "/scan 话题已存在（有节点声明了它）" "手动确认数据流：timeout 20 ros2 topic hz /scan；确认 frame_id：ros2 topic echo /scan --field header.frame_id --once"
      else
        info "当前没有 /scan ⇒ 雷达驱动还没起来（或起来了但没声明该话题）"
      fi
    fi
    info "手动核对 /scan 与 TF（需要节点都在跑时执行）："
    info "  timeout 20 ros2 topic hz /scan"
    info "  ros2 topic echo /scan --field header.frame_id --once    # 必须是 laser"
    info "  ros2 run tf2_ros tf2_echo odom base_link                # 桥应发这个"
    info "  ros2 run tf2_ros tf2_echo base_link laser              # 需要 URDF/static transform（本工程还没补，见报告）"
    info "  ros2 run tf2_ros tf2_echo map odom                     # 建图时 slam_toolbox 发；定位时 AMCL 发，绝不同时"
  fi

  # ---------------------------------------------------------- 9 判定
  sec "9. 判定与下一步（把这一整段抄回给 Lead）"
  local which
  which=""
  if [[ ${N7522:-0} -eq 0 && ${NCP210:-0} -eq 0 ]]; then
    which="A 枚举不到"
  elif [[ $LIDAR_NODE_OK -eq 0 ]]; then
    which="B 枚举到但没有 $LIDAR_NODE"
  elif [[ -z $DRIVER_FOUND ]]; then
    which="C 有设备节点但驱动包没装"
  elif [[ $PROBE_RUN -eq 1 && $PROBE_STTY_FAIL -eq 1 ]]; then
    which="D2 探测条件不满足（stty 设不了设备，≠ 0 字节）"
  elif [[ $PROBE_RUN -eq 1 && $PROBE_BYTES -eq 0 ]]; then
    which="D 驱动可能已装但串口没数据（探测 0 字节）"
  else
    which="E 设备/驱动/数据似乎齐全 → 下一步验 /scan 与 TF"
  fi
  printf '\n>>> 本次判定：%s\n' "$which"
  printf '    （依据：7522=%d 7523=%d CP210x=%d，ttyLIDAR=%s，驱动=[%s]，探测=%s）\n' \
    "${N7522:-0}" "${N7523:-0}" "${NCP210:-0}" \
    "$([[ $LIDAR_NODE_OK -eq 1 ]] && printf '存在' || printf '不存在')" \
    "${DRIVER_FOUND:-无}" \
    "$([[ $PROBE_RUN -eq 1 ]] && printf '%d 字节' "$PROBE_BYTES" || printf '未探测')"
  if [[ $PROBE_STTY_FAIL -eq 1 ]]; then
    printf '    （探测未执行：stty 设不了设备 —— 与「0 字节」分开报告，见失败树 D2）\n'
  fi

  printf '\n失败树速查（现象 → 查什么 → 下一步最小实验）：\n'
  printf '  A 枚举不到      → lsusb/dmesg 无 1a86:7522 → 换线换口、查供电；笔记本 Get-PnpDevice 只证明枚举\n'
  printf '  B 有枚举无节点  → udev 规则没装 / ID_PATH 变了 → 装规则 reload+trigger；或 identify_ports.sh 重取 ID_PATH 报 Lead\n'
  printf '  C 节点在无驱动  → ros2 pkg list 无雷达包 → 拍照定型号 → 离线装包（Pi 无外网，属阻塞）\n'
  printf '  D 有驱动无数据  → --probe 全 0 字节 → 换波特率/查线序/--request-rplidar；查转接板芯片\n'
  printf '  D2 探测条件不满足 → stty 设不了设备（打不开/不是 tty/无权限）→ 先 ls -l 与权限，不是 0 字节\n'
  printf '  E 有数据无 /scan → launch 报错 → 串口名是否 %s、波特率、product_name；ros2 node list / topic list\n' "$LIDAR_NODE"
  printf '  F 有 /scan 无 TF → frame_id 是否 laser → 本工程还没有 base_link→laser 的 URDF/static transform（已知缺口，见 reports/B_ros2.md §5.2 C1 / §6.3，M4 前补）\n'

  printf '\n证据保存（把日志文件 + 照片一起回传）：\n'
  printf '  bash %s %s 2>&1 | tee ~/check_lidar_$(date +%%F_%%H%%M).log\n' \
    "${0##*/}" "$([[ $DO_PROBE -eq 1 ]] && printf -- '--probe' || printf '')"
  printf '  然后：cat ~/check_lidar_*.log  （或直接把文件拷走）\n'

  printf '\n================================================================\n'
  printf ' 汇总：PASS=%d  FAIL=%d  WARN=%d\n' "$PASS" "$FAIL" "$WARN"
  printf ' 本脚本没有安装任何包、没有改 /etc、没有写文件；--probe 只改设备运行时波特率。\n'
  printf '================================================================\n'
  if [[ $FAIL -gt 0 ]]; then return 1; fi
  return 0
}

if ! need_pi >/dev/null 2>&1; then
  need_pi
  exit 2
fi

main
rc=$?
exit $rc
