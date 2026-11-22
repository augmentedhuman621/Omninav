#!/usr/bin/env bash
# =============================================================================
# OmniNav · preflight.sh —— 树莓派上机自检
# -----------------------------------------------------------------------------
# 做什么 ：逐项打印 PASS / FAIL / WARN，把"树莓派现在到底缺什么"一次说清。
#          **每一项 FAIL 都会打印它对应的那一行修法命令。**
# 在哪跑 ：树莓派 5（aarch64）。**必须在树莓派上跑。**
# 用法   ：
#            ./preflight.sh              # 常规自检
#            ./preflight.sh --scan       # 额外等 12 秒看 /scan 有没有数据（需雷达驱动已在跑）
# 退出码 ：0 = 没有 FAIL；1 = 有 FAIL；2 = 不在树莓派上（或用了 --help）
# 依据   ：11_接口契约.md §2/§5.3、AGENTS.md §7（版本对齐三件套）、
#          09_新对话交接.md §2.3、树莓派5_上位机烧录与配置清单.md
# =============================================================================
set -euo pipefail

DO_SCAN=0
for arg in "$@"; do
  case $arg in
    --scan) DO_SCAN=1 ;;
    -h | --help)
      printf '用法：%s [--scan]\n' "${0##*/}"
      printf '  --scan   额外检查 /scan 是否有数据流（默认跳过，因为要先起雷达驱动）\n'
      exit 2
      ;;
    *)
      printf '✖ 不认识的选项：%s（试 --help）\n' "$arg" >&2
      exit 2
      ;;
  esac
done

PASS=0
FAIL=0
WARN=0

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
  local m
  m=$(uname -m)
  if [[ $m != aarch64 ]]; then
    printf '✖ preflight.sh 要在树莓派 5 上跑（uname -m 应为 aarch64），当前是 %s。\n' "$m" >&2
    printf '  它检查的是树莓派的网络、设备节点与 ROS2 包清单。\n' >&2
    printf '  若只想在笔记本上验证脚本逻辑：OMNINAV_SKIP_ARCH_CHECK=1 %s\n' "$0" >&2
    return 2
  fi
  return 0
}

# ROS2 包是否存在：先看 /opt/ros/jazzy，再看 $HOME/ros2_ws 的 overlay
pkg_present() {
  local p=$1 d
  if [[ -d /opt/ros/jazzy/share/$p ]]; then return 0; fi
  if [[ -d $HOME/ros2_ws/install/$p/share/$p ]]; then return 0; fi
  for d in "$HOME"/ros2_ws/install/*/share/"$p"; do
    if [[ -d $d ]]; then return 0; fi
  done
  return 1
}

bashrc_exports() {
  local var=$1 val=$2
  grep -qE "^[[:space:]]*export[[:space:]]+${var}=${val}([[:space:]]|\$)" "$HOME/.bashrc" 2>/dev/null
}

main() {
  printf '================================================================\n'
  printf ' OmniNav · 树莓派上机自检 preflight.sh\n'
  printf '================================================================\n'

  # ---------------------------------------------------------------- 0 基础信息
  sec "0. 基础信息"
  info "uname -m   = $(uname -m)"
  info "主机名     = $(uname -n)"
  info "用户       = $(id -un) (uid=$(id -u))"
  info "内核       = $(uname -r)"
  info "系统       = $(. /etc/os-release 2>/dev/null && printf '%s %s' "$NAME" "$VERSION_ID" || printf '(读不到 /etc/os-release)')"
  info "内存       = $(free -h 2>/dev/null | awk 'NR==2{print $2" 总 / "$7" 可用"}' || printf '(无 free)')"
  info "根分区     = $(df -h / 2>/dev/null | awk 'NR==2{print $2" 总 / "$4" 可用 ("$5" 已用)"}' || printf '(无 df)')"

  sec "1. 网络（★ 当前第一阻塞）"
  local defroute v4 gw
  defroute=$(ip route show default 2>/dev/null || true)
  if [[ -n $defroute ]]; then
    ok "有默认路由" "$defroute"
  else
    bad "没有默认路由（ip route show default 是空的）⇒ 上不了外网 ⇒ apt 装不了任何东西" \
      '见 pi_setup/README.md 第 1 节。三条路任选：① Windows 把 ICS 取消勾选再勾回来；② 给树莓派配静态 192.168.137.50/24 + 网关 192.168.137.1；③ 让树莓派连手机热点。修完必须看到 `ip route` 里有 `default via ...`'
  fi

  v4=$(ip -4 -o addr show scope global 2>/dev/null || true)
  if [[ -n $v4 ]]; then
    ok "有全局 IPv4 地址" "$(printf '%s' "$v4" | awk '{print $2" = "$4}')"
  else
    bad "没有任何全局 IPv4 地址（只有 IPv6 链路本地）⇒ 跨机 DDS / foxglove 都连不上" \
      '同上，见 README 第 1 节'
  fi

  gw=$(ip route show default 2>/dev/null | awk '/default/{print $3; exit}')
  if [[ -n $gw ]]; then
    if ping -c 1 -W 2 -- "$gw" >/dev/null 2>&1; then
      ok "默认网关 $gw 能 ping 通"
    else
      warn "默认网关 $gw ping 不通" "可能是防火墙挡了 ICMP；只要 `ip route` 和 DNS 正常就不用管"
    fi
  fi

  if command -v getent >/dev/null 2>&1; then
    if getent hosts archive.ubuntu.com >/dev/null 2>&1; then
      ok "DNS 能解析 archive.ubuntu.com"
    else
      bad "DNS 解析失败（getent hosts archive.ubuntu.com 无结果）" \
        'DNS 一般跟默认路由一起来。若默认路由已修好但 DNS 仍失败，在 netplan 里加 nameservers: addresses: [8.8.8.8, 1.1.1.1]，再 sudo netplan apply'
    fi
  else
    warn "没有 getent 命令，跳过 DNS 检查"
  fi

  if timeout 5 bash -c 'exec 3<>/dev/tcp/1.1.1.1/443' >/dev/null 2>&1; then
    ok "能建外网 TCP 连接（1.1.1.1:443）"
  else
    bad "连不上外网 TCP（1.1.1.1:443，5 秒超时）" '默认路由 + NAT 没通，见 README 第 1 节'
  fi

  # ---------------------------------------------------------------- 2 串口权限
  sec "2. 串口权限（dialout 组）"
  local me dialout_line in_session=0 in_file=0
  me=$(id -un)
  if id -nG 2>/dev/null | tr ' ' '\n' | grep -qx dialout; then in_session=1; fi
  dialout_line=$(getent group dialout 2>/dev/null || true)
  if printf '%s' "$dialout_line" | grep -qE "(^|[ ,:])${me}(,|\$)"; then in_file=1; fi

  if [[ $in_session -eq 1 ]]; then
    ok "当前用户 $me 已在 dialout 组" "$dialout_line"
  elif [[ $in_file -eq 1 ]]; then
    bad "已在 /etc/group 的 dialout 里，但**当前这个会话还没生效**" \
      "★ 这正是最常见的误判。必须**重新登录**：退出 SSH 再进（不是重开一个终端窗口），或临时用 `newgrp dialout`。然后用 `id -nG | tr ' ' '\\n' | grep dialout` 确认能看到 dialout"
  else
    bad "当前用户 $me 不在 dialout 组 ⇒ 读 /dev/ttyUSB* 会 Permission denied" \
      "sudo usermod -aG dialout $me   然后 ★**必须重新登录**（退出 SSH 再进）才生效"
  fi

  # ---------------------------------------------------------------- 3 设备节点
  sec "3. 冻结的设备节点（契约 §2）"
  local d pair
  for pair in "ttyLIDAR:激光雷达（已插）" "ttyVISION:视觉模块通信板（已插）"; do
    d=/dev/${pair%%:*}
    if [[ -e $d ]]; then
      if [[ -r $d && -w $d ]]; then
        ok "$d 存在且可读写" "$(ls -l -- "$d" 2>/dev/null || true)"
      else
        bad "$d 存在但当前用户读不了/写不了" "①先修 dialout 组（上一节）；②确认 udev 规则里写了 GROUP=\"dialout\" MODE=\"0660\""
      fi
    else
      bad "$d 不存在（${pair##*:}）" \
        '① 先跑 pi_setup/identify_ports.sh 定口；② 把生成的规则 cp 到 /etc/udev/rules.d/；③ sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty'
    fi
  done

  d=/dev/ttySTM32
  if [[ -e $d ]]; then
    ok "$d 存在" "$(ls -l -- "$d" 2>/dev/null || true)"
  else
    warn "$d 不存在" '★ **现在可以不管**：STM32 那根独立 USB-TTL 还没接上（接了之后再补 udev 规则里注释掉的那一行）'
  fi

  local raw
  raw=$(ls -1 /dev/ttyUSB* 2>/dev/null || true)
  if [[ -n $raw ]]; then
    info "裸设备节点：$(printf '%s' "$raw" | tr '\n' ' ')"
    if [[ ! -e /dev/ttyLIDAR || ! -e /dev/ttyVISION ]]; then
      info "  ↑ 设备是**插着的**，只是还没有被命名成 ttyLIDAR / ttyVISION ⇒ 问题在 udev 规则，不在硬件。"
    fi
  else
    info "裸设备节点：/dev/ttyUSB* 一个都没有"
  fi

  if [[ -f /etc/udev/rules.d/99-omninav-serial.rules ]]; then
    ok "udev 规则已安装：/etc/udev/rules.d/99-omninav-serial.rules"
  else
    bad "udev 规则没装（/etc/udev/rules.d/99-omninav-serial.rules 不存在）" \
      '见 README 第 3 节：identify_ports.sh 生成 → sudo cp → udevadm reload + trigger'
  fi

  # ---------------------------------------------------------------- 4 共享库
  sec "4. firmware_core 共享库（契约 §5.3）"
  local so=/opt/omninav/lib/libomninav_core.so
  if [[ -n ${OMNINAV_CORE_SO:-} ]]; then
    info "检测到环境变量 OMNINAV_CORE_SO=$OMNINAV_CORE_SO（契约 §5.3 说它优先于本路径）"
  fi
  if [[ -f $so ]]; then
    ok "$so 存在" "$(ls -l -- "$so" 2>/dev/null || true)"
    if command -v nm >/dev/null 2>&1; then
      local defined missing='' sym
      defined=$(nm -D --defined-only -- "$so" 2>/dev/null || true)
      for sym in mecanum_forward mecanum_inverse mecanum_l_mm mecanum_default_geom mecanum_geom_is_valid odom_update odom_reset; do
        if ! printf '%s\n' "$defined" | grep -qw -- "$sym"; then missing="$missing $sym"; fi
      done
      if [[ -z $missing ]]; then
        ok "七个必需符号全在（契约 §5.2）"
      else
        bad "缺导出符号：$missing" '这版 .so 不完整，报给 core-algo；桥会明确报错退出，不会自己拿 Python 兜底'
      fi
    else
      warn "没有 nm，跳过符号校验" 'sudo apt install -y binutils'
    fi
  else
    bad "$so 不存在 ⇒ 桥（stm32_bridge）起不来" \
      '见 README 第 4 节：把 firmware_core 用 scp 送过去，然后跑 sudo ./install_core.sh'
  fi

  # ---------------------------------------------------------------- 5 ROS2
  sec "5. ROS2 Jazzy 环境"
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    ok "/opt/ros/jazzy/setup.bash 存在"
  else
    bad "/opt/ros/jazzy/setup.bash 不存在 ⇒ ROS2 Jazzy 没装或装到别处了" '按 树莓派5_上位机烧录与配置清单.md 重装 ROS2 Jazzy'
  fi

  local p
  for p in slam_toolbox nav2_bringup nav2_amcl nav2_controller nav2_planner nav2_bt_navigator teleop_twist_keyboard; do
    if pkg_present "$p"; then
      ok "必需包已装：$p"
    else
      bad "必需包缺失：$p" "有外网时：sudo apt install -y ros-jazzy-${p//_/-}   没外网时见 README 第 6 节的离线路线"
    fi
  done

  for p in foxglove_bridge robot_localization micro_ros_agent twist_mux; do
    if pkg_present "$p"; then
      ok "可选包已装：$p"
    else
      warn "可选包未装：$p" "$(case $p in
        foxglove_bridge) printf '建图/导航必须看得见 → README 第 7 节（apt install ros-jazzy-foxglove-bridge，清华 tuna 源里有 3.5.0）' ;;
        robot_localization) printf '★ 本项目**暂时用不着**：IMU 已砍（03_底盘物料清单.md:162），不要为它写 EKF' ;;
        micro_ros_agent) printf '只有"视觉模块刷成 ROS 版固件"那条路才需要（契约外的架构分叉）' ;;
        twist_mux) printf '可选：给遥控/导航做速度仲裁' ;;
      esac)"
    fi
  done

  if [[ -f /opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so ]]; then
    ok "RMW 实现库在：librmw_cyclonedds_cpp.so"
  else
    bad "缺 librmw_cyclonedds_cpp.so，但环境要求 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
      'sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp（需要外网）。★ 缺了它 ROS2 会起不来或静默换实现，跨机通信就废了'
  fi

  info "当前环境变量：ROS_DOMAIN_ID='${ROS_DOMAIN_ID:-（未设）}'  RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION:-（未设）}'"
  local dom_ok=1 rmw_ok=1
  if bashrc_exports ROS_DOMAIN_ID 42; then
    ok "~/.bashrc 里有 export ROS_DOMAIN_ID=42"
  else
    dom_ok=0
    bad "~/.bashrc 里没有 export ROS_DOMAIN_ID=42" \
      'printf "\nexport ROS_DOMAIN_ID=42\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n" >> ~/.bashrc && source ~/.bashrc'
  fi
  if bashrc_exports RMW_IMPLEMENTATION rmw_cyclonedds_cpp; then
    ok "~/.bashrc 里有 export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
  else
    rmw_ok=0
    bad "~/.bashrc 里没有 export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
      'printf "\nexport ROS_DOMAIN_ID=42\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n" >> ~/.bashrc && source ~/.bashrc'
  fi
  if [[ $dom_ok -eq 1 && $rmw_ok -eq 1 ]]; then
    info "  ★ 这一对必须与笔记本/虚拟机**完全一致**，否则跨机 DDS 静默失败、不报任何错（AGENTS.md §7）。"
  fi

  if grep -q '/opt/ros/jazzy/setup.bash' "$HOME/.bashrc" 2>/dev/null; then
    ok "~/.bashrc 里 source 了 /opt/ros/jazzy/setup.bash"
  else
    warn "~/.bashrc 里没有 source /opt/ros/jazzy/setup.bash" 'echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc'
  fi

  local sw
  sw=$(swapon --show 2>/dev/null || true)
  if [[ -n $sw ]]; then
    info "交换空间：$(printf '%s' "$sw" | tr '\n' ' ')"
  else
    warn "没有交换空间（2GB 内存的机器上编 C++ 很容易 OOM）" '编大包时用 colcon build --parallel-workers 1，并考虑开 zram'
  fi

  # ---------------------------------------------------------------- 6 可选 /scan
  if [[ $DO_SCAN -eq 1 ]]; then
    sec "6. 可选：/scan 数据流（--scan）"
    info "等最多 12 秒……（前提：雷达驱动已经在另一个终端里跑着）"
    local out
    out=$(timeout 12 bash -lc 'source /opt/ros/jazzy/setup.bash 2>/dev/null; ros2 topic hz /scan' 2>&1 || true)
    if printf '%s' "$out" | grep -q 'average rate'; then
      ok "/scan 有数据流" "$(printf '%s' "$out" | grep 'average rate' | head -1)"
    else
      bad "/scan 没有数据流（12 秒内没看到 average rate）" \
        '① 雷达驱动起了吗（ros2 topic list 里有没有 /scan）；② 驱动用的串口是不是 /dev/ttyLIDAR；③ 雷达型号与驱动是否配套（rplidar_ros ↔ A1M8，ldlidar_stl_ros2 ↔ LD19）'
    fi
  fi

  # ---------------------------------------------------------------- 汇总
  sec "汇总"
  printf 'PASS = %d    FAIL = %d    WARN = %d\n' "$PASS" "$FAIL" "$WARN"
  if [[ $FAIL -gt 0 ]]; then
    printf '\n有 %d 项必须修。按 pi_setup/README.md 里的顺序修（第 1 节网络优先，它会解锁后面所有安装），修完重跑本脚本。\n' "$FAIL"
    printf '把上面每一行 [FAIL] 连同它的「修法」一起贴出来，就能直接对上 README 的哪一节。\n'
    exit 1
  fi
  printf '\n没有 FAIL。WARN 项按需处理（标了「现在可以不管」的可以先放着）。\n'
  return 0
}

if ! need_pi; then
  exit 2
fi

main
