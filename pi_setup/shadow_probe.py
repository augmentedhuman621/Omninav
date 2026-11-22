#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""shadow_probe.py —— 让 STM32 的影子模式真的算几条 V 帧（B1 实机验收 · 喂帧侧）

背景
----
`stm32_fw/Src/usart2_bench.c` 里新加了**影子模式**（本地键 `.s` 切换，默认关）：

    收到 `V,<vx>,<vy>,<w>` -> mecanum_solve()（逆解/限幅/死区/装配映射）
                          -> fixp_mm_s_to_counts()（mm/s -> 编码器计数/控制窗口）
                          -> 只打印，**不驱动任何电机**

★ 结果的去向：影子模式的输出走 `logf()`，而 `logf()` 是**两个口都发**
  （USART1/COM9 + USART2/树莓派），所以本脚本的 `drain()` 里也能看到它，
  COM9 那侧同样能看到 —— 两边对照着看最省事。

所以本脚本只干"喂帧"这一件事，判据在 COM9 那侧看：
    每条 V 帧应该打印 1 行 `[SHADOW] V,... -> L=... scale=... sat=... `
    加 4 行 `[SHADOW]   FL/FR/RL/RR wheel=... motor=... target=... cnt/20ms`

用法
----
    python3 pi_setup/shadow_probe.py [设备名]      # 默认 /dev/ttySTM32

只用标准库 os + termios，不依赖 pyserial（树莓派上不一定装了）。
输出全 ASCII。退出码 0 = 四条帧都收到了 #ACK。
"""

import os
import sys
import termios
import time
import re

DEV = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttySTM32"
BAUD = 115200                    # 给人看的
# ★ 给 termios 的必须是 B115200 这个常量，不能填 115200 这个整数：
#   a[4]/a[5] 期待的是 c_cflag 里的 CBAUD 编码，填裸数字会让 tcsetattr 报
#   errno 22 Invalid argument（第一版就是这么挂的）。test_stm32_link.py 用的是同一个常量。
BAUD_CONST = termios.B115200

# 影子模式的 4 条测试帧。★ 都是契约 §3 的合法帧（mm/s、mrad/s、全整数）。
FRAMES = [
    ("直行 forward",      "V,300,0,0"),
    ("左平移 strafe",     "V,0,300,0"),
    ("原地左转 rotate",   "V,0,0,1000"),
    ("斜行+旋转 combo",   "V,300,300,500"),
]

COUNTS_PER_M = 10505   # firmware_core fixedpoint.h 的名义值
PERIOD_MS = 20         # FIXP_CONTROL_PERIOD_MS_NOMINAL


def counts_of(mm_s):
    """mm/s -> 计数/控制窗口，与 fixp_mm_s_to_counts() 同一个式子（对称四舍五入）。"""
    num = mm_s * COUNTS_PER_M * PERIOD_MS
    den = 1000 * 1000
    return (num + den // 2) // den if num >= 0 else -((-num + den // 2) // den)


def open_port(path):
    """与 test_stm32_link.py 完全相同的串口配置（那里已经实机验证过）。"""
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = 0                                              # iflag : raw
    a[1] = 0                                              # oflag : raw
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL   # 8N1, ignore modem
    a[3] = 0                                              # lflag : raw, no echo
    a[4] = BAUD_CONST
    a[5] = BAUD_CONST
    a[6][termios.VMIN] = 0
    a[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def drain(fd, seconds):
    """读 seconds 秒内到达的全部字节（端口是 O_NONBLOCK，空读要自己让一下）。"""
    out = b""
    # ★ 用 monotonic 而不是 time.time()：后者会被 NTP 拉前/拉后，drain 就会
    #   提前返回（读不完整）或者死等。test_stm32_link.py 里用的也是 monotonic。
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            r = os.read(fd, 512)
        except (BlockingIOError, OSError):
            time.sleep(0.02)
            continue
        if r:
            out += r
        else:
            time.sleep(0.005)
    return out.decode("ascii", "replace")


SHADOW_STATE_RE = re.compile(r"shadow.*=\s*(ON|OFF)")


def set_shadow(fd, want_on):
    """把影子模式设成 want_on 并**校验**是否真的到位。

    ★ 固件的本地键 ``.s`` 是 **toggle**，不是幂等的：如果上次异常退出把影子留在
    "开"，再发一次 ``.s`` 反而会把它**关掉** —— 然后四条 V 帧全部不打印
    ``[SHADOW]`` 行，排查方向会被彻底带偏（会被误判成"影子模式坏了"）。
    固件会把新状态打成 ``[U2] shadow (...) = ON/OFF``，所以这里读回来确认，
    状态不对就再 toggle 一次，最多三轮。

    :returns: (是否达成, 最后一次 drain 到的文本)
    """
    back = ""
    for _ in range(3):
        os.write(fd, b".s")
        time.sleep(0.4)
        back = drain(fd, 0.6)
        states = SHADOW_STATE_RE.findall(back)
        if states and (states[-1] == "ON") == bool(want_on):
            return True, back
    return False, back


def main():
    print("=" * 62)
    print(" OmniNav shadow probe (B1)  --  feed V frames, watch COM9")
    print("=" * 62)
    print("device : %s" % DEV)
    print("baud   : %d 8N1" % BAUD)
    print("result : [SHADOW] lines appear BOTH here (USART2) and on COM9")
    print("")
    print("expected counts/20ms if a wheel is asked for a clean speed:")
    for mm in (150, 300, 450, 750):
        print("    %4d mm/s -> %d cnt/%dms" % (mm, counts_of(mm), PERIOD_MS))
    print("")

    fd = open_port(DEV)
    time.sleep(0.3)

    # ① 打开影子模式（本地键必须带 '.' 前缀，否则 's' 会被当成协议字节）
    shadow_on, back = set_shadow(fd, True)
    print("--- shadow mode -> ON (verified=%s) ---" % ("YES" if shadow_on else "NO"))
    print(back if back.strip() else "(nothing)")
    print("")
    if not shadow_on:
        print("!! 三次都没能确认影子模式已打开 —— 下面四条 V 帧大概率不会打印 [SHADOW]。")
        print("!! 先在 COM9 上手动发 '.s'，确认回显是 '= ON'；再看固件是不是这一版。")
        print("")

    ok = 0
    for name, frame in FRAMES:
        os.write(fd, (frame + "\n").encode("ascii"))
        time.sleep(0.6)
        back = drain(fd, 0.6)
        got_ack = re.search(r"#ACK\s+V\s+3", back) is not None
        if got_ack:
            ok += 1
        print("--- %-18s send %-16s ack=%s ---" % (name, frame, "YES" if got_ack else "no"))
        for line in back.splitlines():
            if line.strip():
                print("      " + line)
        print("")

    # ② 关掉影子模式，把台架恢复成基线状态
    shadow_off, back = set_shadow(fd, False)
    print("--- shadow mode -> OFF (verified=%s) ---" % ("YES" if shadow_off else "NO"))
    print(back if back.strip() else "(nothing)")
    print("")

    os.close(fd)

    print("=" * 62)
    print("RESULT: %d/4 frames acknowledged" % ok)
    print("Now read COM9 on the PC: each frame should have produced")
    print("  1x [SHADOW] summary line + 4x [SHADOW] wheel lines.")
    print("=" * 62)
    return 0 if ok == len(FRAMES) else 1


if __name__ == "__main__":
    sys.exit(main())
