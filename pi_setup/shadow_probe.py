#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""shadow_probe.py -- make the STM32 shadow mode really compute a few V frames
(B1 on-target acceptance, frame-feeding side)

Background
----------
`stm32_fw/Src/usart2_bench.c` gained a **shadow mode** (toggled locally by key
`.s`, off by default):

    receives `V,<vx>,<vy>,<w>` -> mecanum_solve() (inverse kinematics / clamp / deadband / assembly mapping)
                              -> fixp_mm_s_to_counts() (mm/s -> encoder counts per control window)
                              -> prints only, **drives no motor**

★ Where the result goes: shadow-mode output goes through `logf()`, which **transmits
  on both ports** (USART1/COM9 + USART2/Raspberry Pi), so this script's `drain()`
  sees it too, and the COM9 side sees it as well -- comparing both is the easiest.

So this script does one thing -- feed frames; the judgement is on the COM9 side:
    each V frame should print 1 line `[SHADOW] V,... -> L=... scale=... sat=... `
    plus 4 lines `[SHADOW]   FL/FR/RL/RR wheel=... motor=... target=... cnt/20ms`

Usage
-----
    python3 pi_setup/shadow_probe.py [device]      # default /dev/ttySTM32

Standard library only, os + termios; no pyserial dependency (it may not be
installed on the Pi). Output is all ASCII. Exit code 0 = all four frames got #ACK.
"""

import os
import sys
import termios
import time
import re

DEV = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttySTM32"
BAUD = 115200                    # for humans
# ★ The value handed to termios must be the constant B115200, never the integer
#   115200: a[4]/a[5] expect the CBAUD encoding in c_cflag, and a bare number makes
#   tcsetattr fail with errno 22 Invalid argument (the first version died this way).
#   test_stm32_link.py uses the same constant.
BAUD_CONST = termios.B115200

# The 4 shadow-mode test frames. ★ All are legal contract §3 frames (mm/s, mrad/s, all integers).
FRAMES = [
    ("forward straight",  "V,300,0,0"),
    ("left strafe",       "V,0,300,0"),
    ("rotate left",       "V,0,0,1000"),
    ("diagonal+rotate",   "V,300,300,500"),
]

COUNTS_PER_M = 10505   # nominal value from firmware_core fixedpoint.h
PERIOD_MS = 20         # FIXP_CONTROL_PERIOD_MS_NOMINAL


def counts_of(mm_s):
    """mm/s -> counts per control window, same formula as fixp_mm_s_to_counts() (symmetric rounding)."""
    num = mm_s * COUNTS_PER_M * PERIOD_MS
    den = 1000 * 1000
    return (num + den // 2) // den if num >= 0 else -((-num + den // 2) // den)


def open_port(path):
    """Exact same serial configuration as test_stm32_link.py (verified on the real device there)."""
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
    """Read every byte that arrives within `seconds` (port is O_NONBLOCK, so yield manually when empty)."""
    out = b""
    # ★ Use monotonic rather than time.time(): the latter can jump forward/backward
    #   due to NTP, making drain return early (incomplete read) or hang. test_stm32_link.py
    #   also uses monotonic.
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
    """Set shadow mode to want_on and **verify** that it really took effect.

    ★ The firmware's local key ``.s`` is a **toggle**, not idempotent: if a previous
    abnormal exit left shadow mode "on", sending ``.s`` again actually turns it
    **off** -- then all four V frames stop printing ``[SHADOW]`` lines and the
    investigation goes completely off track (it looks like "shadow mode is broken").
    The firmware prints the new state as ``[U2] shadow (...) = ON/OFF``, so read it
    back here and toggle again if the state is wrong, at most three rounds.

    :returns: (reached desired state, text captured by the last drain)
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

    # 1) turn shadow mode on (the local key must carry the '.' prefix, or 's' is taken as a protocol byte)
    shadow_on, back = set_shadow(fd, True)
    print("--- shadow mode -> ON (verified=%s) ---" % ("YES" if shadow_on else "NO"))
    print(back if back.strip() else "(nothing)")
    print("")
    if not shadow_on:
        print("!! Could not confirm shadow mode turned on in three attempts -- the four V frames below")
        print("!! most likely will not print [SHADOW]. First send '.s' by hand on COM9 and confirm the")
        print("!! echo is '= ON'; then check whether the firmware is this version.")
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

    # 2) turn shadow mode off, restoring the bench to its baseline state
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
