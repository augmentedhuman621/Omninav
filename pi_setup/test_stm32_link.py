#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OmniNav - STM32 <-> Raspberry Pi  USART2 link self-test

WHAT IT PROVES
  Runs ON the Raspberry Pi and drives /dev/ttySTM32 (the independent USB-TTL
  module wired to the STM32's USART2, PA2=TX / PA3=RX).  It checks, in order:

    T1  heartbeat   STM32 -> Pi   #HB <tick_ms> <n> every 1000 ms
    T2  echo        Pi -> STM32   every received byte is echoed back verbatim
    T3  kick        byte-exactness of a 62-char payload
    T4  protocol V  V,300,0,0\\n  -> #ACK V 3 + cmd=V argc=3 err=0x0
    T5  protocol S / reset
    T6  bad frames  argc error + unknown command are reported, not crashed
    T7  local keys  ?  prints a #STAT line; drop=0 rxerr=0

WHY NO pyserial
  This box may have no internet at test time.  Only the Python standard
  library (os + termios) is used, so it always runs.

USAGE
  python3 pi_setup/test_stm32_link.py            # default /dev/ttySTM32
  python3 pi_setup/test_stm32_link.py /dev/ttyUSB0

PRECONDITIONS
  * P5 jumper on the STM32 board REMOVED  (else PA2/PA3 are tied to SP3485)
  * no serial-getty / ModemManager holding the port
  * user in the dialout group (udev rule grants root:dialout 0660)

EXIT CODE 0 = all passed, 1 = at least one failed.
"""

import os
import re
import sys
import time
import termios

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttySTM32"
BAUD_CONST = termios.B115200

KICK_PAYLOAD = b"0123456789 ABCDEFGHIJKLMNOPQRSTUVWXYZ abcdefghijklmnopqrstuvwxyz"

results = []          # (name, ok, detail)
evidence = []         # raw lines worth showing


def record(name, ok, detail=""):
    results.append((name, bool(ok), detail))
    print("  [%s] %-34s %s" % ("PASS" if ok else "FAIL", name, detail))


def note(text):
    evidence.append(text)


# --------------------------------------------------------------- port plumbing
def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = 0                                   # iflag : raw
    a[1] = 0                                   # oflag : raw
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL   # 8N1, ignore modem
    a[3] = 0                                   # lflag : raw, no echo
    a[4] = BAUD_CONST
    a[5] = BAUD_CONST
    a[6][termios.VMIN] = 0
    a[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def collect(fd, seconds):
    """Accumulate bytes for `seconds`, non-blocking."""
    buf = bytearray()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            chunk = os.read(fd, 4096)
            if chunk:
                buf.extend(chunk)
                continue
        except BlockingIOError:
            pass
        except OSError:
            pass
        time.sleep(0.01)
    return bytes(buf)


def send(fd, data):
    if isinstance(data, str):
        data = data.encode("ascii")
    os.write(fd, data)


def lines_of(buf):
    out = []
    for raw in buf.split(b"\n"):
        s = raw.rstrip(b"\r").decode("ascii", "replace")
        if s:
            out.append(s)
    return out


def show(tag, buf, keep=14):
    ls = lines_of(buf)
    for s in ls[:keep]:
        note("    %-4s | %s" % (tag, s))
    if len(ls) > keep:
        note("    %-4s | ... (%d more lines)" % (tag, len(ls) - keep))
    return ls


def raw_text(buf):
    """Whole stream as one string.

    ★ The criteria must use this, never per-line matching -- the STM32 echoes each
    byte as it arrives, verbatim, and does not add a newline for us. So when a
    frame is sent without a trailing newline, the echo merges into the same line
    as the STM32's next unsolicited output (#HB / [U2] / #STAT). Matching with
    per-line startswith gives false failures (that is how T2/T7 first failed).
    """
    return buf.decode("ascii", "replace")


def local_key(fd, key):
    """Send a local command. ★ It must carry the '.' prefix.

    The first version sent bare local commands, so the 'e' in the protocol command
    `reset` was eaten by the local "toggle echo" command and the parser received
    only 'rst' -> cmd=UNKNOWN. The firmware side has been changed to a prefix state
    machine; if the prefix is missing here, the counter clears in T1/T4 break and
    T7 feeds '?' straight into the parser as a protocol byte.
    """
    send(fd, "." + key)


# --------------------------------------------------------------- main
def main():
    if not os.path.exists(PORT):
        print("FATAL: %s does not exist." % PORT)
        print("       Is the USB-TTL module plugged in?  ls -l /dev/ttyUSB*")
        return 1

    print("OmniNav USART2 link self-test")
    print("port = %s   baud = 115200 8N1   (pure stdlib, no pyserial)" % PORT)
    real = os.path.realpath(PORT)
    print("realpath = %s" % real)

    fd = open_port(PORT)
    try:
        # ---------------------------------------------------------- T1 heartbeat
        print("\n--- T1  heartbeat: STM32 -> Pi, expect one #HB per 1000 ms ---")
        local_key(fd, "c")                  # clear STM32 counters + parser
        collect(fd, 0.3)
        b = collect(fd, 3.4)
        ls = show("T1", b)
        hb = [s for s in ls if s.startswith("#HB ")]
        ticks = []
        for s in hb:
            m = re.match(r"^#HB\s+(\d+)\s+(\d+)$", s)
            if m:
                ticks.append(int(m.group(1)))
        gaps = [ticks[i + 1] - ticks[i] for i in range(len(ticks) - 1)]
        # ★ Tolerance is ±30 ms, not ±5 ms. Why:
        #   The STM32 uses blocking HAL_UART_Transmit, and the heartbeat timestamp is
        #   taken at **the moment of transmission**. Besides the heartbeat, the main
        #   loop also prints a `[U2] rx=...` status line (about 110 chars, ~9.5 ms of
        #   air time at 115200). When a status line lands right on a heartbeat's
        #   deadline, that heartbeat's **transmit instant** is pushed back 10-20 ms --
        #   measured `gaps=[1000, 1000, 1015]`, and that 1015 immediately follows the
        #   status line. => This measures "transmit instants", not "the timebase"; the
        #   timebase itself is accurate (without status-line interference adjacent
        #   beats are always exactly 1000). The final firmware will use DMA/interrupt
        #   transmission, so the jitter disappears naturally.
        JITTER_MS = 30
        ok = len(hb) >= 3 and bool(gaps) and all(
            1000 - JITTER_MS <= g <= 1000 + JITTER_MS for g in gaps)
        record("T1 heartbeat 1000 ms period", ok,
               "%d frames, gaps=%s" % (len(hb), gaps))
        if gaps and any(g > 1000 + JITTER_MS or g < 1000 - JITTER_MS for g in gaps):
            note("    T1   | note: outlier > tolerance. If it tracks an [U2] status")
            note("    T1   |       line, it is blocking-TX jitter, not timebase drift.")
        if not hb:
            print("\n*** No #HB at all -> STM32 is not transmitting on USART2.")
            print("*** Check: P5 jumper removed? PA2 -> module RXD? firmware flashed?")

        # ---------------------------------------------------------- T2 echo
        print("\n--- T2  echo: every byte sent comes back verbatim ---")
        collect(fd, 0.2)
        local_key(fd, "c")
        collect(fd, 0.3)
        send(fd, "OMNINAV_ECHO_0123456789")
        b = collect(fd, 0.8)
        ls = show("T2", b)
        ok = "OMNINAV_ECHO_0123456789" in raw_text(b)
        record("T2 echo verbatim", ok,
               "found" if ok else "not found")

        # ---------------------------------------------------------- T3 kick
        print("\n--- T3  kick: byte-exact 62-char payload from the STM32 ---")
        collect(fd, 0.2)
        local_key(fd, "k")
        b = collect(fd, 0.8)
        ls = show("T3", b)
        got = [s for s in ls if s.startswith("#KICK ")]
        ok = bool(got) and got[0][6:].encode("ascii", "replace") == KICK_PAYLOAD
        record("T3 kick payload byte-exact", ok,
               "%d chars" % (len(got[0]) - 6 if got else 0))

        # ---------------------------------------------------------- T4 protocol V
        print("\n--- T4  protocol frame  V,300,0,0\\n ---")
        collect(fd, 0.2)
        local_key(fd, "c")
        collect(fd, 0.3)
        send(fd, "V,300,0,0\n")
        b = collect(fd, 1.0)
        ls = show("T4", b)
        blob = "\n".join(ls)
        c1 = "#ACK V 3" in ls
        c2 = "cmd=V" in blob and "argc=3" in blob and "err=0x0" in blob
        c3 = "arg: 300 0 0" in blob
        record("T4a #ACK V 3", c1, "" if c1 else "missing")
        record("T4b cmd=V argc=3 err=0x0", c2, "" if c2 else "missing")
        record("T4c args decoded 300 0 0", c3, "" if c3 else "missing")

        # ---------------------------------------------------------- T5 S / reset
        print("\n--- T5  protocol frames  S\\n  and  reset\\n ---")
        collect(fd, 0.2)
        send(fd, "S\n")
        b1 = collect(fd, 0.7)
        send(fd, "reset\n")
        b2 = collect(fd, 0.7)
        ls = show("T5", b1 + b2)
        blob = "\n".join(ls)
        ok = ("cmd=S" in blob and "argc=0" in blob
              and "cmd=reset" in blob)
        record("T5 cmd=S / cmd=reset", ok, "" if ok else "missing")

        # ---------------------------------------------------------- T6 bad frames
        print("\n--- T6  bad frames are reported, not crashed ---")
        collect(fd, 0.2)
        send(fd, "V,1,2\n")            # V needs 3 args -> ARGC error
        b1 = collect(fd, 0.7)
        send(fd, "X,1\n")              # unknown command
        b2 = collect(fd, 0.7)
        ls = show("T6", b1 + b2)
        blob = "\n".join(ls)
        # PROTO_ERR_ARGC = 0x0010
        ok1 = "err=0x10" in blob
        ok2 = "cmd=UNKNOWN" in blob
        record("T6a argc error reported", ok1, "" if ok1 else "missing")
        record("T6b unknown command reported", ok2, "" if ok2 else "missing")

        # ---------------------------------------------------------- T7 local keys
        print("\n--- T7  local key '.?' -> #STAT line, drop=0 rxerr=0 ---")
        collect(fd, 0.2)
        local_key(fd, "?")
        b = collect(fd, 1.0)
        ls = show("T7", b)
        blob = raw_text(b)
        # ★ Use substring matching: the '.' echo merges into the same line as '[U2] rx=' (see raw_text's comment)
        has_stat = "#STAT" in blob
        has_u2 = "[U2] rx=" in blob
        ok = has_stat and has_u2
        m = re.search(r"#STAT[^\r\n]*", blob)
        record("T7a #STAT line", ok, m.group(0) if m else "no #STAT")

        m = re.search(r"drop=(\d+) rxerr=(\d+)", blob)
        drop0 = int(m.group(1)) if m else None
        err0 = int(m.group(2)) if m else None
        ok = (drop0 == 0 and err0 == 0)
        record("T7b no dropped bytes / no RX errors", ok,
               "drop=%s rxerr=%s" % (drop0, err0))

        # ---------------------------------------------------------- summary
        print("\n" + "=" * 62)
        bad = [r for r in results if not r[1]]
        print("RESULT: %d/%d passed" % (len(results) - len(bad), len(results)))
        if bad:
            print("FAILED:")
            for name, _, detail in bad:
                print("   - %s   %s" % (name, detail))
        else:
            print("ALL PASS - STM32 USART2 <-> Raspberry Pi link is fully OK")
        print("=" * 62)

        if evidence:
            print("\n--- raw evidence ---")
            for line in evidence:
                print(line)

        return 1 if bad else 0
    finally:
        os.close(fd)


if __name__ == "__main__":
    sys.exit(main())
