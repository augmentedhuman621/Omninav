#!/usr/bin/env python3
"""B-03 验证夹具（只在本机 WSL 跑）：用 pty 模拟“雷达在吐数据”，回归 check_lidar.sh。

为什么用 pty：`pty.openpty()` 的从端（slave）对脚本来说就是一个 tty 设备——
  * `stty -F <slave>` 能设置 termios（与真串口行为一致）；
  * 主端（master）写进去的字节会出现在从端的读取流里。
所以它能证明 `probe_read()` 的 `timeout cat | od` 管道在“有数据”时确实能把字节带回，
而不是被 pipefail + `|| out=""` 吞成 0 字节（B-03 / B2 回归），也能证明修好后的参数
解析对 `--device DEV` 与 `--device=DEV` 两种写法一视同仁（B1 回归）。

边界：不碰树莓派、不碰真实硬件、不改系统；只在 /tmp 级别用 pty，不改仓库其他文件。
输出：`pi_setup/check_lidar_fix_pty.log`（UTF-8，全部原始输出），退出码 0=断言全过，1=有失败。

用法（WSL Ubuntu）：`python3 pi_setup/check_lidar_fix_pty_test.py`
"""
import os
import pty
import re
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(ROOT, "pi_setup", "check_lidar.sh")
LOG = os.path.join(ROOT, "pi_setup", "check_lidar_fix_pty.log")

PATTERN = bytes([0x54, 0x2C]) + b"\x00" * 46  # LD19 特征包头 54 2c 的假数据
ENV = dict(os.environ, OMNINAV_SKIP_ARCH_CHECK="1")

checks = []  # (ok: bool, name, detail)
raw_chunks = []


def check(ok, name, detail=""):
    checks.append((bool(ok), name, detail))
    print(("[PASS] " if ok else "[FAIL] ") + name + (("  | " + detail) if detail else ""))


def run_script(args, timeout=120):
    p = subprocess.run(
        ["bash", SCRIPT] + args,
        capture_output=True, text=True, timeout=timeout, env=ENV,
    )
    out = p.stdout + p.stderr
    raw_chunks.append("$ bash %s %s\nEXIT=%d\n%s\n" % (SCRIPT, " ".join(args), p.returncode, out))
    return p.returncode, out


def main():
    if not os.path.exists(SCRIPT):
        print("找不到脚本：%s" % SCRIPT)
        return 1

    # ---------- 0. 语法 / 完整性（原始证据单独落一个 log） ----------
    import hashlib
    with open(SCRIPT, "rb") as f:
        raw = f.read()
    sha = hashlib.sha256(raw).hexdigest()
    lines = raw.count(b"\n")
    syntax = subprocess.run(["bash", "-n", SCRIPT], capture_output=True, text=True)
    check(syntax.returncode == 0, "bash -n 语法检查通过", "EXIT=%d %s" % (syntax.returncode, syntax.stderr.strip()))
    syntax_log = [
        "# check_lidar.sh 语法与完整性检查（B-03，由 check_lidar_fix_pty_test.py 生成）\n",
        "time=%s\n" % time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "命令：bash -n %s\n" % SCRIPT,
        "SYNTAX_EXIT=%d\n" % syntax.returncode,
        "sha256=%s\n" % sha,
        "lines=%d\n" % lines,
    ]
    with open(os.path.join(ROOT, "pi_setup", "check_lidar_fix_syntax.log"), "w", encoding="utf-8") as f:
        f.write("".join(syntax_log))

    results = {}

    # ---------- B1：参数解析（空格形式 vs 等号形式） ----------
    rc_help, out_help = run_script(["--help"])
    check(rc_help == 2, "B1 --help 退出码 = 2", "实际 %d" % rc_help)
    check("--device DEV" in out_help and "--device=DEV" in out_help,
          "B1 usage 同时写明 --device DEV 与 --device=DEV", "")

    rc_bad, out_bad = run_script(["--device"])
    check(rc_bad == 2, "B1 `--device`（缺参数）退出码 = 2", "实际 %d" % rc_bad)
    check("需要带参数" in out_bad, "B1 `--device` 缺参数有明确提示", "")

    rc_empty, out_empty = run_script(["--device="])
    check(rc_empty == 2, "B1 `--device=`（空值）退出码 = 2", "实际 %d" % rc_empty)
    check("不能为空" in out_empty, "B1 `--device=` 空值有明确提示", "")

    rc_dash, out_dash = run_script(["--device", "--probe"])
    check(rc_dash == 2, "B1 `--device --probe`（值像选项）退出码 = 2", "实际 %d" % rc_dash)

    rc_unknown, out_unknown = run_script(["--bogus"])
    check(rc_unknown == 2, "B1 未知选项退出码 = 2", "实际 %d" % rc_unknown)

    # 两个形式都指向 /dev/null（能打开、但 stty 必然失败 ⇒ 快速且不碰硬件）
    rc_eq, out_eq = run_script(["--probe", "--device=/dev/null"])
    rc_sp, out_sp = run_script(["--probe", "--device", "/dev/null"], timeout=180)
    check(rc_eq == rc_sp, "B1 `--device=X` 与 `--device X` 退出码一致",
          "=形式 %d / 空格形式 %d" % (rc_eq, rc_sp))
    check("探测设备 = /dev/null" in out_eq and "探测设备 = /dev/null" in out_sp,
          "B1 两种形式都真的把 /dev/null 当探测设备", "")
    check(rc_eq != 2, "B1 等号形式不再被当参数错误（退出码≠2）", "实际 %d" % rc_eq)
    check(rc_sp != 2, "B1 空格形式不再被当参数错误（退出码≠2，修复前是 2）", "实际 %d" % rc_sp)

    # ---------- B2：stty 失败与 0 字节分开报（/dev/null = 能打开但不是 tty） ----------
    check("stty 设 /dev/null 失败" in out_eq, "B2 stty 失败有独立提示（不是 0 字节）", "")
    check("本次判定：D2" in out_eq or "探测未执行：stty 设不了设备" in out_eq,
          "B2 判定区独立报告 D2/探测未执行（与 0 字节区分）", "")
    check("本轮探测未执行" in out_eq, "B2 明说探测未执行", "")
    check("所有候选波特率下 3 秒内都是 0 字节" not in out_eq,
          "B2 stty 失败时不再误报“全部 0 字节”", "")

    # ---------- B2：pty 模拟数据（核心回归：修复前恒 0 字节） ----------
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    stop = threading.Event()

    def writer():
        while not stop.is_set():
            try:
                os.write(master, PATTERN)
            except OSError:
                break
            time.sleep(0.02)

    t = threading.Thread(target=writer, daemon=True)
    t.start()
    try:
        rc_pty, out_pty = run_script(["--probe", "--device", slave_name], timeout=180)
    finally:
        stop.set()
        t.join(timeout=2)
        os.close(master)
        os.close(slave)

    m = re.search(r"被动收到 (\d+) 字节", out_pty)
    bytes_seen = int(m.group(1)) if m else 0
    check(bytes_seen > 0, "B2 pty 有数据时 --probe 读到字节（修复前恒 0）",
          "读到 %d 字节，pty=%s" % (bytes_seen, slave_name))
    check("含 54 2c（LD19 包头特征）" in out_pty, "B2 能识别 54 2c 包头特征", "")
    check("本次判定：D2 探测条件不满足" not in out_pty, "B2 pty 数据路径不误判 D2", "")
    check(rc_pty != 2, "B2 pty 探测运行未被参数/环境错误中断", "EXIT=%d" % rc_pty)

    # ---------- 只读边界：脚本不应真的执行安装/改系统动作 ----------
    with open(SCRIPT, "r", encoding="utf-8", errors="replace") as f:
        src = f.read()
    # 只看“会被执行的行”：行首才是命令；提示文字里的 apt install 不算
    exec_lines = [ln.strip() for ln in src.splitlines()
                  if not ln.lstrip().startswith("#") and re.match(r"^\s*(sudo\s+)?(apt|apt-get|pip3?|dnf|yum|snap)\s+(install|remove)\b", ln)]
    check(not exec_lines, "只读边界：脚本无安装/卸载命令执行行", "命中 %s" % exec_lines if exec_lines else "")
    danger = re.findall(r"(?m)^\s*(?:sudo\s+)?(?:tee|cp|mv|rm|chmod|chown|usermod|udevadm\s+control)\b[^\n]*/(?:etc|usr|opt|lib)\b", src)
    check(not danger, "只读边界：脚本无写 /etc /usr /opt 的执行行", "命中 %s" % danger if danger else "")
    check(src.count('>"$PROBE_DEV"') == 1 and '>"$dev"' not in src,
          "只读边界：唯一写设备的是 --request-rplidar 的 2 字节请求", "")

    # ---------- 汇总 ----------
    ok_n = sum(1 for c in checks if c[0])
    print("\n==== 断言汇总：%d/%d 通过 ====" % (ok_n, len(checks)))
    raw_chunks.append("\n==== 断言汇总：%d/%d 通过 ====\n" % (ok_n, len(checks)))
    for ok, name, detail in checks:
        raw_chunks.append(("[PASS] " if ok else "[FAIL] ") + name + (("  | " + detail) if detail else "") + "\n")
    with open(LOG, "w", encoding="utf-8") as f:
        f.write("".join(raw_chunks))
    print("原始输出已写入：%s" % LOG)
    return 0 if ok_n == len(checks) else 1


if __name__ == "__main__":
    sys.exit(main())
