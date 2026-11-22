#!/usr/bin/env python3
"""`libomninav_core.so` 的**自洽性**自检 —— 桥能不能真的加载它、数学对不对得上。

这个脚本是 T1（``core-algo``）与 T2（``bridge-dev``）之间的**交接验收工具**：
它不问"实现得漂亮吗"，只问四件事：

1. ``.so`` 在不在契约 §5.3 规定的三个路径之一；
2. 契约 §5.2 要求的 **7 个符号**在不在；
3. ``ctypes.Structure`` 与 C 的 ABI 对不对得上（特别是 ``odom_pose_t`` 的尾部 padding）；
4. ``mecanum_inverse()`` 与 ``odom_update()`` **互相一致** —— 用逆解算出的四轮位移喂给
   里程计，应该原样还原成当初那条 twist；
5. 航向归一化的**常量语义**对不对 —— 连续自转 ≥ 2 整圈后，``pose.theta_mrad`` 既要落在
   ``(-ODOM_HALF_TURN_MRAD, ODOM_HALF_TURN_MRAD]``，又要与 ``total.theta_mrad`` 相差
   **非零整数圈**（契约 §5.1 的验收判据，见文件里 ``ODOM_HALF_TURN_MRAD`` 的说明）。

★★ 第 4 条是刻意设计的：它**只用 C 的数学**（Python 这边一个公式都不写），
所以它同时验证了两件事 —— 桥的 ctypes 绑定是对的，以及正/逆解与里程计没有互相打架。

退出码：``0`` 全通过、``2`` 找不到 ``.so``、``3`` 少符号、``4`` 自检失败。

跑法::

    # 树莓派（装好之后）
    python3 tools/check_core.py

    # 笔记本 / WSL（开发时指向 firmware_core 的产物）
    OMNINAV_CORE_SO=/mnt/d/dsh/工程设计/firmware_core/build/libomninav_core.so \
        python3 tools/check_core.py
"""

from __future__ import annotations

import os
import sys
from typing import List, Optional, Sequence, Tuple

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(_HERE, "..", "src", "stm32_bridge")))

import ctypes  # noqa: E402

from stm32_bridge import kinematics  # noqa: E402

EXIT_OK = 0
EXIT_NO_LIBRARY = 2
EXIT_NO_SYMBOL = 3
EXIT_CHECK_FAILED = 4

# ``odom_update()`` 返回 0 的记录。C 返回 0 = 几何无效 / 指针为空（器质性错误），
# 一旦发生，后面所有位姿断言都会跟着红 —— 单独留个痕，好让人一眼看出
# "不是积分算错了，是根本一帧都没收下"。
_REJECTIONS: List[str] = []

#: ★★ ``odom.h`` 的宏 ``ODOM_HALF_TURN_MRAD`` —— 航向归一化的"半圈"常量（契约 §5.1）。
#:
#: 来源：``firmware_core/include/odom.h: ODOM_HALF_TURN_MRAD``，真实现在
#: ``firmware_core/src/odom.c`` 的 ``odom_normalize_mrad()``。
#:
#: **这不是契约 §6 禁的那种"第二份魔数"。** 契约 §6 禁的是几何/标定数字（轮径、轴距、
#: 编码器分辨率）在第二个地方被重写 —— 那些数字 Python 侧一个字都不该有。而这里是一个
#: **C 宏**：它在编译期就烧进 ``.so`` 了，**ctypes 读不到宏**（宏不是符号，导出表里没有），
#: 所以 Python 侧只能复写一份。为了不让"复写"退化成"猜"，本脚本会做两件事：
#:   1. 优先尝试从 ``.so`` 里读访问器 ``odom_half_turn_mrad()``（现在没有，将来若加上就自动采用）；
#:   2. 读不到时用本常量，并由 `main()` 里"航向归一化：契约 §5.1 的验收判据"那三条
#:      在**运行时证明它**：连续自转 ≥ 2 整圈后，``(pose.theta_mrad - total.theta_mrad)``
#:      必须恰是 ``2*HALF`` 的非零整数倍。常量抄错（或 C 侧没同步）时这条会立刻红。
#:
#: ★ 修订史（2026-10-07 契约 §5.1 拍板）：原值 ``180000`` 是**真 bug**，不只是注释错 ——
#: ``180000 mrad = 180 rad ≈ 10313°``，与契约原文"即 (-180, 180] 度"自相矛盾；更要命的是
#: 归一化要保持角度等价，**回绕量必须是整圈的整数倍**，而 ``2 × 180000 = 360000 mrad``
#: = 57.2958 圈（余 106.48°）⇒ 一旦 |θ| 越过 180000，航向会**跳变 106.48°**。
#: 现值 ``3142`` 的回绕量 ``2 × 3142 = 6284 mrad`` ≈ 2π rad，每次回绕只引入
#: ``6284 - 6283.185 = 0.815 mrad`` ≈ 0.047° 的量化误差。
ODOM_HALF_TURN_MRAD = 3142

#: 历史值，只用于**诊断**：如果 ``pose.theta_mrad`` 恰好等于"按 180000 归一化"的期望值，
#: 就说明 ``.so`` 是用旧的 ``odom.c`` 编出来的（宏没同步）。不参与任何通过/失败判定。
_LEGACY_HALF_TURN_MRAD = 180000


def _expected_theta(total_mrad: int, half_turn: int) -> int:
    """``total_mrad`` 在 ``(-half_turn, half_turn]`` 里的那个唯一代表元。

    这是**纯模运算**，不是"第二份里程计算法"：它只回答"哪个角与 total 相差整数圈、
    且落在规范区间里"，不碰任何几何/积分。用来把期望值**算出来写进断言**，
    而不是靠人肉心算（契约 §5.1 的判据①）。
    """
    full = 2 * half_turn
    remainder = (total_mrad + half_turn) % full  # 归到 [0, full)
    if remainder <= 0:
        remainder += full
    return remainder - half_turn


def _resolve_half_turn_mrad(core: "kinematics.Core") -> Tuple[int, str]:
    """能读到就用 ``.so`` 里的值，读不到就用本文件的常量。

    返回 ``(值, 来源说明)``。来源会打印进检查详情里，好让人一眼看出这次到底是
    "照着 C 自己的宏验的"还是"照着 Python 侧抄的那份验的"。
    """
    lib = getattr(core, "_lib", None)
    for name in ("odom_half_turn_mrad", "omninav_odom_half_turn_mrad"):
        candidate = getattr(lib, name, None)
        if candidate is None:
            continue
        candidate.restype = ctypes.c_int32
        candidate.argtypes = []
        value = int(candidate())
        if value > 0:
            return value, "lib:%s()" % name
    return ODOM_HALF_TURN_MRAD, "常量 ODOM_HALF_TURN_MRAD（源 odom.h；.so 里没有访问器）"


class Checks:
    def __init__(self) -> None:
        self.rows: List[Tuple[str, bool, str]] = []

    def check(self, name: str, ok: bool, detail: str = "") -> bool:
        self.rows.append((name, bool(ok), detail))
        return bool(ok)

    def report(self) -> int:
        print("")
        print("-" * 96)
        for name, ok, detail in self.rows:
            print("%-4s %-46s %s" % ("ok" if ok else "FAIL", name, detail))
        print("-" * 96)
        failed = [row for row in self.rows if not row[1]]
        print("共 %d 条检查，%d 条失败" % (len(self.rows), len(failed)))
        return EXIT_OK if not failed else EXIT_CHECK_FAILED


def _twist(vx: int = 0, vy: int = 0, omega: int = 0) -> kinematics.MecanumTwist:
    return kinematics.MecanumTwist(vx_mm_s=vx, vy_mm_s=vy, omega_mrad_s=omega)


def _drive(
    core: kinematics.Core,
    geom: kinematics.MecanumGeom,
    twist: kinematics.MecanumTwist,
    frames: int,
) -> Tuple[kinematics.OdomPose, kinematics.OdomTotal, Optional[kinematics.MecanumTwist]]:
    """把一条 twist 通过 **C 的逆解** 变成四轮位移，累加 ``frames`` 帧喂给里程计。

    ★ 全程没有一行 Python 数学：轮值来自 ``mecanum_inverse()``，
    位姿来自 ``odom_update()``。
    """
    wheels = core.inverse(geom, twist)
    step = [int(wheels.wheel_mm_s[index]) for index in range(kinematics.WHEEL_COUNT)]
    pose, total = core.odom_reset()
    d_prev = [0] * kinematics.WHEEL_COUNT
    d_now = [0] * kinematics.WHEEL_COUNT
    last = None
    for index in range(frames):
        d_now = [d_now[wheel] + step[wheel] for wheel in range(kinematics.WHEEL_COUNT)]
        last = core.odom_update(pose, total, geom, d_now, d_prev)
        if last is None:
            _REJECTIONS.append("第 %d 帧（共 %d 帧）" % (index + 1, frames))
        d_prev = list(d_now)
    return pose, total, last


def main(argv: Optional[Sequence[str]] = None) -> int:
    print("候选路径（契约 §5.3 的顺序）：")
    for index, path in enumerate(kinematics.candidate_paths(), start=1):
        print("  %d. %s%s" % (index, path, "  <- 存在" if os.path.isfile(path) else ""))

    try:
        core = kinematics.Core()
    except kinematics.CoreLibraryNotFound as exc:
        print("\n找不到 %s：\n%s" % (kinematics.SONAME, exc))
        return EXIT_NO_LIBRARY
    except kinematics.CoreSymbolMissing as exc:
        print("\n%s" % exc)
        return EXIT_NO_SYMBOL
    except kinematics.CoreLibraryError as exc:
        print("\n%s" % exc)
        return EXIT_NO_LIBRARY

    print("\n已加载：%s" % core.describe())
    checks = Checks()

    # ---- ABI：结构体大小 ------------------------------------------------
    sizes = {
        "MecanumGeom": (ctypes.sizeof(kinematics.MecanumGeom), 12),
        "MecanumTwist": (ctypes.sizeof(kinematics.MecanumTwist), 12),
        "MecanumWheels": (ctypes.sizeof(kinematics.MecanumWheels), 16),
        "OdomPose": (ctypes.sizeof(kinematics.OdomPose), 24),
        "OdomTotal": (ctypes.sizeof(kinematics.OdomTotal), 24),
    }
    for name, (actual, expected) in sizes.items():
        checks.check(
            "sizeof(%s) == %d" % (name, expected),
            actual == expected,
            "实际 %d 字节" % actual,
        )

    # ---- 几何 -----------------------------------------------------------
    geom = core.default_geom()
    checks.check(
        "mecanum_default_geom() 是合法几何",
        core.geom_is_valid(geom),
        "half_wheelbase=%d half_track=%d vy_polarity=%d omega_polarity=%d"
        % (
            geom.half_wheelbase_mm,
            geom.half_track_mm,
            geom.vy_polarity,
            geom.omega_polarity,
        ),
    )
    l_mm = core.l_mm(geom)
    checks.check("mecanum_l_mm() > 0", l_mm > 0, "L = %d mm" % l_mm)

    # ---- 逆解 -> 正解 往返 ----------------------------------------------
    for vx, vy, omega in ((200, 0, 0), (0, 150, 0), (0, 0, 300), (-200, 120, -300)):
        wheels = core.inverse(geom, _twist(vx, vy, omega))
        back = core.forward(geom, [int(wheels.wheel_mm_s[i]) for i in range(4)])
        ok = (
            abs(back.vx_mm_s - vx) <= 3
            and abs(back.vy_mm_s - vy) <= 3
            and abs(back.omega_mrad_s - omega) <= 3
        )
        checks.check(
            "逆解->正解往返 (%d,%d,%d)" % (vx, vy, omega),
            ok,
            "wheels=%s -> (%d,%d,%d)"
            % (
                [int(wheels.wheel_mm_s[i]) for i in range(4)],
                back.vx_mm_s,
                back.vy_mm_s,
                back.omega_mrad_s,
            ),
        )

    # ---- 逆解 -> 里程计 一致性（单帧） ---------------------------------
    for vx, vy, omega, label in (
        (200, 0, 0, "正前方"),
        (0, 150, 0, "正左/右横移"),
        (0, 0, 300, "原地自转"),
    ):
        pose, total, twist = _drive(core, geom, _twist(vx, vy, omega), 1)
        ok = (
            abs(pose.x_mm - vx) <= 3
            and abs(pose.y_mm - vy) <= 3
            and abs(pose.theta_mrad - omega) <= 3
        )
        checks.check(
            "单帧里程计还原 %s (%d,%d,%d)" % (label, vx, vy, omega),
            ok,
            "pose=(%d mm, %d mm, %d mrad) twist=%s"
            % (
                pose.x_mm,
                pose.y_mm,
                pose.theta_mrad,
                None if twist is None else (twist.vx_mm_s, twist.vy_mm_s, twist.omega_mrad_s),
            ),
        )

    # ---- 100 帧直行：位置应线性累加 ------------------------------------
    pose, total, _ = _drive(core, geom, _twist(200, 0, 0), 100)
    checks.check(
        "100 帧 x 200 mm/s 直行 -> x ≈ 20000 mm",
        abs(pose.x_mm - 20000) <= 60,
        "x=%d mm y=%d mm theta=%d mrad" % (pose.x_mm, pose.y_mm, pose.theta_mrad),
    )
    checks.check(
        "直行不产生横向假漂移",
        abs(pose.y_mm) <= 5,
        "y=%d mm" % pose.y_mm,
    )

    # ---- 100 帧原地自转：★ 圆弧积分的关键考点 --------------------------
    # 欧拉积分在这里最容易出错：原地自转不该产生任何平移。
    pose, total, _ = _drive(core, geom, _twist(0, 0, 300), 100)
    checks.check(
        "100 帧原地自转 -> theta_total ≈ 30000 mrad",
        abs(total.theta_mrad - 30000) <= 300,
        "theta_total=%d mrad" % total.theta_mrad,
    )
    checks.check(
        "原地自转不产生假位移（圆弧积分的核心考点）",
        abs(pose.x_mm) <= 5 and abs(pose.y_mm) <= 5,
        "x=%d mm y=%d mm（欧拉积分在这里通常会漂出几百 mm）" % (pose.x_mm, pose.y_mm),
    )
    # ---- ★★ 航向归一化：契约 §5.1 的验收判据（累计 >= 2 整圈）-----------
    #
    # 上面那次 100 帧自转累计了 theta_total ≈ 30000 mrad；按新常量 6284 mrad/圈算
    # 约 4.77 圈（>= 2 整圈），正好够验归一化。两条判据：
    #   ① pose.theta_mrad 落在 (-HALF, HALF]；
    #   ② (pose.theta_mrad - total.theta_mrad) 是 2*HALF 的非零整数倍。
    # ★ ② 才是"归一化保持角度等价"的**证明式**：它只在 HALF 的回绕量真是整圈时才成立。
    #   旧值 180000 能过 ①（区间够大，值原样不变自然"落在里面"），但过不了 ② ——
    #   例如 total=200000 → pose=-160000，差值 360000，360000 % 6284 = 1744 != 0。
    half_turn, half_turn_src = _resolve_half_turn_mrad(core)
    full_turn = 2 * half_turn
    total_theta = int(total.theta_mrad)
    pose_theta = int(pose.theta_mrad)
    expected_theta = _expected_theta(total_theta, half_turn)
    delta_theta = pose_theta - total_theta
    turns = delta_theta // full_turn

    # 先挡住"这次自转不够 2 圈、上面两条判断题其实没有意义"这种情况，
    # 免得判据在将来被人改小自转量之后**静默地变成永远通过**。
    checks.check(
        "本次自转累计 >= 2 整圈（否则下面两条判据无意义）",
        abs(total_theta) >= 2 * full_turn,
        "累计 %d mrad = %.2f 圈；2 整圈 = %d mrad（半圈常量 %d，来源 %s）"
        % (total_theta, total_theta / full_turn, 2 * full_turn, half_turn, half_turn_src),
    )

    legacy_expected = _expected_theta(total_theta, _LEGACY_HALF_TURN_MRAD)
    stale_hint = ""
    if pose_theta == legacy_expected and legacy_expected != expected_theta:
        stale_hint = (
            "；★★ pose 恰好等于『按旧常量 180000 归一化』的期望值 %d"
            " ⇒ .so 是用旧 odom.c 编的（firmware_core/src/odom.c 里 ODOM_HALF_TURN_MRAD 还没同步成 3142；"
            "不写行号，行号会漂）"
            % legacy_expected
        )

    checks.check(
        ">= 2 整圈后 pose.theta_mrad 落在 (-%d, %d] 且等于算出的期望值" % (half_turn, half_turn),
        -half_turn < pose_theta <= half_turn and pose_theta == expected_theta,
        "pose.theta=%d mrad；按 %d 归一化应得 %d mrad（累计 %d mrad，差 %d mrad）%s"
        % (
            pose_theta,
            half_turn,
            expected_theta,
            total_theta,
            pose_theta - expected_theta,
            stale_hint,
        ),
    )
    checks.check(
        "归一化保持角度等价：差值恰为 %d mrad 的非零整数倍" % full_turn,
        delta_theta % full_turn == 0 and delta_theta != 0,
        "pose.theta - total.theta = %d mrad = %d 圈（应非零；期望 pose=%d mrad）"
        % (delta_theta, turns, expected_theta),
    )

    # ---- 零位移：位姿必须一动不动 --------------------------------------
    pose_a, total_a = core.odom_reset()
    zero = [0] * kinematics.WHEEL_COUNT
    core.odom_update(pose_a, total_a, geom, zero, zero)
    checks.check(
        "Δd == 0 时位姿保持不变",
        pose_a.x_mm == 0 and pose_a.y_mm == 0 and pose_a.theta_mrad == 0,
        "pose=(%d, %d, %d)" % (pose_a.x_mm, pose_a.y_mm, pose_a.theta_mrad),
    )

    # ---- C 有没有拒收过帧 ----------------------------------------------
    checks.check(
        "odom_update() 全程没有返回 0（几何有效性）",
        not _REJECTIONS,
        "%d 次拒绝：%s" % (len(_REJECTIONS), _REJECTIONS[0]) if _REJECTIONS else "0 次拒绝",
    )

    return checks.report()


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
