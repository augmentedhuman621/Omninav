#!/usr/bin/env python3
"""`libomninav_core.so` — **self-consistency** check: can the bridge really load it, and does the math add up.

This script is the **handover acceptance tool** between T1 (``core-algo``) and T2 (``bridge-dev``):
It does not ask "is the implementation pretty", only these things:

1. Whether the ``.so`` is at one of the three paths specified by contract §5.3;
2. Whether the **7 symbols** required by contract §5.2 are present;
3. Whether the ``ctypes.Structure`` layout matches the C ABI (especially the tail padding of ``odom_pose_t``);
4. Whether ``mecanum_inverse()`` and ``odom_update()`` are **mutually consistent** — feed the four-wheel displacements from the inverse solution into
   the odometry and they should restore exactly the original twist;
5. Whether the **constant semantics** of heading normalization are right — after >= 2 full turns of continuous rotation, ``pose.theta_mrad`` must fall in
   ``(-ODOM_HALF_TURN_MRAD, ODOM_HALF_TURN_MRAD]`` and must differ from ``total.theta_mrad`` by
   a **non-zero integer number of turns** (contract §5.1 acceptance criterion; see the ``ODOM_HALF_TURN_MRAD`` note in this file).

★★ Item 4 is deliberate: it **uses only the C math** (not a single formula on the Python side),
so it verifies two things at once — the bridge's ctypes bindings are right, and the forward/inverse solutions do not fight the odometry.

Exit codes: ``0`` all passed, ``2`` ``.so`` not found, ``3`` missing symbols, ``4`` self-check failed.

Usage::

    # Raspberry Pi (after installation)
    python3 tools/check_core.py

    # Laptop / WSL (point at the firmware_core build output during development)
    OMNINAV_CORE_SO=/mnt/d/dsh/<workspace>/firmware_core/build/libomninav_core.so \
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

# Records of ``odom_update()`` returning 0. C returning 0 = invalid geometry / null pointer (structural error);
# once it happens every later pose assertion turns red too — keep a separate trace so it is obvious at a glance
# that "it is not wrong integration, it is that not a single frame was accepted".
_REJECTIONS: List[str] = []

#: ★★ The ``odom.h`` macro ``ODOM_HALF_TURN_MRAD`` — the "half turn" constant for heading normalization (contract §5.1).
#:
#: Source: ``firmware_core/include/odom.h: ODOM_HALF_TURN_MRAD``; the real implementation is in
#: ``odom_normalize_mrad()`` in ``firmware_core/src/odom.c``.
#:
#: **This is not the "second copy of a magic number" that contract §6 forbids.** §6 forbids duplicating geometry/calibration numbers (wheel radius, wheelbase,
#: encoder resolution) in a second place — Python must not contain a single one of those. This is a
#: **C macro**: it is baked into the ``.so`` at compile time and **ctypes cannot read macros** (a macro is not a symbol, it is not in the export table),
#: so the Python side has to restate it. To keep that restatement from degenerating into a guess, this script does two things:
#:   1. First try to read the accessor ``odom_half_turn_mrad()`` from the ``.so`` (it does not exist yet; if added later it is adopted automatically);
#:   2. Otherwise use this constant, and let the three "heading normalization: contract §5.1 acceptance" checks in `main()`
#:      **prove it at runtime**: after >= 2 full turns of continuous rotation, ``(pose.theta_mrad - total.theta_mrad)``
#:      must be exactly a non-zero integer multiple of ``2*HALF``. If the constant is copied wrong (or the C side is not synced) this check goes red immediately.
#:
#: ★ Revision history (contract §5.1 decision, 2026-10-07): the original ``180000`` was a **real bug**, not just a wrong comment —
#: ``180000 mrad = 180 rad ≈ 10313°``, contradicting the contract's own "i.e. (-180, 180] degrees"; worse,
#: normalization must preserve angular equivalence, so **the wrap amount must be an integer multiple of a full turn**, but ``2 × 180000 = 360000 mrad``
#: = 57.2958 turns (remainder 106.48°) ⇒ once |θ| crosses 180000 the heading **jumps by 106.48°**.
#: The current ``3142`` gives a wrap of ``2 × 3142 = 6284 mrad`` ≈ 2π rad, introducing only
#: a quantization error of ``6284 - 6283.185 = 0.815 mrad`` ≈ 0.047° per wrap.
ODOM_HALF_TURN_MRAD = 3142

#: Legacy value, used for **diagnostics** only: if ``pose.theta_mrad`` happens to equal the expectation for "normalized by 180000",
#: it means the ``.so`` was built from the old ``odom.c`` (macro not synced). It never affects pass/fail.
_LEGACY_HALF_TURN_MRAD = 180000


def _expected_theta(total_mrad: int, half_turn: int) -> int:
    """The unique representative of ``total_mrad`` in ``(-half_turn, half_turn]``.

    This is **pure modular arithmetic**, not a "second odometry algorithm": it only answers "which angle differs from total by an integer number of turns
    and falls in the canonical interval"; it touches no geometry/integration. It is used to **compute** the expected value for the assertion
    instead of relying on hand calculation (contract §5.1 criterion 1).
    """
    full = 2 * half_turn
    remainder = (total_mrad + half_turn) % full  # map into [0, full)
    if remainder <= 0:
        remainder += full
    return remainder - half_turn


def _resolve_half_turn_mrad(core: "kinematics.Core") -> Tuple[int, str]:
    """Use the value from the ``.so`` when readable, otherwise the constant in this file.

    Return ``(value, source description)``. The source is printed in the check detail so it is obvious whether this run was
    "verified against C's own macro" or "verified against the Python-side copy".
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
    return ODOM_HALF_TURN_MRAD, "constant ODOM_HALF_TURN_MRAD (source odom.h; no accessor in the .so)"


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
        print("%d checks, %d failed" % (len(self.rows), len(failed)))
        return EXIT_OK if not failed else EXIT_CHECK_FAILED


def _twist(vx: int = 0, vy: int = 0, omega: int = 0) -> kinematics.MecanumTwist:
    return kinematics.MecanumTwist(vx_mm_s=vx, vy_mm_s=vy, omega_mrad_s=omega)


def _drive(
    core: kinematics.Core,
    geom: kinematics.MecanumGeom,
    twist: kinematics.MecanumTwist,
    frames: int,
) -> Tuple[kinematics.OdomPose, kinematics.OdomTotal, Optional[kinematics.MecanumTwist]]:
    """Turn one twist into four-wheel displacements through the **C inverse solution** and feed ``frames`` accumulated frames into the odometry.

    ★ Not a single line of Python math: the wheel values come from ``mecanum_inverse()``,
    the pose comes from ``odom_update()``.
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
            _REJECTIONS.append("frame %d (of %d)" % (index + 1, frames))
        d_prev = list(d_now)
    return pose, total, last


def main(argv: Optional[Sequence[str]] = None) -> int:
    print("Candidate paths (contract §5.3 order):")
    for index, path in enumerate(kinematics.candidate_paths(), start=1):
        print("  %d. %s%s" % (index, path, "  <- exists" if os.path.isfile(path) else ""))

    try:
        core = kinematics.Core()
    except kinematics.CoreLibraryNotFound as exc:
        print("\nCannot find %s:\n%s" % (kinematics.SONAME, exc))
        return EXIT_NO_LIBRARY
    except kinematics.CoreSymbolMissing as exc:
        print("\n%s" % exc)
        return EXIT_NO_SYMBOL
    except kinematics.CoreLibraryError as exc:
        print("\n%s" % exc)
        return EXIT_NO_LIBRARY

    print("\nLoaded: %s" % core.describe())
    checks = Checks()

    # ---- ABI: structure sizes ------------------------------------------------
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
            "actual %d bytes" % actual,
        )

    # ---- Geometry -----------------------------------------------------------
    geom = core.default_geom()
    checks.check(
        "mecanum_default_geom() returns valid geometry",
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

    # ---- Inverse -> forward round trip ----------------------------------------------
    for vx, vy, omega in ((200, 0, 0), (0, 150, 0), (0, 0, 300), (-200, 120, -300)):
        wheels = core.inverse(geom, _twist(vx, vy, omega))
        back = core.forward(geom, [int(wheels.wheel_mm_s[i]) for i in range(4)])
        ok = (
            abs(back.vx_mm_s - vx) <= 3
            and abs(back.vy_mm_s - vy) <= 3
            and abs(back.omega_mrad_s - omega) <= 3
        )
        checks.check(
            "inverse->forward round trip (%d,%d,%d)" % (vx, vy, omega),
            ok,
            "wheels=%s -> (%d,%d,%d)"
            % (
                [int(wheels.wheel_mm_s[i]) for i in range(4)],
                back.vx_mm_s,
                back.vy_mm_s,
                back.omega_mrad_s,
            ),
        )

    # ---- Inverse -> odometry consistency (single frame) ---------------------------------
    for vx, vy, omega, label in (
        (200, 0, 0, "straight ahead"),
        (0, 150, 0, "pure left/right strafe"),
        (0, 0, 300, "rotate in place"),
    ):
        pose, total, twist = _drive(core, geom, _twist(vx, vy, omega), 1)
        ok = (
            abs(pose.x_mm - vx) <= 3
            and abs(pose.y_mm - vy) <= 3
            and abs(pose.theta_mrad - omega) <= 3
        )
        checks.check(
            "single-frame odometry restores %s (%d,%d,%d)" % (label, vx, vy, omega),
            ok,
            "pose=(%d mm, %d mm, %d mrad) twist=%s"
            % (
                pose.x_mm,
                pose.y_mm,
                pose.theta_mrad,
                None if twist is None else (twist.vx_mm_s, twist.vy_mm_s, twist.omega_mrad_s),
            ),
        )

    # ---- 100 frames straight: position should accumulate linearly ------------------------------------
    pose, total, _ = _drive(core, geom, _twist(200, 0, 0), 100)
    checks.check(
        "100 frames at 200 mm/s straight -> x ≈ 20000 mm",
        abs(pose.x_mm - 20000) <= 60,
        "x=%d mm y=%d mm theta=%d mrad" % (pose.x_mm, pose.y_mm, pose.theta_mrad),
    )
    checks.check(
        "straight driving produces no lateral phantom drift",
        abs(pose.y_mm) <= 5,
        "y=%d mm" % pose.y_mm,
    )

    # ---- 100 frames rotating in place: the key arc-integration test --------------------------
    # Euler integration is easiest to get wrong here: rotation in place must produce no translation.
    pose, total, _ = _drive(core, geom, _twist(0, 0, 300), 100)
    checks.check(
        "100 frames rotating in place -> theta_total ≈ 30000 mrad",
        abs(total.theta_mrad - 30000) <= 300,
        "theta_total=%d mrad" % total.theta_mrad,
    )
    checks.check(
        "rotation in place produces no phantom translation (core arc-integration test)",
        abs(pose.x_mm) <= 5 and abs(pose.y_mm) <= 5,
        "x=%d mm y=%d mm (Euler integration typically drifts hundreds of mm here)" % (pose.x_mm, pose.y_mm),
    )
    # ---- ★★ Heading normalization: contract §5.1 acceptance (cumulative >= 2 full turns)-----------
    #
    # The 100-frame rotation above accumulated theta_total ≈ 30000 mrad; at the new constant of 6284 mrad/turn that is
    # about 4.77 turns (>= 2 full turns), just enough to verify normalization. Two criteria:
    #   (1) pose.theta_mrad falls in (-HALF, HALF];
    #   (2) (pose.theta_mrad - total.theta_mrad) is a non-zero integer multiple of 2*HALF.
    # ★ (2) is the **proof** that "normalization preserves angular equivalence": it only holds when HALF's wrap amount is truly a full turn.
    #   The old value 180000 passes (1) (the interval is large enough that an unchanged value trivially "falls inside") but fails (2) —
    #   e.g. total=200000 → pose=-160000, difference 360000, and 360000 % 6284 = 1744 != 0.
    half_turn, half_turn_src = _resolve_half_turn_mrad(core)
    full_turn = 2 * half_turn
    total_theta = int(total.theta_mrad)
    pose_theta = int(pose.theta_mrad)
    expected_theta = _expected_theta(total_theta, half_turn)
    delta_theta = pose_theta - total_theta
    turns = delta_theta // full_turn

    # First guard against the situation where this rotation is under 2 turns and the two criteria above are meaningless,
    # so that a future reduction of the rotation amount does not **silently make them always pass**.
    checks.check(
        "this rotation accumulated >= 2 full turns (otherwise the two criteria below are meaningless)",
        abs(total_theta) >= 2 * full_turn,
        "accumulated %d mrad = %.2f turns; 2 full turns = %d mrad (half-turn constant %d, source %s)"
        % (total_theta, total_theta / full_turn, 2 * full_turn, half_turn, half_turn_src),
    )

    legacy_expected = _expected_theta(total_theta, _LEGACY_HALF_TURN_MRAD)
    stale_hint = ""
    if pose_theta == legacy_expected and legacy_expected != expected_theta:
        stale_hint = (
            "; ★★ pose equals exactly the expectation for 'normalized by the legacy constant 180000' %d"
            " ⇒ the .so was built from the old odom.c (ODOM_HALF_TURN_MRAD in firmware_core/src/odom.c has not been synced to 3142;"
            "no line numbers, they drift)"
            % legacy_expected
        )

    checks.check(
        "after >= 2 full turns, pose.theta_mrad falls in (-%d, %d] and equals the computed expectation" % (half_turn, half_turn),
        -half_turn < pose_theta <= half_turn and pose_theta == expected_theta,
        "pose.theta=%d mrad; normalizing by %d should give %d mrad (accumulated %d mrad, difference %d mrad)%s"
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
        "normalization preserves angular equivalence: the difference is a non-zero integer multiple of %d mrad" % full_turn,
        delta_theta % full_turn == 0 and delta_theta != 0,
        "pose.theta - total.theta = %d mrad = %d turns (must be non-zero; expected pose=%d mrad)"
        % (delta_theta, turns, expected_theta),
    )

    # ---- Zero displacement: the pose must not move at all --------------------------------------
    pose_a, total_a = core.odom_reset()
    zero = [0] * kinematics.WHEEL_COUNT
    core.odom_update(pose_a, total_a, geom, zero, zero)
    checks.check(
        "pose stays unchanged when Δd == 0",
        pose_a.x_mm == 0 and pose_a.y_mm == 0 and pose_a.theta_mrad == 0,
        "pose=(%d, %d, %d)" % (pose_a.x_mm, pose_a.y_mm, pose_a.theta_mrad),
    )

    # ---- Did C reject any frames? ----------------------------------------------
    checks.check(
        "odom_update() never returned 0 (geometry validity)",
        not _REJECTIONS,
        "%d rejections: %s" % (len(_REJECTIONS), _REJECTIONS[0]) if _REJECTIONS else "0 rejections",
    )

    return checks.report()


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
