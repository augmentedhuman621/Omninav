"""stm32_bridge.kinematics — loads ``libomninav_core.so`` built from ``firmware_core`` through ``ctypes``.

★★★ Iron rule (end of ``11_interface_contract.md`` §3 / §5.3 + the T2 task card "soft dependency")

    Mecanum forward/inverse kinematics and arc-integration odometry may have **exactly one implementation**, in the C code of ``firmware_core``
    (``mecanum.c`` / ``odom.c``). This file **does no math at all**; it only wraps C symbols as Python functions.
    When the ``.so`` cannot be found it **must raise a clear exception**, and must **never** reimplement the math in Python —
    that would be a second implementation, bound to diverge from the C version eventually, and no test would warn on the day it diverges.

``.so`` lookup order (contract §5.3, **frozen**; the first existing path wins):

    1. Environment variable ``OMNINAV_CORE_SO`` (handy for pointing elsewhere while developing on a laptop / WSL)
    2. ``<package dir>/lib/libomninav_core.so``
    3. ``/opt/omninav/lib/libomninav_core.so``   <- installed here by ``pi_setup/install_core.sh``

If none exist -> :class:`CoreLibraryNotFound`; if the ``.so`` exists but lacks symbols -> :class:`CoreSymbolMissing`.
Both are **explicit errors, never a silent fallback**.
"""

from __future__ import annotations

import ctypes
import os
from typing import List, Optional, Sequence, Tuple

__all__ = [
    "WHEEL_COUNT",
    "SONAME",
    "CORE_SO_ENV",
    "SYSTEM_CORE_SO",
    "REQUIRED_SYMBOLS",
    "CoreLibraryError",
    "CoreLibraryNotFound",
    "CoreSymbolMissing",
    "MecanumGeom",
    "MecanumTwist",
    "MecanumWheels",
    "OdomPose",
    "OdomTotal",
    "candidate_paths",
    "find_library",
    "Core",
    "default_core",
    "reset_default_core",
]

#: Number of wheels (FL, FR, RL, RR).
WHEEL_COUNT = 4

#: Artifact name frozen by contract §5.2.
SONAME = "libomninav_core.so"

#: Contract §5.3 item 1: override with an environment variable during development.
CORE_SO_ENV = "OMNINAV_CORE_SO"

#: Contract §5.3 item 3: where ``pi_setup/install_core.sh`` installs it.
SYSTEM_CORE_SO = "/opt/omninav/lib/" + SONAME

#: The "must export" symbols frozen by contract §5.2.
REQUIRED_SYMBOLS: Tuple[str, ...] = (
    "mecanum_forward",
    "mecanum_inverse",
    "mecanum_l_mm",
    "mecanum_default_geom",
    "mecanum_geom_is_valid",
    "odom_update",
    "odom_reset",
)

#: ``<package dir>/lib/<SONAME>`` (contract §5.3 item 2).
PACKAGE_LIB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib")


class CoreLibraryError(RuntimeError):
    """Any ``libomninav_core.so``-related error — always propagated, never swallowed."""


class CoreLibraryNotFound(CoreLibraryError):
    """None of the three candidate paths exists."""


class CoreSymbolMissing(CoreLibraryError):
    """The ``.so`` loaded, but not all contract-required symbols are exported."""


# --------------------------------------------------------------------------
# ctypes structs (field order / types must match the firmware_core headers verbatim)
# --------------------------------------------------------------------------


class MecanumGeom(ctypes.Structure):
    """Mirrors C ``mecanum_geom_t`` (``firmware_core/include/mecanum.h``)."""

    _fields_ = [
        ("half_wheelbase_mm", ctypes.c_int32),
        ("half_track_mm", ctypes.c_int32),
        ("vy_polarity", ctypes.c_int8),
        ("omega_polarity", ctypes.c_int8),
    ]


class MecanumTwist(ctypes.Structure):
    """Mirrors C ``mecanum_twist_t``. Units mm/s, mrad/s."""

    _fields_ = [
        ("vx_mm_s", ctypes.c_int32),
        ("vy_mm_s", ctypes.c_int32),
        ("omega_mrad_s", ctypes.c_int32),
    ]


class MecanumWheels(ctypes.Structure):
    """Mirrors C ``mecanum_wheels_t``: four wheel linear velocities, order FL, FR, RL, RR."""

    _fields_ = [("wheel_mm_s", ctypes.c_int32 * WHEEL_COUNT)]


class OdomPose(ctypes.Structure):
    """Mirrors C ``odom_pose_t``: ``{int64_t x_mm, int64_t y_mm, int32_t theta_mrad}``.

    ★ Note the trailing padding: with natural alignment the C side is **24 bytes** (4 bytes of tail padding),
    not 20. ``ctypes.Structure`` pads according to the same ABI rules, so declaring the fields as-is is correct;
    but elsewhere **do not** assume ``sizeof(OdomPose) == 20``.
    """

    _fields_ = [
        ("x_mm", ctypes.c_int64),
        ("y_mm", ctypes.c_int64),
        ("theta_mrad", ctypes.c_int32),
    ]


class OdomTotal(ctypes.Structure):
    """Mirrors C ``odom_total_t``: accumulated quantities without angle normalization."""

    _fields_ = [
        ("x_mm", ctypes.c_int64),
        ("y_mm", ctypes.c_int64),
        ("theta_mrad", ctypes.c_int64),
    ]


# --------------------------------------------------------------------------
# Library lookup
# --------------------------------------------------------------------------


def _normalize_path(raw: str) -> str:
    """Expand ``~`` and ``$VAR``, then return an absolute path.

    ★ The contract §5.3 error hint tells people to write ``OMNINAV_CORE_SO=~/omninav/lib/libomninav_core.so``,
      but ``os.path.isfile()`` **does not understand** ``~`` (it is just an ordinary character). Without expansion,
      following the hint would actually produce "cannot find the .so".
    """
    return os.path.abspath(os.path.expanduser(os.path.expandvars(raw)))


def candidate_paths() -> List[str]:
    """Return the candidate paths in contract §5.3 order (item 1 expands ~ / $VAR)."""
    paths: List[str] = []
    override = os.environ.get(CORE_SO_ENV, "").strip()
    if override:
        paths.append(_normalize_path(override))
    paths.append(os.path.join(PACKAGE_LIB_DIR, SONAME))
    paths.append(SYSTEM_CORE_SO)
    return paths


def find_library() -> str:
    """Return the first existing candidate path; raise :class:`CoreLibraryNotFound` if none exists."""
    tried = candidate_paths()
    for path in tried:
        if os.path.isfile(path):
            return path
    raise CoreLibraryNotFound(
        "Cannot find %s. Looked at these paths in contract §5.3 order:\n  %s\n"
        "How to fix:\n"
        "  * Raspberry Pi: let pi_setup/install_core.sh install the .so to %s\n"
        "  * Laptop / WSL development: %s=/absolute/path/%s\n"
        "  * Or build the .so first by running `make lib` in firmware_core/"
        % (SONAME, "\n  ".join(tried), SYSTEM_CORE_SO, CORE_SO_ENV, SONAME)
    )


class Core:
    """Python facade for ``libomninav_core.so``. **Contains no math**."""

    def __init__(self, path: Optional[str] = None) -> None:
        self.path = path or find_library()
        try:
            self._lib = ctypes.CDLL(self.path)
        except OSError as exc:
            raise CoreLibraryError(
                "ctypes failed to load %s: %s\n"
                "Common causes: architecture mismatch (an x86_64 .so cannot be used by the Raspberry Pi aarch64;"
                "on the Pi you must run make lib locally on the Pi)" % (self.path, exc)
            ) from exc
        self._bind()

    # -- Symbol binding ----------------------------------------------------

    def _bind(self) -> None:
        missing = [name for name in REQUIRED_SYMBOLS if not hasattr(self._lib, name)]
        if missing:
            raise CoreSymbolMissing(
                "%s is missing symbols required by contract §5.2: %s\n"
                "(``make lib`` must be built with -fPIC -shared, and do not make the symbols static)"
                % (self.path, ", ".join(missing))
            )

        lib = self._lib

        lib.mecanum_default_geom.restype = MecanumGeom
        lib.mecanum_default_geom.argtypes = []

        lib.mecanum_l_mm.restype = ctypes.c_int32
        lib.mecanum_l_mm.argtypes = [ctypes.POINTER(MecanumGeom)]

        lib.mecanum_geom_is_valid.restype = ctypes.c_int
        lib.mecanum_geom_is_valid.argtypes = [ctypes.POINTER(MecanumGeom)]

        lib.mecanum_forward.restype = None
        lib.mecanum_forward.argtypes = [
            ctypes.POINTER(MecanumGeom),
            ctypes.POINTER(MecanumWheels),
            ctypes.POINTER(MecanumTwist),
        ]

        lib.mecanum_inverse.restype = None
        lib.mecanum_inverse.argtypes = [
            ctypes.POINTER(MecanumGeom),
            ctypes.POINTER(MecanumTwist),
            ctypes.POINTER(MecanumWheels),
        ]

        lib.odom_reset.restype = None
        lib.odom_reset.argtypes = [ctypes.POINTER(OdomPose), ctypes.POINTER(OdomTotal)]

        lib.odom_update.restype = ctypes.c_int
        lib.odom_update.argtypes = [
            ctypes.POINTER(OdomPose),
            ctypes.POINTER(OdomTotal),
            ctypes.POINTER(MecanumGeom),
            ctypes.POINTER(ctypes.c_int32),
            ctypes.POINTER(ctypes.c_int32),
            ctypes.POINTER(MecanumTwist),
        ]

    # -- High-level wrappers -----------------------------------------------

    def default_geom(self) -> MecanumGeom:
        """C-side default geometry (**the single source of geometry and calibration parameters**, contract §6)."""
        return self._lib.mecanum_default_geom()

    def make_geom(
        self,
        half_wheelbase_mm: int,
        half_track_mm: int,
        vy_polarity: int,
        omega_polarity: int,
    ) -> MecanumGeom:
        geom = MecanumGeom(
            half_wheelbase_mm=int(half_wheelbase_mm),
            half_track_mm=int(half_track_mm),
            vy_polarity=int(vy_polarity),
            omega_polarity=int(omega_polarity),
        )
        if not self.geom_is_valid(geom):
            raise ValueError(
                "Invalid geometry (mecanum_geom_is_valid returned 0):"
                "half_wheelbase_mm=%d half_track_mm=%d vy_polarity=%d omega_polarity=%d"
                % (half_wheelbase_mm, half_track_mm, vy_polarity, omega_polarity)
            )
        return geom

    def geom_is_valid(self, geom: MecanumGeom) -> bool:
        return bool(self._lib.mecanum_geom_is_valid(ctypes.byref(geom)))

    def l_mm(self, geom: MecanumGeom) -> int:
        """``L = half_wheelbase_mm + half_track_mm`` (computed by C, not recomputed in Python)."""
        return int(self._lib.mecanum_l_mm(ctypes.byref(geom)))

    def forward(self, geom: MecanumGeom, wheels_mm_s: Sequence[int]) -> MecanumTwist:
        """Forward kinematics: wheel linear velocities (FL,FR,RL,RR) -> body velocity."""
        wheels = MecanumWheels((ctypes.c_int32 * WHEEL_COUNT)(*_check_wheels(wheels_mm_s)))
        twist = MecanumTwist()
        self._lib.mecanum_forward(ctypes.byref(geom), ctypes.byref(wheels), ctypes.byref(twist))
        return twist

    def inverse(self, geom: MecanumGeom, twist: MecanumTwist) -> MecanumWheels:
        """Inverse kinematics: body velocity -> wheel linear velocities (FL,FR,RL,RR)."""
        wheels = MecanumWheels()
        self._lib.mecanum_inverse(ctypes.byref(geom), ctypes.byref(twist), ctypes.byref(wheels))
        return wheels

    def odom_reset(self) -> Tuple[OdomPose, OdomTotal]:
        """Reset pose and accumulated quantities; return a fresh pair of structs for the caller to own."""
        pose = OdomPose()
        total = OdomTotal()
        self._lib.odom_reset(ctypes.byref(pose), ctypes.byref(total))
        return pose, total

    def odom_update(
        self,
        pose: OdomPose,
        total: OdomTotal,
        geom: MecanumGeom,
        d_now: Sequence[int],
        d_prev: Sequence[int],
        twist_out: Optional[MecanumTwist] = None,
    ) -> Optional[MecanumTwist]:
        """Update the pose from the four-wheel **accumulated distance** increments (arc integration, computed in C).

        :returns: on success, the body velocity estimate for this step (mm/s, mrad/s);
          returns ``None`` when C returns 0 (invalid geometry / null pointer), with the **pose left unchanged**.
          ⇒ the caller must **not** advance ``d_prev`` in that case, otherwise odometry is silently lost.
        """
        now = (ctypes.c_int32 * WHEEL_COUNT)(*_check_wheels(d_now))
        prev = (ctypes.c_int32 * WHEEL_COUNT)(*_check_wheels(d_prev))
        twist = twist_out if twist_out is not None else MecanumTwist()
        ok = self._lib.odom_update(
            ctypes.byref(pose),
            ctypes.byref(total),
            ctypes.byref(geom),
            now,
            prev,
            ctypes.byref(twist),
        )
        return twist if ok else None

    # -- Diagnostics -------------------------------------------------------

    def describe(self) -> str:
        return "%s (OdomPose=%d B, OdomTotal=%d B)" % (
            self.path,
            ctypes.sizeof(OdomPose),
            ctypes.sizeof(OdomTotal),
        )


def _check_wheels(values: Sequence[int]) -> Tuple[int, int, int, int]:
    if len(values) != WHEEL_COUNT:
        raise ValueError("expected %d wheel values, got %d" % (WHEEL_COUNT, len(values)))
    out = []
    for index, value in enumerate(values):
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError("wheel[%d] must be an int, got %r" % (index, type(value).__name__))
        if not -(2 ** 31) <= value <= 2 ** 31 - 1:
            raise ValueError("wheel[%d]=%d is outside int32" % (index, value))
        out.append(value)
    return (out[0], out[1], out[2], out[3])


# --------------------------------------------------------------------------
# Module-level default instance (shared by the node and tool scripts to avoid repeated dlopen)
# --------------------------------------------------------------------------

_DEFAULT_CORE: Optional[Core] = None


def default_core() -> Core:
    """Lazily loaded default :class:`Core`. Raises :class:`CoreLibraryNotFound` when the ``.so`` is missing."""
    global _DEFAULT_CORE
    if _DEFAULT_CORE is None:
        _DEFAULT_CORE = Core()
    return _DEFAULT_CORE


def reset_default_core() -> None:
    """Drop the cached default instance (for unit tests / after changing ``OMNINAV_CORE_SO``)."""
    global _DEFAULT_CORE
    _DEFAULT_CORE = None
