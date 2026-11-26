"""Wheel joint angle integration for the RViz2 wheel animation (display only).

Why this module exists: the Gazebo ``planar_move`` plug-in moves the whole model
kinematically, so the four wheel joints never turn and RViz2 always shows frozen
wheels. This helper turns the commanded chassis twist into four joint angles, so the
model looks alive in RViz2 while the rest of the stack keeps working exactly as before.

Iron rule (same as everywhere else in this package): the wheel linear speeds come from
``mecanum_inverse()`` in ``libomninav_core.so`` -- this module does **no mecanum math of
its own**. Its only arithmetic is the trivial rolling relation ``omega_joint =
wheel_mm_s / 1000 / wheel_radius`` and the integral of that omega over time, both of
which live in the URDF world (the URDF wheel radius is the single source for them).

Display only: it never publishes ``/odom``, never feeds the serial bridge, and is not
part of the frozen ROS interface (no new frame, no new control topic).
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence

from stm32_bridge import kinematics

__all__ = ["WHEEL_COUNT", "WHEEL_JOINTS", "WheelAngleIntegrator"]

#: Number of wheels (FL, FR, RL, RR) -- the order frozen by ``mecanum.h``.
WHEEL_COUNT = kinematics.WHEEL_COUNT

#: URDF joint names, **in the same FL/FR/RL/RR order** as ``mecanum_inverse()``.
WHEEL_JOINTS = (
    "wheel_fl_joint",
    "wheel_fr_joint",
    "wheel_rl_joint",
    "wheel_rr_joint",
)

#: Default wheel radius, in metres (CAD R = 40.376 mm; must match ``wheel_r`` in the URDF).
DEFAULT_WHEEL_RADIUS_M = 0.040376


class WheelAngleIntegrator:
    """Integrate a chassis twist into four wheel-joint angles (radians) and rates (rad/s)."""

    def __init__(
        self,
        wheel_radius_m: float = DEFAULT_WHEEL_RADIUS_M,
        core: Optional[kinematics.Core] = None,
        geom: Optional[kinematics.MecanumGeom] = None,
    ) -> None:
        if not wheel_radius_m > 0.0:
            raise ValueError("wheel_radius_m must be > 0, got %r" % (wheel_radius_m,))
        self._core = core if core is not None else kinematics.default_core()
        self._geom = geom if geom is not None else self._core.default_geom()
        if not self._core.geom_is_valid(self._geom):
            raise ValueError("invalid mecanum geometry from the core library; refusing to animate")
        self._radius_m = float(wheel_radius_m)
        self._angles = [0.0] * WHEEL_COUNT
        self._rates = [0.0] * WHEEL_COUNT

    # -- state -------------------------------------------------------------

    def reset(self) -> None:
        """Return the wheels to angle 0 / rate 0 (e.g. after an odometry reset)."""
        self._angles = [0.0] * WHEEL_COUNT
        self._rates = [0.0] * WHEEL_COUNT

    @property
    def angles(self) -> List[float]:
        """Current joint angles, radians, FL/FR/RL/RR."""
        return list(self._angles)

    @property
    def rates(self) -> List[float]:
        """Current joint rates, rad/s, FL/FR/RL/RR."""
        return list(self._rates)

    @property
    def wheel_radius_m(self) -> float:
        return self._radius_m

    @property
    def core_description(self) -> str:
        """Human-readable core-library identity (path + struct sizes), for start-up logs."""
        return self._core.describe()

    @property
    def l_mm(self) -> int:
        """``L = half_wheelbase_mm + half_track_mm`` as computed by the C core (diagnostics)."""
        return self._core.l_mm(self._geom)

    # -- kinematics --------------------------------------------------------

    def wheel_speeds_mm_s(
        self, vx_mm_s: int, vy_mm_s: int, omega_mrad_s: int
    ) -> List[int]:
        """Body twist -> the four wheel linear speeds (mm/s), straight from the C core."""
        twist = kinematics.MecanumTwist(
            vx_mm_s=int(vx_mm_s),
            vy_mm_s=int(vy_mm_s),
            omega_mrad_s=int(omega_mrad_s),
        )
        wheels = self._core.inverse(self._geom, twist)
        return [int(wheels.wheel_mm_s[index]) for index in range(WHEEL_COUNT)]

    def step(
        self, vx_mm_s: int, vy_mm_s: int, omega_mrad_s: int, dt_s: float
    ) -> List[float]:
        """Advance the joint angles by ``dt_s`` for the given body twist.

        :returns: the new four joint angles (radians, FL/FR/RL/RR).
        """
        dt = float(dt_s)
        if not math.isfinite(dt) or dt < 0.0:
            dt = 0.0
        speeds = self.wheel_speeds_mm_s(vx_mm_s, vy_mm_s, omega_mrad_s)
        # omega_joint = v_wheel / R: rolling without slipping in the URDF frame. The wheel
        # joints are ``continuous`` about +y, and a positive rotation about +y rolls the
        # wheel forward (+x), which is exactly what a positive wheel speed means.
        self._rates = [speed / 1000.0 / self._radius_m for speed in speeds]
        if dt > 0.0:
            for index in range(WHEEL_COUNT):
                self._angles[index] += self._rates[index] * dt
        return list(self._angles)
