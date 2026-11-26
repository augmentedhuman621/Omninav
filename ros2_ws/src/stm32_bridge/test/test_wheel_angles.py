"""Unit tests for the display-only wheel-angle integrator (no ROS graph involved).

These tests need ``libomninav_core.so`` (the integrator must never reimplement mecanum
math). They are skipped, not failed, when the shared library is absent -- exactly like
``run_e2e.py`` does with return code 77.

Run with:

    OMNINAV_CORE_SO=/path/to/libomninav_core.so python3 -m pytest test/test_wheel_angles.py -q
"""

from __future__ import annotations

import math

import pytest

from stm32_bridge import kinematics
from stm32_bridge.wheel_angles import DEFAULT_WHEEL_RADIUS_M, WHEEL_JOINTS, WheelAngleIntegrator


def _make_integrator() -> WheelAngleIntegrator:
    try:
        return WheelAngleIntegrator()
    except kinematics.CoreLibraryNotFound:
        pytest.skip("libomninav_core.so not available; set OMNINAV_CORE_SO to run this test")


def test_joint_names_match_the_urdf_order() -> None:
    # Order is frozen by mecanum.h: FL, FR, RL, RR.
    assert list(WHEEL_JOINTS) == [
        "wheel_fl_joint",
        "wheel_fr_joint",
        "wheel_rl_joint",
        "wheel_rr_joint",
    ]


def test_invalid_wheel_radius_is_rejected() -> None:
    with pytest.raises(ValueError):
        WheelAngleIntegrator(wheel_radius_m=0.0)


def test_pure_forward_motion_spins_every_wheel_equally() -> None:
    integrator = _make_integrator()
    speeds = integrator.wheel_speeds_mm_s(vx_mm_s=200, vy_mm_s=0, omega_mrad_s=0)
    assert speeds == [200, 200, 200, 200]
    angles = integrator.step(vx_mm_s=200, vy_mm_s=0, omega_mrad_s=0, dt_s=0.1)
    expected = 0.2 / DEFAULT_WHEEL_RADIUS_M * 0.1
    for angle in angles:
        assert math.isclose(angle, expected, rel_tol=1e-9)


def test_lateral_motion_follows_the_mecanum_roller_pairing() -> None:
    integrator = _make_integrator()
    # S_VY = {-1, +1, +1, -1}: a positive vy drives FL/RR backwards and FR/RL forwards.
    speeds = integrator.wheel_speeds_mm_s(vx_mm_s=0, vy_mm_s=100, omega_mrad_s=0)
    assert speeds == [-100, 100, 100, -100]


def test_spin_in_place_follows_the_mecanum_spin_pairing() -> None:
    integrator = _make_integrator()
    # S_OMG = {-1, +1, -1, +1} and rot = omega_mrad_s * L / 1000 (mm/s).
    speeds = integrator.wheel_speeds_mm_s(vx_mm_s=0, vy_mm_s=0, omega_mrad_s=1000)
    magnitude = abs(speeds[0])
    assert magnitude > 0
    assert [1 if value > 0 else -1 for value in speeds] == [-1, 1, -1, 1]
    assert all(abs(abs(value) - magnitude) <= 1 for value in speeds)


def test_negative_time_step_does_not_rewind_or_crash() -> None:
    integrator = _make_integrator()
    integrator.step(vx_mm_s=100, vy_mm_s=0, omega_mrad_s=0, dt_s=0.2)
    before = integrator.angles
    integrator.step(vx_mm_s=100, vy_mm_s=0, omega_mrad_s=0, dt_s=-0.5)
    assert integrator.angles == before
    # The rates still reflect the last command, so the next positive step continues smoothly.
    assert all(rate > 0 for rate in integrator.rates)


def test_angles_are_linear_in_time_and_reset_clears_them() -> None:
    integrator = _make_integrator()
    integrator.step(vx_mm_s=150, vy_mm_s=0, omega_mrad_s=0, dt_s=0.1)
    integrator.step(vx_mm_s=150, vy_mm_s=0, omega_mrad_s=0, dt_s=0.3)
    split = integrator.angles

    one_shot = _make_integrator()
    one_shot.step(vx_mm_s=150, vy_mm_s=0, omega_mrad_s=0, dt_s=0.4)
    for angle_split, angle_shot in zip(split, one_shot.angles):
        assert math.isclose(angle_split, angle_shot, rel_tol=1e-9, abs_tol=1e-9)

    integrator.reset()
    assert integrator.angles == [0.0, 0.0, 0.0, 0.0]
    assert integrator.rates == [0.0, 0.0, 0.0, 0.0]


def test_stop_command_zeroes_the_rates() -> None:
    integrator = _make_integrator()
    integrator.step(vx_mm_s=300, vy_mm_s=-100, omega_mrad_s=200, dt_s=0.1)
    integrator.step(vx_mm_s=0, vy_mm_s=0, omega_mrad_s=0, dt_s=0.1)
    assert integrator.rates == [0.0, 0.0, 0.0, 0.0]
