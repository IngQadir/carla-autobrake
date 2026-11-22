"""Unit tests for aeb_logic.py - run with:  python -m pytest -v"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from aeb_logic import AebConfig, AebController, AebState, compute_ttc  # noqa: E402


# ---------------------------------------------------------------- compute_ttc

def test_ttc_static_obstacle():
    assert compute_ttc(20.0, 10.0, 0.0) == pytest.approx(2.0)


def test_ttc_moving_obstacle_uses_relative_speed():
    # 30 m gap, ego 20 m/s, lead car 10 m/s -> closing 10 m/s -> 3 s
    assert compute_ttc(30.0, 20.0, 10.0) == pytest.approx(3.0)


def test_ttc_no_obstacle_is_infinite():
    assert compute_ttc(None, 15.0) == math.inf


def test_ttc_not_closing_is_infinite():
    assert compute_ttc(10.0, 10.0, 10.0) == math.inf   # same speed
    assert compute_ttc(10.0, 5.0, 10.0) == math.inf    # lead car faster
    assert compute_ttc(10.0, 0.0, 0.0) == math.inf     # standing still


def test_ttc_zero_distance_is_zero():
    assert compute_ttc(0.0, 10.0) == 0.0


def test_ttc_zero_distance_at_standstill_is_infinite():
    # regression: an obstacle reported at 0 m while the car is stopped must not lock the brakes
    assert compute_ttc(0.0, 0.0) == math.inf


def test_standstill_with_obstacle_at_zero_distance_releases_brake():
    ctrl = AebController()
    assert ctrl.update(0.0, 0.0) == (AebState.NORMAL, 0.0)


# ---------------------------------------------------------------- stages

@pytest.mark.parametrize("ttc, expected_state, expected_brake", [
    (5.0, AebState.NORMAL, 0.0),
    (2.0, AebState.FCW, 0.0),
    (1.2, AebState.PARTIAL_BRAKE, 0.4),
    (0.5, AebState.FULL_BRAKE, 1.0),
])
def test_stage_for_each_ttc_band(ttc, expected_state, expected_brake):
    ctrl = AebController()
    ego_speed = 10.0
    state, brake = ctrl.update(distance=ttc * ego_speed, ego_speed=ego_speed)
    assert state == expected_state
    assert brake == pytest.approx(expected_brake)


def test_no_obstacle_stays_normal():
    ctrl = AebController()
    assert ctrl.update(None, 30.0) == (AebState.NORMAL, 0.0)


# ---------------------------------------------------------------- state machine behaviour

def test_escalates_through_all_stages_while_approaching():
    ctrl = AebController()
    states = [ctrl.update(d, 10.0)[0] for d in (40, 20, 12, 5)]
    assert states == [AebState.NORMAL, AebState.FCW,
                      AebState.PARTIAL_BRAKE, AebState.FULL_BRAKE]


def test_jumps_straight_to_full_brake_if_obstacle_appears_late():
    ctrl = AebController()
    state, brake = ctrl.update(3.0, 10.0)        # TTC 0.3 s, no warning before
    assert state == AebState.FULL_BRAKE and brake == 1.0


def test_hysteresis_prevents_flicker_at_threshold():
    ctrl = AebController()
    ctrl.update(25.0, 10.0)                      # TTC 2.5 s -> FCW
    assert ctrl.state == AebState.FCW
    ctrl.update(27.0, 10.0)                      # TTC 2.7 s: above 2.6 but below 2.6 + 0.3
    assert ctrl.state == AebState.FCW            # still warning
    ctrl.update(30.0, 10.0)                      # TTC 3.0 s: clearly safe
    assert ctrl.state == AebState.NORMAL


def test_full_brake_is_held_until_standstill():
    ctrl = AebController()
    ctrl.update(5.0, 10.0)                       # FULL_BRAKE
    # obstacle disappears (e.g. pedestrian walked away) but car is still moving
    state, brake = ctrl.update(None, 4.0)
    assert state == AebState.FULL_BRAKE and brake == 1.0
    # car has stopped -> released
    state, brake = ctrl.update(None, 0.0)
    assert state == AebState.NORMAL and brake == 0.0


def test_custom_config_thresholds():
    cfg = AebConfig(ttc_fcw=4.0, ttc_partial=2.0, ttc_full=1.0, partial_brake=0.5)
    ctrl = AebController(cfg)
    state, brake = ctrl.update(15.0, 10.0)       # TTC 1.5 s
    assert state == AebState.PARTIAL_BRAKE and brake == 0.5
