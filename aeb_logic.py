"""
aeb_logic.py - Forward Collision Warning (FCW) + Automatic Emergency Braking (AEB)

Pure Python, no CARLA dependency -> can be unit-tested anywhere.

Stages (escalation by Time-to-Collision, TTC):
    NORMAL         TTC >= 2.6 s   no action
    FCW            TTC <  2.6 s   warn the driver
    PARTIAL_BRAKE  TTC <  1.6 s   40 % brake
    FULL_BRAKE     TTC <  0.8 s   100 % brake, held until the car stops
"""

from dataclasses import dataclass
from enum import IntEnum
import math


class AebState(IntEnum):
    NORMAL = 0
    FCW = 1
    PARTIAL_BRAKE = 2
    FULL_BRAKE = 3


@dataclass(frozen=True)
class AebConfig:
    ttc_fcw: float = 2.6            # s  - warning threshold
    ttc_partial: float = 1.6        # s  - partial braking threshold
    ttc_full: float = 0.8           # s  - full braking threshold
    partial_brake: float = 0.4      # brake command in PARTIAL_BRAKE (0..1)
    hysteresis: float = 0.3         # s  - TTC must rise this much above a threshold to step down
    min_closing_speed: float = 0.1  # m/s - below this we are not approaching
    standstill_speed: float = 0.1   # m/s - FULL_BRAKE is released only below this speed


def compute_ttc(distance, ego_speed, object_speed=0.0, min_closing_speed=0.1):
    """Time-to-Collision in seconds.

    distance      gap to the object ahead [m], None if nothing detected
    ego_speed     own speed [m/s]
    object_speed  speed of the object ahead in the same direction [m/s]

    Returns math.inf if nothing is detected or we are not closing in.
    """
    if distance is None:
        return math.inf
    closing_speed = ego_speed - object_speed
    if closing_speed <= min_closing_speed:
        return math.inf          # not approaching -> no collision risk (also at standstill)
    if distance <= 0.0:
        return 0.0
    return distance / closing_speed


class AebController:
    """State machine: escalates immediately, de-escalates only with hysteresis."""

    def __init__(self, config=None):
        self.cfg = config or AebConfig()
        self.state = AebState.NORMAL
        self.ttc = math.inf

    def _state_for_ttc(self, ttc, margin=0.0):
        c = self.cfg
        if ttc < c.ttc_full + margin:
            return AebState.FULL_BRAKE
        if ttc < c.ttc_partial + margin:
            return AebState.PARTIAL_BRAKE
        if ttc < c.ttc_fcw + margin:
            return AebState.FCW
        return AebState.NORMAL

    def update(self, distance, ego_speed, object_speed=0.0):
        """Run one cycle. Returns (state, brake_command 0..1)."""
        c = self.cfg
        self.ttc = compute_ttc(distance, ego_speed, object_speed, c.min_closing_speed)

        target = self._state_for_ttc(self.ttc)
        if target > self.state:
            # Escalate immediately - safety first
            self.state = target
        elif self.state == AebState.FULL_BRAKE:
            # Full braking is held until the vehicle is at standstill
            if ego_speed < c.standstill_speed:
                self.state = self._state_for_ttc(self.ttc, c.hysteresis)
        else:
            # Step down only when TTC is clearly above the threshold (no flickering)
            self.state = min(self.state, self._state_for_ttc(self.ttc, c.hysteresis))

        return self.state, self.brake_command()

    def brake_command(self):
        if self.state == AebState.FULL_BRAKE:
            return 1.0
        if self.state == AebState.PARTIAL_BRAKE:
            return self.cfg.partial_brake
        return 0.0
