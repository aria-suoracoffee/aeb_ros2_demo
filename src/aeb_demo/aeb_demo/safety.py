"""Pure AEB decision logic -- no ROS imports so it can be unit-tested.

The controller is deliberately simple and explainable. Two independent gates
can each request a brake (defense in depth):

  * Time-to-collision (TTC) gate:   brake if TTC < ttc_brake
  * Stopping-distance gate:          brake if the constant deceleration needed
                                     to stop within (range - standoff) metres
                                     exceeds a fraction of the vehicle's
                                     braking capability.

Two mechanisms keep the state stable:

  * Hysteresis -- leaving a more-severe state needs the signal to recover past
    a margin, so a noisy measurement sitting on the threshold does not chatter.
  * Brake-hold -- once braking has brought the vehicle to a stop, the brake is
    held while an obstacle is still close ahead (this is what a real "AEB stop
    and hold" does); it releases only when the obstacle clears.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields
from typing import Optional

CLEAR = 0
WARN = 1
BRAKE = 2

STATE_LABELS = {CLEAR: "CLEAR", WARN: "WARN", BRAKE: "BRAKE"}


@dataclass
class AEBParams:
    """Tunable thresholds. Mirrors the aeb_node ROS parameters."""

    max_brake_decel: float = 8.0     # m/s^2  vehicle braking capability
    ttc_warn: float = 2.0            # s      annunciate a warning below this TTC
    ttc_brake: float = 1.2           # s      command full braking below this TTC
    warn_decel_frac: float = 0.4     # -      fraction of max_brake_decel -> WARN
    brake_decel_frac: float = 0.7    # -      fraction of max_brake_decel -> BRAKE
    standoff: float = 2.0            # m      target stopping margin to obstacle
    stop_speed: float = 0.3          # m/s    "stopped" threshold
    min_closing_speed: float = 0.2   # m/s    ignore closing speed noise below this
    hold_distance: float = 15.0      # m      hold the brake at a standstill while
    #                                          an obstacle is within this range
    hysteresis: float = 1.25         # -      signal must recover by this factor
    #                                          before a state is downgraded

    def field_names(self):
        return [f.name for f in fields(self)]


def time_to_collision(range_m: float, closing_speed: float) -> float:
    """Seconds until impact assuming constant closing speed. inf if not closing."""
    if closing_speed <= 0.0 or math.isinf(range_m):
        return math.inf
    return max(range_m, 0.0) / closing_speed


def required_deceleration(range_m: float, closing_speed: float, standoff: float) -> float:
    """Constant deceleration (m/s^2) needed to stop `standoff` metres short.

    Derived from v^2 = 2 * a * d  ->  a = v^2 / (2 d).
    """
    if closing_speed <= 0.0:
        return 0.0
    margin = range_m - standoff
    if margin <= 0.0:
        return math.inf
    return (closing_speed ** 2) / (2.0 * margin)


@dataclass
class Decision:
    state: int
    brake_request: bool
    target_speed_override: Optional[float]  # None => pass the driver command through
    ttc: float
    required_decel: float


def decide(
    prev_state: int,
    valid: bool,
    range_m: float,
    closing_speed: float,
    ego_speed: float,
    p: AEBParams,
) -> Decision:
    """Run one control step of the AEB state machine."""
    closing = closing_speed if (valid and closing_speed > p.min_closing_speed) else 0.0
    ttc = time_to_collision(range_m, closing) if valid else math.inf
    req = required_deceleration(range_m, closing, p.standoff) if valid else 0.0

    stopped = ego_speed <= p.stop_speed
    obstacle_close = valid and range_m < p.hold_distance

    # --- brake latch / hold ---------------------------------------------------
    if prev_state == BRAKE:
        if not stopped:
            return Decision(BRAKE, True, 0.0, ttc, req)       # still braking to a stop
        if obstacle_close:
            return Decision(BRAKE, True, 0.0, ttc, req)       # stop-and-hold
        # obstacle has cleared -> fall through and release

    warn_decel = p.warn_decel_frac * p.max_brake_decel
    brake_decel = p.brake_decel_frac * p.max_brake_decel
    h = p.hysteresis

    # Entering a state uses the nominal threshold; staying in it (prev_state
    # already at/above that level) uses a relaxed threshold.
    if prev_state >= BRAKE:
        brake = (ttc < p.ttc_brake * h) or (req > brake_decel / h)
    else:
        brake = (ttc < p.ttc_brake) or (req > brake_decel)

    if prev_state >= WARN:
        warn = (ttc < p.ttc_warn * h) or (req > warn_decel / h)
    else:
        warn = (ttc < p.ttc_warn) or (req > warn_decel)

    if brake:
        return Decision(BRAKE, True, 0.0, ttc, req)
    if warn:
        return Decision(WARN, False, None, ttc, req)
    return Decision(CLEAR, False, None, ttc, req)
