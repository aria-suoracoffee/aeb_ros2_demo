"""Unit tests for the pure AEB decision logic (no ROS runtime needed)."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from aeb_demo.safety import (  # noqa: E402
    BRAKE,
    CLEAR,
    WARN,
    AEBParams,
    decide,
    required_deceleration,
    time_to_collision,
)

P = AEBParams()


def test_ttc_not_closing_is_inf():
    assert time_to_collision(30.0, 0.0) == math.inf
    assert time_to_collision(30.0, -1.0) == math.inf


def test_ttc_basic():
    assert time_to_collision(20.0, 10.0) == 2.0


def test_required_decel_matches_kinematics():
    # 10 m/s, 10 m of usable margin -> v^2 / (2 d) = 100 / 20 = 5 m/s^2
    assert required_deceleration(12.0, 10.0, standoff=2.0) == 5.0


def test_required_decel_inside_standoff_is_inf():
    assert required_deceleration(1.0, 5.0, standoff=2.0) == math.inf


def test_clear_when_no_target():
    d = decide(CLEAR, valid=False, range_m=math.inf, closing_speed=0.0,
               ego_speed=14.0, p=P)
    assert d.state == CLEAR
    assert d.target_speed_override is None


def test_clear_when_far_and_slow_closing():
    d = decide(CLEAR, valid=True, range_m=70.0, closing_speed=1.0,
               ego_speed=14.0, p=P)
    assert d.state == CLEAR


def test_brake_on_low_ttc():
    # 12 m away, closing 12 m/s -> TTC 1.0 s < ttc_brake (1.2)
    d = decide(CLEAR, valid=True, range_m=12.0, closing_speed=12.0,
               ego_speed=12.0, p=P)
    assert d.state == BRAKE
    assert d.brake_request
    assert d.target_speed_override == 0.0


def test_warn_before_brake():
    # 60 m, closing 25 m/s: TTC 2.4 s (> ttc_warn), but required decel
    # ~5.4 m/s^2 sits between warn_decel_frac and brake_decel_frac of 8.
    d = decide(CLEAR, valid=True, range_m=60.0, closing_speed=25.0,
               ego_speed=25.0, p=P)
    assert d.state == WARN
    assert d.target_speed_override is None


def test_brake_latches_until_stopped():
    latched = decide(BRAKE, valid=False, range_m=math.inf, closing_speed=0.0,
                     ego_speed=3.0, p=P)
    assert latched.state == BRAKE

    released = decide(BRAKE, valid=False, range_m=math.inf, closing_speed=0.0,
                      ego_speed=0.1, p=P)
    assert released.state == CLEAR
