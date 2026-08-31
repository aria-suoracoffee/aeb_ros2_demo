"""Unit tests for the Kalman-filter fusion math (no ROS runtime needed)."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from aeb_demo.fusion import (  # noqa: E402
    RangeRateKF,
    full_measurement,
    range_measurement,
    rate_measurement,
)


def make_kf():
    kf = RangeRateKF(sigma_a=3.0, r_var_init=4.0, c_var_init=100.0)
    kf.init(60.0, 0.0)
    return kf


def test_predict_moves_range_by_closing_speed():
    kf = make_kf()
    kf.x[1] = 10.0            # closing at 10 m/s
    kf.predict(0.5)
    assert kf.range == 55.0   # 60 - 10 * 0.5
    assert kf.closing_speed == 10.0


def test_predict_grows_uncertainty():
    kf = make_kf()
    before = np.trace(kf.P)
    kf.predict(0.1)
    assert np.trace(kf.P) > before


def test_constant_velocity_model_holds_closing_speed():
    kf = make_kf()
    kf.x[1] = 14.0
    for _ in range(50):
        kf.predict(0.02)                 # no control input
    assert abs(kf.closing_speed - 14.0) < 0.1


def test_control_input_tracks_ego_braking():
    kf = make_kf()
    kf.x[1] = 14.0
    for _ in range(50):                   # ego brakes at 8 m/s^2 for 1 s
        kf.predict(0.02, a_ego=-8.0)
    assert 5.0 < kf.closing_speed < 7.0   # dropped ~8 m/s, not held at 14
    assert kf.range >= 0.0


def test_range_only_updates_pull_range_and_shrink_variance():
    kf = make_kf()
    var_before = kf.P[0, 0]
    for _ in range(20):
        kf.update(*range_measurement(50.0, 0.02))
    assert abs(kf.range - 50.0) < 0.5
    assert kf.P[0, 0] < var_before


def test_rate_only_updates_converge_closing_speed():
    kf = make_kf()
    for _ in range(30):
        kf.update(*rate_measurement(12.0, 0.02))
    assert abs(kf.closing_speed - 12.0) < 0.5


def test_mahalanobis_gate_rejects_outlier():
    kf = make_kf()                       # state range = 60, P[0,0] = 4
    z, H, R = range_measurement(120.0, 0.36)   # ~100 sigma away
    accepted = kf.update(z, H, R, gate2=9.21)
    assert accepted is False
    assert kf.range == 60.0              # state untouched

    z, H, R = range_measurement(60.5, 0.36)    # plausible
    assert kf.update(z, H, R, gate2=9.21) is True


def test_full_measurement_uses_both_components():
    kf = make_kf()
    for _ in range(15):
        kf.update(*full_measurement(40.0, 0.36, 9.0, 0.02))
    assert abs(kf.range - 40.0) < 1.0
    assert abs(kf.closing_speed - 9.0) < 0.5


def test_complementary_weighting():
    """Lidar range (tight) + radar rate (tight) should both win over the other
    sensor's loose estimate of the same quantity."""
    kf = make_kf()
    for _ in range(40):
        kf.update(*range_measurement(30.0, 0.02))          # lidar: range, tight
        kf.update(*rate_measurement(15.0, 0.02))           # radar: rate, tight
        kf.update(*range_measurement(34.0, 0.36))          # radar: range, loose
        kf.update(*rate_measurement(11.0, 4.0))            # lidar: rate, loose
    assert abs(kf.range - 30.0) < 1.0                      # follows the tight range
    assert abs(kf.closing_speed - 15.0) < 1.0             # follows the tight rate
