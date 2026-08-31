"""Pure sensor-fusion math -- a 2-state Kalman filter, no ROS imports.

State:   x = [range, closing_speed]
         range decreases as the gap closes;  d(range)/dt = -closing_speed
Model:   constant closing speed; acceleration enters as process noise.

Measurements are supplied as (z, H, R):
  * lidar  -> accurate range, low-confidence rate
  * radar  -> noisy range, accurate rate (Doppler)
The filter weights them by their reported variance, gates outliers with a
Mahalanobis test, and coasts on the model when a sensor drops out.
"""

from __future__ import annotations

import numpy as np


class RangeRateKF:
    def __init__(self, sigma_a: float, r_var_init: float, c_var_init: float):
        self.sigma_a = float(sigma_a)          # m/s^2, 1-sigma unmodelled accel
        self.r_var_init = float(r_var_init)
        self.c_var_init = float(c_var_init)
        self.x: np.ndarray | None = None       # [range, closing_speed]
        self.P: np.ndarray | None = None

    @property
    def initialized(self) -> bool:
        return self.x is not None

    def init(self, range0: float, closing0: float = 0.0) -> None:
        self.x = np.array([range0, closing0], dtype=float)
        self.P = np.diag([self.r_var_init, self.c_var_init]).astype(float)

    def reset(self) -> None:
        self.x = None
        self.P = None

    def predict(self, dt: float, a_ego: float = 0.0) -> None:
        """Propagate the state.

        `a_ego` is the ego's own longitudinal acceleration (< 0 when braking),
        used as a *known control input*: closing_speed changes by (a_ego - a_lead)
        per second, and a_lead is the only unknown -> it becomes the process
        noise (`sigma_a`). Without this term the filter fights the very braking
        manoeuvre AEB commanded and rejects the measurements that disagree.
        """
        if not self.initialized or dt <= 0.0:
            return
        F = np.array([[1.0, -dt], [0.0, 1.0]])
        B = np.array([-0.5 * dt * dt, dt])          # coefficient of a_ego
        g = np.array([0.5 * dt * dt, -dt])          # coefficient of unknown a_lead
        Q = np.outer(g, g) * self.sigma_a ** 2
        self.x = F @ self.x + B * float(a_ego)
        self.x[0] = max(self.x[0], 0.0)             # range cannot go negative
        self.P = F @ self.P @ F.T + Q

    def mahalanobis2(self, z: np.ndarray, H: np.ndarray, R: np.ndarray) -> float:
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        return float(y @ np.linalg.solve(S, y))

    def update(self, z, H, R, gate2: float | None = None) -> bool:
        """Kalman update. Returns False (and does nothing) if the measurement
        fails the Mahalanobis gate."""
        if not self.initialized:
            return False
        z = np.asarray(z, dtype=float)
        H = np.asarray(H, dtype=float)
        R = np.asarray(R, dtype=float)
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        if gate2 is not None and float(y @ np.linalg.solve(S, y)) > gate2:
            return False
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        ident = np.eye(2)
        self.P = (ident - K @ H) @ self.P
        return True

    def inflate(self, factor: float) -> None:
        if self.initialized:
            self.P = self.P * float(factor)

    # convenience accessors -------------------------------------------------
    @property
    def range(self) -> float:
        return float(self.x[0])

    @property
    def closing_speed(self) -> float:
        return float(self.x[1])


def range_measurement(value: float, var: float):
    """(z, H, R) for a range-only sensor reading."""
    return (np.array([value]), np.array([[1.0, 0.0]]), np.array([[max(var, 1e-4)]]))


def rate_measurement(value: float, var: float):
    """(z, H, R) for a closing-speed-only sensor reading."""
    return (np.array([value]), np.array([[0.0, 1.0]]), np.array([[max(var, 1e-4)]]))


def full_measurement(range_v: float, range_var: float, rate_v: float, rate_var: float):
    """(z, H, R) for a sensor reporting both range and closing speed."""
    return (
        np.array([range_v, rate_v]),
        np.eye(2),
        np.diag([max(range_var, 1e-4), max(rate_var, 1e-4)]),
    )
