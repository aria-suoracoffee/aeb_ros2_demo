"""Fuse the lidar and radar Measurements into one Hazard.

  sub  /aeb/meas/lidar   aeb_interfaces/Measurement
  sub  /aeb/meas/radar   aeb_interfaces/Measurement
  pub  /aeb/hazard       aeb_interfaces/Hazard   (same message aeb_node already consumes)

A constant-closing-speed Kalman filter (aeb_demo.fusion) carries the track.
Each measurement is Mahalanobis-gated before it is applied, so radar false
alarms are dropped. If no measurement passes for `coast_timeout` the filter
coasts on the model with inflating covariance; after `drop_timeout` the track
is declared invalid.
"""

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from aeb_interfaces.msg import Hazard, Measurement
from aeb_demo.fusion import RangeRateKF, range_measurement

NO_TARGET = 1000.0


class FusionNode(Node):
    def __init__(self):
        super().__init__("fusion_node")
        self.declare_parameter("sigma_a", 4.0)        # m/s^2, unknown lead accel
        self.declare_parameter("gate_chi2", 9.21)     # 2-dof, ~99%
        self.declare_parameter("coast_timeout", 0.6)  # s
        self.declare_parameter("drop_timeout", 1.5)   # s
        self.declare_parameter("publish_rate", 50.0)  # Hz
        self.declare_parameter("r_var_init", 4.0)
        self.declare_parameter("c_var_init", 100.0)
        self.declare_parameter("accel_alpha", 0.3)    # smoothing on the ego accel estimate

        g = self.get_parameter
        self.kf = RangeRateKF(
            sigma_a=float(g("sigma_a").value),
            r_var_init=float(g("r_var_init").value),
            c_var_init=float(g("c_var_init").value),
        )
        self.last_t = None            # time the filter state is valid for
        self.last_update_t = None     # time of the last accepted measurement
        self.fresh = {"lidar": 0.0, "radar": 0.0}

        # Ego motion -> control input for the predict step.
        self.v_ego = 0.0
        self.a_ego = 0.0
        self.odom_t = None

        self.hazard_pub = self.create_publisher(Hazard, "aeb/hazard", 10)
        self.create_subscription(Odometry, "ego/odom", self._on_odom, 10)
        self.create_subscription(Measurement, "aeb/meas/lidar", self._on_meas, 10)
        self.create_subscription(Measurement, "aeb/meas/radar", self._on_meas, 10)
        self.create_timer(1.0 / float(g("publish_rate").value), self._tick)
        self.get_logger().info("fusion_node up")

    def _on_odom(self, m: Odometry):
        v = float(m.twist.twist.linear.x)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.odom_t is not None:
            dt = t - self.odom_t
            if dt > 1e-3:
                a = (v - self.v_ego) / dt
                alpha = float(self.get_parameter("accel_alpha").value)
                self.a_ego += alpha * (a - self.a_ego)
        self.v_ego = v
        self.odom_t = t

    # ------------------------------------------------------------------ time
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _predict_to(self, t: float) -> None:
        if self.last_t is not None:
            self.kf.predict(t - self.last_t, a_ego=self.a_ego)
        self.last_t = t

    # ----------------------------------------------------------- measurement
    def _on_meas(self, m: Measurement):
        try:
            self._integrate(m)
        except Exception as exc:
            self.get_logger().error(f"fusion update failed: {exc!r}")

    def _integrate(self, m: Measurement):
        if not m.valid:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

        rows, z, diag = [], [], []
        if m.has_range:
            rows.append([1.0, 0.0]); z.append(m.range); diag.append(max(m.range_var, 1e-4))
        if m.has_rate:
            rows.append([0.0, 1.0]); z.append(m.closing_speed); diag.append(max(m.rate_var, 1e-4))
        if not rows:
            return

        if not self.kf.initialized:
            if m.has_range:
                self.kf.init(m.range, m.closing_speed if m.has_rate else 0.0)
                self.last_t = t
                self.last_update_t = t
                self.fresh[m.source] = self._now()
            return

        gate = float(self.get_parameter("gate_chi2").value)
        self._predict_to(t)
        ok = self.kf.update(np.array(z), np.array(rows), np.diag(diag), gate2=gate)

        if not ok and m.has_range and len(rows) > 1:
            # Joint gate failed -- fall back to the safety-critical range alone.
            ok = self.kf.update(*range_measurement(m.range, m.range_var), gate2=gate)
            if ok:
                self.get_logger().info(f"{m.source}: rate rejected, range accepted")

        if ok:
            self.last_update_t = t
            self.fresh[m.source] = self._now()
        else:
            self.get_logger().warn(f"gated {m.source} outlier "
                                   f"(range={m.range:.1f} v_close={m.closing_speed:.1f})")

    # ------------------------------------------------------------------ tick
    def _tick(self):
        now = self._now()
        if not self.kf.initialized:
            self._publish(now, valid=False)
            return

        self._predict_to(now)
        stale = now - (self.last_update_t if self.last_update_t is not None else now)
        if stale > float(self.get_parameter("drop_timeout").value):
            self.kf.reset()
            self.last_t = None
            self._publish(now, valid=False)
            return
        if stale > float(self.get_parameter("coast_timeout").value):
            self.kf.inflate(1.02)   # per tick while coasting

        self._publish(now, valid=True)

    def _publish(self, now: float, valid: bool):
        h = Hazard()
        h.header.stamp = self.get_clock().now().to_msg()
        h.header.frame_id = "ego"
        if valid:
            r = max(self.kf.range, 0.0)
            c = self.kf.closing_speed
            h.range = float(r)
            h.closing_speed = float(c)
            h.ttc = float(r / c) if c > 0.1 else NO_TARGET
            h.valid = True
        else:
            h.range = NO_TARGET
            h.closing_speed = 0.0
            h.ttc = NO_TARGET
            h.valid = False
        self.hazard_pub.publish(h)

        if valid:
            def tag(src):
                return "ok" if (now - self.fresh[src]) < 0.3 else "--"
            self.get_logger().info(
                f"fused range={h.range:6.1f} v_close={h.closing_speed:5.1f} "
                f"ttc={min(h.ttc, 999.9):6.1f} | lidar {tag('lidar')} radar {tag('radar')}",
                throttle_duration_sec=1.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
