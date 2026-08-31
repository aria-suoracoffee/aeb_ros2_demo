"""Raw radar returns -> a gated per-sensor Measurement.

Radar gives an accurate closing speed (Doppler) but a noisy range, and it both
drops out and occasionally reports false alarms. This node only does simple
validity gating; the Mahalanobis gate in the fusion node rejects the false
alarms that slip through.
"""

import rclpy
from rclpy.node import Node

from aeb_interfaces.msg import Measurement, RadarReturn


class RadarNode(Node):
    def __init__(self):
        super().__init__("radar_node")
        self.declare_parameter("range_var", 0.36)   # (0.6 m)^2
        self.declare_parameter("rate_var", 0.02)    # (~0.14 m/s)^2
        self.declare_parameter("max_range", 120.0)

        self.pub = self.create_publisher(Measurement, "aeb/meas/radar", 10)
        self.create_subscription(RadarReturn, "radar", self._on_radar, 10)
        self.get_logger().info("radar_node up")

    def _on_radar(self, r: RadarReturn):
        m = Measurement()
        m.header = r.header
        m.source = "radar"

        max_range = float(self.get_parameter("max_range").value)
        if not r.detected or not (0.0 < r.range <= max_range):
            m.valid = False
            self.pub.publish(m)
            return

        m.valid = True
        m.has_range = True
        m.range = float(r.range)
        m.range_var = float(self.get_parameter("range_var").value)
        m.has_rate = True
        m.closing_speed = float(-r.range_rate)   # approaching -> positive
        m.rate_var = float(self.get_parameter("rate_var").value)
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = RadarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
