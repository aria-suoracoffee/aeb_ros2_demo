"""LaserScan -> a per-sensor Measurement for the fusion node.

Lidar contributes an accurate in-path *range* only. (It has no direct velocity,
and a differenced range rate proved too noisy to be worth fusing -- the radar's
Doppler owns closing speed.)

  pub  /aeb/meas/lidar   aeb_interfaces/Measurement  (source = "lidar")
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from aeb_interfaces.msg import Measurement


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        self.declare_parameter("path_half_angle_deg", 4.0)
        self.declare_parameter("valid_margin", 0.98)
        self.declare_parameter("range_var", 0.02)   # m^2  -- lidar range is tight

        self.pub = self.create_publisher(Measurement, "aeb/meas/lidar", 10)
        self.create_subscription(LaserScan, "scan", self._on_scan, 10)
        self.get_logger().info("perception_node up")

    def _on_scan(self, scan: LaserScan):
        try:
            self._process(scan)
        except Exception as exc:
            self.get_logger().error(f"scan processing failed: {exc!r}")

    def _process(self, scan: LaserScan):
        half = math.radians(float(self.get_parameter("path_half_angle_deg").value))
        margin = float(self.get_parameter("valid_margin").value)

        best = math.inf
        for i, r in enumerate(scan.ranges):
            a = scan.angle_min + i * scan.angle_increment
            if abs(a) <= half and scan.range_min <= r <= scan.range_max:
                best = min(best, r * math.cos(a))   # project onto the longitudinal axis

        valid = math.isfinite(best) and best < scan.range_max * margin

        m = Measurement()
        m.header = scan.header
        m.source = "lidar"
        m.valid = bool(valid)
        if valid:
            m.has_range = True
            m.range = float(best)
            m.range_var = float(self.get_parameter("range_var").value)
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
