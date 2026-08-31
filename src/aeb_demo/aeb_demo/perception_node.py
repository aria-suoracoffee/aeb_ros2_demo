"""Turn a raw LaserScan into a single in-path hazard estimate.

Output: aeb_interfaces/Hazard on /aeb/hazard with
  * range         nearest obstacle in the vehicle's path (projected onto the
                  longitudinal axis), inf if the path is clear
  * closing_speed range rate estimated by differentiating successive scans and
                  low-pass filtering -- i.e. a sensor-only closing speed, no
                  dependence on the ego odometry
  * ttc           range / closing_speed
  * valid         whether a real target is present
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from aeb_interfaces.msg import Hazard


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        self.declare_parameter("path_half_angle_deg", 4.0)
        self.declare_parameter("range_rate_alpha", 0.4)
        self.declare_parameter("valid_margin", 0.98)

        self.prev_range = None
        self.prev_stamp = None
        self.closing_speed = 0.0

        self.pub = self.create_publisher(Hazard, "aeb/hazard", 10)
        self.create_subscription(LaserScan, "scan", self._on_scan, 10)

    def _on_scan(self, scan: LaserScan):
        half = math.radians(float(self.get_parameter("path_half_angle_deg").value))
        alpha = float(self.get_parameter("range_rate_alpha").value)
        margin = float(self.get_parameter("valid_margin").value)

        best = math.inf
        for i, r in enumerate(scan.ranges):
            a = scan.angle_min + i * scan.angle_increment
            if abs(a) <= half and scan.range_min <= r <= scan.range_max:
                # Project the beam onto the longitudinal axis.
                best = min(best, r * math.cos(a))

        valid = best < scan.range_max * margin
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9

        if valid and self.prev_range is not None and self.prev_stamp is not None:
            dt = stamp - self.prev_stamp
            if dt > 1e-3:
                rate = (best - self.prev_range) / dt          # <0 => approaching
                inst_closing = -rate
                self.closing_speed += alpha * (inst_closing - self.closing_speed)
        elif not valid:
            self.closing_speed = 0.0

        self.prev_range = best if valid else None
        self.prev_stamp = stamp if valid else None

        msg = Hazard()
        msg.header = scan.header
        msg.range = float(best if valid else math.inf)
        msg.closing_speed = float(self.closing_speed if valid else 0.0)
        if valid and self.closing_speed > 0.1:
            msg.ttc = float(best / self.closing_speed)
        else:
            msg.ttc = float("inf")
        msg.valid = bool(valid)
        self.pub.publish(msg)


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
