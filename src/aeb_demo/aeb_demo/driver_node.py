"""Stand-in for a human driver / upstream planner.

Publishes a constant target speed on /driver/cmd_vel. The AEB node supervises
this command. When AEB is disabled in the launch file this topic is remapped
straight to /cmd_vel so the ego just cruises into whatever is ahead.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class DriverNode(Node):
    def __init__(self):
        super().__init__("driver_node")
        self.declare_parameter("target_speed", 14.0)
        self.declare_parameter("rate_hz", 20.0)
        self.pub = self.create_publisher(Twist, "driver/cmd_vel", 10)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)

    def _tick(self):
        msg = Twist()
        msg.linear.x = float(self.get_parameter("target_speed").value)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
