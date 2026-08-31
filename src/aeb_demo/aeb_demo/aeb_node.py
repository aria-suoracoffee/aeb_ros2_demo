"""Automatic Emergency Braking controller.

Relays the driver's speed command to /cmd_vel under normal conditions, and
overrides it with a full-braking command when the hazard estimate crosses the
thresholds in aeb_demo.safety.

Topics:
  sub  /aeb/hazard      aeb_interfaces/Hazard
  sub  /ego/odom        nav_msgs/Odometry     (ego speed, for the stop latch)
  sub  /driver/cmd_vel  geometry_msgs/Twist   (the command being supervised)
  pub  /cmd_vel         geometry_msgs/Twist
  pub  /aeb/status      aeb_interfaces/AEBStatus
"""

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from aeb_interfaces.msg import AEBStatus, Hazard
from aeb_demo.safety import BRAKE, STATE_LABELS, AEBParams, decide


class AEBNode(Node):
    def __init__(self):
        super().__init__("aeb_node")

        self.params = AEBParams()
        for name in self.params.field_names():
            self.declare_parameter(name, getattr(self.params, name))
            setattr(self.params, name, float(self.get_parameter(name).value))

        self.state = 0
        self.ego_speed = 0.0
        self.driver_target = 0.0

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.status_pub = self.create_publisher(AEBStatus, "aeb/status", 10)
        self.create_subscription(Odometry, "ego/odom", self._on_odom, 10)
        self.create_subscription(Twist, "driver/cmd_vel", self._on_driver, 10)
        self.create_subscription(Hazard, "aeb/hazard", self._on_hazard, 10)

        self.get_logger().info("aeb_node up")

    def _on_odom(self, msg: Odometry):
        self.ego_speed = float(msg.twist.twist.linear.x)

    def _on_driver(self, msg: Twist):
        self.driver_target = max(0.0, float(msg.linear.x))

    def _on_hazard(self, hz: Hazard):
        d = decide(
            self.state,
            hz.valid,
            hz.range,
            hz.closing_speed,
            self.ego_speed,
            self.params,
        )
        prev, self.state = self.state, d.state

        out = Twist()
        if d.target_speed_override is None:
            out.linear.x = self.driver_target
        else:
            out.linear.x = d.target_speed_override
        self.cmd_pub.publish(out)

        status = AEBStatus()
        status.header = hz.header
        status.state = d.state
        status.state_label = STATE_LABELS[d.state]
        status.ttc = hz.ttc
        status.required_decel = min(d.required_decel, 99.9)
        status.braking = d.brake_request
        self.status_pub.publish(status)

        if d.state != prev:
            self.get_logger().warn(
                f"{STATE_LABELS[prev]} -> {STATE_LABELS[d.state]}  "
                f"ttc={hz.ttc:.2f}s  req_decel={d.required_decel:.1f} m/s^2  "
                f"v_ego={self.ego_speed:.1f} m/s"
            )


def main(args=None):
    rclpy.init(args=args)
    node = AEBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
