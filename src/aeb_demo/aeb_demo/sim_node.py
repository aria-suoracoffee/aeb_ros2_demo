"""Lightweight longitudinal driving simulator.

Publishes just enough for the AEB pipeline to run without Gazebo:

  * /scan          sensor_msgs/LaserScan   forward-facing fake lidar
  * /ego/odom      nav_msgs/Odometry       ego position + speed
  * /sim/collision std_msgs/Bool           latches true if the ego hits the lead
  * /sim/markers   visualization_msgs/MarkerArray  boxes for RViz
  * TF map -> base_link, map -> ego_lidar

Subscribes:

  * /cmd_vel       geometry_msgs/Twist     linear.x is treated as target speed

Scenarios (parameter `scenario`):
  * stationary_lead  - a stopped car 60 m ahead
  * slower_lead       - a car ahead cruising slower than the ego
  * hard_brake        - a car ahead matching speed, then braking hard at t=4 s
"""

import math
import random

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from aeb_interfaces.msg import RadarReturn


class SimNode(Node):
    def __init__(self):
        super().__init__("sim_node")

        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("scenario", "stationary_lead")
        self.declare_parameter("ego_start_speed", 14.0)   # m/s ~ 50 km/h
        self.declare_parameter("lead_start_gap", 60.0)    # m, bumper to bumper
        self.declare_parameter("lead_speed", 0.0)         # m/s
        self.declare_parameter("lead_brake_time", 4.0)    # s (hard_brake)
        self.declare_parameter("lead_decel", 6.0)         # m/s^2 (hard_brake)
        self.declare_parameter("max_accel", 2.0)          # m/s^2
        self.declare_parameter("max_brake_decel", 8.0)    # m/s^2
        self.declare_parameter("car_length", 4.5)         # m
        self.declare_parameter("lead_width", 1.8)         # m
        self.declare_parameter("scan_range_max", 80.0)    # m
        self.declare_parameter("scan_fov_deg", 60.0)      # deg
        self.declare_parameter("scan_rays", 121)
        self.declare_parameter("scan_noise_std", 0.03)    # m
        # Radar: coarse range, good Doppler rate, plus dropouts and false alarms.
        self.declare_parameter("radar_decim", 3)          # publish every Nth sim step
        self.declare_parameter("radar_range_std", 0.6)    # m
        self.declare_parameter("radar_rate_std", 0.12)    # m/s
        self.declare_parameter("radar_p_dropout", 0.12)   # fraction of cycles with no return
        self.declare_parameter("radar_p_false", 0.03)     # fraction with a spurious return
        self.declare_parameter("radar_range_max", 120.0)  # m

        g = self.get_parameter
        self.dt = 1.0 / float(g("rate_hz").value)
        self.scenario = str(g("scenario").value)
        self.car_length = float(g("car_length").value)

        self.t = 0.0
        self.x_ego = 0.0
        self.v_ego = float(g("ego_start_speed").value)
        self.x_lead = self.x_ego + self.car_length + float(g("lead_start_gap").value)
        self.v_lead = float(g("lead_speed").value)
        if self.scenario == "hard_brake" and self.v_lead == 0.0:
            self.v_lead = self.v_ego  # travel matched, then brake at lead_brake_time
        elif self.scenario == "slower_lead" and self.v_lead == 0.0:
            self.v_lead = max(0.0, self.v_ego - 8.0)  # default: 8 m/s slower than ego
        self.cmd_target_speed = self.v_ego
        self.collided = False
        self._radar_count = 0

        self.scan_pub = self.create_publisher(LaserScan, "scan", 10)
        self.radar_pub = self.create_publisher(RadarReturn, "radar", 10)
        self.odom_pub = self.create_publisher(Odometry, "ego/odom", 10)
        self.collision_pub = self.create_publisher(Bool, "sim/collision", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "sim/markers", 10)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(self.dt, self._step)
        self.get_logger().info(
            f"sim_node up  scenario={self.scenario}  "
            f"v_ego={self.v_ego:.1f}  gap={g('lead_start_gap').value:.1f}"
        )

    # ------------------------------------------------------------------ IO
    def _on_cmd(self, msg: Twist):
        self.cmd_target_speed = max(0.0, float(msg.linear.x))

    # ---------------------------------------------------------------- model
    def _step(self):
        g = self.get_parameter
        max_accel = float(g("max_accel").value)
        max_brake = float(g("max_brake_decel").value)

        # Ego: first-order tracking of the commanded target speed with
        # asymmetric accel / brake limits.
        dv_cmd = self.cmd_target_speed - self.v_ego
        dv = max(-max_brake * self.dt, min(max_accel * self.dt, dv_cmd))
        self.v_ego = max(0.0, self.v_ego + dv)
        self.x_ego += self.v_ego * self.dt

        # Lead vehicle.
        if self.scenario == "hard_brake" and self.t >= float(g("lead_brake_time").value):
            self.v_lead = max(0.0, self.v_lead - float(g("lead_decel").value) * self.dt)
        self.x_lead += self.v_lead * self.dt
        self.t += self.dt

        gap = self.x_lead - self.x_ego - self.car_length
        if gap <= 0.0 and not self.collided:
            self.collided = True
            self.get_logger().error(
                f"COLLISION at t={self.t:.1f}s  impact speed={self.v_ego:.1f} m/s"
            )
        self.collision_pub.publish(Bool(data=self.collided))

        self._publish_scan(max(gap, 0.0))
        self._radar_count += 1
        if self._radar_count % max(1, int(g("radar_decim").value)) == 0:
            self._publish_radar(max(gap, 0.0), self.v_lead - self.v_ego)
        self._publish_odom()
        self._publish_tf()
        self._publish_markers()

        self.get_logger().info(
            f"t={self.t:5.1f}  v_ego={self.v_ego:5.1f}  v_lead={self.v_lead:5.1f}  "
            f"gap={gap:6.1f}",
            throttle_duration_sec=0.5,
        )

    # ------------------------------------------------------------- sensors
    def _publish_scan(self, gap: float):
        g = self.get_parameter
        n = int(g("scan_rays").value)
        fov = math.radians(float(g("scan_fov_deg").value))
        rmax = float(g("scan_range_max").value)
        width = float(g("lead_width").value)
        noise = float(g("scan_noise_std").value)

        amin = -fov / 2.0
        ainc = fov / (n - 1)

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ego_lidar"
        msg.angle_min = amin
        msg.angle_max = fov / 2.0
        msg.angle_increment = ainc
        msg.range_min = 0.05
        msg.range_max = rmax

        # Angular half-width the lead subtends at distance `gap`.
        half_angular = math.atan2(width / 2.0, gap) if gap > 0.1 else math.pi / 2.0

        ranges = []
        for i in range(n):
            a = amin + i * ainc
            if gap > 0.1 and abs(a) <= half_angular:
                r = gap / math.cos(a) + random.gauss(0.0, noise)
                r = min(max(r, msg.range_min), rmax)
            else:
                r = rmax
            ranges.append(float(r))
        msg.ranges = ranges
        self.scan_pub.publish(msg)

    def _publish_radar(self, gap: float, range_rate_true: float):
        g = self.get_parameter
        m = RadarReturn()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "ego_radar"

        if random.random() < float(g("radar_p_dropout").value) or gap <= 0.1:
            m.detected = False
            self.radar_pub.publish(m)
            return

        if random.random() < float(g("radar_p_false").value):
            # spurious return at a random range / rate
            m.detected = True
            m.range = random.uniform(4.0, float(g("radar_range_max").value))
            m.range_rate = random.uniform(-28.0, 4.0)
            self.radar_pub.publish(m)
            return

        m.detected = True
        m.range = max(0.0, gap + random.gauss(0.0, float(g("radar_range_std").value)))
        m.range_rate = range_rate_true + random.gauss(0.0, float(g("radar_rate_std").value))
        self.radar_pub.publish(m)

    def _publish_odom(self):
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = "map"
        o.child_frame_id = "base_link"
        o.pose.pose.position.x = self.x_ego
        o.pose.pose.orientation.w = 1.0
        o.twist.twist.linear.x = self.v_ego
        self.odom_pub.publish(o)

    def _publish_tf(self):
        now = self.get_clock().now().to_msg()
        for child in ("base_link", "ego_lidar"):
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = "map"
            tf.child_frame_id = child
            tf.transform.translation.x = self.x_ego
            tf.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(tf)

    def _publish_markers(self):
        arr = MarkerArray()
        for idx, (x, color) in enumerate(
            ((self.x_ego, (0.1, 0.6, 1.0)), (self.x_lead, (1.0, 0.3, 0.1)))
        ):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "vehicles"
            m.id = idx
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = x - self.car_length / 2.0
            m.pose.position.z = 0.75
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = self.car_length, 1.8, 1.5
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 0.85
            arr.markers.append(m)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
