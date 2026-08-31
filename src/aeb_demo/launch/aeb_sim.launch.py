"""Bring up the AEB demo: sim + perception + driver (+ AEB controller).

Examples:
  ros2 launch aeb_demo aeb_sim.launch.py
  ros2 launch aeb_demo aeb_sim.launch.py scenario:=hard_brake
  ros2 launch aeb_demo aeb_sim.launch.py scenario:=stationary_lead enable_aeb:=false
  ros2 launch aeb_demo aeb_sim.launch.py rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("aeb_demo")
    params = PathJoinSubstitution([pkg, "config", "params.yaml"])

    scenario = LaunchConfiguration("scenario")
    enable_aeb = LaunchConfiguration("enable_aeb")
    rviz = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "scenario", default_value="stationary_lead",
            description="stationary_lead | slower_lead | hard_brake",
        ),
        DeclareLaunchArgument("enable_aeb", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),

        Node(
            package="aeb_demo", executable="sim_node", name="sim_node",
            output="screen",
            parameters=[params, {"scenario": scenario}],
        ),
        Node(
            package="aeb_demo", executable="perception_node", name="perception_node",
            output="screen", parameters=[params],
        ),

        # AEB on: driver -> /driver/cmd_vel, supervised by aeb_node.
        Node(
            package="aeb_demo", executable="driver_node", name="driver_node",
            output="screen", parameters=[params],
            condition=IfCondition(enable_aeb),
        ),
        Node(
            package="aeb_demo", executable="aeb_node", name="aeb_node",
            output="screen", parameters=[params],
            condition=IfCondition(enable_aeb),
        ),

        # AEB off: driver command goes straight to /cmd_vel.
        Node(
            package="aeb_demo", executable="driver_node", name="driver_node",
            output="screen", parameters=[params],
            remappings=[("driver/cmd_vel", "cmd_vel")],
            condition=UnlessCondition(enable_aeb),
        ),

        Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=["-d", PathJoinSubstitution([pkg, "rviz", "aeb.rviz"])],
            condition=IfCondition(rviz),
        ),
    ])
