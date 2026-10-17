"""Launch the full simulated AMR bench.

Fault injection examples:
  ros2 launch ros2_amr_bench bench.launch.py fault:=stuck_left fault_start:=5.0
  ros2 launch ros2_amr_bench bench.launch.py slip_factor:=0.5 slip_start:=5.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("ros2_amr_bench"), "config", "amr.yaml")
    arg = LaunchConfiguration
    args = [DeclareLaunchArgument("fault", default_value="none"),
            DeclareLaunchArgument("fault_start", default_value="0.0"),
            DeclareLaunchArgument("slip_factor", default_value="1.0"),
            DeclareLaunchArgument("slip_start", default_value="0.0")]

    def node(exe, name, extra=None):
        return Node(package="ros2_amr_bench", executable=exe, name=name, output="screen",
                    parameters=[params] + ([extra] if extra else []))

    return LaunchDescription(args + [
        node("amr_sim", "amr_sim", {"slip_factor": arg("slip_factor"), "slip_start": arg("slip_start")}),
        node("can_bridge", "can_bridge", {"fault": arg("fault"), "fault_start": arg("fault_start")}),
        node("waypoint_follower", "waypoint_follower"),
        node("amr_diagnostics", "amr_diagnostics"),
    ])
