# Attach FAST_LIO to a running sim. Mirrors vio.launch.py.
#   ros2 launch explorer_r2_sim lio.launch.py
#
# Subscribes to /lidar/points + /imu, publishes /fast_lio/{Odometry, path,
# cloud_registered}. The sim's RViz (rviz/sim.rviz) already has displays
# for those topics, so this launch does NOT spawn its own RViz.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("explorer_r2_sim")
    default_cfg = os.path.join(pkg_share, "config", "lio.yaml")

    config_path = LaunchConfiguration("config_path")

    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        parameters=[config_path, {"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("config_path", default_value=default_cfg,
                              description="FAST_LIO YAML config"),
        fast_lio_node,
    ])
