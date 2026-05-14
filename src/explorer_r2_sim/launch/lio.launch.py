# Attach FAST_LIO to a running sim.
#   ros2 launch explorer_r2_sim lio.launch.py
#
# Two nodes:
#   1. lidar_field_adapter.py: enriches /lidar/points (gz output, x/y/z/i
#      only) with synthesized per-point ring + t fields → /lidar/points_lio,
#      because FAST_LIO's Ouster preprocessor refuses to process scans
#      without those fields.
#   2. fastlio_mapping: the actual LIO. Subscribes to /lidar/points_lio
#      + /imu, publishes /Odometry, /path, /cloud_registered. The sim's
#      RViz (rviz/sim.rviz) already has displays for those topics, so
#      this launch does NOT spawn its own RViz.

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

    # Geometry params match models/explorer_r2/model.sdf's gpu_lidar.
    lidar_adapter = Node(
        package="explorer_r2_sim",
        executable="lidar_field_adapter.py",
        name="lidar_field_adapter",
        output="screen",
        parameters=[{
            "scan_lines":     16,
            "min_elev_deg":  -15.0,
            "max_elev_deg":   15.0,
            "scan_rate_hz":   15.0,
            "input_topic":   "/lidar/points",
            "output_topic":  "/lidar/points_lio",
            "use_sim_time":  True,
        }],
    )

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
        lidar_adapter,
        fast_lio_node,
    ])
