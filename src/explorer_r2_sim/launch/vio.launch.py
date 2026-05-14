# Wraps OpenVINS' subscribe.launch.py with our explorer_r2 config.
# Use:
#   ros2 launch explorer_r2_sim vio.launch.py rviz:=true
#
# This launch is independent of cave.launch.py — start the sim first
# (or in another terminal), then start this so OV connects to the live
# /rs_front/image + /imu topics. Or compose both into one launch.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("explorer_r2_sim")
    default_cfg = os.path.join(
        pkg_share, "config", "openvins", "estimator_config.yaml")
    default_rviz = os.path.join(pkg_share, "rviz", "vio.rviz")

    config_path = LaunchConfiguration("config_path")
    rviz = LaunchConfiguration("rviz")

    # Re-use OpenVINS' own subscribe.launch.py — it knows how to bring up
    # run_subscribe_msckf with the right parameter wiring.
    ov_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ov_msckf"), "launch", "subscribe.launch.py"
            ])
        ),
        launch_arguments={
            "config_path":  config_path,
            "use_stereo":   "false",
            "max_cameras":  "1",
            "verbosity":    "INFO",
            "rviz_enable":  "false",     # we use our own rviz config below
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_vio",
        arguments=["-d", default_rviz],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=__import__("launch.conditions",
                             fromlist=["IfCondition"]).IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("config_path", default_value=default_cfg,
                              description="OpenVINS estimator_config.yaml"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="Launch RViz with the VIO layout"),
        ov_launch,
        rviz_node,
    ])
