# Attach OpenVINS to a running sim.
#   ros2 launch explorer_r2_sim vio.launch.py
#
# Subscribes to /imu + /rs_front/image, publishes /ov_msckf/*. The sim's
# RViz (rviz/sim.rviz) already has displays for those topics, so this
# launch does NOT spawn its own RViz.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("explorer_r2_sim")
    default_cfg = os.path.join(
        pkg_share, "config", "openvins", "estimator_config.yaml")

    config_path = LaunchConfiguration("config_path")

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
            "rviz_enable":  "false",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("config_path", default_value=default_cfg,
                              description="OpenVINS estimator_config.yaml"),
        ov_launch,
    ])
