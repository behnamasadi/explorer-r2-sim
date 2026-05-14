# Attach VINS-Fusion to a running sim.
#
#   ros2 launch explorer_r2_sim vins.launch.py
#
# Subscribes to /imu + /rs_front/image + /rs_front_right/image; publishes
# /vins_estimator/odometry, /vins_estimator/path, /vins_estimator/feature.
# The unified rviz/sim.rviz already has displays for those topics, so
# this launch does NOT spawn its own RViz.
#
# VINS-Fusion has its own startup quirks:
#   - It reads cam0_calib / cam1_calib as paths *relative to the dir
#     containing the main config*, not as ros2 launch substitutions.
#   - It expects the output_path directory to exist on disk before the
#     node starts (it writes pose-graph + logs there). The entrypoint
#     creates /ws/runs/vins_output/ on first run.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Same 8 s startup delay as OpenVINS — the spawn-drop impact happens
# in the first ~1 s after gz comes up, and VINS-Fusion's init is also
# sensitive to it.
VINS_START_DELAY_SEC = 8.0


def generate_launch_description():
    pkg_share = get_package_share_directory("explorer_r2_sim")
    default_cfg = os.path.join(pkg_share, "config", "vins", "vins.yaml")

    config_path = LaunchConfiguration("vins_config_path")

    def _make_nodes(_context, *_args, **_kwargs):
        # vins_node: the actual VIO estimator. Without the explicit
        # remappings VINS publishes to relative names (odometry, path,
        # …) that don't end up under /vins_estimator/*. The remap list
        # below matches what zinuok/VINS-Fusion-ROS2's own
        # euroc.launch.py does — keeps the published topics in the
        # /vins_estimator/* namespace that scripts/analyze_bag.py
        # subscribes to and that the rviz layout expects.
        vins_node = Node(
            package="vins",
            executable="vins_node",
            name="vins_estimator",
            arguments=[config_path],
            output="screen",
            parameters=[{"use_sim_time": True}],
            remappings=[
                ("imu_propagate",      "/vins_estimator/imu_propagate"),
                ("path",               "/vins_estimator/path"),
                ("odometry",           "/vins_estimator/odometry"),
                ("point_cloud",        "/vins_estimator/point_cloud"),
                ("margin_cloud",       "/vins_estimator/margin_cloud"),
                ("key_poses",          "/vins_estimator/key_poses"),
                ("camera_pose",        "/vins_estimator/camera_pose"),
                ("camera_pose_visual", "/vins_estimator/camera_pose_visual"),
                ("keyframe_pose",      "/vins_estimator/keyframe_pose"),
                ("keyframe_point",     "/vins_estimator/keyframe_point"),
                ("extrinsic",          "/vins_estimator/extrinsic"),
                ("image_track",        "/vins_estimator/image_track"),
            ],
        )

        # Glue static transform: VINS-Fusion publishes its odometry in
        # frame "world" by default. Anchor it to explorer_r2/odom for
        # RViz, same idea as the OpenVINS global → explorer_r2/odom glue.
        vins_world_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="vins_world_to_odom",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "explorer_r2/odom",
                "--child-frame-id", "world",
            ],
            output="screen",
        )

        return [vins_world_tf, TimerAction(period=VINS_START_DELAY_SEC, actions=[vins_node])]

    return LaunchDescription([
        DeclareLaunchArgument("vins_config_path", default_value=default_cfg,
                              description="Path to VINS-Fusion YAML config"),
        OpaqueFunction(function=_make_nodes),
    ])
