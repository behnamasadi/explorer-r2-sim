# Mode 1: one-command full stack.
#
#   gz sim + world      ─►  world.launch.py
#   spawn EXPLORER_R2   ─►  spawn_robot.launch.py
#   OpenVINS (VIO)      ─►  vio.launch.py
#   VINS-Fusion (VIO)   ─►  vins.launch.py    (optional, for head-to-head)
#   FAST_LIO (LIO)      ─►  lio.launch.py
#
# All sub-launches run as one composite launch. To run the same stack
# manually step-by-step, see "Mode 2" in the README — launch each of the
# four files above in separate terminals.
#
# Launch arguments (forwarded to the sub-launches):
#   world:=<preset|path|fuel-url>      tunnel (default) | cave | rubicon
#                                      | tugbot_depot | singapore_river,
#                                      or a local .sdf path, or a Fuel URL.
#   spawn_x / spawn_y / spawn_z / spawn_yaw
#                                      Override per-world default pose.
#                                      "auto" = use SPAWN_POSES[<world>].
#   gui:=true|false                    Gazebo GUI (default true).
#   rviz:=true|false                   Open RViz with sim.rviz (default true).
#   joy:=true|false                    Joystick stack (default true).
#   teleop:=true|false                 xterm with teleop_twist_keyboard.
#   rqt_steering:=true|false           rqt sliders for /cmd_vel.
#   verbose:=<0-4>                     Gazebo verbosity.

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# Args forwarded verbatim to world.launch.py.
_WORLD_ARGS = ("world", "gui", "rviz", "joy", "teleop", "rqt_steering", "verbose")
# Args forwarded verbatim to spawn_robot.launch.py.
_SPAWN_ARGS = ("world", "spawn_x", "spawn_y", "spawn_z", "spawn_yaw")


def _forward(context, arg_names):
    """Capture current values of named launch args for an IncludeLaunchDescription."""
    return {n: LaunchConfiguration(n).perform(context) for n in arg_names}


def generate_launch_description():
    pkg_share = get_package_share_directory("explorer_r2_sim")
    launch_dir = os.path.join(pkg_share, "launch")

    def _setup(context, *_args, **_kwargs):
        actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "world.launch.py")),
                launch_arguments=_forward(context, _WORLD_ARGS).items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "spawn_robot.launch.py")),
                launch_arguments=_forward(context, _SPAWN_ARGS).items(),
            ),
        ]

        # vio.launch.py — only include if OpenVINS is actually installed.
        try:
            get_package_share_directory("ov_msckf")
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "vio.launch.py"))))
        except PackageNotFoundError:
            print("[cave.launch.py] ov_msckf not installed — skipping VIO. "
                  "Initialise third_party/open_vins and rebuild to enable.")

        # vins.launch.py — VINS-Fusion (the second VIO, for head-to-head
        # comparison). Optional submodule at third_party/VINS-Fusion-ROS2.
        try:
            get_package_share_directory("vins")
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "vins.launch.py"))))
        except PackageNotFoundError:
            print("[cave.launch.py] vins not installed — skipping VINS-Fusion. "
                  "Initialise third_party/VINS-Fusion-ROS2 and rebuild to enable.")

        # lio.launch.py — same guard for FAST_LIO.
        try:
            get_package_share_directory("fast_lio")
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "lio.launch.py"))))
        except PackageNotFoundError:
            print("[cave.launch.py] fast_lio not installed — skipping LIO. "
                  "Initialise third_party/FAST_LIO and rebuild to enable.")

        return actions

    return LaunchDescription([
        DeclareLaunchArgument("world",         default_value="tunnel"),
        DeclareLaunchArgument("gui",           default_value="true"),
        DeclareLaunchArgument("rviz",          default_value="true"),
        DeclareLaunchArgument("joy",           default_value="true"),
        DeclareLaunchArgument("teleop",        default_value="false"),
        DeclareLaunchArgument("rqt_steering",  default_value="false"),
        DeclareLaunchArgument("verbose",       default_value="3"),
        DeclareLaunchArgument("spawn_x",       default_value="auto"),
        DeclareLaunchArgument("spawn_y",       default_value="auto"),
        DeclareLaunchArgument("spawn_z",       default_value="auto"),
        DeclareLaunchArgument("spawn_yaw",     default_value="auto"),
        OpaqueFunction(function=_setup),
    ])
