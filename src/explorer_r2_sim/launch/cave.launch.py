# Brings up the full sim in one shot:
#   gz sim  ─►  cave.sdf  (EXPLORER_R2 in a SubT cave)
#   parameter_bridge  ─►  config/bridge.yaml
#   RViz2  ─►  rviz/cave.rviz
#   teleop_twist_keyboard  (optional, opens in its own xterm)
#
# Launch arguments:
#   gui:=true|false        run Gazebo with GUI (default true)
#   rviz:=true|false       open RViz2 (default true)
#   teleop:=true|false     spawn an xterm running teleop_twist_keyboard
#   world:=<path>          override the world SDF
#   verbose:=true|false    Gazebo verbose flag (-v 4)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("explorer_r2_sim")
    # Default world is the 119-tile SubT tunnel (tunnel.sdf). Switch to the
    # smaller open cave with: world:=…/cave.sdf
    default_world = os.path.join(pkg_share, "worlds", "tunnel.sdf")
    bridge_cfg = os.path.join(pkg_share, "config", "bridge.yaml")
    joy_cfg = os.path.join(pkg_share, "config", "joy_teleop.yaml")
    rviz_cfg = os.path.join(pkg_share, "rviz", "cave.rviz")

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    teleop = LaunchConfiguration("teleop")
    joy = LaunchConfiguration("joy")
    world = LaunchConfiguration("world")
    verbose = LaunchConfiguration("verbose")

    # Use ros_gz_sim's gz_sim.launch.py to start Gazebo correctly across
    # Fortress / Garden / Harmonic — it picks the right binary for the
    # ros_gz version installed in the container.
    # When gui:=false, pass -s to run Gazebo server-only (no rendering UI).
    server_only = PythonExpression(["'' if '", gui, "' == 'true' else ' -s'"])

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            # -r: run immediately. -v: verbosity level (0-4).
            "gz_args": [world, " -r", server_only, " -v ", verbose],
            "on_exit_shutdown": "true",
        }.items(),
    )

    parameter_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_cfg, "use_sim_time": True}],
    )

    # image_bridge gives you compressed image topics for free if you ever
    # want to record bags — leave it out by default to save CPU.
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(rviz),
    )

    # teleop_twist_keyboard requires a TTY — spawn it in its own xterm so it
    # stays interactive even when launched alongside gz/rviz.
    teleop_proc = ExecuteProcess(
        cmd=[
            "xterm", "-e",
            "ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard",
            "--ros-args", "-r", "cmd_vel:=/cmd_vel",
        ],
        output="screen",
        condition=IfCondition(teleop),
    )

    # Joystick stack: joy_node (reads /dev/input/jsX) → teleop_twist_joy_node
    # → /cmd_vel. Container must bind-mount /dev/input and run with the
    # `input` group; compose.yml handles that.
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[joy_cfg, {"use_sim_time": True}],
        output="screen",
        condition=IfCondition(joy),
    )
    joy_teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[joy_cfg, {"use_sim_time": True}],
        remappings=[("/cmd_vel", "/cmd_vel")],
        output="screen",
        condition=IfCondition(joy),
    )

    # rqt_robot_steering: a simple GUI window with two sliders (linear / angular)
    # publishing to /cmd_vel. Handy when no joystick is available.
    rqt_steering = ExecuteProcess(
        cmd=["rqt", "--standalone", "rqt_robot_steering"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rqt_steering")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui",     default_value="true"),
        DeclareLaunchArgument("rviz",    default_value="true"),
        DeclareLaunchArgument("teleop",  default_value="false",
                              description="Open xterm with teleop_twist_keyboard"),
        DeclareLaunchArgument("joy",     default_value="true",
                              description="Start joy + teleop_twist_joy nodes"),
        DeclareLaunchArgument("rqt_steering", default_value="false",
                              description="Open rqt_robot_steering GUI sliders"),
        DeclareLaunchArgument("world",   default_value=default_world),
        DeclareLaunchArgument("verbose", default_value="3"),

        # Make the Fuel + workspace model paths discoverable.
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            os.path.join(pkg_share, "worlds") + ":" +
            os.path.join(pkg_share, "models") + ":" +
            os.environ.get("GZ_SIM_RESOURCE_PATH", "")),
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            os.path.join(pkg_share, "worlds") + ":" +
            os.path.join(pkg_share, "models") + ":" +
            os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")),

        gz_sim_launch,
        parameter_bridge,
        rviz_node,
        teleop_proc,
        joy_node,
        joy_teleop_node,
        rqt_steering,
    ])
