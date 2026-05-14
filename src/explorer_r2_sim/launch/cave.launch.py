# Brings up the full sim in one shot:
#   gz sim  ─►  <world>.sdf  (EXPLORER_R2 in a SubT environment)
#   parameter_bridge  ─►  config/bridge.yaml
#   RViz2  ─►  rviz/sim.rviz       (unified layout: sim + VIO + LIO + GNSS)
#   teleop_twist_keyboard  (optional, opens in its own xterm)
#
# Launch arguments:
#   gui:=true|false        run Gazebo with GUI (default true)
#   rviz:=true|false       open RViz2 (default true)
#   teleop:=true|false     spawn an xterm running teleop_twist_keyboard
#   world:=<preset|path|fuel-url>
#                          tunnel | cave (preset short names), or a local
#                          SDF path, or a Fuel world URL. Default: tunnel.
#   verbose:=true|false    Gazebo verbose flag (-v 4)

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Short-name presets. Values are either:
#   • a string ending in ".sdf"   → resolved against the package's worlds/
#   • a https://fuel.gazebosim.org/1.0/... URL → passed to gz sim directly
#   • a https://app.gazebosim.org/... URL     → rewritten to the API form
#
# Note: tunnel and cave include the EXPLORER_R2 model + plugins inline.
# Third-party Fuel worlds do *not* — you'll need to spawn the robot
# separately (see README "Worlds" section).
WORLD_PRESETS: "dict[str, str]" = {
    "tunnel":          "tunnel.sdf",
    "cave":            "cave.sdf",
    "rubicon":         "https://app.gazebosim.org/abdsemiz/fuel/worlds/Rubicon%20World",
    "tugbot_depot":    "https://app.gazebosim.org/Aiosama/fuel/worlds/tugbot_depot%201",
    "singapore_river": "https://app.gazebosim.org/monkescripts/fuel/worlds/Singapore%20River%20Robot%20X%202026%20world",
}

# https://app.gazebosim.org/{user}/fuel/worlds/{name}
#   → https://fuel.gazebosim.org/1.0/{user}/worlds/{name}
_APP_FUEL_RE = re.compile(
    r"^https?://app\.gazebosim\.org/([^/]+)/fuel/worlds/(.+)$"
)


def _rewrite_fuel_url(url):
    m = _APP_FUEL_RE.match(url)
    if not m:
        return url
    user, name = m.group(1), m.group(2)
    return f"https://fuel.gazebosim.org/1.0/{user}/worlds/{name}"


def resolve_world(world_arg, pkg_share):
    """Map a user-supplied world arg to something gz sim can consume.

    Accepts:
      • a preset short name from WORLD_PRESETS → resolved per the table
      • an absolute or relative SDF path → passed through
      • a Fuel API URL (fuel.gazebosim.org/1.0/.../worlds/...) → passed through
      • a Fuel web URL (app.gazebosim.org/.../fuel/worlds/...) → rewritten
        to the matching fuel.gazebosim.org/1.0 API URL
    """
    value = WORLD_PRESETS[world_arg] if world_arg in WORLD_PRESETS else world_arg
    if value.endswith(".sdf") and not os.path.isabs(value):
        return os.path.join(pkg_share, "worlds", value)
    if value.startswith("http"):
        return _rewrite_fuel_url(value)
    return value


def generate_launch_description():
    pkg_share = get_package_share_directory("explorer_r2_sim")
    bridge_cfg = os.path.join(pkg_share, "config", "bridge.yaml")
    joy_cfg = os.path.join(pkg_share, "config", "joy_teleop.yaml")
    rviz_cfg = os.path.join(pkg_share, "rviz", "sim.rviz")

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    teleop = LaunchConfiguration("teleop")
    joy = LaunchConfiguration("joy")
    verbose = LaunchConfiguration("verbose")

    # Use ros_gz_sim's gz_sim.launch.py to start Gazebo correctly across
    # Fortress / Garden / Harmonic — it picks the right binary for the
    # ros_gz version installed in the container.
    # When gui:=false, pass -s to run Gazebo server-only (no rendering UI).
    server_only = PythonExpression(["'' if '", gui, "' == 'true' else ' -s'"])

    def _make_gz_launch(context, *_args, **_kwargs):
        world_resolved = resolve_world(
            LaunchConfiguration("world").perform(context), pkg_share)
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
                ])
            ),
            launch_arguments={
                # -r: run immediately. -v: verbosity level (0-4).
                "gz_args": [world_resolved, " -r", server_only, " -v ", verbose],
                "on_exit_shutdown": "true",
            }.items(),
        )]

    gz_sim_launch = OpaqueFunction(function=_make_gz_launch)

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
        DeclareLaunchArgument(
            "world", default_value="tunnel",
            description=("World to load. Preset short names: "
                         + ", ".join(WORLD_PRESETS.keys())
                         + ". Or pass a local .sdf path or a Fuel world URL.")),
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
