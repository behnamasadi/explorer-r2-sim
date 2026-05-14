# ros2_ws

ROS 2 + Gazebo simulation workspace for the EXPLORER_R2 rover. Everything
runs in Docker — no host install of ROS or Gazebo is required.

## Versions

| Component             | Version                                       |
|-----------------------|-----------------------------------------------|
| ROS 2 distro          | **Jazzy** (Ubuntu 24.04) — LTS until May 2029 |
| Gazebo                | **Harmonic** (`gz sim`) — LTS until Sep 2028  |
| ros_gz bridge         | `ros-jazzy-ros-gz-bridge`                     |
| Default ROS_DOMAIN_ID | `42`                                          |
| Container base        | `osrf/ros:jazzy-desktop-full`                 |

### Switching ROS / Gazebo versions

`src/explorer_r2_sim/env/` ships two presets, both currently-supported
ROS 2 + Gazebo pairings:

| Preset                       | ROS 2   | Gazebo   | OpenVINS                           | SDF plugins        |
|------------------------------|---------|----------|------------------------------------|--------------------|
| `env/jazzy-harmonic.env`     | Jazzy   | Harmonic | needs .h→.hpp patch (auto-applied) | `gz-sim-*-system`  |
| `env/rolling-ionic.env`      | Rolling | Ionic    | builds clean                       | `gz-sim-*-system`  |

To switch:

```bash
cd ~/ros2_ws/src/explorer_r2_sim

# 1. Pick a preset (this is the one the compose file reads):
ln -sf env/rolling-ionic.env .env

# 2. Rebuild the image:
docker compose build sim
```

Both presets are on the modern `gz-sim-*-system` plugin family, so no SDF
rewriting is needed when flipping between them. Older Ignition Gazebo
combinations (Fortress and earlier) are intentionally not supported here —
upstream OSRF support for them is winding down.

## Layout

```
ros2_ws/
├── src/
│   └── explorer_r2_sim/        ← cave/tunnel sim + sensor suite + VIO wiring
├── third_party/
│   └── open_vins/              ← OpenVINS VIO (git submodule, optional)
├── build/   install/   log/    ← colcon outputs (only populated if you build
│                                 on the host instead of via Docker)
└── README.md                   ← you are here
```

`explorer_r2_sim` carries its own `docker-compose.yml`, `Dockerfile` and
`.env` so it can be built and shipped independently of the workspace root.

## Clone

```bash
# Fresh clone — pulls OpenVINS into third_party/ in one shot:
git clone --recurse-submodules <repo-url> ~/ros2_ws

# Already cloned without submodules? Initialise OpenVINS now:
cd ~/ros2_ws && git submodule update --init --recursive
```

OpenVINS is optional. If you only need the simulator + bridge + RViz + teleop,
skip the submodule init and the sim will build without VIO support.

## explorer_r2_sim — what it is

CMU Team Explorer's **EXPLORER_R2** SubT-challenge rover (4-wheel
rocker-bogie) inside a Gazebo SubT cave or 119-tile tunnel network, with a
ROS 2 ↔ GZ bridge, RViz layouts, joystick teleop, and OpenVINS VIO.

### What flows through the system

```
              ┌────────────────────────── Gazebo Harmonic ──────────────────────────┐
              │                                                                    │
              │   worlds/tunnel.sdf  (or worlds/cave.sdf)                          │
              │                                                                    │
              │   ┌── EXPLORER_R2  (models/explorer_r2/model.sdf) ──┐              │
              │   │   • IMU @ 250 Hz       (with bias model)        │              │
              │   │   • gpu_lidar           Ouster OS0-32, top mast │              │
              │   │   • RGBD camera × 4     rs_front/back/left/right│              │
              │   │   • magnetometer        (added)                 │              │
              │   │   • NavSat (GPS)        (added)                 │              │
              │   │   • DiffDrive plugin    /cmd_vel → wheels       │              │
              │   │   • PosePublisher       sensor TF               │              │
              │   └─────────────────────────────────────────────────┘              │
              └────────────────────┬───────────────────────────────────────────────┘
                                   │  GZ Transport
                                   ▼
                  ┌──────── ros_gz_bridge ─────────┐  (config/bridge.yaml)
                  └────────────────┬───────────────┘
                                   │  ROS 2 / DDS
        ┌──────────────────────────┼─────────────────────────────┐
        │                          │                             │
        ▼                          ▼                             ▼
   /cmd_vel               /lidar/points                  /ov_msckf/odomimu
   /imu                   /lidar/scan                    /ov_msckf/pathimu
   /magnetometer          /rs_front/{image,depth,        ↑
   /navsat                          points,camera_info}  │
   /tf  (single tree)     /model/explorer_r2/odometry    │   OpenVINS (mono VIO)
   /ground_truth/pose                                    │   eats /imu + /rs_front/image
        │                                                │
        │              ┌─── joy_node ───────────┐        │
        ├─────────────►│ teleop_twist_joy_node ├────────►│
        │              └────────────────────────┘        │
        │                                                │
        ▼                                                │
   RViz2  (rviz/cave.rviz, rviz/vio.rviz)  ◄─────────────┘
```

### Topic table (most useful subset)

| ROS topic                      | type                            | source           |
|--------------------------------|---------------------------------|------------------|
| `/cmd_vel`                     | `geometry_msgs/Twist`           | joystick / teleop |
| `/imu`                         | `sensor_msgs/Imu` @ 250 Hz       | gz IMU           |
| `/magnetometer`                | `sensor_msgs/MagneticField`      | gz mag           |
| `/navsat`                      | `sensor_msgs/NavSatFix` @ 10 Hz  | gz NavSat        |
| `/lidar/points`                | `sensor_msgs/PointCloud2` @ 15 Hz | front_laser     |
| `/lidar/scan`                  | `sensor_msgs/LaserScan` @ 15 Hz  | front_laser      |
| `/rs_front/image`              | `sensor_msgs/Image` @ 30 Hz      | rs_front (RGBD)  |
| `/rs_front/depth`              | `sensor_msgs/Image`              | rs_front         |
| `/rs_front/points`             | `sensor_msgs/PointCloud2`        | rs_front         |
| `/rs_front/camera_info`        | `sensor_msgs/CameraInfo`         | rs_front         |
| `/model/explorer_r2/odometry`  | `nav_msgs/Odometry` @ 50 Hz      | DiffDrive (drifty) |
| `/tf`                          | `tf2_msgs/TFMessage`             | DiffDrive + PosePublisher |
| `/ground_truth/pose`           | `tf2_msgs/TFMessage`             | scene_broadcaster (true pose) |
| `/ov_msckf/odomimu`            | `nav_msgs/Odometry`              | OpenVINS VIO     |
| `/ov_msckf/pathimu`            | `nav_msgs/Path`                  | OpenVINS VIO     |
| `/ov_msckf/points_msckf`       | `sensor_msgs/PointCloud2`        | OpenVINS features |
| `/joy`                         | `sensor_msgs/Joy`                | Logitech F310    |

## Quickstart — explorer_r2_sim

```bash
cd ~/ros2_ws/src/explorer_r2_sim

# Allow X clients from the container.
xhost +local:root

# Build (~3-5 min first time; openvins is the slowest step).
docker compose build sim

# Bring it up: gz sim + bridge + RViz + joy + teleop_twist_joy.
docker compose up
```

### How to drive the robot

Three independent control paths — pick whichever you have at hand. All of
them publish on the same `/cmd_vel` topic.

#### 1. Joystick — Logitech F310 (auto-loaded by `cave.launch.py`)

| Control                  | What it does            |
|--------------------------|-------------------------|
| **LB** (left bumper)     | **Deadman** — hold to allow any motion. Release = instant stop. |
| **RB** (right bumper)    | **Turbo** — hold for ×3 speed |
| **Left stick — Y axis**  | Forward / back (push up = forward) |
| **Right stick — X axis** | Yaw left / right        |

Limits: 1.0 m/s linear, 1.0 rad/s yaw normal; 3.0 m/s, 3.0 rad/s with turbo.

> If the joystick was unplugged after the container started, restart it:
> `docker compose restart sim` (joy_node doesn't auto-reconnect).

#### 2. Keyboard — `teleop_twist_keyboard` (opt in)

```bash
docker compose run --rm sim \
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Key map (printed on stdout when it starts):
- `i` / `,` — forward / back
- `j` / `l` — yaw left / right
- `u` `o` `m` `.` — diagonals
- `k` — stop
- `q`/`z` — increase / decrease both scales by 10%

#### 3. GUI sliders — `rqt_robot_steering` (opt in)

A small window with two sliders (linear + angular). Useful when you want
to send a constant velocity hands-free:

```bash
# Add it to the main launch:
docker compose down && docker compose run --rm sim \
  ros2 launch explorer_r2_sim cave.launch.py rqt_steering:=true
# Or run it standalone in another terminal:
docker compose exec sim rqt --standalone rqt_robot_steering
```

In the rqt window set the topic to `/cmd_vel` (default) and slide.

### Run VIO

```bash
docker compose exec sim ros2 launch explorer_r2_sim vio.launch.py rviz:=true
```

Switch from the tunnel to the smaller cave:
```bash
docker compose run --rm sim \
  ros2 launch explorer_r2_sim cave.launch.py \
    world:=/ws/install/explorer_r2_sim/share/explorer_r2_sim/worlds/cave.sdf
```

## Seeing OpenVINS output + comparing against ground truth

### What OpenVINS publishes

| Topic                      | Type                       | What it is                          |
|----------------------------|----------------------------|-------------------------------------|
| `/ov_msckf/odomimu`        | `nav_msgs/Odometry`        | Filter pose + velocity (IMU frame)  |
| `/ov_msckf/poseimu`        | `geometry_msgs/PoseStamped`| Same, pose only                     |
| `/ov_msckf/pathimu`        | `nav_msgs/Path`            | Trajectory line (visualizable)      |
| `/ov_msckf/points_msckf`   | `sensor_msgs/PointCloud2`  | Tracked MSCKF features              |
| `/ov_msckf/points_slam`    | `sensor_msgs/PointCloud2`  | SLAM features in state              |
| `/ov_msckf/trackhist`      | `sensor_msgs/Image`        | Feature tracks overlaid on the image |
| `/ov_msckf/loop_pose`      | `geometry_msgs/PoseWithCov`| Latest loop closure pose            |

### Live visual comparison in RViz

`rviz/vio.rviz` overlays three things in the same orbit view:

- **Red arrow**  → `/model/explorer_r2/odometry` — wheel odometry from the
  DiffDrive plugin (drifts during turns / wheel slip; representative of
  what real encoders give you).
- **Green arrow + green path** → `/ov_msckf/odomimu` and
  `/ov_msckf/pathimu` — OpenVINS' MSCKF estimate.
- **Ground truth** — comes through `/ground_truth/pose` (a `TFMessage`
  carrying every dynamic model's true pose). Add a TF display in RViz
  pinned to frame `explorer_r2` to see the absolute true position.

```bash
# Bring up the sim (terminal 1):
cd ~/ros2_ws/src/explorer_r2_sim && docker compose up

# Start OpenVINS + its RViz overlay (terminal 2):
docker compose exec sim ros2 launch explorer_r2_sim vio.launch.py rviz:=true
```

Drive the robot a bit (joystick, keyboard, or rqt_robot_steering). After
~10 seconds of motion the green VIO trail should snake along behind the
red wheel-odom trail — divergence between green and red over time tells
you how much wheel odometry is drifting, while divergence between green
and the ground-truth TF tells you how much VIO is drifting.

### Quantitative comparison (rosbag → evo)

```bash
# Record both estimates + ground truth while driving (terminal 3):
docker compose exec sim bash -c "
  source /ws/install/setup.bash &&
  ros2 bag record -o /ws/vio_run \
    /ov_msckf/odomimu \
    /model/explorer_r2/odometry \
    /ground_truth/pose
"

# When done, install evo on the host or in a python venv:
pip install evo --user

# Convert /ground_truth/pose (TFMessage) into a Path topic if needed, or
# extract directly with evo's bag interface:
evo_traj bag2 ~/ros2_ws/src/explorer_r2_sim/vio_run \
  --topic_names /ov_msckf/odomimu /model/explorer_r2/odometry \
  --plot --plot_mode xy
```

For the absolute-pose-error (APE) metric, you'll need both trajectories
on the same time base — the simplest workflow is:
1. Convert each topic to TUM format (`evo_traj bag2 … --save_as_tum`).
2. Pick the ground-truth TUM file as `--ref` and run
   `evo_ape tum gt.tum est.tum -va --plot`.

### Quick sanity check (no evo, just terminal)

```bash
# Live distance between VIO estimate and ground truth (publishes /vio_error):
docker compose exec sim bash -c '
  source /ws/install/setup.bash &&
  python3 - <<PY
import rclpy, math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

class Cmp(Node):
  def __init__(self):
    super().__init__("vio_cmp")
    self.gt = None; self.est = None
    self.create_subscription(TFMessage, "/ground_truth/pose", self.cb_gt, 50)
    self.create_subscription(Odometry, "/ov_msckf/odomimu", self.cb_est, 50)
    self.create_timer(1.0, self.report)
  def cb_gt(self, m):
    for t in m.transforms:
      if t.child_frame_id == "explorer_r2":
        self.gt = t.transform.translation
  def cb_est(self, m):
    self.est = m.pose.pose.position
  def report(self):
    if self.gt and self.est:
      d = math.dist((self.gt.x, self.gt.y, self.gt.z),
                    (self.est.x, self.est.y, self.est.z))
      self.get_logger().info(f"||gt - vio|| = {d:.3f} m")

rclpy.init(); n = Cmp(); rclpy.spin(n)
PY
'
```

This prints the live position error in metres every second. A healthy
mono-VIO run on this rig stays under a metre of drift over a few-tens-of-
metre traverse.

## Exact sim calibration (cam ↔ IMU ↔ LiDAR)

Because everything comes from the SDF, we have **perfect** ground-truth
extrinsics — feed them straight into VIO, LIO, or any sensor-fusion node
without running calibration. All poses are relative to `base_link`
(IMU = base_link, no rotation, no offset).

### Sensor positions on `base_link`

| Sensor          | Pose (x, y, z, roll, pitch, yaw)            | Notes                                      |
|-----------------|---------------------------------------------|--------------------------------------------|
| `imu_sensor`    | `(0.000, 0.000, 0.000, 0, 0, 0)`            | Origin = base_link                         |
| `magnetometer`  | `(0.000, 0.000, 0.000, 0, 0, 0)`            | Same link as IMU                           |
| `navsat`        | `(0.000, 0.000, 0.000, 0, 0, 0)`            | Same link as IMU                           |
| `front_laser`   | `(0.000, 0.000, 1.050, 0, 0, 0)`            | Mast-mounted; clears the payload box       |
| `rs_front`      | `(0.565, 0.000, 0.245, 0, 0, 0)`            | RGBD, forward                              |
| `rs_back`       | `(0.250, 0.000, 0.432, 0, 0, π)`            | RGBD, aft                                  |
| `rs_left`       | `(0.365, 0.133, 0.426, 0, 0, +π/2)`         | RGBD, port                                 |
| `rs_right`      | `(0.365,-0.133, 0.426, 0, 0, -π/2)`         | RGBD, starboard                            |

(Gz frame convention: x forward, y left, z up. Optical-frame convention
adds a -π/2 about X then -π/2 about Z to convert from gz-cam → opt.)

### IMU noise model

Pulled directly from `models/explorer_r2/model.sdf`:

| Quantity                       | Value (per axis)                           |
|--------------------------------|--------------------------------------------|
| Update rate                    | 250 Hz                                     |
| Gyro white noise σ             | 2.0 × 10⁻⁴ rad/s                           |
| Gyro bias random walk          | 8.0 × 10⁻⁷ rad/s²                          |
| Accel white noise σ            | 1.0 × 10⁻² m/s²                            |
| Accel bias random walk         | 1.0 × 10⁻³ m/s³                            |

These are converted to continuous-time noise *densities* in
`config/openvins/kalibr_imu_chain.yaml`:

```yaml
gyroscope_noise_density:        1.27e-5
gyroscope_random_walk:          8.0e-7
accelerometer_noise_density:    6.32e-4
accelerometer_random_walk:      1.0e-3
```

### Camera intrinsics (rs_front)

| Param  | Value          |
|--------|----------------|
| width  | 320            |
| height | 240            |
| fx, fy | 277.1, 277.1   |
| cx, cy | 160.5, 120.5   |
| H-FOV  | 1.0472 rad (60°) |
| Distortion | none (perfect pinhole) |

### LiDAR (front_laser) — Ouster OS0-32 spec

| Param        | Value                |
|--------------|----------------------|
| Type         | gpu_lidar            |
| Update rate  | 15 Hz                |
| Horizontal   | 1024 samples × 2π    |
| Vertical     | 16 samples × ±15°    |
| Range        | 0.05 – 100 m         |
| Range noise σ| 0.01 m               |

### Using these in OpenVINS (already wired)

`config/openvins/kalibr_imucam_chain.yaml` contains the cam-to-IMU
SE(3) for `rs_front`:

```yaml
T_imu_cam:
  - [ 0.0,  0.0,  1.0,  0.565]
  - [-1.0,  0.0,  0.0,  0.000]
  - [ 0.0, -1.0,  0.0,  0.245]
  - [ 0.0,  0.0,  0.0,  1.000]
```

(Rows: rotation block converts optical→base_link gz-frame; last column is
the camera's translation in base_link.)

### LIO recommendation

Three solid options, ranked by what I'd reach for first on this stack:

| Package    | Strengths                              | Weakness                          |
|------------|----------------------------------------|-----------------------------------|
| **DLIO**   | Modern, ROS 2 native, very fast & robust, no IMU pre-integration tuning needed | Cave-style featureless walls can stress it |
| FAST-LIO2  | Slightly more accurate in feature-rich scenes, well-documented | ROS 2 fork is community-maintained |
| LIO-SAM    | Most mature, integrates GPS easily     | Heavier; tuning matters; older code |

**Recommendation: DLIO.** Your earlier `tunnel` cheat-sheet already had a
`dlio.launch.py` line so you've used it before; it's a drop-in fit for
this rig (publishes on `/lidar/points` + `/imu`, exactly what DLIO
expects).

For LiDAR-IMU odometry you need the **lidar-IMU** SE(3). For our rig:

```
T_imu_lidar:
  - [1, 0, 0, 0.000]   # lidar 1.05 m above base_link
  - [0, 1, 0, 0.000]   # no rotation
  - [0, 0, 1, 1.050]
  - [0, 0, 0, 1]
```

YAML for FAST-LIO2 (`config/lio.yaml` when added):

```yaml
mapping:
  imu_topic: "/imu"
  lidar_topic: "/lidar/points"
  extrinsic_T: [ 0.0, 0.0, 1.05 ]
  extrinsic_R: [ 1, 0, 0,  0, 1, 0,  0, 0, 1 ]
  acc_cov: 0.4
  gyr_cov: 0.02
  b_acc_cov: 0.001
  b_gyr_cov: 8.0e-7
```

When the LIO package lands, drop a `lio.launch.py` next to `vio.launch.py`
and remap the topics — the rig already publishes everything LIO needs.

## Native install (no Docker)

The Docker image is the recommended path because it pins every version. If
you'd rather run on bare metal, replicate what the Dockerfile does:

```bash
# 1. ROS 2 Jazzy on Ubuntu 24.04 (one-shot installer):
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop-full python3-colcon-common-extensions

# 2. Gazebo Harmonic + ros_gz bridge:
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install -y gz-harmonic \
  ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-image

# 3. Teleop + GUI tools:
sudo apt install -y \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-joy ros-jazzy-teleop-twist-joy \
  ros-jazzy-rqt-robot-steering ros-jazzy-rqt ros-jazzy-rqt-graph

# 4. OpenVINS deps (only if you want VIO):
sudo apt install -y libceres-dev libeigen3-dev libboost-all-dev \
  ros-jazzy-image-transport ros-jazzy-image-transport-plugins \
  ros-jazzy-tf2-geometry-msgs ros-jazzy-cv-bridge

# 5. Pull OpenVINS in as a submodule (with the Jazzy header patch):
cd ~/ros2_ws
git submodule update --init --recursive third_party/open_vins
sed -i 's|image_transport/image_transport\.h|image_transport/image_transport.hpp|' \
  third_party/open_vins/ov_msckf/src/ros/{ROS2Visualizer,ROS1Visualizer}.h
sed -i 's|tf2_geometry_msgs/tf2_geometry_msgs\.h|tf2_geometry_msgs/tf2_geometry_msgs.hpp|' \
  third_party/open_vins/ov_msckf/src/ros/{ROSVisualizerHelper,ROS2Visualizer,ROS1Visualizer}.h
sed -i 's|cv_bridge/cv_bridge\.h|cv_bridge/cv_bridge.hpp|' \
  third_party/open_vins/ov_msckf/src/ros/{ROS1Visualizer,ROS2Visualizer}.h \
  third_party/open_vins/ov_core/src/test_tracking.cpp
ln -sf ../third_party/open_vins ~/ros2_ws/src/open_vins   # let colcon see it

# 6. Build the workspace:
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select explorer_r2_sim ov_core ov_init ov_msckf
source install/setup.bash

# 7. Run:
ros2 launch explorer_r2_sim cave.launch.py
```

For Rolling + Ionic instead of Jazzy + Harmonic, swap `jazzy` → `rolling`
and `gz-harmonic` → `gz-ionic`. Plugin filenames stay the same
(`gz-sim-*-system`).

## Troubleshooting

- **RViz says "no transform" / orange status**: confirm `/tf` is being
  published (`ros2 topic hz /tf` should be ~50 Hz). If not, check that gz
  is running (`docker compose ps`).
- **Pointcloud "not available"**: usually a fixed-frame mismatch. The
  default fixed frame is `explorer_r2/odom`.
- **Wheels sinking into the floor**: the world's vehicle spawn pose must
  set `Z=0.4` because `<include><pose>` overrides the model's internal
  `0.398` Z offset.
- **Lidar sees the robot itself**: front_laser sits at `z=1.05` above
  `base_link` to clear the sensor payload — if you lower it below ~0.65
  it'll raycast into the chassis.
- **OpenVINS won't build on Jazzy**: header rename (`.h` → `.hpp`) for
  `image_transport`, `cv_bridge`, `tf2_geometry_msgs`. The Dockerfile
  patches them with `sed` after cloning — see
  `src/explorer_r2_sim/docker/sim.Dockerfile`.
- **Joystick not detected**: `ls /dev/input/js*` on the host. The
  compose file bind-mounts `/dev/input:/dev/input`. Container must run
  with `privileged: true` (default).

## License

BSD-3-Clause for `explorer_r2_sim`. OpenVINS is GPLv3 (see
`third_party/open_vins/LICENSE`).
