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
│   └── explorer_r2_sim/        ← cave/tunnel sim + sensor suite + VIO/LIO wiring
├── third_party/
│   ├── open_vins/              ← OpenVINS VIO    (git submodule, optional)
│   └── FAST_LIO/               ← hku-mars/FAST_LIO, LIO (git submodule, optional, ROS2 branch)
├── build/   install/   log/    ← colcon outputs (only populated if you build
│                                 on the host instead of via Docker)
└── README.md                   ← you are here
```

`explorer_r2_sim` carries its own `docker-compose.yml`, `Dockerfile` and
`.env` so it can be built and shipped independently of the workspace root.

## Clone

Both submodules (`third_party/open_vins` for VIO and `third_party/FAST_LIO`
for LIO) are **optional** — the simulator, bridge, RViz, and teleop all
build without either of them. Choose the clone style that fits what you
need today; you can always add the other submodule later.

### Fresh clone with both submodules

```bash
git clone --recurse-submodules git@github.com:behnamasadi/explorer-r2-sim.git ~/ros2_ws
```

### Fresh clone, sim only — add VIO / LIO later as needed

```bash
git clone git@github.com:behnamasadi/explorer-r2-sim.git ~/ros2_ws
```

### Adding (or updating) the submodules later

If you cloned without `--recurse-submodules`, or you initially skipped
one and want to add it now:

```bash
cd ~/ros2_ws

# Add OpenVINS (VIO) only:
git submodule update --init --recursive third_party/open_vins

# Add FAST_LIO (LIO) only — must be --recursive, FAST_LIO has a
# nested ikd-Tree submodule:
git submodule update --init --recursive third_party/FAST_LIO

# Or both at once:
git submodule update --init --recursive

# To pull a newer commit on the submodule's tracked branch:
git submodule update --remote third_party/FAST_LIO   # follows ROS2 branch
git submodule update --remote third_party/open_vins  # follows default branch
```

After initialising a submodule, force a colcon rebuild so the new package
is wired in:

```bash
cd ~/ros2_ws/src/explorer_r2_sim
BUILD=force docker compose up
```

The system image (`docker compose build sim`) doesn't need to be rebuilt
— only the in-container colcon overlay does. The entrypoint detects new
submodules in `third_party/` automatically and adds them to the build.

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

## Running modes

The simulator and the two odometry packages are decoupled. Start with mode 1;
add VIO and/or LIO on top whenever you want — they attach to a running sim
without restarting it.

| Mode | What it gives you | Recipe |
|------|------------------|--------|
| **1. Sim only** (no sensor fusion) | gz sim + bridge + RViz + teleop. Drive the robot, see raw sensor topics. | [Quickstart — Mode 1](#quickstart--mode-1-sim-only) |
| **2. + OpenVINS (VIO)** | Adds OpenVINS to the running sim. Its odometry + trajectory show up in the same RViz. | [Run VIO — Mode 2](#run-vio--mode-2-sim--openvins) |
| **3. + FAST_LIO (LIO)** | Adds FAST_LIO. Its odometry + map show up in the same RViz. | [Run LIO — Mode 3](#run-lio--mode-3-sim--fast_lio) |

> **GNSS (`/navsat`, `sensor_msgs/NavSatFix` @ 10 Hz) and magnetometer
> (`/magnetometer`, `sensor_msgs/MagneticField`)** are bridged out of the
> sim by default — no extra mode needed. To *fuse* them with wheel odom +
> IMU into a single drift-corrected pose estimate, drop in
> `ros-jazzy-robot-localization` and wire `navsat_transform_node` +
> `ekf_node` (see the `robot_localization` docs).

**One RViz layout for everything.** `rviz/sim.rviz` opens with the sim in
Mode 1 and already has displays subscribed to the VIO and LIO topics —
but those displays stay empty in Mode 1 because no estimator is running.
Mode 2 and Mode 3 don't open a second RViz; they just start their
respective nodes, and the existing layout's displays light up as the
topics start publishing. The three modes are additive: you can run Mode
1 + 2 + 3 together and see wheel-odom (red), VIO (green), and LIO (blue)
trails overlaid on the same view.

## Quickstart — Mode 1: sim only

```bash
cd ~/ros2_ws/src/explorer_r2_sim

# Allow X clients from the container.
xhost +local:root

# Build the system image (~2-3 min, apt installs only).
docker compose build sim

# Bring it up: first run also colcon-builds the workspace (~3-5 min,
# longer if open_vins/FAST_LIO submodules are initialised). Subsequent
# runs reuse the build cache and start in seconds.
docker compose up
```

The host's `~/ros2_ws` is bind-mounted into the container at `/ws`. Source
edits show up live; `build/`, `install/`, `log/` end up on the host
(gitignored). To force a clean rebuild after pulling submodule updates:
```bash
BUILD=force docker compose up
```

### Worlds

**The robot and the environment are fully decoupled.** Every world SDF —
both the local presets and any third-party Fuel world — is a pure
environment. `cave.launch.py` always spawns the EXPLORER_R2 dynamically
via `ros_gz_sim create` after gz is up; the spawn pose comes from
`SPAWN_POSES[<preset>]` (in `launch/cave.launch.py`), and you can
override any component via `spawn_x` / `spawn_y` / `spawn_z` / `spawn_yaw`.

The `world:=` argument accepts a **preset short name**, a **local
`.sdf` path**, or a **Gazebo Fuel URL**. Presets:

| Preset             | Source                                                                                                   | Default spawn pose `(x, y, z, yaw)` |
|--------------------|----------------------------------------------------------------------------------------------------------|--------------------------------------|
| `tunnel` (default) | `worlds/tunnel.sdf` — 119-tile SubT tunnel network, hand-assembled from OpenRobotics Fuel tiles          | `(10.0, 0.0, 0.4, 0.0)`              |
| `cave`             | `worlds/cave.sdf` — smaller open SubT cave. Lighter on the GPU                                            | `( 2.0, 0.0, 0.4, 0.0)`              |
| `rubicon`          | [`abdsemiz/Rubicon World`](https://app.gazebosim.org/abdsemiz/fuel/worlds/Rubicon%20World)                | `( 0.0, 0.0, 0.5, 0.0)`              |
| `tugbot_depot`     | [`Aiosama/tugbot_depot 1`](https://app.gazebosim.org/Aiosama/fuel/worlds/tugbot_depot%201)                | `( 0.0, 0.0, 0.5, 0.0)`              |
| `singapore_river`  | [`monkescripts/Singapore River Robot X 2026`](https://app.gazebosim.org/monkescripts/fuel/worlds/Singapore%20River%20Robot%20X%202026%20world) | `( 0.0, 0.0, 0.5, 0.0)`              |

Anything outside the table (a custom local SDF, an arbitrary Fuel URL)
falls back to `(0.0, 0.0, 0.4, 0.0)`. Override with `spawn_x:=… spawn_y:=…
spawn_z:=… spawn_yaw:=…`.

```bash
# Default — tunnel preset, robot at (10, 0, 0.4):
docker compose run --rm sim ros2 launch explorer_r2_sim cave.launch.py

# Different world, default spawn for that world:
docker compose run --rm sim ros2 launch explorer_r2_sim cave.launch.py world:=cave

# Third-party Fuel world — robot is spawned automatically at the
# fallback pose unless you supply spawn coordinates:
docker compose run --rm sim ros2 launch explorer_r2_sim cave.launch.py \
  world:=rubicon spawn_x:=3.0 spawn_y:=1.5 spawn_z:=0.6

# Any Fuel URL works the same way (app.gazebosim.org → fuel API rewrite
# is handled by the launch):
docker compose run --rm sim ros2 launch explorer_r2_sim cave.launch.py \
  world:=https://app.gazebosim.org/OpenRobotics/fuel/worlds/Empty
```

Set `spawn_robot:=false` to load the world without any robot at all
(useful when you want to inspect a world or spawn a different robot
from another launch).

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

## Run VIO — Mode 2: sim + OpenVINS

Prerequisite: the `open_vins` submodule is initialised (see
[Clone](#clone)) and you have a running sim from [Mode 1](#quickstart--mode-1-sim-only).

OpenVINS is environment-agnostic — it consumes `/imu` and
`/rs_front/image` regardless of which world the rover is in. It publishes
to the `/ov_msckf/*` topics that the sim's RViz layout already has
displays for, so **don't open a second RViz** — the existing one will
populate as soon as the VIO node starts.

> **Why `bash -ic`?** `docker compose exec` bypasses the container's
> entrypoint, so it doesn't inherit the sourced ROS environment. The
> image's `/root/.bashrc` does source ROS + the workspace overlay, but
> bash only reads `.bashrc` for interactive shells — hence `bash -ic`
> (interactive, command). Equivalent: drop into `docker compose exec
> sim bash` and run commands there, since the interactive shell
> auto-sources.

`compose exec` needs the `sim` container to already be running, so this
is a two-terminal recipe (both terminals `cd` into the compose dir):

```bash
# Terminal 1 — bring up the sim (mode 1, leave running):
cd ~/ros2_ws/src/explorer_r2_sim
docker compose up

# Terminal 2 — attach OpenVINS to the running sim:
cd ~/ros2_ws/src/explorer_r2_sim
docker compose exec sim bash -ic "ros2 launch explorer_r2_sim vio.launch.py"
```

Prefer a single terminal? Use detached mode:
```bash
cd ~/ros2_ws/src/explorer_r2_sim
docker compose up -d
docker compose exec sim bash -ic "ros2 launch explorer_r2_sim vio.launch.py"
docker compose logs -f sim    # tail sim logs in this terminal
```

## Run LIO — Mode 3: sim + FAST_LIO

Prerequisite: the `FAST_LIO` submodule is initialised (see
[Clone](#clone)) and you have a running sim from [Mode 1](#quickstart--mode-1-sim-only).

FAST_LIO is environment-agnostic — it consumes `/lidar/points` + `/imu`
and publishes its own odometry, path, and map. The sim's RViz layout
already has displays for those topics, so no second RViz is needed.

```bash
# Terminal 1 — sim already running from Mode 1.

# Terminal 2 — attach FAST_LIO to the running sim:
cd ~/ros2_ws/src/explorer_r2_sim
docker compose exec sim bash -ic "ros2 launch explorer_r2_sim lio.launch.py"
```

Mode 2 and Mode 3 can run together — open a third terminal and launch the
other one. They don't share topics.

## Evaluating VIO / LIO against ground truth

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

The single `rviz/sim.rviz` layout overlays three things in the same orbit
view (the VIO displays sit empty in Mode 1 and light up when you start
Mode 2):

- **Red arrow**  → `/model/explorer_r2/odometry` — wheel odometry from the
  DiffDrive plugin (drifts during turns / wheel slip; representative of
  what real encoders give you).
- **Green arrow + green path** → `/ov_msckf/odomimu` and
  `/ov_msckf/pathimu` — OpenVINS' MSCKF estimate.
- **Ground truth** — comes through `/ground_truth/pose` (a `TFMessage`
  carrying every dynamic model's true pose). Add a TF display in RViz
  pinned to frame `explorer_r2` to see the absolute true position.

Bring up the sim + OpenVINS using the recipe in
[Run VIO — Mode 2](#run-vio--mode-2-sim--openvins) above, then drive the
robot a bit (joystick, keyboard, or rqt_robot_steering). After ~10
seconds of motion the green VIO trail should snake along behind the red
wheel-odom trail — divergence between green and red over time tells you
how much wheel odometry is drifting, while divergence between green and
the ground-truth TF tells you how much VIO is drifting.

### Quantitative comparison (`evo`)

Three scripts in `scripts/` automate the record-and-evaluate loop:

| Script              | What it does                                                                                                       |
|---------------------|--------------------------------------------------------------------------------------------------------------------|
| `gt_to_path.py`     | Subscribes to `/ground_truth/pose` (TFMessage), extracts the `explorer_r2` transform, republishes `/ground_truth/path` (`nav_msgs/Path`) so `evo` can ingest it. |
| `record_run.sh tag` | Records a bag with `/ground_truth/path`, `/model/explorer_r2/odometry`, `/ov_msckf/*`, `/Odometry`, `/path`, `/cmd_vel` to `~/.local/share/evo/<tag>/<UTC>/`. |
| `eval.sh <bag-dir>` | Converts each trajectory topic to TUM, runs `evo_ape` (APE) and `evo_rpe` (RPE) per estimator vs ground truth, writes per-estimator PNG plots and a `summary.txt`. |

Install `evo` once (inside the container or on the host — your call):
```bash
pip install evo --user
```

Three-terminal workflow:

```bash
# Terminal 1 — sim:
docker compose up

# Terminal 2 — VIO + LIO + ground-truth-path publisher:
docker compose exec sim bash -ic "ros2 launch explorer_r2_sim vio.launch.py" &
docker compose exec sim bash -ic "ros2 launch explorer_r2_sim lio.launch.py" &
docker compose exec sim bash -ic "ros2 run explorer_r2_sim gt_to_path.py"

# Terminal 3 — drive a scenario + record:
docker compose exec sim bash -ic "ros2 run explorer_r2_sim record_run.sh slow_loop"
# Drive the joystick / keyboard to execute the slow_loop profile.
# Ctrl-C when done.

# Then evaluate:
docker compose exec sim bash -ic \
  "ros2 run explorer_r2_sim eval.sh ~/.local/share/evo/slow_loop/<UTC>"
```

`scenarios/README.md` documents five canonical drive profiles (`static`,
`slow_loop`, `fast_straight`, `sharp_turn`, `featureless_wall`) and rough
sanity bands for what counts as a healthy APE RMSE on this rig.

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

| Param            | Value                                      |
|------------------|--------------------------------------------|
| width            | 320                                        |
| height           | 240                                        |
| fx, fy           | 277.1, 277.1                               |
| cx, cy           | 160.5, 120.5                               |
| H-FOV            | 1.0472 rad (60°)                           |
| Distortion model | radial-tangential (`radtan`)                |
| k1, k2           | −0.02, 0.003 (mild barrel; RealSense-like) |
| p1, p2           | 0, 0                                       |
| Image noise σ    | 0.01 (norm. 0–1; ≈ 2.5 grey levels of 255) |

These distortion + noise values live in `models/explorer_r2/model.sdf`
(the gz sensor) and `config/openvins/kalibr_imucam_chain.yaml` (what
OpenVINS uses to undistort). **Keep them in sync** when tuning — a
mismatch shows up as systematic VIO drift on straight-line drives.

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

### LIO — FAST_LIO

LIO ships as an optional submodule at `third_party/FAST_LIO/`, cloned
from [`hku-mars/FAST_LIO`](https://github.com/hku-mars/FAST_LIO) and
pinned to its `ROS2` branch. The colcon package is `fast_lio`. It
subscribes to `/lidar/points` + `/imu` and publishes its own odometry +
map, so it's environment-agnostic exactly like VIO.

For LiDAR-IMU odometry you need the **lidar-IMU** SE(3). For our rig:

```
T_imu_lidar:
  - [1, 0, 0, 0.000]   # lidar 1.05 m above base_link
  - [0, 1, 0, 0.000]   # no rotation
  - [0, 0, 1, 1.050]
  - [0, 0, 0, 1]
```

YAML for `fast_lio` (`config/lio.yaml` when wired):

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

Drop a `lio.launch.py` next to `vio.launch.py` that loads the config and
remaps the topics — the rig already publishes everything `fast_lio` needs.

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

# 3. Teleop + GUI tools (xterm is needed for cave.launch.py teleop:=true):
sudo apt install -y \
  xterm \
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

# 6. (Optional) FAST_LIO for LIO — submodule already on the ROS2 branch:
git submodule update --init --recursive third_party/FAST_LIO
sudo apt install -y libpcl-dev ros-jazzy-pcl-conversions ros-jazzy-pcl-ros
ln -sf ../third_party/FAST_LIO ~/ros2_ws/src/fast_lio

# 7. Build the workspace:
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select explorer_r2_sim ov_core ov_init ov_msckf fast_lio
source install/setup.bash

# 8. Run:
ros2 launch explorer_r2_sim cave.launch.py
```

For Rolling + Ionic instead of Jazzy + Harmonic, swap `jazzy` → `rolling`
and `gz-harmonic` → `gz-ionic`. Plugin filenames stay the same
(`gz-sim-*-system`).

## License

BSD-3-Clause for `explorer_r2_sim`. OpenVINS is GPLv3 (see
`third_party/open_vins/LICENSE`).
