# Analysis: VIO vs LIO vs GT on a 155-s forward-drive run

A case study from `runs/run_20260514T154145Z/` — a real bag the user
recorded while teleoperating the rover roughly straight forward across
the tunnel world. This file is the worked example referenced from the
[Quantitative comparison](../README.md#quantitative-comparison-via-rosbag--evo)
section of the README. The point isn't the specific numbers; it's the
*methodology* — how to spot scale drift, init-induced yaw error, and
mono-VIO failure modes from the raw trajectory data.

## Setup

| | |
|---|---|
| Bag duration | 155 s |
| Topics recorded | `/imu`, `/rs_front/image`, `/lidar/{points,points_lio}`, `/ground_truth/{pose,odom,path}`, `/ov_msckf/{odomimu,pathimu}`, `/Odometry`, `/path`, `/cmd_vel`, `/tf`, `/tf_static` |
| Drive profile | Mostly straight forward, then some curving |
| World | `tunnel` (default) |
| Mode | 1 — full stack via `cave.launch.py` |

## Results

```
Name      N   start  (x, y, z)            end    (x, y, z)            path length
------  ----  -------------------------  --------------------------  -----------
GT      6892  (-0.01, -0.00, -0.01)      (+30.57, +0.53,  -2.69)        40.27 m
LIO      499  (-0.03, -0.00, -0.01)      (+30.46, +0.43,  -3.31)        40.61 m
VIO     9790  (+0.02, -0.00, +0.07)      (-164.62, +6.17, -4.49)       290.23 m
```

```
LIO  end-position error vs GT:  (-0.12, -0.10, -0.62) m   |err| =   0.64 m
VIO  end-position error vs GT:  (-195.19, +5.65, -1.80) m  |err| = 195.28 m
```

LIO tracks GT to within 0.64 m over a 40 m drive — that's 1.5 % final
drift, mostly a 0.62 m Z error. **LIO is healthy.**

VIO accumulates 195 m of end-point error and reports 290 m of path
length while the rover physically travelled 40 m (7× over-estimate of
total motion). **VIO is fundamentally broken in this configuration.**

## Why VIO fails — three compounding causes

### 1. Mono camera + forward-driving = worst-case parallax

A monocular VIO algorithm needs **parallax** to triangulate features
and estimate their depth. When the camera's optical axis is aligned
with the direction of motion (which it is here: `rs_front` looks
forward, the rover drives forward), features near the optical center
get **no parallax** — they just expand in the image as the camera
approaches them.

The information content for depth recovery is concentrated in points
near the image edges, and those points are sparse and fall out of view
quickly. The filter can correctly track those features but it can't
tell whether they're 5 m away or 50 m away, so **scale is very weakly
observable**.

Over a 40 m drive that compounds into a ~7× scale error. The 290 m
path-length VIO reports is consistent with the filter believing each
metre travelled is actually ~7 m.

This is a textbook mono-VIO limitation, not a bug in OpenVINS. Stereo
VIO (or RGBD VIO) fixes it because depth becomes directly observable.

### 2. Init contaminated by the spawn drop

From OpenVINS' init log at startup:

```
[init]: successful initialization in 0.0003 seconds
[init]: orientation = 0.0105, 0.0000, 0.9999, 0.0000      ← (x, y, z, w) = 180° yaw
[init]: bias gyro  = -0.0001, -0.0013,  0.0000
[init]: velocity   =  0.0000,  0.0000,  0.0000
[init]: bias accel = -0.0001, -0.0000, -0.0066
```

Two things are wrong:

- **Init completed in 0.3 milliseconds.** That's not a measurement
  window — that's OpenVINS catching the IMU jerk from the rover's
  spawn drop (model spawns at `z=0.4`, settles around `z≈0.25`, with a
  brief impact transient on the wheels). The "static" init triggered
  on motion data, not on stationary data, so the gravity vector
  estimate is whatever direction the impact happened to push the IMU.
- **`orientation = (0.01, 0, 1.00, 0)`** in `(x, y, z, w)` quaternion
  order is a 180° rotation about Z — i.e. **OpenVINS thinks `global`
  is yawed 180° relative to the rover body at startup**. This is the
  *unobservable yaw* problem: IMU-only static init can recover pitch
  and roll from the gravity vector (gravity has a known direction in
  the world), but yaw is rotation around the gravity axis and gravity
  carries no information about it. OpenVINS picked an arbitrary value
  and happened to land on ~180° this run. Next run it might be 35°
  or −90°.

End result: drive forward → robot moves world +X → VIO reports motion
in `-X_global`. The green RViz trail going the "wrong direction" is
this yaw offset, **not** a code bug.

### 3. Cave environment is texture-poor

SubT cave walls have *some* texture (cracks, gradient shading) but are
much less feature-rich than a typical outdoor or indoor scene. Combined
with the 320×240 image at 30 Hz and OpenVINS' default `fast_threshold:
20`, feature tracks get lost frequently. Between tracking events the
filter propagates on IMU only, and IMU bias drift compounds quickly.

## Confirming the failure mode

Three quick checks distinguish the mono-VIO failures above from
"VIO has a real bug":

1. **Path length ratio.** VIO/GT ≈ 7 → scale drift. If it were close to
   1.0, the issue would be pure rotation. If it were 0.5–2.0, it'd be
   moderate drift. 7 is the smoking gun.
2. **Final pose sign flip.** VIO `x = −164 m` while GT `x = +30 m` →
   the trajectory is in a rotated frame. Run `evo_ape -a` (Umeyama
   alignment) on the same bag and the aligned APE drops dramatically
   if it's *just* yaw. If aligned APE is also huge, then VIO has
   scale drift on top of the rotation (which is what's happening
   here).
3. **LIO comparison.** LIO's 0.64 m error over the same drive
   confirms the rover, the sim, the bridge, and the timing are all
   fine. The issue is specific to the VIO algorithm + this motion
   pattern + this camera.

## Mono-VIO tuning already applied

These changes shipped to `config/openvins/estimator_config.yaml` and
`launch/vio.launch.py` to address the three failure modes above. They
**only help mono VIO** — for stereo VIO (the real fix) we'd still need
to wire up a second camera. Apply or revert by flipping a single key
in each case.

| Where                              | Knob                | Before    | After     | Reason                                                                                                                                                     |
|------------------------------------|---------------------|-----------|-----------|------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `estimator_config.yaml`            | `histogram_method`  | HISTOGRAM | **CLAHE** | Local contrast equalisation. Standard low-light VIO knob; large gain in dim cave scenes.                                                                   |
| `estimator_config.yaml`            | `num_pts`           | 200       | **400**   | More features per frame to compensate for parallax-starved forward driving.                                                                                |
| `estimator_config.yaml`            | `fast_threshold`    | 20        | **10**    | More aggressive corner detection on texture-poor cave walls.                                                                                               |
| `estimator_config.yaml`            | `min_px_dist`       | 10        | **7**     | Allow tighter feature packing.                                                                                                                             |
| `estimator_config.yaml`            | `track_frequency`   | 21 Hz     | **30 Hz** | Match the camera; don't drop frames the front-end could use.                                                                                               |
| `estimator_config.yaml`            | `init_window_time`  | 2.0 s     | **5.0 s** | Average over enough stationary samples to nail gravity direction even if init triggers slightly inside the spawn-drop window.                              |
| `vio.launch.py`                    | `VIO_START_DELAY_SEC` | 0       | **8.0 s** | Don't even subscribe to /imu until the spawn drop has settled. The big one — combined with the longer init window, this is what kills the "init on impact" failure. |
| `estimator_config.yaml`            | `calib_cam_*`       | true      | **false** | Our SDF-derived calibration is exact; online refinement just adds numerical noise the filter integrates into pose.                                         |
| `estimator_config.yaml`            | `try_zupt`          | false     | **true**  | Zero-velocity update when stationary. Required to prevent IMU-bias drift from compounding into position while idle.                                        |
| `estimator_config.yaml`            | `init_dyn_use`      | false     | **true**  | Dynamic-init fallback if static path misses.                                                                                                              |

If after these changes the trail is still wrong, the only remaining
mono-VIO levers are: a more parallax-friendly drive profile (figure-8
beats forward-straight), longer stationary wait before commanding the
first motion, and bumping `max_clones` / `max_slam` for a larger
sliding window. After those, stereo is the only path that genuinely
fixes the underlying problem.

## Comparing multiple VIO implementations side by side

Possible — and a natural next step once we're done squeezing OpenVINS.
The system architecture already supports it:

- `scripts/analyze_bag.py` reads odometry topics by name. To compare a
  second VIO (e.g. VINS-Fusion, Kimera-VIO, OKVIS2) the only changes
  are:
  1. Add the second VIO as a `third_party/` submodule + a small
     `<name>.launch.py`.
  2. Tell `analyze_bag.py` the new topic in its `ODOM_TOPICS` dict and
     give it a colour.
  3. Optionally: add a corresponding RViz display.
- Each VIO is just another process subscribed to `/imu` and
  `/rs_front/image`. They don't interfere with each other.
- `evo_traj bag2 ... /ground_truth/odom /ov_msckf/odomimu /vins/odom
  /kimera/odom --save_as_tum` compares all of them in one shot;
  `evo_res *.zip` gives a side-by-side APE table.

Candidates worth comparing at the mono-cam stage:

| Package        | Repo                                       | Strengths                                                            |
|----------------|--------------------------------------------|----------------------------------------------------------------------|
| **OpenVINS**   | rpng/open_vins (already wired)             | MSCKF, clean code, easy to debug.                                    |
| **VINS-Fusion**| HKUST-Aerial-Robotics/VINS-Fusion          | Tightly-coupled BA + loop closure. Often better on long sequences.   |
| **Kimera-VIO** | MIT-SPARK/Kimera-VIO                       | Built for SLAM + semantic mapping; needs slightly different config.  |
| **DLIOM/SVIN** | various                                    | Newer entrants worth tracking.                                       |

If you want to commit to this, the workflow is the same as for adding
LIO earlier: `git submodule add` under `third_party/`, write the
launch, add the topic to the analyzer.

## Accept that LIO wins on this rig

Cave + lidar + forward driving is LIO's home turf. VIO is here mostly
as a comparison target / sanity check / learning vehicle. If you're
trying to *use* the rover's pose estimate for navigation, fuse LIO
into wheel-odom with `robot_localization` and trust that; treat VIO
as the experimental loop.

## Reproducing this analysis on your own bag

```bash
# Install evo once (host or container):
pip3 install evo --user --break-system-packages

# Record (Mode 1 already up, leave VIO/LIO settle 10-15 s before drive):
mkdir -p ~/ros2_ws/runs
docker compose exec sim bash -ic "ros2 bag record \
  -o /ws/runs/run_$(date -u +%Y%m%dT%H%M%SZ) \
  /imu /rs_front/image /rs_front/camera_info \
  /lidar/points /lidar/points_lio \
  /ground_truth/odom /ground_truth/path /ground_truth/pose \
  /ov_msckf/odomimu /ov_msckf/pathimu \
  /Odometry /path \
  /cmd_vel /tf /tf_static"

# Drive (40-60 s mix); Ctrl-C the bag.

# Extract + APE:
mkdir -p /tmp/eval && cd /tmp/eval
evo_traj bag2 ~/ros2_ws/runs/run_<UTC> \
  /ground_truth/odom /ov_msckf/odomimu /Odometry --save_as_tum
evo_ape tum ground_truth_odom.tum ov_msckf_odomimu.tum -va --plot --plot_mode xy
evo_ape tum ground_truth_odom.tum Odometry.tum            -va --plot --plot_mode xy
```

The `-a` flag (Umeyama alignment) handles the unobservable-yaw offset
in OpenVINS' world frame — without it VIO will always look "wrong"
even when its actual drift is small.
