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

## What to try next

In rough order of "small fix first, big rewire later":

1. **Switch the camera tracker to CLAHE.** Edit
   `config/openvins/estimator_config.yaml`, set
   `histogram_method: CLAHE`. Equalises local contrast — usually a
   big win in dim cave scenes. Cheap to try.
2. **Drive a more parallax-friendly profile.** Forward driving is the
   absolute worst case for mono VIO. A figure-8, or even sustained
   yaw rate with a small forward component, gives the camera much
   more useful parallax. Even with mono, well-driven motion gets
   significantly better scale.
3. **Wait 10–15 s stationary before driving.** Lets the rover settle
   so OpenVINS' init grabs clean gravity instead of impact data.
4. **Bump up `init_window_time` to 5+ s** to require more samples
   before init can succeed. Combined with the wait above, this lets
   the impact transient pass.
5. **Stereo VIO** is the real fix. The model already has `rs_left`
   and `rs_right`. Add a `cam1` block to `kalibr_imucam_chain.yaml`,
   flip `use_stereo: true` and `max_cameras: 2` in
   `estimator_config.yaml`. Stereo fixes scale ambiguity for good.
6. **Accept that LIO wins on this rig.** Cave + lidar + forward
   driving is LIO's home turf. VIO is here mostly as a comparison
   target / sanity check / learning vehicle.

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
