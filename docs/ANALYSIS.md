# Trajectory analysis — current state, workflow, and case studies

This doc has three parts:

1. **[Current state](#current-state)** — what the codebase ships right
   now, the OpenVINS knobs that are on/off, and the latest measured
   numbers.
2. **[Workflow](#workflow-record--analyse--compare)** — exact commands
   to record a bag, run the analyser, and compare against a previous
   run.
3. **[Case studies](#case-study-1--forward-drive-mono-vio-fails)** —
   what specific bags taught us, with the failure modes and the fixes
   that came out of each. New iterations get appended at the bottom.

---

## Current state

### Numbers from the most recent runs

| Run                             | GT path | LIO err | LIO ratio | VIO err | VIO ratio | Notes |
|---------------------------------|--------:|--------:|----------:|--------:|----------:|-------|
| `run_20260514T154145Z` (v1)     | 40.27 m | 0.64 m (1.6 %) | 1.01 | 195.28 m (485 %) | 7.21 | Forward drive. VIO init contaminated by spawn drop. Pre-CLAHE config. |
| `run_20260514T162146Z` (v2)     | 58.52 m | **0.15 m (0.3 %)** | **1.01** | **4030 m (6887 %)** | **378** | Figure-8. CLAHE + aggressive tracker → catastrophic VIO blow-up. Reverted. |

LIO has been steadily healthy. VIO is the open problem.

### What changed since defaults — currently shipping

These are the deltas from upstream OpenVINS defaults that are
**currently in the config / launch files**. The reasoning for each
is in [`docs/PARAMETERS.md`](PARAMETERS.md).

| Where                         | Knob                  | Default     | Currently   | Why                                                                          |
|-------------------------------|-----------------------|-------------|-------------|------------------------------------------------------------------------------|
| `launch/vio.launch.py`        | `VIO_START_DELAY_SEC` | 0           | **8.0 s**   | Don't subscribe to `/imu` until the spawn-drop transient is over.            |
| `config/.../estimator_config` | `init_window_time`    | 2.0 s       | **5.0 s**   | Average over enough stationary samples for clean gravity-vector estimate.    |
| `config/.../estimator_config` | `init_imu_thresh`     | 1.5         | **0.3**     | Lower threshold matches our low-noise gz IMU (σ ≈ 1e-2 m/s²).                |
| `config/.../estimator_config` | `init_dyn_use`        | false       | **true**    | Dynamic-init fallback if static path misses.                                 |
| `config/.../estimator_config` | `calib_cam_extrinsics` | true       | **false**   | Sim calibration is exact; online refinement just adds noise.                  |
| `config/.../estimator_config` | `calib_cam_intrinsics` | true       | **false**   | Same.                                                                        |
| `config/.../estimator_config` | `calib_cam_timeoffset` | true       | **false**   | Same.                                                                        |
| `config/.../estimator_config` | `try_zupt`            | false       | **true**    | Zero-velocity update prevents bias drift while idle.                         |
| `config/.../kalibr_imu_chain` | `Tw, Ta, R_*, Tg`     | (omitted)   | **identity/zero** | Required even when no correction needed — omitting them crashes OpenVINS. |

The front-end tracker knobs (`histogram_method`, `num_pts`,
`fast_threshold`, `min_px_dist`, `track_frequency`) are at **upstream
defaults**, after a one-iteration experiment where flipping them to
"low-light cave" values made VIO 50× worse — see
[Case study #2](#case-study-2--clahe-aggressive-tracker-backfire) below.

### Scripts that ship with the repo

| Script                              | Purpose                                                                                                |
|-------------------------------------|--------------------------------------------------------------------------------------------------------|
| `scripts/gt_to_path.py`             | Auto-started by `world.launch.py`. Bridges `/ground_truth/pose` (gz) → `/ground_truth/odom` + `/path`. |
| `scripts/lidar_field_adapter.py`    | Auto-started by `lio.launch.py`. Adds `ring` + `t` fields gz lidar omits but FAST_LIO needs.           |
| `scripts/record_run.sh <tag>`       | Wraps `ros2 bag record` for all the topics analyse_bag wants.                                          |
| `scripts/analyze_bag.py <bag>`      | **The main analyser.** Reads bag → produces plots + summary.md.                                        |
| `scripts/eval.sh <bag>`             | Wraps `evo_traj` + `evo_ape` for offline Umeyama-aligned error.                                        |

---

## Workflow: record → analyse → compare

After making config changes you want to evaluate, run this loop.

### 1. Bring up Mode 1 and wait for VIO to start

```bash
cd ~/ros2_ws/src/explorer_r2_sim
docker compose down            # clean state
xhost +local:root
docker compose up
```

VIO is intentionally delayed 8 s after launch. Watch terminal 1 for:

```
[run_subscribe_msckf-X]: process started with pid [...]
```

That's t+8s. Stay stationary another 5–10 s so static init has clean
data.

### 2. Record

```bash
# In another terminal:
mkdir -p ~/ros2_ws/runs
docker compose exec sim bash -ic "ros2 bag record \
  -o /ws/runs/run_$(date -u +%Y%m%dT%H%M%SZ) \
  /imu /rs_front/image /rs_front/camera_info \
  /lidar/points /lidar/points_lio \
  /ground_truth/pose /ground_truth/odom /ground_truth/path \
  /ov_msckf/odomimu /ov_msckf/pathimu \
  /Odometry /path \
  /cmd_vel /tf /tf_static"
```

### 3. Drive a useful profile

Suggested 80–90 s mix:

| 0–15 s     | Stationary             | Lets VIO init on clean IMU data |
| 15–30 s    | Slow forward, ~0.3 m/s | Easiest motion                  |
| 30–40 s    | **STOP**               | Watch RViz: green should freeze if ZUPT works |
| 40–70 s    | Figure-8 / strong yaw  | Mono VIO's best parallax case   |
| 70–80 s    | Stop                   | Second ZUPT test                |

Ctrl-C the bag recorder when done.

### 4. Analyse

`analyze_bag.py` is a pure-Python script with **no ROS dependency**.
Its requirements are just `numpy`, `matplotlib`, `rosbags`, and
optionally `opencv-python` (for the camera-frame samples). You can run
it from any of three places — pick whichever matches your habits:

#### Option A — conda env on the host (recommended for repeat use)

If you have a conda env for this project (e.g. `explorer-r2-sim`):

```bash
conda activate explorer-r2-sim
pip install numpy matplotlib rosbags opencv-python    # once

cd ~/ros2_ws
python src/explorer_r2_sim/scripts/analyze_bag.py runs/run_<UTC>
```

#### Option B — system Python on the host

```bash
pip3 install --user --break-system-packages numpy matplotlib rosbags opencv-python   # once

cd ~/ros2_ws
python3 src/explorer_r2_sim/scripts/analyze_bag.py runs/run_<UTC>
```

#### Option C — inside the running container

The package already declares `python3-numpy` and friends as deps, and
the script is installed to `lib/explorer_r2_sim/` by colcon. Use it
via `ros2 run`:

```bash
docker compose exec sim bash -ic \
  "pip3 install rosbags opencv-python --break-system-packages 2>/dev/null; \
   ros2 run explorer_r2_sim analyze_bag.py /ws/runs/run_<UTC>"
```

(`rosbags` and `opencv-python` aren't in the system image yet — the
inline pip install is idempotent; once it's run once in a running
container the cache hits on subsequent runs.)

#### Output (all three options)

The script auto-creates `<bag>_report/` next to the bag and writes:

| File                    | What it shows                                                           |
|-------------------------|-------------------------------------------------------------------------|
| `summary.md`            | Topic rates, trajectory stats, end-position error, path-length ratio, IMU sanity, auto-interpretation. |
| `trajectory_xy.png`     | Top-down XY of all three estimators. Circle = start, square = end.       |
| `trajectory_3d.png`     | Full 3D view (X, Y, Z), same colour palette as RViz.                     |
| `trajectory_xyz.png`    | Per-axis (X, Y, Z) over normalised time — best for spotting drift on one axis. |
| `imu_accel.png`         | \|accel\| vs the 9.81 m/s² line + per-axis. Stationary should sit at 9.81. |
| `imu_gyro.png`          | Gyro components over time.                                              |
| `camera_frames/*.png`   | Six evenly-spaced /rs_front/image samples — visual sanity check.        |

### 5. (Optional) Umeyama-aligned APE with `evo`

`evo_ape -va` aligns the estimator's frame to GT before computing
error — that subtracts out the unobservable initial yaw and tells you
the *true* drift, separately from frame misalignment.

```bash
pip3 install evo --user --break-system-packages
mkdir -p /tmp/eval && cd /tmp/eval
evo_traj bag2 ~/ros2_ws/runs/run_<UTC> \
  /ground_truth/odom /ov_msckf/odomimu /Odometry --save_as_tum
evo_ape tum ground_truth_odom.tum ov_msckf_odomimu.tum -va --plot --plot_mode xy
evo_ape tum ground_truth_odom.tum Odometry.tum            -va --plot --plot_mode xy
```

### 6. Compare against a previous run

Append the new row to the [numbers table](#numbers-from-the-most-recent-runs)
above. The analyser flags whether each estimator looks healthy / has
scale drift / has yaw offset; comparing two bags' `summary.md` files
tells you whether a config change helped or hurt.

---

## Aside: what is parallax, and why is forward driving the worst case?

The short version: **a monocular camera estimates depth from how
things move across the image when the camera itself moves.** Stuff
near you swings across the image fast; stuff far away barely moves.
The *difference* between those motion rates is parallax, and it's the
only information a single 2D image gives you about 3D depth.

You wrote it right: "stationary observer, something near and something
far — the near one looks like it moved more." Same idea for a moving
observer.

### Why driving forward kills it

The bad case isn't *distance* per se — it's that the camera moves
**along its own optical axis** (the line the camera is pointing).

Imagine driving toward a brick wall:

- The brick **directly in front** of you (on the optical axis) just
  gets *bigger* as you approach. It doesn't slide sideways in the
  image; it expands radially. **Zero parallax.**
- A brick **off to one side** does move across the image — but the
  motion is along a radial line outward from the image centre. The
  amount of motion depends on how close the brick is *and* how far
  off-axis it is.
- A brick **at the edge of the image** (90° off-axis) gives the
  maximum parallax, just like a sideways camera move would.

Mathematically, if you're at distance `D` from a point, and you move
forward by `Δd`, the **on-axis** point doesn't shift at all — it just
scales. The **off-axis** point at angle `θ` shifts by roughly
`(Δd · sin θ) / D` in image angle. Plug in `θ = 0` (dead ahead) and
you get zero, regardless of `D`. That's the blind spot.

### The tunnel makes it worse

The geometry of our sim makes this scenario pathological:

- The rover drives forward.
- The camera faces forward (`rs_front`).
- **The tunnel walls are within a few metres of the camera.** The
  ceiling, floor, and side walls are *near* the optical axis — they
  occupy most of the 60° HFOV — so most of what the camera sees is
  close to "dead ahead."
- Tunnel walls are also pretty self-similar: rock texture, dim
  lighting. KLT can track a feature, but if the filter only sees the
  same wall move radially outward as you approach, it doesn't know
  whether the wall is 2 m away or 5 m. **Scale is weakly observable.**

The result: VIO can correctly track *which* features are moving, but
it can't pin down *how fast* the camera is actually moving in metres,
because every consistent (velocity, depth) pair produces the same
image motion. Over a 40 m drive that uncertainty compounds into the
7× scale drift we saw in [Case study 1](#case-study-1--forward-drive-mono-vio-fails).

### Quick numerical example

Camera at origin looking along +X. Two features:

| Feature | Position (X, Y, Z) | Image position before move | Image after camera moves +1 m in X | Image shift |
|---|---|---|---|---|
| Dead ahead, 10 m | (10, 0, 0) | centre | centre | **0 pixels** |
| Dead ahead, 5 m  | ( 5, 0, 0) | centre | centre | **0 pixels** |
| 2 m to the side, 10 m ahead | (10, 2, 0) | 11.3° off-axis | 12.5° off-axis | 1.2° (~3 px) |
| 2 m to the side, 5 m ahead  | ( 5, 2, 0) | 21.8° off-axis | 26.6° off-axis | 4.8° (~13 px) |

The first two rows have **identical image motion** despite being at
different depths. From a single mono camera, you can't tell which is
which. The third and fourth rows do differentiate, but only because
they're off-axis — and points right at the image edge are sparse in a
narrow-corridor scene.

### What fixes it

Three things, in order of how much they help:

1. **Sideways motion** (translation perpendicular to the optical axis).
   A pure sideways move turns every point into a row 4 of the table
   above — strong parallax, strong depth signal. Rotation alone
   doesn't help (it changes pixel positions without changing depth),
   but a yaw + small forward velocity (the figure-8 we recommended)
   gives you continuous sideways translation.
2. **A second camera** (stereo). With a baseline of, say, 10 cm
   between two cameras, you get depth-from-disparity at *every*
   frame — no motion required. Scale is recoverable from the first
   frame onward. This is why the user's stated next step (stereo
   VIO) is the real fix and the other tweaks are just palliatives.
3. **Depth from another sensor** (RGBD camera, lidar). The depth
   image from `rs_front` is already published as `/rs_front/depth`;
   feeding it into a depth-aware VIO (like Kimera) would fix scale
   directly. FAST_LIO does this implicitly with lidar — that's why
   LIO is converging to 1.6 % drift while VIO at 7× is parallax-
   limited.

---

## Case study 1 — forward drive, mono VIO fails

A case study from `runs/run_20260514T154145Z/` — a real bag the user
recorded while teleoperating the rover roughly straight forward across
the tunnel world. The point isn't the specific numbers; it's the
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

## Mono-VIO tuning history (what's currently in the config)

Two categories of changes have been tried. **Init / timing fixes
stayed in** because they cured a real bug (init on spawn-drop impact).
**Front-end tracker tweaks were reverted** because they backfired —
see the next section.

| Category | Where | Knob | Default | Currently set to | Notes |
|---|---|---|---|---|---|
| Init / timing  | `vio.launch.py`         | `VIO_START_DELAY_SEC` | 0       | **8.0 s**   | Don't subscribe to `/imu` until spawn drop settles. |
| Init / timing  | `estimator_config.yaml` | `init_window_time`    | 2.0 s   | **5.0 s**   | Average over more stationary samples. |
| Init / timing  | `estimator_config.yaml` | `init_imu_thresh`     | 1.5     | **0.3**     | Match our low-noise gz IMU. |
| Init / timing  | `estimator_config.yaml` | `init_dyn_use`        | false   | **true**    | Dynamic-init fallback if static fails. |
| Sim has exact calibration | `estimator_config.yaml` | `calib_cam_*`         | true    | **false**   | Don't refine SDF-exact calibration online. |
| Stationary drift | `estimator_config.yaml` | `try_zupt`           | false   | **true**    | Required to prevent bias drift while idle. |
| Front-end (REVERTED) | `estimator_config.yaml` | `histogram_method` | HISTOGRAM | HISTOGRAM | Was CLAHE; reverted — see below. |
| Front-end (REVERTED) | `estimator_config.yaml` | `num_pts` | 200 | 200 | Was 400; reverted. |
| Front-end (REVERTED) | `estimator_config.yaml` | `fast_threshold` | 20 | 20 | Was 10; reverted. |
| Front-end (REVERTED) | `estimator_config.yaml` | `min_px_dist` | 10 | 10 | Was 7; reverted. |
| Front-end (REVERTED) | `estimator_config.yaml` | `track_frequency` | 21.0 Hz | 21.0 Hz | Was 30 Hz; reverted. |

## The CLAHE / aggressive-tracker backfire (case study #2)

After the first analysis I tuned the OpenVINS front-end for "low-light
cave" — CLAHE, more features, lower `fast_threshold`. On the v2 bag
(`run_20260514T162146Z`), VIO got **much worse**:

|              | v1 (forward drive)         | v2 (figure-8 + stops)     | Δ          |
|--------------|----------------------------|---------------------------|------------|
| GT path      | 40.27 m                    | 58.52 m                   | longer     |
| LIO end-err  | 0.64 m  (1.6 %)            | 0.15 m  (0.3 %)           | **✅ better** |
| LIO ratio    | 1.01                       | 1.01                      | unchanged  |
| **VIO end-err** | 195.28 m (485 %)        | **4030.04 m (6886 %)**    | **❌ 20× worse** |
| **VIO ratio**   | 7.21                    | **378.80**                | **❌ 52× worse** |

VIO reported **22.2 km** of motion on a 58 m drive — the green trail
exploded out to `(+1073, +1948, −3360) m`.

**Why:** real-camera tuning assumes real-camera characteristics
(rolling shutter, vignetting, dim corners, structured noise). Our gz
camera is *artificially clean*: uniform lighting, no rolling shutter,
only the small σ=0.01 Gaussian noise floor we added in the SDF. The
combination of CLAHE (amplifies local contrast = amplifies that
Gaussian noise into pseudo-texture), `fast_threshold: 10` (accepts the
amplified noise as corners), and `num_pts: 400` (chases all of them)
fed the filter a flood of false feature observations. The tracker
locked onto ghosts; triangulation produced wildly wrong 3D positions;
the filter integrated the resulting "motion."

The lesson: **don't transplant real-camera tuning onto sim data
without thinking about how the image content differs.** Our sim image
is *easier* than real-world, not harder. Conservative defaults work
better.

If you ever switch to:
- a *real* RealSense camera, or
- a textured/dim outdoor Fuel world with more realistic image noise,

then flipping back to CLAHE + 400 pts + threshold 10 is the right
move. For the current gz cave sim, defaults win.

## What's left to try on mono VIO

After the timing fixes and the tracker revert, the levers still on
the table for pure-mono are:

1. **Drive a more parallax-friendly profile.** Forward-straight is
   the worst-case for any mono VIO (no parallax along the optical
   axis). The v2 run already mixed in some yaw and got better LIO
   numbers — VIO would benefit too if it weren't being thrown off by
   the tracker chaos. With defaults restored, try v2's profile again.
2. **Longer stationary wait** before the first command (15–20 s),
   especially on bag start. The 8 s delay handles the spawn drop;
   another 5–10 s of stationary lets the filter accumulate clean
   static observations.
3. **Bump `max_clones`** (11 → 15) and **`max_slam`** (50 → 75) for a
   larger sliding window. Costs CPU but stabilises the filter when
   tracks die in the cave.
4. **Stereo VIO** is the real cure. The model has `rs_left` +
   `rs_right`. Adding a `cam1` block to `kalibr_imucam_chain.yaml`
   and flipping `use_stereo: true` + `max_cameras: 2` fixes the scale
   ambiguity for good.

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

---

The workflow above ([Workflow: record → analyse → compare](#workflow-record--analyse--compare))
supersedes a duplicate "Reproducing this analysis" section that used
to live here.
