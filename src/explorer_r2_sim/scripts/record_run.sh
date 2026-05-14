#!/usr/bin/env bash
# Record a ground-truth + estimators rosbag for offline VIO/LIO evaluation.
#
# Topics recorded:
#   /ground_truth/path          (from gt_to_path.py — must be running)
#   /model/explorer_r2/odometry (wheel odom from DiffDrive)
#   /ov_msckf/odomimu           (OpenVINS VIO, if mode 2 is up)
#   /ov_msckf/pathimu           (OpenVINS path)
#   /Odometry                   (FAST_LIO, if mode 3 is up)
#   /path                       (FAST_LIO path)
#   /cmd_vel                    (commanded twist — useful for scenario tagging)
#
# Usage (inside the sim container):
#   ros2 run explorer_r2_sim gt_to_path.py &       # publishes /ground_truth/path
#   ros2 run explorer_r2_sim record_run.sh slow_loop
#
# The argument is a scenario tag — drops into ~/.local/share/evo/<tag>/<UTC>.
set -e

TAG="${1:-run}"
OUT_DIR="${OUT_DIR:-${HOME}/.local/share/evo/${TAG}}"
mkdir -p "${OUT_DIR}"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
BAG_PATH="${OUT_DIR}/${STAMP}"

echo "[record_run] Writing rosbag to ${BAG_PATH}"
echo "[record_run] Stop with Ctrl-C when the drive scenario is over."

exec ros2 bag record -o "${BAG_PATH}" \
    /ground_truth/path \
    /model/explorer_r2/odometry \
    /ov_msckf/odomimu /ov_msckf/pathimu \
    /Odometry /path \
    /cmd_vel
