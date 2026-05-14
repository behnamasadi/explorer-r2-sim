#!/usr/bin/env bash
# Sim container entrypoint.
#
# Expects the host's ros2_ws to be bind-mounted at /ws (see compose.yml).
# Steps each container start does:
#   1. Wire optional submodules from /ws/third_party/* into /ws/src/* via
#      symlinks so colcon picks them up.
#   2. Apply Jazzy-compatibility header patches to OpenVINS in-place
#      (idempotent — sed is a no-op once .hpp form is in the files).
#   3. Run `colcon build` if /ws/install isn't populated yet, or if the
#      user passes BUILD=force in the env. Otherwise reuse the cached
#      build.
#   4. Source the overlay and exec whatever was passed as CMD.
#
# Env knobs:
#   BUILD=force   rebuild from scratch even if install/ exists
#   BUILD=skip    never build (assume install/ is up to date)
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Persist Fuel cache between runs so cave tiles aren't re-downloaded.
mkdir -p /root/.gz/fuel /root/.ignition/fuel

cd /ws
mkdir -p src

PKGS=(explorer_r2_sim)

# ─── Optional: OpenVINS (VIO) ───────────────────────────────────────────
if [ -e third_party/open_vins/ov_msckf ]; then
    # Idempotent: re-running sed on already-patched files is a no-op.
    sed -i 's|image_transport/image_transport\.h|image_transport/image_transport.hpp|' \
        third_party/open_vins/ov_msckf/src/ros/ROS2Visualizer.h \
        third_party/open_vins/ov_msckf/src/ros/ROS1Visualizer.h
    sed -i 's|tf2_geometry_msgs/tf2_geometry_msgs\.h|tf2_geometry_msgs/tf2_geometry_msgs.hpp|' \
        third_party/open_vins/ov_msckf/src/ros/ROSVisualizerHelper.h \
        third_party/open_vins/ov_msckf/src/ros/ROS2Visualizer.h \
        third_party/open_vins/ov_msckf/src/ros/ROS1Visualizer.h
    sed -i 's|cv_bridge/cv_bridge\.h|cv_bridge/cv_bridge.hpp|' \
        third_party/open_vins/ov_msckf/src/ros/ROS1Visualizer.h \
        third_party/open_vins/ov_msckf/src/ros/ROS2Visualizer.h \
        third_party/open_vins/ov_core/src/test_tracking.cpp
    ln -sfn /ws/third_party/open_vins /ws/src/open_vins
    PKGS+=(ov_core ov_init ov_msckf)
else
    rm -f /ws/src/open_vins
fi

# ─── Optional: FAST_LIO (LIO) ───────────────────────────────────────────
if [ -e third_party/FAST_LIO/package.xml ]; then
    ln -sfn /ws/third_party/FAST_LIO /ws/src/fast_lio
    PKGS+=(fast_lio)
else
    rm -f /ws/src/fast_lio
fi

# ─── Build ──────────────────────────────────────────────────────────────
case "${BUILD:-auto}" in
    skip)
        echo "[entrypoint] BUILD=skip — using existing install/"
        ;;
    force)
        echo "[entrypoint] BUILD=force — full rebuild: ${PKGS[*]}"
        rm -rf build install log
        colcon build --symlink-install --packages-select "${PKGS[@]}"
        ;;
    *)
        if [ ! -f install/setup.bash ]; then
            echo "[entrypoint] First-time build: ${PKGS[*]}"
            colcon build --symlink-install --packages-select "${PKGS[@]}"
        else
            echo "[entrypoint] install/ present — skipping build (set BUILD=force to rebuild)"
        fi
        ;;
esac

source install/setup.bash
exec "$@"
