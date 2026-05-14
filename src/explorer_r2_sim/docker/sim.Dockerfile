# Single-image dev container: ROS 2 desktop + Gazebo (Fortress or Harmonic) +
# ros_gz_bridge + RViz + teleop. Workspace is COPY'd in and built with colcon.
ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop-full

ARG ROS_DISTRO
ARG GZ_PKG=gz-harmonic
ARG DEBIAN_FRONTEND=noninteractive

# OSRF Gazebo apt repo + Gazebo + ros_gz packages + xterm for teleop UI.
RUN apt-get update && apt-get install -y --no-install-recommends \
      curl gnupg lsb-release ca-certificates xterm \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y --no-install-recommends \
      ${GZ_PKG} \
      ros-${ROS_DISTRO}-ros-gz-bridge \
      ros-${ROS_DISTRO}-ros-gz-sim \
      ros-${ROS_DISTRO}-ros-gz-image \
      ros-${ROS_DISTRO}-teleop-twist-keyboard \
      ros-${ROS_DISTRO}-joy \
      ros-${ROS_DISTRO}-teleop-twist-joy \
      ros-${ROS_DISTRO}-rqt-robot-steering \
      ros-${ROS_DISTRO}-rqt \
      ros-${ROS_DISTRO}-rqt-graph \
      git libceres-dev libeigen3-dev libboost-all-dev \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-image-transport-plugins \
      ros-${ROS_DISTRO}-tf2-geometry-msgs \
      ros-${ROS_DISTRO}-cv-bridge \
 && rm -rf /var/lib/apt/lists/*

# OpenVINS for VIO. Pinned to master because rpng/open_vins doesn't tag
# ROS 2 releases — this fetches whatever's current on first build.
RUN git clone --depth 1 https://github.com/rpng/open_vins.git /ws/src/open_vins \
 && sed -i 's|image_transport/image_transport\.h|image_transport/image_transport.hpp|' \
      /ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.h \
      /ws/src/open_vins/ov_msckf/src/ros/ROS1Visualizer.h \
 && sed -i 's|tf2_geometry_msgs/tf2_geometry_msgs\.h|tf2_geometry_msgs/tf2_geometry_msgs.hpp|' \
      /ws/src/open_vins/ov_msckf/src/ros/ROSVisualizerHelper.h \
      /ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.h \
      /ws/src/open_vins/ov_msckf/src/ros/ROS1Visualizer.h \
 && sed -i 's|cv_bridge/cv_bridge\.h|cv_bridge/cv_bridge.hpp|' \
      /ws/src/open_vins/ov_msckf/src/ros/ROS1Visualizer.h \
      /ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.h \
      /ws/src/open_vins/ov_core/src/test_tracking.cpp

ENV ROS_DISTRO=${ROS_DISTRO}
WORKDIR /ws

# .dockerignore strips build/, install/, log/, .git, .env, README.md
COPY . /ws/src/explorer_r2_sim

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
             colcon build --symlink-install \
               --packages-select explorer_r2_sim ov_core ov_init ov_msckf"

COPY docker/sim-entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "explorer_r2_sim", "cave.launch.py"]
