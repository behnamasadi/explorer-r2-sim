#!/usr/bin/env bash
# Source the workspace overlay then exec whatever was passed as CMD.
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "/ws/install/setup.bash"

# Persist Fuel cache between runs so the cave tiles aren't re-downloaded
# every container start. Mounted to a host volume in compose.yml.
mkdir -p /root/.gz/fuel /root/.ignition/fuel

exec "$@"
