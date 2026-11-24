#!/usr/bin/env bash
set -euo pipefail
project="$(cd -- "$(dirname -- "$0")/.." && pwd -P)"
workspace="$project/ros2_ws"
podman run --rm --pull=never --network=none --userns=keep-id \
    --user "$(id -u):$(id -g)" --security-opt label=disable \
    -v "$workspace:/work:rw" -v "$project/rapid:/imt342-rapid:ro" \
    -v "$project/experimental/trajectory_control/rws_rapid:/imt342-rws-trajectory:ro" \
    --env IMT342_RWS_TRAJECTORY_SOURCE=/imt342-rws-trajectory/Imt342TrajectoryRapid.mod \
    --env IMT342_RAPID_SOURCE=/imt342-rapid/imt342/Imt342JointServer.mod -w /work "${IMT342_JAZZY_IMAGE:-localhost/ros-jazzy-imt342-rws:latest}" bash -ec '
        source /opt/ros/jazzy/setup.bash
        colcon build --base-paths src --event-handlers console_direct+
        source install/setup.bash
        colcon test --event-handlers console_direct+
        colcon test-result --verbose
    '
