#!/usr/bin/env bash
# Build (if needed) and launch a scenario.
#   scripts/run_demo.sh [scenario] [enable_aeb]
# e.g.
#   scripts/run_demo.sh stationary_lead true
#   scripts/run_demo.sh stationary_lead false
#   scripts/run_demo.sh hard_brake
#   scripts/run_demo.sh slower_lead
set -eo pipefail   # not -u: ROS setup scripts reference unset variables

SCENARIO="${1:-stationary_lead}"
ENABLE_AEB="${2:-true}"

source /opt/ros/humble/setup.bash
[ -d install ] || colcon build
source install/setup.bash

exec ros2 launch aeb_demo aeb_sim.launch.py \
    scenario:="${SCENARIO}" enable_aeb:="${ENABLE_AEB}"
