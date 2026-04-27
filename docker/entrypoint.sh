#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

NAV2_PARAMS="${NAV2_PARAMS_FILE:-/ws/config/nav2_params.yaml}"

echo "[navigation_rbnx] starting Nav2 with params: $NAV2_PARAMS"

ros2 launch /ws/launch/nav2_bringup.launch.py \
    params_file:="$NAV2_PARAMS" \
    use_sim_time:=false &

sleep 5
echo "[navigation_rbnx] Nav2 launched, starting atlas bridge..."

export PYTHONPATH="/ws/src:/ws/proto_stubs:/ws/proto_gen:${PYTHONPATH:-}"
exec python3 -m navigation_rbnx.atlas_bridge
