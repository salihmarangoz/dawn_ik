#!/usr/bin/env bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# # SELF COLLISION EXPERIMENTS
# WAYPOINT_FILES=("draw_square"  "lower_x"  "lower_y"  "mid_x"  "mid_y"  "upper_x"  "upper_y"  "upper_y_alt")

# for WAYPOINT_FILE in ${WAYPOINT_FILES[*]}; do
#     ###################################################################
#     export METHOD_NAME="dawn_ik"
#     #export WAYPOINT_FILE="draw_square"
#     export USE_DAWN_IK="true"
#     export USE_COLLISION_IK="false"
#     export OUT_FILENAME="$SCRIPT_DIR""/""$METHOD_NAME"_"$WAYPOINT_FILE"".csv"
#     if [ ! -f "$OUT_FILENAME" ]; then
#         roslaunch marangoz23humanoids_ws experiment_waypoint.launch out_filename:="$OUT_FILENAME" waypoints_file:="$WAYPOINT_FILE.txt" use_dawn_ik:="$USE_DAWN_IK" use_collision_ik:="$USE_COLLISION_IK"
#     fi
#     sleep 1
#     ###################################################################

#     ###################################################################
#     export METHOD_NAME="collision_ik"
#     #export WAYPOINT_FILE="draw_square"
#     export USE_DAWN_IK="false"
#     export USE_COLLISION_IK="true"
#     export OUT_FILENAME="$SCRIPT_DIR""/""$METHOD_NAME"_"$WAYPOINT_FILE"".csv"
#     if [ ! -f "$OUT_FILENAME" ]; then
#         roslaunch marangoz23humanoids_ws experiment_waypoint.launch out_filename:="$OUT_FILENAME" waypoints_file:="$WAYPOINT_FILE.txt" use_dawn_ik:="$USE_DAWN_IK" use_collision_ik:="$USE_COLLISION_IK"
#     fi
#     sleep 1
#     ###################################################################
# done

