#!/usr/bin/env bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# # SELF COLLISION EXPERIMENTS
WAYPOINT_FILES=("circle_xy"  "circle_yz"  "eight_xy"  "eight_yz"  "square_xy"  "square_yz")
END_POINT_FRAME="link_eef"
ROBOT_NAME="lite6"

for WAYPOINT_FILE in ${WAYPOINT_FILES[*]}; do
    ###################################################################
    export METHOD_NAME="dawn_ik"
    if [ ! -f "$OUT_FILENAME" ]; then
        roslaunch dawn_ik run_experiment.launch robot_name:="$ROBOT_NAME" solver:="$METHOD_NAME" waypoints_file:="$WAYPOINT_FILE" endpoint_frame:="$END_POINT_FRAME"
    fi
    sleep 1
    ###################################################################

    ###################################################################
    export METHOD_NAME="collision_ik"
    if [ ! -f "$OUT_FILENAME" ]; then
        roslaunch dawn_ik run_experiment.launch robot_name:="$ROBOT_NAME" solver:="$METHOD_NAME" waypoints_file:="$WAYPOINT_FILE" endpoint_frame:="$END_POINT_FRAME"
    fi
    sleep 1
    ###################################################################
done

