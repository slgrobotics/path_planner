#!/bin/bash

#
# This script generates all derivative files in 'paths' from the original data files (for my specific use case).
#
# Note: the data files are not included in this repository. Generate them using your data and the appropriate tools.
#

# Configuration variables - adjust these as needed
MISSION_WP_SEPARATION=${MISSION_WP_SEPARATION:-1.0}
CRUISE_ALTITUDE=${CRUISE_ALTITUDE:-20.0}
CRUISE_SPEED=${CRUISE_SPEED:-1.0}
CIRCLE_SEGMENTS=${CIRCLE_SEGMENTS:-16}
PATH_SEPARATION=${PATH_SEPARATION:-0.25}
PATH_ANGLE=${PATH_ANGLE:-0}
SAFETY_MARGIN=${SAFETY_MARGIN:-0.5}

echo "Generating all derivative files in 'paths'"
echo "Configuration:"
echo "  MISSION_WP_SEPARATION: $MISSION_WP_SEPARATION"
echo "  CRUISE_ALTITUDE: $CRUISE_ALTITUDE"
echo "  CRUISE_SPEED: $CRUISE_SPEED"
echo "  CIRCLE_SEGMENTS: $CIRCLE_SEGMENTS"
echo "  PATH_SEPARATION: $PATH_SEPARATION"
echo "  PATH_ANGLE: $PATH_ANGLE"
echo "  SAFETY_MARGIN: $SAFETY_MARGIN"

set -x

mkdir -p paths

# front_two_trees.ulg
../../ulg_to_mission.py front_two_trees.ulg \
    -o paths/front_two_trees_mission.plan \
    --step $MISSION_WP_SEPARATION --altitude $CRUISE_ALTITUDE --cruise-speed $CRUISE_SPEED

# front-geofence.plan
../../path_planner.py front-geofence.plan \
    -o paths/front-geofence_path.plan \
    --segments $CIRCLE_SEGMENTS --sep $PATH_SEPARATION --angle $PATH_ANGLE --safe $SAFETY_MARGIN

# front-north.geofence.plan
../../path_planner.py front-north.geofence.plan \
    -o paths/front-north.geofence_path.plan \
    --segments $CIRCLE_SEGMENTS --sep $PATH_SEPARATION --angle $PATH_ANGLE --safe $SAFETY_MARGIN

# front-south.geofence.plan
../../path_planner.py front-south.geofence.plan \
    -o paths/front-south.geofence_path.plan \
    --segments $CIRCLE_SEGMENTS --sep $PATH_SEPARATION --angle $PATH_ANGLE --safe $SAFETY_MARGIN

set +x

echo
echo "All derivative files generated in 'paths':"
ls -l paths
