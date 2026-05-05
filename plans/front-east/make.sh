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

# Error handling function
error_exit() {
    set +x
    echo "ERROR: $1" >&2
    echo "Script failed at line $2" >&2
    exit 1
}

# Set trap for better error reporting
trap 'error_exit "Command failed" $LINENO' ERR

echo "Generating all derivative files in 'paths'"
echo "Configuration:"
echo "  MISSION_WP_SEPARATION: $MISSION_WP_SEPARATION"
echo "  CRUISE_ALTITUDE: $CRUISE_ALTITUDE"
echo "  CRUISE_SPEED: $CRUISE_SPEED"
echo "  CIRCLE_SEGMENTS: $CIRCLE_SEGMENTS"
echo "  PATH_SEPARATION: $PATH_SEPARATION"
echo "  PATH_ANGLE: $PATH_ANGLE"
echo "  SAFETY_MARGIN: $SAFETY_MARGIN"

# Script options for robustness
set -e  # Exit on any command failure
set -u  # Exit on undefined variables
set -o pipefail  # Exit if any command in a pipeline fails

# Trap to cleanup on exit (success or failure)
trap 'echo "Script finished"' EXIT

set -x  # Print commands as they execute

mkdir -p paths

# front-east-feel.ulg
../../ulg_to_mission.py front-east-feel.ulg \
    -o paths/front-east-feel_mission.plan \
    --step $MISSION_WP_SEPARATION --altitude $CRUISE_ALTITUDE --cruise-speed $CRUISE_SPEED

# front-east.plan
../../survey_to_geofence.py front-east.plan \
    -o paths/front-east.geofence.plan

# front-east.plan
#../../scan_to_geofence.py front-east.plan \
#    -o paths/front-east.geofence.plan

# front-east.flattened.plan
#../../scan_to_geofence.py front-east.flattened.plan \
#    -o paths/front-east.flattened.geofence.plan

# front-east_geofence.plan
../../path_planner.py front-east_geofence.plan \
    -o paths/front-east_geofence_path.plan \
    --segments $CIRCLE_SEGMENTS --sep $PATH_SEPARATION --angle $PATH_ANGLE --safe $SAFETY_MARGIN

# Produce a combo plan with all geofences and the original "feel" mission:
../../combine_plans.py -o paths/front-combined.plan \
    paths/front-east-feel_mission.plan \
    paths/front-east.geofence.plan \
    #paths/front-east_geofence_path.plan
    #front-east.plan \
    #front-east.flattened.plan \
    #paths/front-east.flattened.geofence.plan \

set +x

echo
echo "All derivative files generated in 'paths':"
ls -l paths
