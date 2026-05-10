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
PATH_SAFETY_MARGIN=${PATH_SAFETY_MARGIN:-0.0}

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
echo "  PATH_SAFETY_MARGIN: $PATH_SAFETY_MARGIN"

# Script options for robustness
set -e  # Exit on any command failure
set -u  # Exit on undefined variables
set -o pipefail  # Exit if any command in a pipeline fails

# Trap to cleanup on exit (success or failure)
trap 'echo "Script finished"' EXIT

set -x  # Print commands as they execute

mkdir -p paths

# back-west.plan
../../survey_to_geofence.py back-west.plan \
    -o paths/back-west.geofence.plan

# back-west.plan
../../path_planner.py paths/back-west.geofence.plan \
    -o paths/back-west.geofence_path.plan \
    --segments $CIRCLE_SEGMENTS --sep $PATH_SEPARATION --angle $PATH_ANGLE --safe $PATH_SAFETY_MARGIN

# make a 1 meter version for combo plan:
../../path_planner.py paths/back-west.geofence.plan \
    -o paths/back-west.geofence_path_1m.plan \
    --segments $CIRCLE_SEGMENTS --sep 1.0 --angle $PATH_ANGLE --safe $PATH_SAFETY_MARGIN

# Produce a combo plan with all geofences and the "1 meter" path:
../../combine_plans.py -o paths/back-west.combined.plan \
    paths/back-west.geofence.plan \
    paths/back-west.geofence_path_1m.plan \

# Display the combined plan in a window:
../../show_plan.py paths/back-west.combined.plan

set +x

echo
echo "All derivative files generated in 'paths':"
ls -l paths
    