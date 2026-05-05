#!/bin/bash

#
# This script generates all derivative files in 'paths' from the original data files (for my specific use case).
#
# Note: the data files are not included in this repository. Generate them using your data and the appropriate tools.
#

echo "Generating all derivative files in 'paths'"

set -x

mkdir -p paths

# front_two_trees.ulg
../../path_planner/ulg_to_mission.py front_two_trees.ulg \
    -o paths/front_two_trees_mission.plan \
    --step 1.0 --altitude 20.0 --cruise-speed 1.0

# front-geofence.plan
../../path_planner.py front-geofence.plan \
    -o paths/front-geofence_path.plan \
    --segments 16 --sep 0.25 --angle 0 --safe 0.5

# front-north.geofence.plan
../../path_planner.py front-north.geofence.plan \
    -o paths/front-north.geofence_path.plan \
    --segments 16 --sep 0.25 --angle 0 --safe 0.5

# front-south.geofence.plan
../../path_planner.py front-south.geofence.plan \
    -o paths/front-south.geofence_path.plan \
    --segments 16 --sep 0.25 --angle 0 --safe 0.5

set +x

echo "All derivative files generated in 'paths'"