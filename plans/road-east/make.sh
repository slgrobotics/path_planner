#!/bin/bash

echo "Generating all derivative files in 'paths'"

set -x

mkdir -p paths

../path_planner/ulg_to_mission.py front-east-feel.ulg \
    -o paths/front-east-feel_mission.plan \
    --step 1.0 --altitude 20.0 --cruise-speed 1.0


set +x
