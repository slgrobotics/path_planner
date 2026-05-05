> This is a modified version of the [original](https://github.com/swepet/path_planner) *Path Planner* by Peter Lehnér.
>
> This version is able to load *".plan"* files created in [QGroundControl](https://qgroundcontrol.com/), and also outputs paths in QGC format (named "*_path.plan")
>
> All credit goes to the original project and its author - Peter Lehnér.
>
> **Modifications:**
> - methods are moved (mostly verbatim) to *helpers* directory
> - added `helpers/qgc_format.py` to handle QGC ".plan" format
> - added sample files to demonstrate functionality

## How to use

**Install prerequisites:**
```
sudo apt-get install python3-shapely python3-pyproj
pip3 install simplekml pyulog --break-system-packages
```

If you intend to use `plan_to_drone.py` install MAVSDK:
```
pip3 install mavsdk --break-system-packages
```

**Clone:**
```
mkdir ~/planner_ws
git clone https://github.com/slgrobotics/path_planner.git
```

**Run sample files in the `plans` directory:**
```
cd ~/planner_ws/path_planner
./path_planner.py plans/test_3.kmz --target GeoCage_1 --sep 0.4 --safe 0.5 --angle 45
  or
./path_planner.py plans/test_3.kml --target GeoCage_1 --sep 0.4 --safe 0.5 --angle 45
./path_planner.py plans/geofence_qgroundcontrol_multi2.plan --sep 0.4 --safe 0.5 --angle 45
```

Use [QGroundControl](https://qgroundcontrol.com/) to display or create *".plan"* files,
or [Google MyMaps](https://www.google.com/maps/d/) to display or create *".kml"* or *".kmz"* files.

`./path_planner.py plans/test_3.kml --sep 0.4 --safe 0.5 --angle 45`
<img width="2018" height="1631" alt="Screenshot from 2026-04-24 13-32-42" src="https://github.com/user-attachments/assets/57dff9fd-db30-497a-b5d4-927bf67cafd9" />

`./path_planner.py plans/test_3.kml --sep 0.4 --safe 0.5 --angle 45`
<img width="1137" height="1264" alt="Screenshot from 2026-04-24 20-16-52" src="https://github.com/user-attachments/assets/9fc17068-518c-420d-b3c3-d09b0fcdb104" />

`./path_planner.py plans/geofence_qgroundcontrol_multi2.plan --target GeoCage_1 --sep 0.4 --safe 0.5 --angle 45`
<img width="1449" height="868" alt="Screenshot from 2026-04-24 21-04-45" src="https://github.com/user-attachments/assets/ab90251e-be44-4049-bdef-2f86a7e42c64" />

## Utilities: Missions and Geofence Converters

There are two converter scripts to assist with "lawn mowing mission" planning. The typical workflow is as follows:

- Use *[QGroundControl](https://qgroundcontrol.com/)* to plan a mission that traces the perimeter of your mowing area. Test it live and adjust as needed.
Alternatively, run `ulg_to_mission.py` to convert a PX4 `.ulg` log file to a mission `.plan` file. 
- Run `mission_to_geofence.py` to convert the mission into a geofence polygon.
- Optionally, use *QGroundControl* to edit the Fence:
  - add exclusion areas (polygons and circles)
  - **Tip:** position the robot near obstacles for better accuracy, as *QGroundControl* displays it live on the plan
- Run `path_planner.py` to generate the final *“scan pattern”* mission file.

#### mission_to_geofence.py

Convert a *QGroundControl* mission path into a geofence polygon.

This script takes a *QGroundControl* `.plan` file containing a mission (waypoints) and converts it into a geofence plan file.

The script transforms mission waypoints into polygon corners. The resulting geofence can be used either as:
- an inclusion zone (allowed area), or
- an exclusion zone (no-fly / no-go area), depending on the *"-e"* argument.

In practice, you can “trace” the boundary of an area by creating a mission around it and then converting that mission into a geofence.

Usage:
```
~/planner_ws/path_planner/mission_to_geofence.py input_mission.plan [-o output_geofence.plan] [-e]
```

#### geofence_to_mission.py

Convert a *QGroundControl* geofence polygon into a mission path.

This script takes a *QGroundControl* `.plan` file containing a geofence polygon and converts it into a mission plan by generating waypoints at the polygon corners.

The resulting mission can be used to “trace” or verify the boundary of a geofence area.

Usage:
```
~/planner_ws/path_planner/geofence_to_mission.py input_geofence.plan [-o output_mission.plan] [-a ALTITUDE]
```

#### ulg_to_mission.py

Convert a PX4 `.ulg` log file into a *QGroundControl* mission plan.

This script extracts GPS positions from a PX4 `.ulg` log and creates a `.plan` mission file. It filters out points that are closer than the provided `--step` distance to reduce waypoint density.

Usage:
```
~/planner_ws/path_planner/ulg_to_mission.py input_log.ulg [-o output_mission.plan] [--step 1.0] [--cruise-speed 1.3] [-a 20.0]
```

#### combine_plans.py

Combine multiple *QGroundControl*  `.plan` files into a single `.plan` output.
Use this script to overlay multiple QGC files and, for example, verify that your robot's paths comply with geofences.

This script merges mission items, geofence polygons, geofence circles, and rally
points from multiple *QGroundControl* `.plan` files into a single combined plan.

Usage:
```
python3 combine_plans.py -o combined.plan input1.plan input2.plan ...
```

#### flatten_plan.py

Flatten *TransectStyleComplexItem* (survey) entries in *QGroundControl* `.plan` files.

The script will replace any item that contains a *TransectStyleComplexItem* (or
*ComplexItem* of type 'survey') with that complex item's inner `Items` list,
preserving order. By default it will renumber `doJumpId` sequentially.

Usage examples:
```
python3 flatten_plan.py input.plan -o output.plan
python3 flatten_plan.py input.plan --inplace --backup
python3 flatten_plan.py input.plan --dry-run
python3 flatten_plan.py front-east.plan --mode clean -o front-east.flattened.plan
```

#### scan_to_geofence.py

Convert a *QGroundControl* survey mission to a geofence inclusion zone.

This script takes a *QGroundControl* `.plan` file containing a survey mission
and creates a geofence inclusion zone based on the convex hull of the mission
waypoints. The convex hull represents the approximate outer boundary of the
surveyed area.

Usage:
```
python3 scan_to_geofence.py input_mission.plan [-o output_geofence.plan]
```

#### plan_to_drone.py

Upload a *QGroundControl* `.plan` mission file to a MAVSDK-compatible drone.

This script connects to a MAVSDK drone via UDP, imports a *QGroundControl* mission
plan, and uploads the mission to the drone.

View the mission in *QGroundControl*'s Plan View after running this script - use "*Download*" button.

Install dependencies with:
```
pip3 install mavsdk --break-system-packages
```

Usage:
```
python3 plan_to_drone.py <path_to_plan_file>
```

------------------

Below is the original README:

# path_planner

![Path image](path.png)

This script generates a boustrophedon (lawnmower pattern) path within specified 
target polygons defined in a KML/KMZ file, while avoiding designated obstacle 
polygons from the same file. It handles coordinate transformations (WGS84 to UTM), 
path generation at a specified angle and separation, obstacle avoidance by 
splitting the path, and stitching the remaining segments together by navigating 
along obstacle boundaries. The final path, along with the target and obstacle 
polygons, is saved to an output KML/KMZ file.

Authors: Peter Lehnér, Gemini-2.5.Pro-03-25
