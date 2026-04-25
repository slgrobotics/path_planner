> This is a modified version of the original *Path Planner* by Peter Lehnér.
>
> This version is able to load *".plan"* files created in [QGroundControl](https://qgroundcontrol.com/), and also outputs paths in QGC format (named "*_path.plan")
>
> All credit goes to the original project and its author - Peter Lehnér.

**Modifications:**
- methods are moved (mostly verbatim) to *helpers* directory
- added `helpers/qgc_format.py` to handle QGC ".plan" format
- added sample files to demonstrate functionality

`./path_planner.py plans/test_3.kml --sep 0.4 --safe 0.5 --angle 45`
<img width="2018" height="1631" alt="Screenshot from 2026-04-24 13-32-42" src="https://github.com/user-attachments/assets/57dff9fd-db30-497a-b5d4-927bf67cafd9" />

`./path_planner.py plans/test_3.kml --sep 0.4 --safe 0.5 --angle 45`
<img width="1137" height="1264" alt="Screenshot from 2026-04-24 20-16-52" src="https://github.com/user-attachments/assets/9fc17068-518c-420d-b3c3-d09b0fcdb104" />

`./path_planner.py plans/geofence_qgroundcontrol_multi2.plan --target GeoCage_1 --sep 0.4 --safe 0.5 --angle 45`
<img width="1449" height="868" alt="Screenshot from 2026-04-24 21-04-45" src="https://github.com/user-attachments/assets/ab90251e-be44-4049-bdef-2f86a7e42c64" />

## How to use

**Install prerequisites:**
```
sudo apt-get install python3-shapely python3-pyproj
pip3 install simplekml --break-system-packages
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
