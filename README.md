> This is a modified version of the original Path Planner by Peter Lehnér.
>
> This version is able to load ".plan" files created in QGroundControl, and outputs paths in QGC format, named "*_path.plan"
>
> All credit goes to the original project and its author.

`./path_planner.py plans/test_3.kml --sep 0.4 --safe 0.5 --angle 45`
<img width="2018" height="1631" alt="Screenshot from 2026-04-24 13-32-42" src="https://github.com/user-attachments/assets/57dff9fd-db30-497a-b5d4-927bf67cafd9" />

`./path_planner.py plans/test_3.kml --sep 0.4 --safe 0.5 --angle 45`
<img width="1137" height="1264" alt="Screenshot from 2026-04-24 20-16-52" src="https://github.com/user-attachments/assets/9fc17068-518c-420d-b3c3-d09b0fcdb104" />


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
