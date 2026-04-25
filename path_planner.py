#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This script generates a boustrophedon (lawnmower pattern) path within specified 
target polygons defined in a KML/KMZ file, while avoiding designated obstacle 
polygons from the same file. It handles coordinate transformations (WGS84 to UTM), 
path generation at a specified angle and separation, obstacle avoidance by 
splitting the path, and stitching the remaining segments together by navigating 
along obstacle boundaries. The final path, along with the target and obstacle 
polygons, is saved to an output KML/KMZ file.

Authors: Peter Lehnér, Gemini-2.5.Pro-03-25
"""
import sys
import argparse
from shapely.geometry import LineString, Point
from shapely.ops import unary_union, transform
from shapely.affinity import rotate
import pyproj
import os # Needed for path manipulation

from helpers.kml import (read_kml_polygons, save_path_to_kml)
from helpers.path_math import (generate_boustrophedon_path, split_path_by_obstacles, _find_intersection_points)

from helpers.stitching import (order_tracks_along_path, stitch_path_segments_proj)
from helpers.tracks import create_augmented_obstacle_tracks


TOLERANCE = 1e-9  # Tolerance for floating point comparisons

# =======================================
#
# sudo apt-get install python3-shapely python3-pyproj
# pip3 install simplekml --break-system-packages
# 
# usage:
#  path_planner.py [-h] [--output OUTPUT_KML_FILE] [--target TARGET_POLYGON_NAMES] [--angle ANGLE] \
#                       [--sep SEP] [--reverse] [--ignore IGNORE_POLYGON_NAMES] [--safe SAFE] input_kml
#
# Generate a boustrophedon path for specified KML polygons, avoiding others as obstacles.
#
# positional arguments:
#   input_kml             Path to the input KML file.
#
# options:
#   -h, --help            show this help message and exit
#   --output OUTPUT_KML_FILE
#                         Path to the output KML file. If not specified, defaults to <input_kml_basename>_path.kml.
#   --target TARGET_POLYGON_NAMES
#                         Name of a polygon within the KML to generate paths for. Can be specified multiple times. If none specified, defaults to the largest polygon by area.
#   --angle ANGLE         Compass angle for the path lines in degrees (0=N, 90=E, 180=S, 270=W). Default is 90.
#   --sep SEP             Separation between path lines in meters. Default is 0.3.
#   --reverse             Reverse the order of points in the final output path.
#   --ignore IGNORE_POLYGON_NAMES
#                         Name of a polygon within the KML to completely ignore. Can be specified multiple times.
#   --safe SAFE           Safety distance in meters to keep away from target edges and obstacles. Default is 0.0.
#
# Example usage:
#   ./path_planner.py --target "Field A" --ignore "Pond" --angle 90 --sep 0.3 --safe 0.5 input_map.kml
#   cd ~/planner_ws/path_planner
#   ./path_planner.py plans/test_1.kml
#   ./path_planner.py plans/test_2.kml
#   ./path_planner.py plans/test_2.kmz
#   ./path_planner.py plans/test_2.kml --target GeoCage_1 --sep 0.1 --safe 0.5 --angle 45 --ignore Exclusion_1
#   ./path_planner.py plans/test_2.kml --target GeoCage_1 --sep 0.1 --safe 0.5 --angle 45
#
# =======================================

def main():
    parser = argparse.ArgumentParser(description='Generate a boustrophedon path for specified KML polygons, avoiding others as obstacles.')
    parser.add_argument('input_kml', help='Path to the input KML file.')
    parser.add_argument('--output', dest='output_kml_file', default=None,
                        help='Path to the output KML file. If not specified, defaults to <input_kml_basename>_path.kml.')
    parser.add_argument('--target', dest='target_polygon_names', action='append', default=[],
                        help='Name of a polygon within the KML to generate paths for. Can be specified multiple times. If none specified, defaults to the largest polygon by area.')
    parser.add_argument('--angle', type=float, default=90.0, 
                        help='Compass angle for the path lines in degrees (0=N, 90=E, 180=S, 270=W). Default is 90.')
    parser.add_argument('--sep', type=float, default=0.3, 
                        help='Separation between path lines in meters. Default is 0.3.')
    parser.add_argument('--reverse', action='store_true', 
                        help='Reverse the order of points in the final output path.')
    parser.add_argument('--ignore', dest='ignore_polygon_names', action='append', default=[],
                        help='Name of a polygon within the KML to completely ignore. Can be specified multiple times.')
    parser.add_argument('--safe', type=float, default=0.0,
                        help='Safety distance in meters to keep away from target edges and obstacles. Default is 0.0.')

    # --- Check for missing arguments (only input_kml is mandatory now) ---
    # This check might need refinement if other args become mandatory again
    # Check if only script name is passed or help flag
    if len(sys.argv) == 1 or (len(sys.argv) >= 2 and sys.argv[1] in ('-h', '--help')):
        parser.print_help(sys.stderr)
        sys.exit(1)
    # Check if the first argument exists (likely the input KML)
    # This prevents running parsing if the main input is missing
    elif len(sys.argv) >= 2 and not sys.argv[1].startswith('-') and not os.path.exists(sys.argv[1]):
         # Basic check if the first argument looks like a file that doesn't exist
         print(f"Error: Input KML file not found: {sys.argv[1]}")
         sys.exit(1)

    args = parser.parse_args()

    input_kml = args.input_kml
    target_polygon_names_arg = args.target_polygon_names 
    ignore_polygon_names_arg = args.ignore_polygon_names # Get the ignored names
    angle_degrees = args.angle
    separation_in_meters = args.sep
    reverse_final_path = args.reverse 
    safety_distance = args.safe # Get the safety distance

    # --- Determine Output KML filename --- 
    if args.output_kml_file:
        output_kml_file = args.output_kml_file
    else:
        input_basename = os.path.splitext(os.path.basename(input_kml))[0]
        output_kml_file = f"{input_basename}_path.kml"
        
    # --- Determine Output Prefix for KML track name --- 
    output_prefix = os.path.splitext(os.path.basename(output_kml_file))[0]

    print(f"Input KML: {input_kml}")
    print(f"Output KML: {output_kml_file}")
    print(f"Internal KML Track Name: {output_prefix}") 
    print(f"Path Angle: {angle_degrees} degrees")
    print(f"Path Separation: {separation_in_meters} meters")
    print(f"Safety Distance: {safety_distance} meters")

    all_polygons_data = read_kml_polygons(input_kml) 
    
    # --- Filter out ignored polygons FIRST --- 
    ignore_names_set = set(ignore_polygon_names_arg) # Use a set for faster lookup
    filtered_polygons_data = []
    ignored_count = 0
    for name, poly in all_polygons_data:
        if name in ignore_names_set:
            ignored_count += 1
        else:
            filtered_polygons_data.append((name, poly))
    
    if ignored_count > 0:
        print(f"Ignored {ignored_count} polygons based on --ignore argument: {ignore_polygon_names_arg}")
    elif ignore_polygon_names_arg: # Print even if none were found matching
        print(f"Specified polygons to ignore, but none matched in the KML: {ignore_polygon_names_arg}")
        
    # --- Use filtered_polygons_data for the rest of the logic --- 
    
    # Keep track of names and geometries together
    target_polygons_with_names = [] 
    obstacles_with_names = [] 
    target_polygon_names_found = [] # Just for the printout
    obstacle_names = [] # Just for the printout

    # --- Target / Obstacle Separation Logic --- 
    if not target_polygon_names_arg: # Default: largest polygon
        print("No target polygons specified via --target. Finding largest polygon by area.")
        # Use filtered data here
        if not filtered_polygons_data:
            print("Error: No polygons remain after filtering ignored ones.")
            sys.exit(1)
            
        largest_poly = None
        largest_area = -1.0
        largest_name = "Unknown"

        # Calculate areas in a suitable projected CRS for better accuracy
        # We need a temporary projection just for area comparison
        # Find a central point for a representative CRS
        # Use filtered data here
        valid_polys_for_bounds = [p[1] for p in filtered_polygons_data if p[1].is_valid]
        if not valid_polys_for_bounds:
             print("Error: No valid polygons remain after filtering to determine bounds/centroid.")
             sys.exit(1)
             
        combined_bounds = unary_union(valid_polys_for_bounds).bounds
        center_lon = (combined_bounds[0] + combined_bounds[2]) / 2
        center_lat = (combined_bounds[1] + combined_bounds[3]) / 2
        
        temp_utm_zone = int((center_lon + 180) / 6) + 1
        temp_hemisphere = 6 if center_lat >= 0 else 7
        temp_crs_proj = f"EPSG:32{temp_hemisphere}{temp_utm_zone:02d}"
        # print(f"Debug: Using temporary CRS {temp_crs_proj} for area calculation.")
        try:
            temp_transformer = pyproj.Transformer.from_crs("EPSG:4326", temp_crs_proj, always_xy=True)
            can_project_for_area = True
        except pyproj.exceptions.CRSError as e:
            print(f"Warning: Could not create temporary projection {temp_crs_proj} for area calculation ({e}). Area comparison might be less accurate.")
            can_project_for_area = False

        for name, poly_deg in filtered_polygons_data:
            current_area = 0.0
            if can_project_for_area:
                 try:
                     poly_proj = transform(temp_transformer.transform, poly_deg)
                     current_area = poly_proj.area
                 except Exception as e:
                     print(f"Warning: Could not project/calculate area for polygon '{name}': {e}. Skipping area check for this polygon.")
                     continue # Skip polygon if area calculation fails
            else:
                 # Fallback to degree-based area (less accurate but better than nothing)
                 current_area = poly_deg.area 
                 
            if current_area > largest_area:
                 largest_area = current_area
                 largest_poly = poly_deg
                 largest_name = name


        if largest_poly is None:
            print("Error: Could not determine the largest polygon (possibly due to projection/area errors).")
            sys.exit(1)
            
        area_unit = "sq meters approx" if can_project_for_area else "sq degrees approx"
        print(f"Largest polygon found: '{largest_name}' (Area: {largest_area:.2f} {area_unit})")
        target_polygons_with_names.append((largest_name, largest_poly))
        target_polygon_names_found.append(largest_name)
        
        # All other polygons (from the filtered list) are obstacles
        # Use filtered data here
        for name, poly in filtered_polygons_data:
             if poly != largest_poly:
                  obstacles_with_names.append((name, poly)) # Store tuple
                  obstacle_names.append(name)

    else: # Specific targets provided
        print(f"Target polygon names specified via --target: {target_polygon_names_arg}")
        target_names_set = set(target_polygon_names_arg)
        # Store found polygons temporarily to allow reordering later
        found_polygons_map = {} 
        
        # Use filtered data here
        for name, poly in filtered_polygons_data:
            if name in target_names_set:
                if name not in found_polygons_map:
                     found_polygons_map[name] = poly
                # Don't add to obstacles if it's a target
            else:
                obstacles_with_names.append((name, poly)) # Store tuple
                obstacle_names.append(name) # Store obstacle name
                
        # Check if all specified names were found
        found_names_set = set(found_polygons_map.keys())
        missing_names = target_names_set - found_names_set
        if missing_names:
            print(f"Warning: The following specified target polygons were not found (or were ignored): {list(missing_names)}")
            
        if not found_polygons_map:
             print(f"Error: None of the specified target polygons ({target_polygon_names_arg}) were found (or all were ignored) in the KML file.")
             sys.exit(1)
        
        # Reorder target polygons based on argument order, storing tuples
        unique_ordered_names_from_args = []
        seen_names = set()
        for name in target_polygon_names_arg:
            if name in found_polygons_map and name not in seen_names:
                 unique_ordered_names_from_args.append(name)
                 seen_names.add(name)

        # Build the final list of target tuples in the specified order
        target_polygons_with_names = [(name, found_polygons_map[name]) for name in unique_ordered_names_from_args]
        target_polygon_names_found = unique_ordered_names_from_args # Update for printout

    print(f"Selected {len(target_polygons_with_names)} target polygons: {target_polygon_names_found}")
    if obstacle_names:
        print(f"Identified {len(obstacles_with_names)} obstacle polygons: {obstacle_names}")
    else:
        print(f"Identified {len(obstacles_with_names)} obstacle polygons.")
    
    # Extract just the geometries for processing, keep the named lists for saving
    target_polygons_deg = [item[1] for item in target_polygons_with_names]
    obstacles_deg = [item[1] for item in obstacles_with_names]
    
    if not target_polygons_deg:
        print("Error: No target polygons selected. Exiting.")
        sys.exit(1)
        
    # --- Determine CRS and DEFINE TRANSFORMERS before use --- 
    crs_deg = "EPSG:4326"
    try:
        combined_targets_for_centroid = unary_union(target_polygons_deg)
        if combined_targets_for_centroid.is_empty:
             print("Warning: Combined target polygons resulted in empty geometry for centroid calculation. Using first target polygon.")
             if target_polygons_deg:
                  centroid_deg = target_polygons_deg[0].centroid
             else: 
                  raise ValueError("No target polygons available for centroid calculation.")
        else:
             centroid_deg = combined_targets_for_centroid.centroid
    except Exception as e:
        print(f"Warning: Could not calculate combined centroid ({e}). Using centroid of the first target polygon.")
        if target_polygons_deg:
             centroid_deg = target_polygons_deg[0].centroid
        else:
              print("Error: Cannot determine centroid because no target polygons are selected.")
              sys.exit(1)
        
    utm_zone = int((centroid_deg.x + 180) / 6) + 1
    hemisphere_code = 6 if centroid_deg.y >= 0 else 7
    crs_proj = f"EPSG:32{hemisphere_code}{utm_zone:02d}" 
    print(f"Using projected CRS: {crs_proj}")

    # --- Create Transformers HERE --- Fix scope issue
    try:
         transformer_to_proj = pyproj.Transformer.from_crs(crs_deg, crs_proj, always_xy=True)
         transformer_to_deg = pyproj.Transformer.from_crs(crs_proj, crs_deg, always_xy=True)
    except pyproj.exceptions.CRSError as e:
         print(f"Error creating coordinate transformers for CRS {crs_proj}: {e}")
         sys.exit(1)
    
    # --- Project Obstacles to UTM --- 
    obstacles_proj_orig = [] 
    valid_obstacles_deg = [obs for obs in obstacles_deg if obs.is_valid and not obs.is_empty]
    for i, obs_deg in enumerate(valid_obstacles_deg):
        try:
            obstacles_proj_orig.append(transform(transformer_to_proj.transform, obs_deg))
        except Exception as e:
            print(f"Warning: Could not project obstacle {i+1} to UTM: {e}")
            
    # --- Apply Safety Buffer to Projected Obstacles --- 
    obstacles_proj_buffered = []
    if safety_distance > TOLERANCE and obstacles_proj_orig:
        print(f"Applying +{safety_distance:.4f}m safety buffer to {len(obstacles_proj_orig)} projected obstacles...")
        buffer_amount = abs(safety_distance)
        for i, obs_proj in enumerate(obstacles_proj_orig):
            try:
                buffered_obs = obs_proj.buffer(buffer_amount, join_style=2)
                if buffered_obs.is_valid and not buffered_obs.is_empty:
                    obstacles_proj_buffered.append(buffered_obs)
                else:
                    print(f"Warning: Buffering obstacle {i+1} resulted in invalid/empty geometry. Using original projected obstacle for safety.")
                    obstacles_proj_buffered.append(obs_proj) # Fallback to original projected
            except Exception as e:
                print(f"Warning: Error buffering obstacle {i+1}: {e}. Using original projected obstacle for safety.")
                obstacles_proj_buffered.append(obs_proj) # Fallback to original projected
    else:
        # If no safety distance or no obstacles, use the original projected list
        print("No positive safety distance specified or no obstacles found, using original obstacle boundaries.")
        obstacles_proj_buffered = obstacles_proj_orig
            
    print(f"Using {len(obstacles_proj_buffered)} obstacles (potentially buffered) for path avoidance.")

    # --- Generate Base Path (Projected) --- MOVED HERE
    base_path_proj = None
    path_centroid_proj = None 
    path_transformer_to_deg = None # Initialize
    
    # --- Check if target polygons intersect (using projected for accuracy) --- 
    targets_intersect = False
    target_polygons_proj = []
    if len(target_polygons_deg) > 1:
        from itertools import combinations
        try:
             target_polygons_proj = [transform(transformer_to_proj.transform, p) for p in target_polygons_deg]
             for poly1_proj, poly2_proj in combinations(target_polygons_proj, 2):
                  # Use a small tolerance for intersection check?
                  if poly1_proj.buffer(TOLERANCE).intersects(poly2_proj.buffer(TOLERANCE)):
                       targets_intersect = True
                       print("Target polygons intersect (or touch). Merging them for path generation.")
                       break 
        except Exception as e:
             print(f"Warning: Error projecting/checking intersection of target polygons: {e}. Proceeding assuming they might intersect.")
             targets_intersect = True
    
    if targets_intersect or len(target_polygons_deg) == 1:
        print("Generating base path for merged/single target area...")
        # Combine the original degree polygons for the path generation input
        main_geometry_deg = unary_union(target_polygons_deg) 
        if not main_geometry_deg.is_valid or main_geometry_deg.is_empty:
             print("Error: Merged target geometry is invalid or empty. Exiting.")
             sys.exit(1)
        
        # Call generate_boustrophedon_path once 
        base_path_proj, path_centroid_proj, _, path_transformer_to_deg = generate_boustrophedon_path(
            main_geometry_deg, 
            separation_meters=separation_in_meters, 
            angle_deg=angle_degrees,
            safety_distance=safety_distance # Pass safety distance
        )
        
    else: # Targets do not intersect, generate paths separately and combine
        print("Generating base paths for separate target areas...")
        all_paths_coords_proj = []
        first_path_transformer_to_deg = None # Store transformer from the first path generated
        
        for i, poly_deg in enumerate(target_polygons_deg): 
            poly_name = target_polygon_names_found[i]
            print(f"  Generating path for target {i+1}/{len(target_polygons_deg)} ('{poly_name}')...")
            
            path_proj, current_centroid_proj, _, current_transformer_to_deg = generate_boustrophedon_path(
                poly_deg, 
                separation_meters=separation_in_meters, 
                angle_deg=angle_degrees,
                safety_distance=safety_distance # Pass safety distance
            )
            
            if not path_centroid_proj: # Store centroid from first path if not set
                 path_centroid_proj = current_centroid_proj
            if not first_path_transformer_to_deg: # Store transformer from first path
                 first_path_transformer_to_deg = current_transformer_to_deg
                 
            if path_proj and not path_proj.is_empty and path_proj.coords:
                 path_coords = list(path_proj.coords)
                 # Simple concatenation - might need smarter joining later if gaps are large
                 all_paths_coords_proj.extend(path_coords)
            else:
                 print(f"Warning: No valid path generated for target '{poly_name}'.")
                 
        if not all_paths_coords_proj:
            print("Error: No path segments generated for any separate target polygon. Exiting.")
            sys.exit(1)
            
        # Clean the combined path
        cleaned_combined_coords = []
        if all_paths_coords_proj:
             cleaned_combined_coords.append(all_paths_coords_proj[0])
             for k in range(1, len(all_paths_coords_proj)):
                 if Point(all_paths_coords_proj[k]).distance(Point(cleaned_combined_coords[-1])) > TOLERANCE:
                     cleaned_combined_coords.append(all_paths_coords_proj[k])
                     
        if len(cleaned_combined_coords) < 2:
             print("Error: Combined path has fewer than 2 points after cleaning. Exiting.")
             sys.exit(1)
             
        base_path_proj = LineString(cleaned_combined_coords)
        path_transformer_to_deg = first_path_transformer_to_deg # Use transformer from first segment for final conversion
        print(f"Combined separate paths into a single base path (Points: {len(base_path_proj.coords)}).")

    # --- Error check for base path generation --- 
    if base_path_proj is None or not base_path_proj.is_valid or base_path_proj.is_empty:
        print("Error: Base path generation failed or resulted in empty path. Exiting.")
        sys.exit(1)
        
    # --- The rest of the process uses the generated base_path_proj and the buffered obstacles --- 
    
    # --- Find Intersection Points (Projected) --- 
    # Use the BUFFERED obstacles to find intersection points
    intersection_points_proj = _find_intersection_points(base_path_proj, obstacles_proj_buffered)
    
    # --- Create Augmented Obstacle Tracks (Projected) ---
    # Use the BUFFERED obstacles to create the tracks for stitching
    obstacle_tracks_proj = create_augmented_obstacle_tracks(obstacles_proj_buffered, intersection_points_proj)
    
    # --- Split the Base Path by Obstacles (Projected) ---
    # Pass the BUFFERED obstacles to split_path_by_obstacles
    split_tracks_proj = split_path_by_obstacles(base_path_proj, obstacles_proj_buffered)
    
    final_stitched_path_proj = None 
    final_path_length_m = 0.0
    final_path_points = 0
    final_path_segments = 0

    if not split_tracks_proj:
        print("Warning: No path tracks generated after splitting by obstacles. Output KML might be empty or incomplete.")
    else:
        # --- Order Tracks Properly (Projected) ---
        ordered_split_tracks_proj = order_tracks_along_path(split_tracks_proj, base_path_proj)
        
        if not ordered_split_tracks_proj:
             print("Error: Failed to order split tracks. Cannot stitch.")
        else:
             # --- Stitch Path Segments (Projected) ---
             final_stitched_path_proj = stitch_path_segments_proj(ordered_split_tracks_proj, obstacle_tracks_proj)

             if not final_stitched_path_proj or final_stitched_path_proj.is_empty:
                  print("Warning: Stitching resulted in an empty or invalid path.")
                  final_stitched_path_proj = None 

    # --- Calculate final metrics IF a stitched path exists --- 
    if final_stitched_path_proj:
        final_path_length_m = final_stitched_path_proj.length
        final_path_points = len(final_stitched_path_proj.coords) if final_stitched_path_proj.coords else 0
        final_path_segments = max(0, final_path_points - 1)

    # --- Transform Results back to Degrees for Output --- 
    final_stitched_path_deg = None
    if final_stitched_path_proj:
        # Use the transformer associated with the final path generation
        # (either from the single call or the first call in the loop)
        if path_transformer_to_deg: 
            try:
                final_stitched_path_deg = transform(path_transformer_to_deg.transform, final_stitched_path_proj)
            except Exception as e:
                print(f"Error transforming final stitched path to degrees: {e}")
                final_stitched_path_deg = None
        else:
            print("Error: Could not determine the correct transformer to convert final path to degrees.")
            
    # --- Save Results to KML --- 
    save_path_to_kml(
        final_stitched_path_deg,
        output_kml_file,
        output_prefix, 
        reverse_final_path,
        target_polygons_with_names, # Pass targets with names
        obstacles_with_names        # Pass obstacles with names
        )

    # --- Final Summary --- 
    print(f"\nProcessing complete.")
    if final_stitched_path_proj: 
        print(f"  Final Path Segments: {final_path_segments}")
        print(f"  Final Path Length:   {final_path_length_m:.2f} meters")
    elif base_path_proj and not split_tracks_proj: 
         print("  Warning: Generated base path was entirely removed by obstacles.")
         print("  No final path generated.")
    elif base_path_proj: 
         if len(split_tracks_proj) == 1 and split_tracks_proj[0].equals(base_path_proj):
              base_len = base_path_proj.length
              base_pts = len(base_path_proj.coords) if base_path_proj.coords else 0
              base_segs = max(0, base_pts - 1)
              print(f"  Path generated (no obstacles intersected or stitching failed/not needed).")
              print(f"  Path Segments: {base_segs}")
              print(f"  Path Length:   {base_len:.2f} meters")
         else: 
              print(f"  Warning: Stitching failed after splitting. Reporting metrics for the base path before splitting.")
              base_len = base_path_proj.length
              base_pts = len(base_path_proj.coords) if base_path_proj.coords else 0
              base_segs = max(0, base_pts - 1)
              print(f"  Base Path Segments: {base_segs}")
              print(f"  Base Path Length:   {base_len:.2f} meters")
              print("  Final output path may be incomplete.")
    else: 
        print("  No final path was generated.")
        
    print(f"  Output KML saved to: {output_kml_file}") 


if __name__ == "__main__":
    main() 