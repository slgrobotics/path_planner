
from shapely.geometry import Polygon, LineString, MultiPolygon, Point, MultiLineString

TOLERANCE = 1e-9  # Tolerance for floating point comparisons

def create_augmented_obstacle_tracks(obstacles_proj, intersection_points_proj):
    """
    Creates LineString tracks for each obstacle (PROJECTED), including original vertices
    and points (PROJECTED) where the base path intersected the obstacle boundary.
    
    Returns:
        List of LineString objects representing the augmented obstacle tracks (PROJECTED).
    """
    # Reverted to previous version before error-introducing change
    augmented_tracks_proj = []
    intersection_points_geom = [Point(p) for p in intersection_points_proj]
    valid_obstacles_proj = [p for p in obstacles_proj 
                           if isinstance(p, (Polygon, MultiPolygon)) and p.is_valid and not p.is_empty]

    for i, obstacle in enumerate(valid_obstacles_proj):
        obstacle_boundary = obstacle.boundary 
        
        if obstacle_boundary.is_empty or not isinstance(obstacle_boundary, (LineString, MultiLineString)):
            continue

        boundaries_to_process = []
        if isinstance(obstacle_boundary, LineString):
            if obstacle_boundary.coords:
                 boundaries_to_process.append(obstacle_boundary)
        elif isinstance(obstacle_boundary, MultiLineString):
             boundaries_to_process.extend([ls for ls in obstacle_boundary.geoms if ls.coords])
             
        if not boundaries_to_process:
            continue

        for b_idx, boundary_ls in enumerate(boundaries_to_process):
            if not boundary_ls.coords: continue
             
            original_coords = list(boundary_ls.coords)
            relevant_intersections = []
            for pt_geom in intersection_points_geom:
                if boundary_ls.distance(pt_geom) < TOLERANCE:
                     relevant_intersections.append(pt_geom)
                     
            combined_points_geom = {Point(p) for p in original_coords} | set(relevant_intersections)
            
            filtered_points_geom = []
            if combined_points_geom:
                temp_list = list(combined_points_geom)
                if temp_list:
                     filtered_points_geom.append(temp_list[0])
                     for k in range(1, len(temp_list)):
                         is_unique = True
                         for existing_pt in filtered_points_geom:
                              if temp_list[k].distance(existing_pt) < TOLERANCE:
                                   is_unique = False
                                   break
                         if is_unique:
                              filtered_points_geom.append(temp_list[k])

            if len(filtered_points_geom) < 2:
                continue
                
            try:
                point_distances = []
                for pt in filtered_points_geom:
                     distance_along = boundary_ls.project(pt)
                     point_distances.append({'point': pt, 'distance': distance_along})
                     
                point_distances.sort(key=lambda item: item['distance'])
                
                sorted_augmented_coords = [(item['point'].x, item['point'].y) for item in point_distances]

                is_original_closed = Point(original_coords[0]).distance(Point(original_coords[-1])) < TOLERANCE
                is_augmented_closed = Point(sorted_augmented_coords[0]).distance(Point(sorted_augmented_coords[-1])) < TOLERANCE

                if is_original_closed and not is_augmented_closed:
                     sorted_augmented_coords.append(sorted_augmented_coords[0])
                elif not is_original_closed and is_augmented_closed and len(sorted_augmented_coords)>1:
                     sorted_augmented_coords = sorted_augmented_coords[:-1]

                if len(sorted_augmented_coords) >= 2:
                     final_track_coords = [sorted_augmented_coords[0]]
                     for k in range(1, len(sorted_augmented_coords)):
                          if Point(sorted_augmented_coords[k]).distance(Point(final_track_coords[-1])) > TOLERANCE:
                               final_track_coords.append(sorted_augmented_coords[k])
                               
                     if len(final_track_coords) >= 2:
                         augmented_tracks_proj.append(LineString(final_track_coords))

            except Exception as e:
                 print(f"Error creating augmented track component for obstacle {i+1} boundary {b_idx}: {e}")
            
    return augmented_tracks_proj

