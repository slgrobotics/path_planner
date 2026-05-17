from shapely.geometry import LineString, Point

from .rdp_simplification import (rdp_simplify)

TOLERANCE = 1e-9  # Tolerance for floating point comparisons


# =======================================
# Stitching Helper Functions START 
# =======================================

def find_point_index_on_track(point_coord, track, tolerance=1e-6):
    """Finds the index of a point coordinate in a LineString's coords using tolerance."""
    target_point = Point(point_coord)
    min_dist = float('inf')
    best_idx = -1
    if not track.coords:
        return -1
        
    for i, coord in enumerate(track.coords):
        dist = target_point.distance(Point(coord))
        if dist < min_dist:
            min_dist = dist
            best_idx = i
            
    if min_dist < tolerance:
        return best_idx
    else:
        # Try projecting point onto the line to find the closest segment, then check endpoints
        # This might help if the exact point isn't a vertex but lies on a segment
        # This adds complexity, let's stick to vertex check for now.
        # print(f"Debug: Point {point_coord} not found within tolerance {tolerance} on track. Min dist: {min_dist}")
        return -1

def calculate_path_length(coords_list):
    """Calculates the length of a path defined by a list of coordinates (assumes projected CRS)."""
    if not coords_list or len(coords_list) < 2:
        return 0.0
    try:
        return LineString(coords_list).length
    except Exception as e:
        # print(f"Error calculating length for coords list (length {len(coords_list)}): {e}") # Removed
        return 0.0

def find_bridging_obstacle_path(end_point_coord, start_point_coord, obstacle_tracks_proj, tolerance=1e-7):
    """
    Finds the shortest path segment along an obstacle track boundary connecting two points.
    Operates on projected coordinates. Returns the list of coordinates for the shortest path.
    """
    end_pt = Point(end_point_coord)
    start_pt = Point(start_point_coord)

    best_bridge_coords = None
    min_bridge_len = float('inf')
    found_on_any_track = False
    bridging_lookup_tolerance = 1e-5 # Use a larger tolerance for lookups within this function

    for i, obstacle_track in enumerate(obstacle_tracks_proj):
        if not obstacle_track.is_valid or not obstacle_track.coords or len(obstacle_track.coords) < 2:
            continue
        coords = list(obstacle_track.coords)
        
        # Check if the track is closed (first and last points are close)
        # This is important for navigating loops correctly
        is_closed = Point(coords[0]).distance(Point(coords[-1])) < tolerance 
        
        # If the track isn't closed, calculating forward/backward paths is trickier.
        # For simplicity, let's only consider closed obstacle tracks for bridging for now.
        # Open tracks (lines) might represent parts of obstacle boundaries but aren't ideal for routing along.
        if not is_closed and len(coords) > 2 : # Need > 2 points to be potentially closed
             # print(f"Debug: Skipping obstacle track {i} for bridging as it's not closed.")
             continue
             
        n = len(coords)
        # Adjust n if closed: number of segments is n-1
        num_segments = n - 1 if is_closed and n > 1 else n 
        if num_segments == 0: continue # Single point track
        
        idx1 = find_point_index_on_track(end_point_coord, obstacle_track, tolerance=bridging_lookup_tolerance)
        idx2 = find_point_index_on_track(start_point_coord, obstacle_track, tolerance=bridging_lookup_tolerance)

        if idx1 != -1 and idx2 != -1:
            found_on_any_track = True
            current_segment = []
            current_len = 0.0
            
            if idx1 == idx2:
                current_segment = coords[idx1:idx1+1] # Path is just the single point
                current_len = 0.0
            elif not is_closed:
                 # Handle open linestring - just take the direct segment
                 start_index = min(idx1, idx2)
                 end_index = max(idx1, idx2)
                 segment = coords[start_index : end_index+1]
                 if idx1 > idx2: # Reverse if start point comes after end point in list
                      segment = segment[::-1]
                 current_segment = segment
                 current_len = calculate_path_length(current_segment)
            else: # Closed loop logic
                # Path forward
                path_fwd_indices = []
                curr = idx1
                while curr != idx2:
                    path_fwd_indices.append(curr)
                    # Use modulo num_segments (which is n-1 for closed loops)
                    curr = (curr + 1) % num_segments 
                path_fwd_indices.append(idx2)
                segment_fwd = [coords[j] for j in path_fwd_indices]
                len_fwd = calculate_path_length(segment_fwd)

                # Path backward
                path_bwd_indices = []
                curr = idx1
                while curr != idx2:
                    path_bwd_indices.append(curr)
                    # Use modulo num_segments for backward movement too
                    curr = (curr - 1 + num_segments) % num_segments 
                path_bwd_indices.append(idx2)
                segment_bwd = [coords[j] for j in path_bwd_indices]
                len_bwd = calculate_path_length(segment_bwd)

                # Choose shorter path
                if len_fwd <= len_bwd:
                    current_segment = segment_fwd
                    current_len = len_fwd
                else:
                    current_segment = segment_bwd
                    current_len = len_bwd

            # Update best bridge if this one is shorter
            if current_len < min_bridge_len:
                min_bridge_len = current_len
                best_bridge_coords = current_segment
                # print(f"Found new best bridge (len {min_bridge_len:.2f}m) on obstacle track {i}") # Removed

    if not found_on_any_track:
        # print(f"Warning: Could not find *both* points {end_point_coord} and {start_point_coord} on any single obstacle track.") # Removed
        # Maybe try finding closest points on tracks instead? More complex. For now, fail.
        return [] 
    if best_bridge_coords is None:
         # print(f"Warning: Failed to determine shortest bridge path between {end_point_coord} and {start_point_coord} despite finding points.") # Removed
         return []
         
    # print(f"Debug: Found bridge path with {len(best_bridge_coords)} points, length {min_bridge_len:.3f}m")
    return best_bridge_coords

def order_tracks_along_path(tracks_proj, base_path_proj):
    """Sorts tracks based on the projection distance of their start points onto a base path."""
    if not tracks_proj:
        return []
    if not base_path_proj.is_valid or not base_path_proj.coords:
        # print("Warning: Cannot order tracks along an invalid or empty base path. Returning original order.") # Removed
        return tracks_proj 
        
    track_distances = []
    for i, track in enumerate(tracks_proj):
        if not track.is_valid or not track.coords:
            # print(f"Warning: Skipping invalid or empty track {i} during ordering.") # Removed
            distance = float('inf')
        else:
            start_point = Point(track.coords[0])
            try:
                 distance = base_path_proj.project(start_point, normalized=False)
            except Exception as e:
                 print(f"Error projecting start point of track {i}: {e}. Placing track at end.")
                 distance = float('inf')
        track_distances.append({'distance': distance, 'track': track, 'original_index': i})
        
    # Sort by distance along the base path
    track_distances.sort(key=lambda item: item['distance'])
    
    ordered_tracks = []
    skipped_indices = []
    for item in track_distances:
        if item['distance'] == float('inf'):
             skipped_indices.append(item['original_index'])
        else:
             ordered_tracks.append(item['track'])
             
    # if skipped_indices:
    #      print(f"Warning: {len(skipped_indices)} tracks (original indices: {skipped_indices}) could not be ordered and were omitted.") # Removed
    # print(f"Ordered {len(ordered_tracks)} tracks along the base path.") # Removed
    return ordered_tracks

def stitch_path_segments_proj(ordered_tracks_proj, obstacle_tracks_proj):
    """
    Stitches ordered path segments together using the shortest path along obstacle boundaries.
    Operates on projected coordinates.

    Returns:
        A single LineString representing the final stitched path in projected coordinates,
        or None if stitching fails.
    """
    simplify_tolerance = 0.01 # Use 1cm for bridge pre-simplification

    if not ordered_tracks_proj:
        print("Error: No path segments provided to stitch.")
        return None
        
    valid_tracks = [t for t in ordered_tracks_proj if t.is_valid and t.coords]
    if len(valid_tracks) != len(ordered_tracks_proj):
        # print(f"Warning: Filtering out {len(ordered_tracks_proj) - len(valid_tracks)} invalid/empty tracks before stitching.") # Removed
        ordered_tracks_proj = valid_tracks
        if not ordered_tracks_proj:
             print("Error: No valid path segments remaining after filtering.")
             return None
             
    # print(f"Starting stitching process for {len(ordered_tracks_proj)} segments...") # Removed
    final_coords = list(ordered_tracks_proj[0].coords)
    # print(f"  Initial coords from track 0: {len(final_coords)}") # Removed

    for i in range(len(ordered_tracks_proj) - 1):
        # print(f"\n  Processing gap {i}:") # Removed
        # print(f"    Coords before bridge: {len(final_coords)}") # Removed
        track_i = ordered_tracks_proj[i]
        track_i_plus_1 = ordered_tracks_proj[i+1]
        
        if not track_i.coords or not track_i_plus_1.coords:
             print(f"Error: Encountered track with no coords during stitching loop (i={i}). Stopping.")
             # Return what we have stitched so far
             return LineString(final_coords) if len(final_coords) >= 2 else None
             
        end_i = track_i.coords[-1]
        start_i_plus_1 = track_i_plus_1.coords[0]
        # print(f"    Stitching gap between track {i} (ends {end_i}) and track {i+1} (starts {start_i_plus_1})") # Removed
        
        bridge_coords = find_bridging_obstacle_path(end_i, start_i_plus_1, obstacle_tracks_proj)
        # print(f"    Found bridge coords: {len(bridge_coords)}") # Removed

        if len(bridge_coords) >= 3:
            try:
                bridge_line = LineString(bridge_coords)
                simplified_bridge = bridge_line.simplify(simplify_tolerance, preserve_topology=True)
                if simplified_bridge.is_valid and not simplified_bridge.is_empty and len(simplified_bridge.coords) >= 2:
                    new_bridge_coords = list(simplified_bridge.coords)
                    if len(new_bridge_coords) < len(bridge_coords):
                        # print(f"    Pre-simplified bridge from {len(bridge_coords)} to {len(new_bridge_coords)} points.") # Removed
                        bridge_coords = new_bridge_coords
                # else:
                    # print(f"    Warning: Pre-simplification of bridge resulted in invalid/empty geometry.") # Removed
            except Exception as e:
                pass
                # print(f"    Warning: Error during bridge pre-simplification: {e}") # Removed

        if not bridge_coords:
            # Fallback: use direct straight-line bridge between end and start to continue stitching.
            # This may cross obstacles but allows generating a continuous path instead of failing.
            print(f"Warning: Could not find bridge path between track {i} and {i+1}. Using straight-line fallback.")
            bridge_coords = [end_i, start_i_plus_1]

        coords_before_bridge_append = len(final_coords)
        if len(bridge_coords) > 0:
             if Point(final_coords[-1]).distance(Point(bridge_coords[0])) < 1e-7: 
                 if len(bridge_coords) > 1: # Add bridge points except the first one
                     final_coords.extend(bridge_coords[1:])
             else: # Gap is too large or points don't match, append the whole bridge
                 # print(f"Warning: Bridge start {bridge_coords[0]} doesn't match current end {final_coords[-1]}. Appending full bridge.") # Removed
                 final_coords.extend(bridge_coords)
        # print(f"    Coords after bridge append: {len(final_coords)} (added {len(final_coords) - coords_before_bridge_append})") # Removed

        coords_before_track_append = len(final_coords)
        next_track_coords = list(track_i_plus_1.coords)
        if len(next_track_coords) > 0:
             # Check distance between last point of current path and first point of next track
             if Point(final_coords[-1]).distance(Point(next_track_coords[0])) < 1e-7:
                 if len(next_track_coords) > 1: # Add next track points except the first one
                     final_coords.extend(next_track_coords[1:])
             else: # Gap is too large or points don't match, append the whole track
                 # print(f"Warning: Next track start {next_track_coords[0]} doesn't match current end {final_coords[-1]}. Appending full track.") # Removed
                 final_coords.extend(next_track_coords)
        # print(f"    Coords after track {i+1} append: {len(final_coords)} (added {len(final_coords) - coords_before_track_append})") # Removed

    # print(f"\nCompleted stitching loop. Coords before final cleanup: {len(final_coords)}") # Removed
    
    if not final_coords:
        print("Error: Final coordinate list is empty after stitching.")
        return None
        
    # --- Final Cleanup and Simplification ---
    # 1. Basic deduplication
    # print(f"Running basic deduplication on {len(final_coords)} points...") # Removed
    deduplicated_coords = [final_coords[0]]
    for j in range(1, len(final_coords)):
        if Point(final_coords[j]).distance(Point(deduplicated_coords[-1])) > TOLERANCE:
            deduplicated_coords.append(final_coords[j])
    # print(f"  Coords after deduplication: {len(deduplicated_coords)}") # Removed
    
    # 2. RDP simplification
    rdp_tolerance = 0.05 # 5cm tolerance for final path simplification
    # print(f"Running RDP simplification with tolerance: {rdp_tolerance}m") # Removed
    if len(deduplicated_coords) >= 3:
        try:
            simplified_coords = rdp_simplify(deduplicated_coords, rdp_tolerance)
            # print(f"  Coords after RDP: {len(simplified_coords)}") # Removed
        except Exception as e:
            print(f"Error during RDP simplification: {e}. Using deduplicated coords.")
            simplified_coords = deduplicated_coords
        else:
            simplified_coords = deduplicated_coords
        
    cleaned_coords = simplified_coords
    # print(f"Cleaned stitched path contains {len(cleaned_coords)} points.") # Removed
    
    if len(cleaned_coords) < 2:
        print("Error: Final stitched path has less than 2 unique points after cleanup.")
        return None
        
    return LineString(cleaned_coords)

# =======================================
# Stitching Helper Functions END 
# =======================================
