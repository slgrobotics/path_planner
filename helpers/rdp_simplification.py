import math

TOLERANCE = 1e-9  # Tolerance for floating point comparisons


# =======================================
# RDP Simplification Helpers START
# =======================================

def perpendicular_distance(point, line_start, line_end):
    """Calculates the perpendicular distance from a point to a line segment defined by start and end points."""
    px, py = point
    ax, ay = line_start
    bx, by = line_end

    # Vector from A to B
    vx = bx - ax
    vy = by - ay
    # Vector from A to P
    wx = px - ax
    wy = py - ay

    mag_v_sq = vx**2 + vy**2
    if mag_v_sq < TOLERANCE**2: # Avoid division by zero if start == end
        return math.sqrt(wx**2 + wy**2)

    # Parameter t representing projection of AP onto AB
    dot = wx * vx + wy * vy
    t = dot / mag_v_sq

    if t < 0.0: # Closest point is A
        closest_x, closest_y = ax, ay
    elif t > 1.0: # Closest point is B
        closest_x, closest_y = bx, by
    else: # Closest point is projection
        closest_x = ax + t * vx
        closest_y = ay + t * vy

    # Distance from P to closest point
    dx = px - closest_x
    dy = py - closest_y
    return math.sqrt(dx**2 + dy**2)

def rdp_simplify(point_list, epsilon):
    """Simplifies a list of points using the Ramer-Douglas-Peucker algorithm."""
    if len(point_list) < 3:
        return point_list # Nothing to simplify

    start_point = point_list[0]
    end_point = point_list[-1]
    max_dist = 0.0
    max_index = 0

    # Find the point furthest from the line segment connecting start and end
    for i in range(1, len(point_list) - 1):
        dist = perpendicular_distance(point_list[i], start_point, end_point)
        if dist > max_dist:
            max_dist = dist
            max_index = i

    # If max distance is greater than epsilon, recursively simplify
    if max_dist > epsilon:
        # Recursive call on the first part
        results1 = rdp_simplify(point_list[:max_index+1], epsilon)
        # Recursive call on the second part
        results2 = rdp_simplify(point_list[max_index:], epsilon)

        # Combine the results, excluding the duplicate middle point
        return results1[:-1] + results2
    else:
        # All intermediate points are within tolerance, keep only start and end
        return [start_point, end_point]

# =======================================
# RDP Simplification Helpers END
# =======================================
