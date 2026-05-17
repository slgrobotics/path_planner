#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate a lawnmower scan path by squeezing the target polygon area with a given
separation and respecting exclusion zones.

Supports QGroundControl .plan files and KML/KMZ polygon files.
"""

import sys
import os
import argparse
from shapely.geometry import Polygon, MultiPolygon
from shapely.ops import unary_union, transform
import pyproj

from helpers.kml_format import read_kml_polygons, save_path_to_kml
from helpers.qgc_format import read_qgc_plan_polygons, save_path_to_qgc_plan
from helpers.path_math import generate_boustrophedon_path, split_path_by_obstacles, _find_intersection_points
from helpers.stitching import order_tracks_along_path, stitch_path_segments_proj
from helpers.tracks import create_augmented_obstacle_tracks


def filter_polygons_by_name(polygons, names_to_ignore):
    if not names_to_ignore:
        return polygons
    return [(name, poly) for name, poly in polygons if name not in names_to_ignore]


def choose_target_and_obstacles_from_kml(polygons, target_names):
    """Choose target polygons and obstacle polygons from a KML/KMZ polygon list."""
    if target_names:
        selected = [(name, poly) for name, poly in polygons if name in target_names]
        missing = [name for name in target_names if name not in {p[0] for p in polygons}]
        if missing:
            print(f"Warning: Target polygon names not found in KML: {missing}")
        if not selected:
            print(f"Warning: No target polygons were selected by --target. Falling back to the largest polygon.")
    else:
        selected = []

    if not selected:
        if not polygons:
            return [], []
        largest = max(polygons, key=lambda item: item[1].area)
        selected = [largest]
        print(f"Selected largest polygon as target: {largest[0]}")

    selected_names = {name for name, _ in selected}
    obstacles = [(name, poly) for name, poly in polygons if name not in selected_names]
    return selected, obstacles


def choose_target_and_obstacles_from_plan(polygons, target_names):
    """Choose target and exclusion polygons from a QGC .plan polygon list."""
    inclusion_polygons = []
    exclusion_polygons = []
    for name, poly in polygons:
        suffix = name.rsplit('_', 1)[-1].lower()
        if suffix == 'exclusion':
            exclusion_polygons.append((name, poly))
        elif suffix == 'inclusion':
            inclusion_polygons.append((name, poly))
        else:
            inclusion_polygons.append((name, poly))

    all_names = {name for name, _ in polygons}
    missing = [name for name in target_names if name not in all_names]
    if missing:
        print(f"Warning: Target polygon names not found in plan file: {missing}")

    if target_names:
        selected = [(name, poly) for name, poly in inclusion_polygons if name in target_names]
        if not selected and inclusion_polygons:
            print("Warning: No specified target names matched inclusion polygons. Using all inclusion polygons.")
            selected = list(inclusion_polygons)
        elif not selected:
            selected = [(name, poly) for name, poly in polygons if name in target_names]
            if selected:
                print("Warning: Selected target names from plan polygons without explicit inclusion/exclusion.")
    else:
        if inclusion_polygons:
            selected = list(inclusion_polygons)
        else:
            if not polygons:
                return [], []
            largest = max(polygons, key=lambda item: item[1].area)
            selected = [largest]
            print(f"Selected largest polygon as target: {largest[0]}")

    exclusion_names = {name for name, _ in selected}
    obstacles = [(name, poly) for name, poly in exclusion_polygons if name not in exclusion_names]
    if not target_names and not inclusion_polygons:
        obstacle_candidates = [(name, poly) for name, poly in polygons if name not in exclusion_names]
        obstacles.extend(obstacle_candidates)
    return selected, obstacles


def parse_input_file(input_file, target_names, ignore_names, num_segments):
    """Read polygons from the input file and separate target and obstacle polygons."""
    if input_file.lower().endswith('.plan'):
        polygons = read_qgc_plan_polygons(input_file, num_segments)
        polygons = filter_polygons_by_name(polygons, ignore_names)
        targets, obstacles = choose_target_and_obstacles_from_plan(polygons, target_names)
    else:
        polygons = read_kml_polygons(input_file)
        polygons = filter_polygons_by_name(polygons, ignore_names)
        targets, obstacles = choose_target_and_obstacles_from_kml(polygons, target_names)

    return targets, obstacles


def build_output_paths(input_file, output_file):
    output_kml_file = None
    output_plan_file = None
    if output_file:
        if output_file.lower().endswith('.kml'):
            output_kml_file = output_file
        elif output_file.lower().endswith('.plan'):
            output_plan_file = output_file
        else:
            output_kml_file = output_file
            print(f"Warning: Unknown output extension, saving as KML: {output_file}")
    else:
        base = os.path.splitext(os.path.basename(input_file))[0]
        output_kml_file = f"{base}_squeeze.kml"
        output_plan_file = f"{base}_squeeze.plan"
    return output_kml_file, output_plan_file


def project_polygons(polygons, transformer):
    projected = []
    for name, poly in polygons:
        try:
            projected.append(transform(transformer.transform, poly))
        except Exception as e:
            print(f"Warning: Could not project polygon '{name}': {e}")
    return projected


def main():
    parser = argparse.ArgumentParser(description='Generate a squeezed lawnmower path from a target polygon while obeying exclusion zones.')
    parser.add_argument('input_file', help='Path to the input KML/KMZ or QGC Plan file.')
    parser.add_argument('-o', '--output', dest='output_file', default=None,
                        help='Path to the output file. Use .kml for KML only, .plan for Plan only, or omit to write both.')
    parser.add_argument('--target', dest='target_polygon_names', action='append', default=[],
                        help='Name of a target polygon to use. Can be specified multiple times.')
    parser.add_argument('--ignore', dest='ignore_polygon_names', action='append', default=[],
                        help='Name of a polygon to ignore completely. Can be specified multiple times.')
    parser.add_argument('--angle', type=float, default=90.0,
                        help='Scan direction angle in degrees (0=N, 90=E, 180=S, 270=W). Default 90.')
    parser.add_argument('--sep', type=float, default=0.3,
                        help='Line separation in meters. Default 0.3.')
    parser.add_argument('--safe', type=float, default=0.0,
                        help='Safety distance in meters from boundaries and obstacles. Default 0.0.')
    parser.add_argument('--reverse', action='store_true', help='Reverse the final path order.')
    parser.add_argument('--segments', type=int, default=8,
                        help='Number of segments to approximate circles in QGC plans. Default 8.')

    if len(sys.argv) == 1 or (len(sys.argv) >= 2 and sys.argv[1] in ('-h', '--help')):
        parser.print_help(sys.stderr)
        sys.exit(1)

    args = parser.parse_args()

    if not os.path.exists(args.input_file):
        print(f"Error: Input file not found: {args.input_file}")
        sys.exit(1)

    targets, obstacles = parse_input_file(args.input_file, args.target_polygon_names, args.ignore_polygon_names, args.segments)
    if not targets:
        print("Error: No target polygons selected for the squeezed path.")
        sys.exit(1)

    target_names = [name for name, _ in targets]
    obstacle_names = [name for name, _ in obstacles]
    print(f"Target polygons: {target_names}")
    if obstacle_names:
        print(f"Exclusion zones: {obstacle_names}")
    else:
        print("No exclusion zones detected.")

    target_geometries = [poly for _, poly in targets]
    obstacle_geometries = [poly for _, poly in obstacles]

    try:
        target_union = unary_union(target_geometries)
    except Exception as e:
        print(f"Error: Failed to union target polygons: {e}")
        sys.exit(1)

    if target_union.is_empty:
        print("Error: Target geometry is empty after union.")
        sys.exit(1)

    if obstacle_geometries:
        try:
            obstacle_union = unary_union(obstacle_geometries)
            if not obstacle_union.is_empty:
                target_union = target_union.difference(obstacle_union)
        except Exception as e:
            print(f"Warning: Could not union obstacle geometries: {e}. Proceeding without obstacle subtraction.")

    if target_union.is_empty:
        print("Error: Coverage area is empty after applying exclusion zones.")
        sys.exit(1)

    base_path_proj, centroid_proj, transformer_to_proj, transformer_to_deg = generate_boustrophedon_path(
        target_union,
        separation_meters=args.sep,
        angle_deg=args.angle,
        safety_distance=args.safe
    )

    if base_path_proj.is_empty or not base_path_proj.is_valid:
        print("Error: Generated base path is empty or invalid.")
        sys.exit(1)

    obstacles_proj = project_polygons(obstacle_geometries, transformer_to_proj)
    if args.safe > 0 and obstacles_proj:
        buffered_obstacles_proj = []
        for obs in obstacles_proj:
            try:
                buffered = obs.buffer(abs(args.safe), join_style=2)
                if buffered.is_valid and not buffered.is_empty:
                    buffered_obstacles_proj.append(buffered)
                else:
                    buffered_obstacles_proj.append(obs)
            except Exception as e:
                print(f"Warning: Could not buffer an exclusion zone: {e}")
                buffered_obstacles_proj.append(obs)
    else:
        buffered_obstacles_proj = obstacles_proj

    split_tracks_proj = split_path_by_obstacles(base_path_proj, buffered_obstacles_proj)
    if not split_tracks_proj:
        print("Warning: No valid path segments remained after obstacle splitting.")

    ordered_tracks_proj = order_tracks_along_path(split_tracks_proj, base_path_proj)
    intersection_points_proj = _find_intersection_points(base_path_proj, buffered_obstacles_proj)
    obstacle_tracks_proj = create_augmented_obstacle_tracks(buffered_obstacles_proj, intersection_points_proj)

    final_path_proj = stitch_path_segments_proj(ordered_tracks_proj, obstacle_tracks_proj)
    if final_path_proj is None or final_path_proj.is_empty:
        print("Warning: Stitching failed or produced no path. Falling back to raw base path.")
        final_path_proj = base_path_proj

    try:
        final_path_deg = transform(transformer_to_deg.transform, final_path_proj)
    except Exception as e:
        print(f"Error: Could not convert final path to geographic coordinates: {e}")
        sys.exit(1)

    output_kml_file, output_plan_file = build_output_paths(args.input_file, args.output_file)
    if output_kml_file:
        save_path_to_kml(final_path_deg, output_kml_file, os.path.splitext(os.path.basename(output_kml_file))[0], args.reverse, targets, obstacles)
    if output_plan_file:
        save_path_to_qgc_plan(final_path_deg, output_plan_file, reverse_path=args.reverse)

    print("\nSqueeze path generation complete.")
    if output_kml_file:
        print(f"  KML output:  {output_kml_file}")
    if output_plan_file:
        print(f"  Plan output: {output_plan_file}")


if __name__ == '__main__':
    main()
