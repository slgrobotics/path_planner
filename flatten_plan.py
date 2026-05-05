#!/usr/bin/env python3

"""
flatten_plan.py

Flatten TransectStyleComplexItem (survey) entries in QGroundControl .plan files.

Usage examples:
  python3 flatten_plan.py input.plan -o output.plan
  python3 flatten_plan.py input.plan --inplace --backup
  python3 flatten_plan.py input.plan --dry-run

  python3 flatten_plan.py front-east.plan --mode clean -o front-east.flattened.plan

The script will replace any item that contains a `TransectStyleComplexItem` (or
ComplexItem of type 'survey') with that complex item's inner `Items` list,
preserving order. By default it will renumber `doJumpId` sequentially.
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
from typing import Any, Dict, List, Tuple


def is_survey_complex_item(item: Dict[str, Any]) -> bool:
    # Detect common shapes: either the item has a TransectStyleComplexItem key
    # or it's a ComplexItem whose complexItemType is 'survey'.
    if 'TransectStyleComplexItem' in item:
        return True
    if item.get('type') == 'ComplexItem' and item.get('complexItemType') == 'survey':
        return True
    return False


def extract_inner_items(item: Dict[str, Any]) -> List[Dict[str, Any]]:
    # Return the list of inner items for a survey complex item.
    if 'TransectStyleComplexItem' in item:
        inner = item['TransectStyleComplexItem'].get('Items', [])
        return [copy.deepcopy(i) for i in inner]
    # Some variants may embed the TransectStyleComplexItem fields at top-level of the object
    t = item.get('TransectStyleComplexItem') or {}
    inner = t.get('Items', [])
    return [copy.deepcopy(i) for i in inner]


def flatten_plan(data: Dict[str, Any], renumber: bool = True, drop_command_206: bool = False) -> Tuple[Dict[str, Any], Dict[str, int]]:
    """Return a copy of `data` with survey complex items flattened.

    Also returns a small stats dict with counts of removed/added items.
    """
    out = copy.deepcopy(data)
    mission = out.get('mission')
    if not mission or 'items' not in mission:
        raise ValueError('input JSON does not contain mission.items')

    items: List[Dict[str, Any]] = mission['items']
    new_items: List[Dict[str, Any]] = []
    removed = 0
    removed_command_206 = 0
    added = 0

    for item in items:
        if is_survey_complex_item(item):
            inner = extract_inner_items(item)
            removed += 1
            # filter out items with command == 206 if requested
            filtered = []
            for it in inner:
                if drop_command_206 and it.get('command') == 206:
                    removed_command_206 += 1
                    continue
                filtered.append(it)
            added += len(filtered)
            new_items.extend(filtered)
        else:
            # optionally drop any top-level items with command == 206
            if drop_command_206 and item.get('command') == 206:
                removed_command_206 += 1
                continue
            new_items.append(item)

    # Optionally renumber doJumpId sequentially (1-based). This avoids id collisions
    # and produces a simple, sequential mission numbering which QGC generally expects.
    if renumber:
        next_id = 1
        for it in new_items:
            if 'doJumpId' in it:
                try:
                    it['doJumpId'] = int(next_id)
                except Exception:
                    it['doJumpId'] = next_id
                next_id += 1

    out['mission']['items'] = new_items
    stats = {
        'removed_complex_items': removed,
        'removed_command_206': removed_command_206,
        'added_items': added,
        'final_item_count': len(new_items),
    }
    return out, stats


def load_json(path: str) -> Dict[str, Any]:
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)


def write_json(path: str, data: Dict[str, Any]) -> None:
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=4, sort_keys=False)


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Flatten Survey (TransectStyleComplexItem) in QGroundControl .plan files')
    parser.add_argument('input', help='Path to input .plan JSON file')
    parser.add_argument('-o', '--output', help='Path to write output file. If omitted and --inplace not given, writes to stdout')
    parser.add_argument('--drop-206', action='store_true', help='Remove mission items where command == 206')
    parser.add_argument('--mode', choices=['default', 'clean'], default='default', help="Shorthand modes; 'clean' is equivalent to --drop-206")
    parser.add_argument('--inplace', action='store_true', help='Modify the input file in place')
    parser.add_argument('--backup', action='store_true', help='When used with --inplace, create a .bak backup of the original')
    parser.add_argument('--no-renumber', dest='renumber', action='store_false', help='Do not renumber doJumpId values')
    parser.add_argument('--dry-run', action='store_true', help='Do not write files; just print what would change')

    args = parser.parse_args(argv)

    try:
        data = load_json(args.input)
    except Exception as e:
        print(f'Error reading {args.input}: {e}', file=sys.stderr)
        return 2

    # mode 'clean' is shorthand for drop-206
    drop_206 = args.drop_206 or (args.mode == 'clean')

    try:
        out, stats = flatten_plan(data, renumber=args.renumber, drop_command_206=drop_206)
    except Exception as e:
        print(f'Error processing plan: {e}', file=sys.stderr)
        return 3

    print('Flatten results:')
    for k, v in stats.items():
        print(f'  {k}: {v}')

    if args.dry_run:
        print('Dry-run: no files written')
        return 0

    if args.inplace:
        if args.backup:
            bak = args.input + '.bak'
            write_json(bak, data)
            print(f'Backup written to {bak}')
        write_json(args.input, out)
        print(f'Input file {args.input} updated in-place')
        return 0

    if args.output:
        write_json(args.output, out)
        print(f'Output written to {args.output}')
        return 0

    # Default: print to stdout
    json.dump(out, sys.stdout, indent=4)
    print()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
