#!/usr/bin/env python3
"""Inspect supplied parking IDs and polygon dimensions without ROS or a road map."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/vip3_parking/src'))
from vip3_parking.gt import load_spaces
from vip3_parking.evaluation import slot_from_points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gt', type=Path, default=ROOT/'src/vip3_parking/data/katri/parking_space_set.json')
    parser.add_argument('--id', action='append', dest='ids', help='exact idx; repeat to select multiple slots')
    args = parser.parse_args()
    try:
        spaces = load_spaces(args.gt)
        selected = list(spaces) if args.ids is None else args.ids
        missing = [idx for idx in selected if idx not in spaces]
        if missing:
            parser.error('unknown parking ID: '+', '.join(missing))
        summary = [dict(idx=idx, **slot_from_points(spaces[idx]['points'])) for idx in selected]
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    print(json.dumps(dict(source=str(args.gt), sha256=hashlib.sha256(args.gt.read_bytes()).hexdigest(),
                          record_count=len(spaces), coordinates='unchanged from supplied GT; alignment unverified',
                          angle='polygon long axis in degrees modulo 180; entry direction unspecified',
                          slots=summary), indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
