"""Read supplied parking polygons by ID without inferring their map frame or entry direction."""
import json
import math
from pathlib import Path


def load_spaces(path):
    """Return idx -> original record; never substitute default width/length/angle.

    Coordinates remain in the supplied file's frame. A caller must establish
    frame alignment before using these points alongside a vehicle pose.
    """
    records = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    if not isinstance(records, list):
        raise ValueError('parking GT must be a list of records')
    spaces = {}
    for record in records:
        if not isinstance(record, dict):
            raise ValueError('parking GT records must be objects')
        idx = record.get('idx')
        if not isinstance(idx, str) or not idx.strip() or idx in spaces:
            raise ValueError('parking GT requires unique non-empty string IDs')
        points = record.get('points')
        try:
            valid = (isinstance(points, list) and len(points) >= 3
                     and all(isinstance(p, list) and len(p) == 3
                             and all(not isinstance(v, bool) and isinstance(v, (int, float))
                                     and math.isfinite(v) for v in p) for p in points))
            distinct = len({(p[0], p[1]) for p in points}) if valid else 0
        except (TypeError, ValueError, OverflowError):
            valid, distinct = False, 0
        if not valid or distinct < 3:
            raise ValueError('{} requires at least three distinct finite XYZ points'.format(idx))
        spaces[idx] = record
    return spaces
