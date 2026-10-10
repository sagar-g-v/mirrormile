"""Summarise build/carla_lane_jumps.json (written by validate_carla_map.py) by cause."""
import collections
import json
import sys

import config as C
from opendrive import OpenDrive

od = OpenDrive(sys.argv[1] if len(sys.argv) > 1 else C.XODR_FILE)
J = json.load(open(C.BUILD / "carla_lane_jumps.json"))
cat = collections.Counter()
ex = {}
for r1, l1, r2, l2, d, dy, dz, x, y in J:
    a, b = od.by_id[str(r1)], od.by_id[str(r2)]
    kind = ("road" if a.junction == "-1" else "conn") + "->" + ("road" if b.junction == "-1" else "conn")
    if d > 3:
        c = "teleport (>3 m)"
    elif dy > 60:
        c = "reversal (>60 deg)"
    elif dy > 15:
        c = "kink 15-60 deg"
    elif dz > 0.2:
        c = "height step"
    else:
        c = "spacing"
    key = (c, kind, r1 == r2)
    cat[key] += 1
    ex.setdefault(key, (r1, l1, r2, l2, d, dy, dz, x, y))
for k, v in cat.most_common():
    print(f"{v:5d}  {k[0]:20s} {k[1]:11s} same_road={k[2]}   e.g. {ex[k]}")
