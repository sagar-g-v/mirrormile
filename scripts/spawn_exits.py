"""Which spawn candidates have a lane that runs off the map within reach? (CARLA client venv: venv38)

CARLA's Traffic Manager removes a vehicle whose path has no next waypoint. A lane cut by the area's clip box ends at
the map edge, so a car spawned shortly before such an end is removed within seconds. For every candidate this follows
the lane graph (every branch, 2 m steps) up to REACH m and reports whether any branch ends within EDGE m of the map's
extent (a lane end inside the map - a cul-de-sac - is real and kept).

    venv38/Scripts/python.exe spawn_exits.py <map.xodr> <candidates.json: [[x, y(UE), z], ...]> [REACH] [EDGE]
prints a JSON list of booleans (True: runs off the map within REACH)
"""
import json
import sys

import carla

REACH = float(sys.argv[3]) if len(sys.argv) > 3 else 150.0
EDGE = float(sys.argv[4]) if len(sys.argv) > 4 else 40.0
STEP = 2.0

m = carla.Map("x", open(sys.argv[1], encoding="utf-8").read())
pts = json.load(open(sys.argv[2]))
grid = m.generate_waypoints(10.0)
x0 = min(w.transform.location.x for w in grid); x1 = max(w.transform.location.x for w in grid)
y0 = min(w.transform.location.y for w in grid); y1 = max(w.transform.location.y for w in grid)


def at_edge(loc):
    return min(loc.x - x0, x1 - loc.x, loc.y - y0, y1 - loc.y) < EDGE


def runs_off(w):
    todo, seen = [(w, 0.0)], set()
    while todo:
        cur, d = todo.pop()
        key = (cur.road_id, cur.section_id, cur.lane_id, round(cur.s))
        if key in seen or len(seen) > 2000:
            continue
        seen.add(key)
        nxt = cur.next(STEP)
        if not nxt:
            if at_edge(cur.transform.location):
                return True
            continue
        if d + STEP < REACH:
            todo += [(n, d + STEP) for n in nxt]
    return False


out = []
for x, y, z in pts:
    w = m.get_waypoint(carla.Location(x, y, z), project_to_road=True, lane_type=carla.LaneType.Driving)
    out.append(bool(w is not None and runs_off(w)))
print(json.dumps(out))
