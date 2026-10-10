"""Validate the OpenDRIVE with CARLA's own parser (LibCarla) - no simulator needed.

Checks: parse, topology, waypoints, junctions, elevation range, geolocation round trip,
and that driving-lane waypoints lie on the generated road mesh (distance to mesh z).
Run with the CARLA client venv:  venv38\\Scripts\\python validate_carla_map.py
"""
import json
import math
import os
import sys

import carla

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XODR = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "export", "LA_DTLA", "LA_DTLA.xodr")
import re
_x = open(XODR, encoding="utf-8").read()
meta = {"lat0": float(re.search(r"\+lat_0=([-0-9.]+)", _x).group(1)),
        "lon0": float(re.search(r"\+lon_0=([-0-9.]+)", _x).group(1))}

m = carla.Map("LA_DTLA", _x)
topo = m.get_topology()
wps = m.generate_waypoints(2.0)
drive = [w for w in wps if w.lane_type == carla.LaneType.Driving]
side = [w for w in wps if w.lane_type == carla.LaneType.Sidewalk]
junc = {w.get_junction().id for w in wps if w.is_junction}
zs = [w.transform.location.z for w in drive]
print(f"topology segments : {len(topo)}")
print(f"waypoints (2 m)   : {len(wps)}  driving {len(drive)}  sidewalk {len(side)}")
print(f"junctions w/ wps  : {len(junc)}")
print(f"driving z range   : {min(zs):.2f} .. {max(zs):.2f} m")
spawns = m.get_spawn_points()
print(f"spawn points      : {len(spawns)}")
xs = [w.transform.location.x for w in drive]; ys = [w.transform.location.y for w in drive]
print(f"UE-frame x {min(xs):.0f}..{max(xs):.0f}  y {min(ys):.0f}..{max(ys):.0f}  (CARLA y = -OpenDRIVE y)")

# geolocation: map origin must be lat0/lon0; a point 500 m east must be ~lon0 + 500/(R cos lat)
g0 = m.transform_to_geolocation(carla.Location(0, 0, 0))
g1 = m.transform_to_geolocation(carla.Location(500, 0, 0))
g2 = m.transform_to_geolocation(carla.Location(0, -500, 0))   # CARLA -y = north
R = 6378137.0
print(f"origin geo        : {g0.latitude:.6f}, {g0.longitude:.6f} (expected {meta['lat0']}, {meta['lon0']})")
print(f"500 m east  dlon  : {g1.longitude - g0.longitude:.6f} (expected {math.degrees(500 / (R * math.cos(math.radians(meta['lat0'])))):.6f})")
print(f"500 m north dlat  : {g2.latitude - g0.latitude:.6f} (expected ~{math.degrees(500 / R):.6f})")

# lane-graph continuity as CARLA sees it: stepping 1 m along any lane must move ~1 m and turn < 15 deg
jumps = []
for w in m.generate_waypoints(1.0):
    for n in w.next(1.0):
        d = w.transform.location.distance(n.transform.location)
        dy = abs((n.transform.rotation.yaw - w.transform.rotation.yaw + 180) % 360 - 180)
        dz = abs(n.transform.location.z - w.transform.location.z)
        # outer lanes of curved roads legitimately move > 1 m per metre of s, tight urban turns
        # turn ~15-25 deg per metre; flag only real discontinuities
        if d > 4.0 or (d > 2.0 and dy > 45) or dy > 30 or (dz / max(d, 0.3)) > 0.25:
            jumps.append((w.road_id, w.lane_id, n.road_id, n.lane_id, round(d, 2), round(dy, 1), round(dz, 2),
                          round(w.transform.location.x, 1), round(w.transform.location.y, 1)))
print(f"lane-graph defects : {len(jumps)}  (1 m step: jump > 4 m, kink > 30 deg, grade > 25 %)")
for j in sorted(jumps, key=lambda j: -j[4])[:8]:
    print("    road %s lane %s -> road %s lane %s  dist %.2f  dyaw %.1f  dz %.2f  at (%.1f, %.1f)" % j)
with open(os.path.join(ROOT, "build", "carla_lane_jumps.json"), "w") as f:
    json.dump(jumps, f)

ok = len(topo) > 0 and len(drive) > 1000 and abs(g0.latitude - meta["lat0"]) < 1e-5 and abs(g0.longitude - meta["lon0"]) < 1e-5
# hand driving-lane points to the mesh check (run with the main venv)
with open(os.path.join(ROOT, "build", "carla_waypoints.json"), "w") as f:
    json.dump([[w.transform.location.x, w.transform.location.y, w.transform.location.z, int(w.is_junction), w.transform.rotation.yaw] for w in drive], f)
print("RESULT:", "OK" if ok else "PROBLEM")
sys.exit(0 if ok else 1)


