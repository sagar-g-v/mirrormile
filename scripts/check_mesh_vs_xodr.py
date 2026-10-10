"""Accuracy checks between CARLA's view of the OpenDRIVE and the generated meshes.

1. road mesh height vs CARLA driving-waypoint height (should agree to a few cm)
2. CARLA geolocation of real waypoints vs pyproj (true lat/lon of the OpenDRIVE point)
3. writes build/spawn_points.json (UE frame, metres) for the Unreal import script
"""
import json
import math
import subprocess
import sys

import numpy as np
import shapely
from pyproj import Transformer

import config as C

wps = np.array(json.load(open(C.BUILD / "carla_waypoints.json")))   # x, y(UE), z, is_junction, yaw
g = np.load(C.GEOM_FILE)
import importlib
GB = importlib.import_module("03_build_geometry")
Vs, Fs, n_ = [], [], 0
for k in g.files:
    if k.startswith("Road__") and k.endswith("|V"):
        Vs.append(g[k]); Fs.append(g[k[:-2] + "|F"] + n_); n_ += len(g[k])
mz = GB.MeshZ(np.concatenate(Vs), np.concatenate(Fs))
P = np.column_stack([wps[:, 0], -wps[:, 1]])              # UE y -> OpenDRIVE y
# the road surface at the waypoint's own level (stacked roads: never the deck above / road below)
zm = mz(P, how="near", zref=wps[:, 2], tol=6.0)
hit = ~np.isnan(zm)
miss = int((~hit).sum())
where = np.flatnonzero(hit)
dz = np.abs(zm[hit] - wps[hit, 2])
jn = wps[hit, 3].astype(bool)
print(f"waypoints on road mesh : {len(dz)}/{len(P)} (missing {miss})")
print(f"|mesh z - xodr z|      : median {np.median(dz)*100:.1f} cm, p95 {np.percentile(dz,95)*100:.1f} cm, max {dz.max()*100:.1f} cm")
print(f"   on road segments    : p95 {np.percentile(dz[~jn],95)*100:.1f} cm   in junctions: p95 {np.percentile(dz[jn],95)*100:.1f} cm")
print(f"   share > 10 cm       : {np.mean(dz > 0.10)*100:.2f} %   > 25 cm: {np.mean(dz > 0.25)*100:.2f} %")
for k in np.argsort(dz)[-5:][::-1]:
    i = where[k]
    print(f"   worst: {dz[k]*100:5.1f} cm at OpenDRIVE ({P[i,0]:.1f}, {P[i,1]:.1f}) junction={bool(wps[i,3])}")

# geolocation: compare CARLA's GNSS for 3 waypoints with the true position
to_ll = Transformer.from_crs(C.MAP_PROJ, 4326, always_xy=True)
probe = wps[[0, len(wps) // 2, -1]]
code = ("import carla,json,sys;m=carla.Map('x',open(sys.argv[1],encoding='utf-8').read());"
        "print(json.dumps([[g.latitude,g.longitude] for g in [m.transform_to_geolocation(carla.Location(*p)) for p in json.loads(sys.argv[2])]]))")
out = subprocess.run([str(C.ROOT / "venv38/Scripts/python.exe"), "-c", code, str(C.XODR_FILE),
                      json.dumps(probe[:, :3].tolist())], capture_output=True, text=True)
carla_ll = json.loads(out.stdout.strip().splitlines()[-1])
for (x, y, z, _, _), (clat, clon) in zip(probe, carla_ll):
    lon, lat = to_ll.transform(x, -y)
    dn = (clat - lat) * 111000; de = (clon - lon) * 111000 * math.cos(math.radians(lat))
    print(f"wp UE({x:7.1f},{y:7.1f}) true {lat:.6f},{lon:.6f}  carla {clat:.6f},{clon:.6f}  err N {dn:6.1f} m  E {de:6.1f} m")

# spawn points: driving lanes outside junctions, >= 30 m apart, heading from neighbouring waypoints
cand = wps[wps[:, 3] == 0]
sel = []
for p in cand:
    if all((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 > 30 ** 2 for q in sel):
        sel.append(p)
print("spawn points:", len(sel))
json.dump({"note": "CARLA/UE frame, metres and degrees (x, y, z, yaw)",
           "points": [[float(p[0]), float(p[1]), float(p[2]), float(p[4])] for p in sel]},
          open(C.BUILD / "spawn_points.json", "w"))

