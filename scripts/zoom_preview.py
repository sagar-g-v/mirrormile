"""Render a top-down close-up of build/geometry.npz around (x, y): python zoom_preview.py x y [radius]"""
import sys
from collections import defaultdict

import numpy as np
from PIL import Image, ImageDraw

import config as C

cx, cy = float(sys.argv[1]), float(sys.argv[2])
R = float(sys.argv[3]) if len(sys.argv) > 3 else 40
S = 12  # px per m
W = int(2 * R * S)
img = Image.new("RGB", (W, W), (30, 60, 30))
dr = ImageDraw.Draw(img)
g = np.load(C.GEOM_FILE)
col = {"Terrain": (60, 90, 60), "Road": (70, 70, 78), "SideWalk__Concrete": (180, 175, 165),
       "SideWalk__Curb": (120, 120, 120), "RoadLine__White": (255, 255, 255), "RoadLine__Yellow": (255, 200, 0),
       "Building": (120, 70, 160)}
order = ["Terrain", "Building", "Road", "SideWalk__Concrete", "RoadLine__White", "RoadLine__Yellow"]
groups = defaultdict(list)
for k in g.files:
    if k.endswith("|V"):
        groups[k.split("|")[0]].append(k[:-2])
for pref in order:
    for name, keys in groups.items():
        if not name.startswith(pref):
            continue
        for key in keys:
            V, F = g[key + "|V"], g[key + "|F"]
            m = (np.abs(V[:, 0] - cx) < R + 20) & (np.abs(V[:, 1] - cy) < R + 20)
            sel = m[F].any(1)
            for f in F[sel]:
                p = [((V[i, 0] - cx + R) * S, (cy + R - V[i, 1]) * S) for i in f]
                if pref == "Terrain":
                    z = V[f, 2].mean()
                    dr.polygon(p, fill=(40, int(min(255, max(0, (z - 70) * 6))), 40))
                else:
                    dr.polygon(p, fill=col.get(name, col.get(pref)))
img.save(C.BUILD / "zoom.png")
print("saved", C.BUILD / "zoom.png")
