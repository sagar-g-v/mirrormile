"""Street props at their OSM positions: benches, litter bins, fire hydrants, bus stops, bollards, parking meters, bike
racks, street cabinets, speed bumps ... (OSM point features; the area's OSM extract holds ways only, so these come from
one Overpass query, cached in data/street_furniture.json).

Each point gets a CARLA prop (06 spawns the static meshes of /Game/Carla/Static/Static), a position on the surface it
stands on (sidewalk / terrain / road for speed bumps) and a heading:
  * along-road props (bench, bus stop, bike rack, cabinet, meter): parallel to the nearest road, facing it
  * speed bumps: across the road they lie on
Points on a driving lane are dropped (except speed bumps): a bench on the carriageway is an OSM position error.

Writes build/street_props.json: {"props": [{kind, x, y, z, heading, osm_id}], "stats": {...}}
"""
import json
import math
import sys
from collections import Counter

import numpy as np
import requests
from pyproj import Transformer

import config as C
import importlib
GB = importlib.import_module("03_build_geometry")

# OSM tags -> prop kind (06 maps a kind to CARLA meshes)
KINDS = [
    (("amenity", "bench"), "bench"),
    (("leisure", "picnic_table"), "bench"),
    (("amenity", "waste_basket"), "bin"),
    (("amenity", "recycling"), "bin"),
    (("emergency", "fire_hydrant"), "hydrant"),
    (("highway", "bus_stop"), "bus_stop"),
    (("amenity", "shelter"), "bus_stop"),
    (("barrier", "bollard"), "bollard"),
    (("vending", "parking_tickets"), "parking_meter"),
    (("amenity", "parking_meter"), "parking_meter"),
    (("amenity", "bicycle_parking"), "bike_rack"),
    (("man_made", "street_cabinet"), "cabinet"),
    (("power", "substation"), "cabinet"),
    (("amenity", "atm"), "atm"),
    (("man_made", "surveillance"), "camera"),
    (("advertising", "column"), "advertising"),
    (("advertising", "board"), "advertising"),
    (("traffic_calming", "bump"), "speed_bump"),
    (("traffic_calming", "hump"), "speed_bump"),
    (("traffic_calming", "table"), "speed_bump"),
    (("amenity", "planter"), "planter"),
]
ALONG_ROAD = {"bench", "bus_stop", "bike_rack", "cabinet", "parking_meter", "atm", "advertising", "planter", "bin"}


def fetch_points():
    f = C.DATA / "street_furniture.json"
    if f.exists():
        return json.loads(f.read_text())
    s, w, n, e = C.outer_bbox()
    bb = f"{s},{w},{n},{e}"
    sel = "\n".join(f'  node["{k}"="{v}"]({bb});' for (k, v), _ in KINDS)
    q = f"[out:json][timeout:120];\n(\n{sel}\n);\nout;"
    for url in ("https://overpass-api.de/api/interpreter", "https://overpass.kumi.systems/api/interpreter"):
        try:
            r = requests.post(url, data={"data": q}, timeout=180, headers={"User-Agent": "mirrormile/1.0"})
            r.raise_for_status()
            els = r.json()["elements"]
            f.write_text(json.dumps(els))
            print(f"street furniture: {len(els)} points, {len(r.content) / 1e3:.0f} kB from {url}")
            return els
        except Exception as ex:  # noqa: BLE001
            print("overpass failed:", url, ex)
    return []


def kind_of(tags):
    for (k, v), kind in KINDS:
        if tags.get(k) == v:
            return kind
    return None


def main():
    pts = fetch_points()
    if "--fetch-only" in sys.argv:
        print(Counter(kind_of(p.get("tags", {})) for p in pts))
        return
    meta = json.loads(C.META_FILE.read_text())
    x0, y0, x1, y1 = meta["extent"]
    tf = Transformer.from_crs(4326, C.MAP_PROJ, always_xy=True)
    g = np.load(C.GEOM_FILE)

    def mesh_of(prefixes):
        Vs, Fs, n = [], [], 0
        for k in g.files:
            if k.startswith(prefixes) and k.endswith("|V"):
                Vs.append(g[k]); Fs.append(g[k[:-2] + "|F"] + n); n += len(g[k])
        return GB.MeshZ(np.concatenate(Vs), np.concatenate(Fs))
    road_z = mesh_of(("Road__Asphalt",))
    ground_z = mesh_of(("SideWalk__", "Terrain__Ortho"))
    dem = GB.Dem(C.DEM_UTM, filt=5)
    # road direction for headings: nearest driving-lane sample of the OpenDRIVE network (02)
    import opendrive
    od = opendrive.OpenDrive(C.XODR_FILE)
    P, H = [], []
    for r in od.roads:
        s = r.sample_s(0, r.length, max_step=4.0)
        if len(s) < 2:
            continue
        x, y, h = r.ref(s)
        P.append(np.column_stack([x, y])); H.append(h)
    P, H = np.concatenate(P), np.concatenate(H)
    from scipy.spatial import cKDTree
    tree = cKDTree(P)
    out, stats = [], Counter()
    for p in pts:
        kind = kind_of(p.get("tags", {}))
        if kind is None:
            continue
        x, y = tf.transform(p["lon"], p["lat"])
        if not (x0 <= x <= x1 and y0 <= y <= y1):
            stats["outside"] += 1; continue
        zr = road_z(np.array([[x, y]]), how="max")[0]
        d, i = tree.query([x, y])
        h = float(H[i])
        if kind != "speed_bump" and not np.isnan(zr):
            # mapped on the road (bus stops often are): moved sideways off the asphalt, away from the road's
            # reference line (right of travel when on it), at most 12 m; else dropped
            away = np.array([x, y]) - P[i]
            if np.hypot(*away) < 0.3:
                away = np.array([math.sin(h), -math.cos(h)])
            away /= np.hypot(*away)
            for step in np.arange(0.5, 12.01, 0.5):
                q = np.array([x, y]) + away * step
                if np.isnan(road_z(q[None], how="max")[0]):
                    x, y = float(q[0] + away[0] * 0.6), float(q[1] + away[1] * 0.6)   # 0.6 m past the edge
                    stats[f"{kind}_moved_off_road"] += 1
                    break
            else:
                stats[f"{kind}_on_road_dropped"] += 1; continue
            d, i = tree.query([x, y])
        if kind == "speed_bump" and np.isnan(zr):
            stats["speed_bump_off_road_dropped"] += 1; continue
        z = zr if kind == "speed_bump" else ground_z(np.array([[x, y]]), how="max")[0]
        if np.isnan(z):
            z = float(dem(np.array([x]), np.array([y]))[0])
        h = float(H[i])
        if kind == "speed_bump":
            heading = math.degrees(h)                    # mesh long axis across the lane (06 adds the mesh's own axis)
        else:                                            # parallel to the road; the front faces it
            rx, ry = P[i] - np.array([x, y])
            side = math.cos(h) * ry - math.sin(h) * rx  # >0: road centre to the left of the road direction
            heading = math.degrees(h + (math.pi / 2 if side > 0 else -math.pi / 2))
        rec = dict(x=round(float(x), 3), y=round(float(y), 3), z=round(float(z), 3), heading=round(heading, 1),
                   road_dist=round(float(d), 1), osm_id=p["id"])
        if kind == "bus_stop":          # what the stop has (OSM shelter / bench / bin); a bare stop gets no prop
            t = p.get("tags", {})
            items = ["bus_shelter"] if t.get("shelter") == "yes" else (["bench"] if t.get("bench") == "yes" else [])
            if t.get("bin") == "yes":
                items.append("bin")
            if not items:
                stats["bus_stop_bare"] += 1
            for k, item in enumerate(items):  # side by side along the road, 2.5 m apart
                along = 2.5 * k
                out.append(dict(rec, kind=item, x=round(float(x + math.cos(h) * along), 3),
                                y=round(float(y + math.sin(h) * along), 3)))
                stats[f"bus_stop_{item}"] += 1
            continue
        out.append(dict(rec, kind=kind))
        stats[kind] += 1
    (C.BUILD / "street_props.json").write_text(json.dumps({"props": out, "stats": dict(stats)}))
    print("street props:", len(out), dict(stats))


if __name__ == "__main__":
    main()
