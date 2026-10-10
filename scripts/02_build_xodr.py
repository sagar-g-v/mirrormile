"""OSM -> SUMO net + OpenDRIVE with real elevation (USGS 3DEP DEM) using netconvert.

Options mirror CARLA's OSM2ODR converter (carla.Osm2Odr) and add --heightmap.geotiff.
"""
import json
from collections import defaultdict
import os
import re
import subprocess
import sys
from pathlib import Path

import config as C

SUMO_BIN = Path(sys.executable).parent.parent / "Lib/site-packages/sumo/bin"
NETCONVERT = SUMO_BIN / "netconvert.exe"

DRIVABLE = ["motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link",
            "secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified",
            "residential", "living_street", "service"]


OSM_ELE = C.BUILD / "area_ele.osm"
SIDEWALK_STREETS = {"primary", "secondary", "tertiary", "residential", "unclassified", "living_street"}


def add_elevation():
    """Write an `ele` tag (DEM height, bilinear, median-filtered) on every OSM node.

    netconvert's --heightmap.geotiff crashes on this build, and sampling ourselves lets us
    filter lidar artefacts (basement pits, leftover building edges) before they reach the road.
    """
    import numpy as np
    import rasterio
    from lxml import etree
    from pyproj import Transformer
    from scipy.ndimage import map_coordinates, median_filter

    with rasterio.open(C.DEM_UTM) as src:
        dem = src.read(1).astype(np.float64)
        inv = ~src.transform
    dem[dem < -1000] = np.nan
    dem = np.where(np.isnan(dem), np.nanmedian(dem), dem)
    dem = median_filter(dem, size=5)                      # 5 m median: kills pits/spikes
    tf = Transformer.from_crs(4326, C.UTM_EPSG, always_xy=True)

    tree = etree.parse(str(C.OSM_FILE))
    root = tree.getroot()
    # Drop parking aisles / driveways / private service roads: they explode the junction
    # count and are not part of the drivable public network.
    dropped = 0
    for wy in root.findall("way"):
        tags = {t.get("k"): t.get("v") for t in wy.findall("tag")}
        if tags.get("highway") == "service" and (
                tags.get("service") in ("parking_aisle", "driveway", "drive-through", "emergency_access")
                or tags.get("access") in ("private", "no")):
            # re-tag rather than delete: relations may still reference the way
            wy.find("tag[@k='highway']").set("v", "dropped_service"); dropped += 1
        elif tags.get("highway") in SIDEWALK_STREETS and tags.get("tunnel") != "yes":
            # DTLA maps sidewalks as separate footways (sidewalk=separate), which gives
            # no sidewalk lanes in OpenDRIVE. CARLA needs them as lanes (curbs, walkers).
            for t in wy.findall("tag"):
                if t.get("k").startswith("sidewalk"):
                    wy.remove(t)
            etree.SubElement(wy, "tag", k="sidewalk", v="both")
    print(f"dropped {dropped} service ways")
    import levels
    nodes = root.findall("node")
    ids = [n.get("id") for n in nodes]
    lon = np.array([float(n.get("lon")) for n in nodes]); lat = np.array([float(n.get("lat")) for n in nodes])
    x, y = tf.transform(lon, lat)
    xy_of = dict(zip(ids, zip(np.asarray(x), np.asarray(y))))
    to_ll = Transformer.from_crs(C.UTM_EPSG, 4326, always_xy=True).transform

    def dem_z(px, py):
        col, row = inv * (np.asarray(px), np.asarray(py))
        return map_coordinates(dem, [np.asarray(row) - 0.5, np.asarray(col) - 0.5], order=1, mode="nearest")

    # bridge deck points (lidar class 17) in UTM
    deck_xy, deck_z = np.zeros((0, 2)), np.zeros(0)
    lid = C.DATA / "lidar" / "points.npz"
    if lid.exists():
        p = np.load(lid)
        m = p["cls"] == 17
        dx, dy = Transformer.from_crs(3857, C.UTM_EPSG, always_xy=True).transform(p["x"][m], p["y"][m])
        deck_xy, deck_z = np.column_stack([dx, dy]), p["z"][m].astype(np.float64)
        g = np.flatnonzero(p["cls"] == 2)[::50]                      # vertical datum check vs DEM
        gx, gy = Transformer.from_crs(3857, C.UTM_EPSG, always_xy=True).transform(p["x"][g], p["y"][g])
        off = float(np.median(p["z"][g] - dem_z(gx, gy)))
        deck_z = deck_z - off
        print(f"lidar bridge-deck points: {len(deck_z):,} (datum offset {off:+.2f} m)")
    added = levels.densify(root, xy_of, to_ll, [9_000_000_000_000])
    zmap, stats, _ = levels.assign_levels(root, xy_of, dem_z, deck_xy, deck_z)
    print(f"densified bridge/tunnel ways with {added} nodes; {stats}")
    n_set = 0
    for n in root.findall("node"):
        nid = n.get("id")
        if nid in zmap:
            h = zmap[nid]
        elif nid in xy_of:
            h = float(dem_z(np.array([xy_of[nid][0]]), np.array([xy_of[nid][1]]))[0])
        else:
            continue
        for t in n.findall("tag[@k='ele']"):
            n.remove(t)
        etree.SubElement(n, "tag", k="ele", v=f"{h:.2f}")
        n_set += 1
    tree.write(str(OSM_ELE), xml_declaration=True, encoding="UTF-8")
    zs = np.array(list(zmap.values()))
    print(f"ele tags written to {n_set} nodes, drivable z {zs.min():.1f}..{zs.max():.1f}")


def run():
    s, w, n, e = C.BBOX
    args = [
        str(NETCONVERT),
        "--osm-files", str(OSM_ELE),
        "--proj", C.MAP_PROJ, "--offset.disable-normalization", "true",
        "--osm.elevation", "true",
        "--keep-edges.in-geo-boundary", f"{w},{s},{e},{n}",
        "--keep-edges.by-type", ",".join(f"highway.{t}" for t in DRIVABLE),
        "--remove-edges.by-vclass", "rail_urban,rail,tram,subway",
        # --- CARLA OSM2ODR defaults ---
        "--geometry.remove", "--ramps.guess", "--edges.join", "--junctions.join",
        "--roundabouts.guess", "--tls.group-signals", "true",
        "--sidewalks.guess", "true", "--osm.sidewalks", "true",
        # --osm.crossings crashes netconvert 1.28 on this data; guess crossings instead
        "--crossings.guess", "true",
        "--default.lanewidth", "3.5",           # US urban lanes (~11.5 ft)
        "--default.sidewalk-width", "3.0",      # DTLA sidewalks are wide
        # --- quality ---
        "--junctions.corner-detail", "6",       # rounded curb returns
        "--no-turnarounds", "true",             # no U-turn connectors (CARLA towns have none; they
                                                # produce 180-degree kinks in the lane graph)
        "--junctions.internal-link-detail", "10",   # smoother connecting-lane curves
        "--junctions.join-dist", "12",
        "--geometry.max-grade.fix", "true",
        "--opendrive-output.straight-threshold", "1e-8",
        "--output.original-names", "true",
        "--osm.all-attributes", "true",
        "--output-file", str(C.NET_FILE),
        "--opendrive-output", str(C.XODR_FILE),
        "--no-warnings", "false", "--aggregate-warnings", "5",
        "--junctions.join-output", str(JOIN_AUTO),
    ]
    if JOIN_FILE.exists():
        args += ["--node-files", str(JOIN_FILE)]
    sumo_home = SUMO_BIN.parent
    env = dict(os.environ, SUMO_HOME=str(sumo_home), PROJ_DATA=str(sumo_home / "data/proj"),
               PROJ_LIB=str(sumo_home / "data/proj"))
    print(" ".join(args[:3]), "...")
    r = subprocess.run(args, capture_output=True, text=True, env=env)
    print(r.stdout[-1500:], r.stderr[-1500:])
    if r.returncode:
        sys.exit(f"netconvert failed (code {r.returncode})")


JOIN_AUTO = C.BUILD / "joins_auto.xml"
JOIN_FILE = C.BUILD / "joins.nod.xml"
MIN_EDGE = 2.5   # m: edges shorter than this between two junctions get their junctions merged


def plan_joins():
    """Find micro-edges between junctions (divided-street clusters that netconvert did not join)
    and write a join file merging their end nodes (plus netconvert's own clusters).
    Returns the number of micro-edges found."""
    from lxml import etree
    join_sets = []
    if JOIN_AUTO.exists():
        join_sets = [j.get("nodes").split() for j in etree.parse(str(JOIN_AUTO)).getroot().iter("join")]

    def members(node_id):
        """Expand a netconvert 'cluster_a_b_c_#2more' id to its full original node list."""
        if not node_id.startswith("cluster_"):
            return [node_id]
        parts = {p for p in node_id[len("cluster_"):].split("_") if p and not p.startswith("#")}
        for js in join_sets:
            if parts <= set(js):
                return js
        return sorted(parts)

    class _C(dict):
        def get(self, k, default=None):
            return members(k)
    clusters = _C()
    prev = []
    if JOIN_FILE.exists():
        prev = [j.get("nodes").split() for j in etree.parse(str(JOIN_FILE)).getroot().iter("join")]
    net = etree.parse(str(C.NET_FILE)).getroot()
    jtype = {j.get("id"): j.get("type") for j in net.iter("junction")}
    groups, n_short = [], 0
    for e in net.iter("edge"):
        if e.get("function") in ("internal", "crossing", "walkingarea"):
            continue
        ln = e.find("lane")
        if ln is None or float(ln.get("length")) >= MIN_EDGE:
            continue
        a, b = e.get("from"), e.get("to")
        if "dead_end" in (jtype.get(a), jtype.get(b)):
            continue
        n_short += 1
        groups.append(clusters.get(a, [a]) + clusters.get(b, [b]))
    groups += join_sets + prev
    parent = {}

    def find(x):
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]; x = parent[x]
        return x

    for g in groups:
        for x in g[1:]:
            parent[find(x)] = find(g[0])
    sets = defaultdict(set)
    for x in list(parent):
        sets[find(x)].add(x)
    root = etree.Element("nodes")
    for s in sets.values():
        if len(s) > 1:
            etree.SubElement(root, "join", nodes=" ".join(sorted(s)))
    etree.ElementTree(root).write(str(JOIN_FILE), xml_declaration=True, encoding="UTF-8", pretty_print=True)
    return n_short


def report():
    net = C.NET_FILE.read_text(encoding="utf-8")
    loc = re.search(r'<location netOffset="([^"]+)" convBoundary="([^"]+)" origBoundary="([^"]+)" projParameter="([^"]+)"', net)
    off = [float(v) for v in loc.group(1).split(",")]
    meta = {"map": C.MAP_NAME, "net_offset": off, "conv_boundary": [float(v) for v in loc.group(2).split(",")],
            "utm_epsg": C.UTM_EPSG, "proj": loc.group(4)}
    C.META_FILE.write_text(json.dumps(meta, indent=2))
    x = C.XODR_FILE.read_text(encoding="utf-8")
    roads = len(re.findall(r"<road ", x)); juncs = len(re.findall(r"<junction ", x))
    elev = re.findall(r'<elevation s="[^"]+" a="([^"]+)"', x)
    zs = [float(v) for v in elev]
    print(f"xodr: {roads} roads, {juncs} junctions, {len(zs)} elevation records, "
          f"z {min(zs):.1f}..{max(zs):.1f} m" if zs else "NO ELEVATION in xodr!")
    print("netOffset:", off)


if __name__ == "__main__":
    add_elevation()
    JOIN_FILE.unlink(missing_ok=True)
    for it in range(6):                  # converge: join micro-edges until none remain
        run()
        k = plan_joins()
        print(f"pass {it + 1}: {k} micro-edges (< {MIN_EDGE} m) between junctions")
        if k == 0:
            break
    report()





