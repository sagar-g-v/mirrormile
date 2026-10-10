"""Download the USGS 3DEP lidar point cloud (LA County 2023, public domain) for the map extent.

Minimal Entwine Point Tile (EPT) client: walks the octree hierarchy, downloads only the LAZ
nodes intersecting the area (all depths = full density), keeps points inside the area.
Output: data/lidar/points.npz  (x, y, z in EPSG:3857 metres, classification, return_number)
"""
import io
import json
import math
from concurrent.futures import ThreadPoolExecutor

import laspy
import numpy as np
import requests
from pyproj import Transformer

import config as C

EPT = "https://s3-us-west-2.amazonaws.com/usgs-lidar-public/CA_LosAngeles_1_B23"
OUT = C.DATA / "lidar" / "points.npz"
S = requests.Session()
S.headers["User-Agent"] = "osm2carla/1.0"


CACHE = C.DATA / "lidar" / "nodes"


def get(url, timeout=300, tries=6):
    import time
    for k in range(tries):
        try:
            r = S.get(url, timeout=timeout); r.raise_for_status(); return r
        except (requests.ConnectionError, requests.Timeout, requests.HTTPError) as e:
            if k == tries - 1:
                raise
            time.sleep(2 * (k + 1))


def get_json(path):
    return get(f"{EPT}/{path}", timeout=120).json()


def area_bounds_3857(margin=130.0):
    """Configured area incl. its margin (WGS84 box) as an EPSG:3857 box."""
    s, w, n, e = C.outer_bbox()
    t = Transformer.from_crs(4326, 3857, always_xy=True)
    xs, ys = t.transform([w, e, w, e], [s, s, n, n])
    return min(xs), min(ys), max(xs), max(ys)


def main():
    info = get_json("ept.json")
    b = info["bounds"]                       # cube: xmin ymin zmin xmax ymax zmax
    root = np.array(b[:3]); size0 = b[3] - b[0]
    qx0, qy0, qx1, qy1 = area_bounds_3857()
    print(f"query box (3857): {qx0:.0f},{qy0:.0f} - {qx1:.0f},{qy1:.0f}")

    def node_box(d, x, y):
        s = size0 / 2 ** d
        return root[0] + x * s, root[1] + y * s, root[0] + (x + 1) * s, root[1] + (y + 1) * s

    def hits(d, x, y):
        a0, b0, a1, b1 = node_box(d, x, y)
        return a0 < qx1 and a1 > qx0 and b0 < qy1 and b1 > qy0

    nodes, pages = [], ["0-0-0-0"]
    seen_pages = set()
    while pages:                              # hierarchy pages (count -1 = separate page)
        p = pages.pop()
        if p in seen_pages:
            continue
        seen_pages.add(p)
        h = get_json(f"ept-hierarchy/{p}.json")
        for key, cnt in h.items():
            d, x, y, z = map(int, key.split("-"))
            if not hits(d, x, y):
                continue
            if cnt == -1:
                pages.append(key)
            elif cnt > 0:
                nodes.append(key)
    print(f"{len(nodes)} octree nodes intersect the area (from {len(seen_pages)} hierarchy pages)")

    CACHE.mkdir(parents=True, exist_ok=True)

    def fetch(key):
        f = CACHE / f"{key}.laz"                      # resumable: tiles are cached on disk
        if f.exists() and f.stat().st_size > 0:
            content = f.read_bytes()
        else:
            content = get(f"{EPT}/ept-data/{key}.laz").content
            f.write_bytes(content)
        las = laspy.read(io.BytesIO(content))
        x, y, z = np.asarray(las.x), np.asarray(las.y), np.asarray(las.z)
        m = (x >= qx0) & (x <= qx1) & (y >= qy0) & (y <= qy1)
        return (x[m], y[m], z[m], np.asarray(las.classification)[m].astype(np.uint8),
                np.asarray(las.return_number)[m].astype(np.uint8), len(content))

    parts, mb = [], 0.0
    with ThreadPoolExecutor(10) as ex:
        for i, res in enumerate(ex.map(fetch, nodes)):
            parts.append(res[:5]); mb += res[5] / 1e6
            if (i + 1) % 50 == 0:
                print(f"  {i + 1}/{len(nodes)} nodes, {mb:.0f} MB")
    x, y, z, cls, ret = (np.concatenate([p[k] for p in parts]) for k in range(5))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(OUT, x=x, y=y, z=z.astype(np.float32), cls=cls, ret=ret)
    u, c = np.unique(cls, return_counts=True)
    area = (qx1 - qx0) * (qy1 - qy0) * math.cos(math.radians(C.LAT0)) ** 2
    print(f"points: {len(x):,} ({len(x) / area:.1f} pts/m2), downloaded {mb:.0f} MB")
    print("classes:", dict(zip(u.tolist(), c.tolist())))


if __name__ == "__main__":
    main()

