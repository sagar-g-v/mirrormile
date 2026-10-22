"""Vegetation from real data: 2023 LA County lidar (height) + NAIP near-infrared (NDVI) + OSM.

Outputs
  build/vegetation.json      tree / shrub instances (map frame, metres): x, y, z, height, crown radius,
                             species (palm | broadleaf_s/m/l | shrub), yaw, variant
  build/vegetation_geom.npz  grass patch meshes (layer "Terrain__Grass", same format as geometry.npz)
  build/vegetation_debug.png top-down check: canopy, detected trees, grass
"""
import json
import math
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import rasterio
import shapely
from PIL import Image, ImageDraw
from pyproj import Transformer
from rasterio.features import rasterize, shapes
from rasterio.transform import from_origin
from rasterio.warp import Resampling, reproject
from scipy.ndimage import gaussian_filter, map_coordinates, maximum_filter, binary_opening
from scipy.spatial import cKDTree

import config as C
import importlib
GB = importlib.import_module("03_build_geometry")
import geom_par as GP

RES = 0.5                       # canopy model resolution (m)
NDVI_VEG = 0.12                 # vegetation threshold (low: urban trees are often in shadow)
TREE_MIN_H, SHRUB_MIN_H = 2.5, 0.4
rng = np.random.default_rng(42)


def load_meta():
    return json.loads(C.META_FILE.read_text())


def grid(extent):
    x0, y0, x1, y1 = extent
    W, H = int(round((x1 - x0) / RES)), int(round((y1 - y0) / RES))
    return from_origin(x0, y1, RES, RES), W, H


def dem_sampler():
    with rasterio.open(C.DEM_UTM) as src:
        d = src.read(1).astype(np.float64); inv = ~src.transform
    d[d < -1000] = np.nanmedian(np.where(d < -1000, np.nan, d))
    tf = Transformer.from_crs(C.MAP_PROJ, C.UTM_EPSG, always_xy=True)

    def f(x, y):
        ux, uy = tf.transform(np.asarray(x, float), np.asarray(y, float))
        c, r = inv * (np.asarray(ux), np.asarray(uy))
        return map_coordinates(d, [np.asarray(r) - 0.5, np.asarray(c) - 0.5], order=1, mode="nearest")
    return f


LANE_CLEAR = 0.5        # m kept free between a trunk and the edge of any vehicle lane
LOW_CROWN = 4.5         # m: a crown starting below this hangs into traffic -> keep the whole crown off the lanes
LEVEL_DZ = 4.0          # m: a lane this far above / below the tree base is another level (bridge, tunnel)


def lane_areas():
    """Vehicle lanes of the OpenDRIVE (03's lane_pieces: STRtree, mean z) for the same-level test.
    The road mesh alone is not enough: a trunk within 0.5 m of its edge still stands in the outer lane."""
    from opendrive import OpenDrive
    tree, _, zs = GB.lane_pieces(OpenDrive(C.XODR_FILE))
    return tree, zs


def on_lane(tree, zs, xy, zb, clear):
    """True where the point lies within clear (per point) of a vehicle lane at its own level."""
    pts = shapely.points(xy)
    hit = np.zeros(len(xy), bool)
    for i, (p, c) in enumerate(zip(pts, clear)):
        for k in tree.query(p, predicate="dwithin", distance=c):
            if abs(zs[k] - zb[i]) < LEVEL_DZ:
                hit[i] = True
                break
    return hit


def canopy_height_model(extent, dem):
    p = np.load(C.DATA / "lidar" / "points.npz")
    tf = Transformer.from_crs(3857, C.MAP_PROJ, always_xy=True)
    x, y = tf.transform(p["x"], p["y"])
    z, cls = p["z"].astype(np.float64), p["cls"]
    keep = (cls != 7) & (cls != 18)                       # noise
    x, y, z, cls = x[keep], y[keep], z[keep], cls[keep]
    ground = cls == 2
    gi = rng.choice(np.flatnonzero(ground), size=min(200000, ground.sum()), replace=False)
    offset = float(np.median(z[gi] - dem(x[gi], y[gi])))   # lidar vertical datum vs DEM (NAVD88)
    hag = z - offset - dem(x, y)
    print(f"lidar: {len(x):,} pts, vertical offset to DEM {offset:+.2f} m, "
          f"ground residual p95 {np.percentile(np.abs(hag[gi]), 95):.2f} m")
    tr, W, H = grid(extent)
    col = ((x - extent[0]) / RES).astype(int); row = ((extent[3] - y) / RES).astype(int)
    ok = (~ground) & (col >= 0) & (col < W) & (row >= 0) & (row < H) & (hag > 0.15) & (hag < 120)
    chm = np.zeros(H * W, np.float32)
    np.maximum.at(chm, row[ok] * W + col[ok], hag[ok].astype(np.float32))
    above = ok & (hag > 1.0)
    pts = (x[above], y[above], hag[above].astype(np.float32))     # for crown-shape analysis
    return chm.reshape(H, W), tr, W, H, pts


def ndvi_grid(tr, W, H):
    with rasterio.open(C.DATA / "rednir_utm.tif") as src:
        dst = np.zeros((2, H, W), np.float32)
        reproject(src.read().astype(np.float32), dst, src_transform=src.transform, src_crs=src.crs,
                  dst_transform=tr, dst_crs=rasterio.crs.CRS.from_proj4(C.MAP_PROJ), resampling=Resampling.bilinear)
    red, nir = dst
    return (nir - red) / np.maximum(nir + red, 1.0)


def surface_meshes(dem=None):
    """Height of the final top surface (sidewalk > road > terrain) at points."""
    g = np.load(C.GEOM_FILE)
    out = {}
    for pref in ("SideWalk__Concrete", "Road__Asphalt", "Terrain__Ortho"):
        Vs, Fs, n = [], [], 0
        for k in g.files:
            if k.startswith(pref) and k.endswith("|V"):
                Vs.append(g[k]); Fs.append(g[k[:-2] + "|F"] + n); n += len(g[k])
        out[pref] = GB.MeshZ(np.concatenate(Vs), np.concatenate(Fs)) if Vs else None
    # one union per mesh tile (road tiles serve both lists), the tiles on threads (GEOS releases the GIL); own
    # executor: robust_union may use the shared geometry pool itself
    pk = [k for pref in ("SideWalk__Concrete", "Road__Asphalt") for k in g.files if k.startswith(pref) and k.endswith("|V")]
    rk = [k for k in g.files if k.startswith("Road__") and k.endswith("|V")]
    keys = list(dict.fromkeys(pk + rk))
    with ThreadPoolExecutor(max_workers=GP.WORKERS) as ex:
        tile_u = dict(zip(keys, ex.map(lambda k: GB.robust_union(shapely.polygons(g[k][g[k[:-2] + "|F"]][:, :, :2])), keys)))
    paved = [tile_u[k] for k in pk]
    road = [tile_u[k] for k in rk]

    def zsurf(P):
        z = np.full(len(P), np.nan)
        for pref in ("SideWalk__Concrete", "Road__Asphalt", "Terrain__Ortho"):
            m = np.isnan(z)
            if out[pref] is not None and m.any():
                if dem is not None:       # the surface at ground level (never a bridge deck above)
                    z[m] = out[pref](P[m], how="near", zref=dem(P[m, 0], P[m, 1]), tol=3.0)
                else:
                    z[m] = out[pref](P[m], how="max")
        return z
    f_paved, f_road = GP.submit(shapely.union_all, paved), GP.submit(shapely.union_all, road)
    return zsurf, f_paved.result(), f_road.result()


def building_polygons():
    polys = []
    for poly, tags in GB.osm_polygons():
        if ("building" in tags or "building:part" in tags) and not GB.is_underground(tags):
            p = shapely.make_valid(poly)
            if not p.is_empty:
                polys.append(p)
    return shapely.union_all(polys)


class _NearPoints:
    """Accepted points in a grid of cells: is any closer than d to p? Only the cells within d are looked at -
    the same answer as the distance to every accepted point (which made the spacing loops quadratic)."""

    def __init__(self, cell):
        self.cell, self.cells = cell, {}

    def add(self, p):
        self.cells.setdefault((int(p[0] // self.cell), int(p[1] // self.cell)), []).append((p[0], p[1]))

    def any_within(self, p, d):
        cx, cy, k = int(p[0] // self.cell), int(p[1] // self.cell), int(d // self.cell) + 1
        pts = [q for i in range(cx - k, cx + k + 1) for j in range(cy - k, cy + k + 1) for q in self.cells.get((i, j), ())]
        return bool(pts) and np.min(np.hypot(*(np.array(pts) - p).T)) < d


def detect_trees(chm, canopy, tr):
    """Crown peaks on the smoothed canopy, height-adaptive spacing, crown radius from Voronoi area."""
    c = np.where(canopy, chm, 0.0)
    cs = gaussian_filter(c, 1.0)
    peaks = (cs == maximum_filter(cs, size=7)) & (cs >= TREE_MIN_H) & canopy
    r, cidx = np.nonzero(peaks)
    h = maximum_filter(c, size=3)[r, cidx]
    order = np.argsort(-h)
    r, cidx, h = r[order], cidx[order], h[order]
    xy = np.column_stack([tr.c + (cidx + 0.5) * RES, tr.f - (r + 0.5) * RES])
    acc = []
    near = _NearPoints(5.0)
    for i, (p, hi) in enumerate(zip(xy, h)):          # non-max suppression: spacing grows with height
        dmin = max(2.5, 0.22 * hi)
        if near.any_within(p, dmin):
            continue
        acc.append((p[0], p[1], hi)); near.add(p)
    T = np.array(acc)
    # crown radius: canopy pixels assigned to the nearest peak (within 9 m)
    rr, cc = np.nonzero(canopy)
    pix = np.column_stack([tr.c + (cc + 0.5) * RES, tr.f - (rr + 0.5) * RES])
    d, j = cKDTree(T[:, :2]).query(pix, distance_upper_bound=9.0)
    cnt = np.bincount(j[np.isfinite(d)], minlength=len(T))[:len(T)]
    radius = np.sqrt(cnt * RES * RES / math.pi)
    return T, radius


def main():
    meta = load_meta()
    extent = tuple(meta["extent"])
    dem = dem_sampler()
    chm, tr, W, H, pts = canopy_height_model(extent, dem)
    ptree = cKDTree(np.column_stack(pts[:2]))
    ndvi = ndvi_grid(tr, W, H)
    print("rasters", W, "x", H, f"@ {RES} m")

    # the building union and the three whole-map buffers run on the geometry threads beside the rest
    f_bld = GP.submit(lambda: (lambda b: (b, b.buffer(0.3)))(building_polygons()))
    zsurf, paved, road = surface_meshes(dem)
    shapely.prepare(road)
    f_road_in, f_paved_out = GP.submit(road.buffer, -0.5), GP.submit(paved.buffer, 0.05)
    bld, bld_out = f_bld.result()
    bmask = rasterize([(bld_out, 1)], out_shape=(H, W), transform=tr).astype(bool)
    pmask = rasterize([(paved, 1)], out_shape=(H, W), transform=tr).astype(bool)

    veg = gaussian_filter(ndvi, 0.7) > NDVI_VEG
    canopy = veg & (chm >= TREE_MIN_H) & ~bmask
    canopy = binary_opening(canopy, iterations=1)
    shrub = veg & (chm >= SHRUB_MIN_H) & (chm < TREE_MIN_H) & ~bmask & ~pmask
    grass = (gaussian_filter(ndvi, 1.0) > 0.18) & (chm < SHRUB_MIN_H) & ~bmask & ~pmask
    grass = binary_opening(grass, iterations=2)

    T, radius = detect_trees(chm, canopy, tr)
    lanes, lane_z = lane_areas()
    trees = []
    on_road = shapely.contains_xy(f_road_in.result(), T[:, 0], T[:, 1])
    base = zsurf(T[:, :2])
    n_lane = 0
    for (x, y, h), r, rd, zb in zip(T, radius, on_road, base):
        if rd or np.isnan(zb) or h > 35:   # over a driving lane / not a tree (structures)
            continue
        r = float(np.clip(r, 0.8, 0.55 * h))
        # crown depth from the point cloud: a palm is a bare trunk with a small crown on top,
        # broadleaf foliage reaches far down the tree
        idx = ptree.query_ball_point((x, y), max(1.0, 0.8 * r))
        hz = pts[2][idx]
        hz = hz[(hz > 1.0) & (hz <= h + 0.5)]
        crown_base = float(np.percentile(hz, 10)) if len(hz) >= 8 else 0.4 * h
        crown_depth = (h - crown_base) / max(h, 1e-6)
        if h >= 7 and crown_depth < 0.30:
            sp = "palm"
        elif h < 7:
            sp = "broadleaf_s"
        elif h < 13:
            sp = "broadleaf_m"
        else:
            sp = "broadleaf_l"
        clear = r if (crown_base < LOW_CROWN and sp != "palm") else LANE_CLEAR
        if on_lane(lanes, lane_z, np.array([[x, y]]), np.array([zb]), [clear])[0]:
            n_lane += 1       # trunk (or a low crown) in a vehicle lane
            continue
        trees.append(dict(x=round(float(x), 2), y=round(float(y), 2), z=round(float(zb), 3), height=round(float(h), 2),
                          radius=round(r, 2), crown_base=round(crown_base, 2), species=sp,
                          yaw=round(float(rng.uniform(0, 360)), 1),
                          variant=int(rng.integers(0, 4))))
    # shrubs: Poisson-like sampling on the shrub mask
    rr, cc = np.nonzero(shrub)
    sp_xy = np.column_stack([tr.c + (cc + 0.5) * RES, tr.f - (rr + 0.5) * RES])
    sp_h = chm[rr, cc]
    shrubs, taken = [], []
    near = _NearPoints(2.0)
    for i in rng.permutation(len(sp_xy)):
        p = sp_xy[i]
        if near.any_within(p, 1.6):
            continue
        taken.append(p); near.add(p)
        if len(taken) > 4000:
            break
    if taken:
        tp = np.array(taken)
        zb = zsurf(tp)
        hh = sp_h[cKDTree(sp_xy).query(tp)[1]]
        lane_hit = on_lane(lanes, lane_z, tp, zb, np.maximum(0.5, 0.6 * hh))   # a shrub's crown is its body
        n_lane += int(lane_hit.sum())
        for p, z, hv, lh in zip(tp, zb, hh, lane_hit):
            if not np.isnan(z) and not lh:
                shrubs.append(dict(x=round(float(p[0]), 2), y=round(float(p[1]), 2), z=round(float(z), 3),
                                   height=round(float(hv), 2), radius=round(float(max(0.5, 0.6 * hv)), 2),
                                   species="shrub", yaw=round(float(rng.uniform(0, 360)), 1), variant=int(rng.integers(0, 4))))
    # grass patches -> mesh 3 cm above the surface underneath
    gpolys = [shapely.geometry.shape(gm) for gm, v in shapes(grass.astype(np.uint8), mask=grass, transform=tr) if v == 1]
    gu = shapely.union_all(gpolys).simplify(0.25)
    gu = GB.polys_only(gu.difference(f_paved_out.result()).difference(bld))
    layer = GB.Layer()
    n_patch = 0
    # triangulate every patch first, then the surface height of all their points in one call (per patch it was
    # ~9000 DEM / mesh queries with their set-up cost each); zsurf works point by point, so the same heights
    tris = [res for p in shapely.get_parts(gu) if p.area >= 3.0
            for res in [GB.triangulate(p, lambda P: np.zeros(len(P)), GB.planar_uv, seg=2.0)] if res]
    if tris:
        Z = zsurf(np.concatenate([V[:, :2] for V, _, _ in tris])) + 0.03
        cut = np.cumsum([len(V) for V, _, _ in tris])[:-1]
        for (V, F, UV), z in zip(tris, np.split(Z, cut)):
            V[:, 2] = z
            ok = ~np.isnan(V[F]).any((1, 2))
            layer.add(V, F[ok], UV); n_patch += 1
    arrays = {}
    res = layer.arrays()
    if res is not None:
        V, F, UV, _ = res
        nx = int(math.ceil((extent[2] - extent[0]) / GB.TILE))
        cen = V[F].mean(1)
        t = ((cen[:, 1] - extent[1]) // GB.TILE).astype(int) * nx + ((cen[:, 0] - extent[0]) // GB.TILE).astype(int)
        for tt in np.unique(t):
            f = F[t == tt]
            used, inv = np.unique(f.ravel(), return_inverse=True)
            key = f"Terrain__Grass|T{tt:02d}"
            arrays[key + "|V"] = V[used]; arrays[key + "|F"] = inv.reshape(-1, 3).astype(np.int32)
            arrays[key + "|UV"] = UV[used].astype(np.float32)
    np.savez_compressed(C.BUILD / "vegetation_geom.npz", **arrays)
    out = dict(trees=trees, shrubs=shrubs, source="USGS 3DEP lidar CA_LosAngeles_1_B23 + NAIP NDVI",
               grass_area_m2=round(float(gu.area), 1))
    (C.BUILD / "vegetation.json").write_text(json.dumps(out))
    from collections import Counter
    print("trees:", len(trees), dict(Counter(t["species"] for t in trees)))
    print("tree height p10/p50/p90:", np.percentile([t["height"] for t in trees], [10, 50, 90]).round(1) if trees else "-")
    print("shrubs:", len(shrubs), "| grass patches:", n_patch, f"({gu.area:.0f} m2)")
    print("dropped from vehicle lanes (OpenDRIVE, same level):", n_lane)

    # debug image
    img = np.zeros((H, W, 3), np.uint8)
    img[..., :] = 30
    img[pmask] = (70, 70, 75); img[bmask] = (110, 80, 130)
    img[grass] = (90, 170, 70); img[shrub] = (40, 120, 40); img[canopy] = (20, 90, 20)
    im = Image.fromarray(img); d = ImageDraw.Draw(im)
    for t in trees:
        px, py = (t["x"] - tr.c) / RES, (tr.f - t["y"]) / RES
        rr_ = t["radius"] / RES
        col = (255, 200, 0) if t["species"] == "palm" else (255, 255, 255)
        d.ellipse([px - rr_, py - rr_, px + rr_, py + rr_], outline=col, width=2)
    im.save(C.BUILD / "vegetation_debug.png")


if __name__ == "__main__":
    main()
