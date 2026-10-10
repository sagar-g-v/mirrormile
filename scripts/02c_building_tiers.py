"""Building massing from lidar: stacked roof tiers per OSM footprint (setbacks, podiums, penthouses).

OSM footprints in LA come from the LA County import and carry ONE height (the highest feature), so
03 would extrude every tower as a single box. The 2023 USGS 3DEP lidar (public domain) sees the
real roof: per footprint we build a 0.5 m surface model (highest non-ground return per cell, trees
removed with NAIP NDVI), find the dominant roof levels and turn each level into a nested region.
03 then builds the building as a stack: footprint up to tier 0, tier k's region from tier k-1 up to
tier k (regions are nested, so walls never cross).

Rules
  * lidar is used only where it covers the footprint well (>= 60 % of cells have roof returns)
  * a footprint the lidar sees as (nearly) empty ground keeps the OSM height (new construction /
    data age mismatch), as does any footprint without enough returns
  * small low buildings (< 15 m, < 400 m2, mostly pitched houses) get one tier at the median roof
    height (the middle of the pitch) instead of a stepped roof

Output: build/building_tiers.json  {building_key: {"h_osm", "h_lidar", "tiers": [{"z", "wkt"}...]}}
z = absolute roof height in the DEM datum; wkt = tier region (map frame), tier 0 = whole footprint.
"""
import json
import math
from collections import Counter

import numpy as np
import rasterio
import shapely
from pyproj import Transformer
from rasterio.features import rasterize, shapes
from rasterio.transform import from_origin
from rasterio.warp import Resampling, reproject
from scipy.ndimage import binary_opening, gaussian_filter, gaussian_filter1d, median_filter, distance_transform_edt

import config as C
import importlib
GB = importlib.import_module("03_build_geometry")

RES = 0.5
MIN_COVER = 0.60          # fraction of footprint cells with a roof return
STEP = 3.0                # m: roof levels closer than this are one tier
TIER_TOL = 1.2            # m: a cell belongs to tier k if its roof is within this of level k (or above)
MIN_TIER_AREA = 50.0      # m2 (smaller: rooftop equipment, not massing)
MIN_TIER_WIDTH = 4.0      # m (opening radius ~ half of it)
MAX_TIERS = 6
NDVI_TREE = 0.20
NEAR = 7.0                # m: footprint-to-lidar misregistration allowance next to taller buildings


def lidar_surface(extent, dem):
    p = np.load(C.DATA / "lidar" / "points.npz")
    tf = Transformer.from_crs(3857, C.MAP_PROJ, always_xy=True)
    x, y = tf.transform(p["x"], p["y"])
    z, cls = p["z"].astype(np.float64), p["cls"]
    keep = (cls != 7) & (cls != 18) & (cls != 17)
    x, y, z, cls = x[keep], y[keep], z[keep], cls[keep]
    ground = cls == 2
    rng = np.random.default_rng(1)
    gi = rng.choice(np.flatnonzero(ground), size=min(200000, int(ground.sum())), replace=False)
    offset = float(np.median(z[gi] - dem(x[gi], y[gi])))
    z = z - offset                                         # -> DEM datum
    x0, y0, x1, y1 = extent
    W, H = int(round((x1 - x0) / RES)), int(round((y1 - y0) / RES))
    col = ((x - x0) / RES).astype(np.int64); row = ((y1 - y) / RES).astype(np.int64)
    ok = (col >= 0) & (col < W) & (row >= 0) & (row < H)
    dsm = np.full(H * W, -np.inf, np.float32)
    np.maximum.at(dsm, row[ok & ~ground] * W + col[ok & ~ground], z[ok & ~ground].astype(np.float32))
    dsm = dsm.reshape(H, W)
    dsm[~np.isfinite(dsm)] = np.nan
    print(f"lidar DSM {W}x{H} @ {RES} m, datum offset {offset:+.2f} m")
    return dsm, from_origin(x0, y1, RES, RES), W, H


def ndvi_grid(tr, W, H):
    with rasterio.open(C.DATA / "rednir_utm.tif") as src:
        dst = np.zeros((2, H, W), np.float32)
        reproject(src.read().astype(np.float32), dst, src_transform=src.transform, src_crs=src.crs,
                  dst_transform=tr, dst_crs=rasterio.crs.CRS.from_proj4(C.MAP_PROJ), resampling=Resampling.bilinear)
    red, nir = dst
    return (nir - red) / np.maximum(nir + red, 1.0)


def roof_levels(h, cell_area):
    """Dominant roof levels (height above ground) from the area histogram of roof heights."""
    lo, hi = float(np.floor(h.min())), float(np.ceil(h.max())) + 1.0
    bins = np.arange(lo, hi + 0.25, 0.25)
    hist, edges = np.histogram(h, bins)
    hs = gaussian_filter1d(hist.astype(float), 2.0) * cell_area          # m2 per 0.25 m bin, smoothed
    centres = 0.5 * (edges[:-1] + edges[1:])
    pk = [i for i in range(len(hs)) if hs[i] > 0 and (i == 0 or hs[i] >= hs[i - 1]) and (i == len(hs) - 1 or hs[i] > hs[i + 1])]
    cand = []
    for i in pk:                                          # area supporting a peak: +-STEP/2 around it
        m = np.abs(centres - centres[i]) <= STEP / 2
        cand.append((centres[i], float(hist[m].sum()) * cell_area))
    cand = [c for c in cand if c[1] >= MIN_TIER_AREA]
    cand.sort(key=lambda c: -c[1])
    levels = []
    for z, a in cand:                                      # biggest first, drop near-duplicates
        if all(abs(z - l) >= STEP for l in levels):
            levels.append(z)
    return sorted(levels)


def polygons_of(mask, tr):
    geoms = [shapely.geometry.shape(g) for g, v in shapes(mask.astype(np.uint8), mask=mask, transform=tr) if v]
    return shapely.union_all(geoms) if geoms else shapely.Polygon()


def tiers_for(poly, dsm, veg, tr, ground_z, h_osm, taller=(), inside=()):
    """taller: [(neighbour polygon, its height)] of taller buildings within NEAR m. Footprints are
    not perfectly registered to the lidar, so cells near a tall neighbour can carry the neighbour's
    roof/facade; such cells (height matching the neighbour, well above this building) are dropped."""
    x0, y0, x1, y1 = poly.bounds
    c0 = max(int((x0 - tr.c) / RES) - 2, 0); c1 = min(int((x1 - tr.c) / RES) + 3, dsm.shape[1])
    r0 = max(int((tr.f - y1) / RES) - 2, 0); r1 = min(int((tr.f - y0) / RES) + 3, dsm.shape[0])
    if c1 - c0 < 3 or r1 - r0 < 3:
        return None, "tiny"
    sub_tr = rasterio.Affine(RES, 0, tr.c + c0 * RES, 0, -RES, tr.f - r0 * RES)
    inner = poly.buffer(-0.75)
    if inner.is_empty:
        return None, "thin"
    fp = rasterize([(inner, 1)], out_shape=(r1 - r0, c1 - c0), transform=sub_tr).astype(bool)
    if fp.sum() < 8:
        return None, "thin"
    if inside:
        n0 = fp.sum()
        fp &= ~rasterize([(q.buffer(1.0), 1) for q in inside], out_shape=fp.shape, transform=sub_tr).astype(bool)
        if fp.sum() < max(8, 0.25 * n0):
            return None, "container"
    d = dsm[r0:r1, c0:c1].copy()
    d[veg[r0:r1, c0:c1]] = np.nan                         # tree crowns over the roof
    hag = d - ground_z
    if taller:
        zones = [(rasterize([(nb.buffer(NEAR), 1)], out_shape=fp.shape, transform=sub_tr).astype(bool), hn) for nb, hn in taller]
        anyz = np.zeros(fp.shape, bool)
        for z_, _ in zones:
            anyz |= z_
        own = fp & ~anyz & np.isfinite(hag) & (hag > 2.0)
        href = float(np.median(hag[own])) if own.sum() >= 8 else (h_osm or 0.0)
        for z_, hn in zones:
            bad = z_ & np.isfinite(hag) & (hag > href + 5.0) & (hag < hn + 3.0)
            hag[bad] = np.nan
    roof = fp & np.isfinite(hag) & (hag > 2.0)
    if fp.sum() and roof.sum() / fp.sum() < MIN_COVER:
        lowish = fp & np.isfinite(hag) & (hag <= 2.0)
        return None, ("ground_in_lidar" if lowish.sum() / fp.sum() > 0.5 else "low_cover")
    # fill holes (dark roofs / glass give no returns) from the nearest roof cell, de-speckle
    idx = distance_transform_edt(~roof, return_distances=False, return_indices=True)
    hf = hag[idx[0], idx[1]]
    hf = median_filter(hf, size=3)
    h = hf[fp]
    h_lidar = float(np.percentile(h, 98))
    area = poly.area
    if h_lidar < 15 and area < 400:                       # houses: one tier at mid-pitch
        return {"h_lidar": h_lidar, "levels": [float(np.median(h))], "regions": [poly]}, "single"
    levels = roof_levels(h, RES * RES) or [float(np.median(h))]
    cap = 2 if h_lidar < 25 else MAX_TIERS             # low/mid-rise: base + one setback at most
    if len(levels) > cap:                              # keep the lowest (base) and the largest-area upper levels
        levels = [levels[0]] + sorted(levels[1:], key=lambda l: -float(np.sum(h >= l - TIER_TOL)))[:cap - 1]
        levels = sorted(levels)
    if len(levels) > MAX_TIERS:
        levels = levels[-MAX_TIERS:]
    regions, prev = [], poly
    full = rasterize([(poly, 1)], out_shape=fp.shape, transform=sub_tr).astype(bool)
    for k, lv in enumerate(levels):
        if k == 0:
            regions.append(poly); continue
        m = full & (hf >= lv - TIER_TOL)
        m = binary_opening(m, iterations=max(1, int(MIN_TIER_WIDTH / 2 / RES)))
        reg = polygons_of(m, sub_tr)
        if reg.is_empty:
            break
        # regularise the raster outline: ~1 m simplification, square corners (mitred close/open)
        reg = shapely.make_valid(reg.simplify(1.0)).buffer(1.0, join_style="mitre").buffer(-2.0, join_style="mitre").buffer(1.0, join_style="mitre")
        reg = shapely.make_valid(reg).intersection(prev).buffer(0)
        reg = shapely.union_all([g for g in shapely.get_parts(reg) if g.geom_type == "Polygon" and g.area >= MIN_TIER_AREA])
        if reg.is_empty or reg.area < MIN_TIER_AREA or reg.area > 0.97 * prev.area:
            if not reg.is_empty and reg.area > 0.97 * prev.area:
                levels[k - 1] = lv                         # the "lower" level is just roof noise: merge up
                continue
            break
        regions.append(reg); prev = reg
    levels = levels[:len(regions)]
    # each tier's roof height: median of its own exposed roof cells (robust to parapets / equipment)
    out_lv = []
    for k, reg in enumerate(regions):
        own = reg.difference(regions[k + 1]) if k + 1 < len(regions) else reg
        m = rasterize([(own, 1)], out_shape=fp.shape, transform=sub_tr).astype(bool) & roof if not own.is_empty else None
        z = float(np.percentile(hag[m], 60)) if m is not None and m.sum() >= 4 else levels[k]
        if out_lv and z < out_lv[-1] + 1.0:
            z = out_lv[-1] + 1.0
        out_lv.append(z)
    return {"h_lidar": h_lidar, "levels": out_lv, "regions": regions}, f"tiers{len(regions)}"


def main():
    # whole download area in the map frame (03 decides later which buildings fall inside the map)
    s, w, n, e = C.outer_bbox()
    tf = Transformer.from_crs(4326, C.MAP_PROJ, always_xy=True)
    xs, ys = tf.transform([w, e, w, e], [s, s, n, n])
    extent = (min(xs), min(ys), max(xs), max(ys))
    dem = GB.Dem(C.DEM_UTM, filt=5)
    dsm, tr, W, H = lidar_surface(extent, dem)
    ndvi = ndvi_grid(tr, W, H)
    veg = gaussian_filter(ndvi, 0.7) > NDVI_TREE
    out, stats = {}, Counter()
    box = shapely.box(*extent)
    items = []
    for raw, tags in GB.osm_polygons():
        key = GB.building_key(raw)
        poly = shapely.make_valid(raw).buffer(0)
        if poly.is_empty or not box.contains(poly.centroid) or poly.area < 4:
            continue
        if poly.geom_type != "Polygon":
            poly = max(shapely.get_parts(poly), key=lambda g: g.area)
        ext = np.asarray(poly.exterior.coords)
        ground = float(np.min(dem(ext[:, 0], ext[:, 1])))
        lv = tags.get("building:levels")
        h_osm = GB.parse_len(tags.get("height")) or (float(lv) * 3.6 + 1.0 if lv and lv.replace(".", "").isdigit() else None)
        minh = GB.parse_len(tags.get("min_height")) or 0.0
        if minh > 0 or tags.get("building") in ("roof", "carport", "canopy"):
            stats["skip_floating"] += 1; continue           # canopies / bridges between towers: OSM as is
        if GB.is_underground(tags):
            stats["skip_underground"] += 1; continue
        items.append((key, poly, ground, h_osm))
    # pass 1: every footprint on its own -> a height per building
    stree = shapely.STRtree([p for _, p, _, _ in items])
    inside = []
    for i, (_, p, _, _) in enumerate(items):
        js = [j for j in stree.query(p, predicate="intersects") if j != i and items[j][1].area < p.area]
        inside.append([items[j][1] for j in js if items[j][1].intersection(p).area > 5.0])
    first = [tiers_for(p, dsm, veg, tr, g, h, inside=inside[i])[0] for i, (_, p, g, h) in enumerate(items)]
    hgt = np.array([(r["h_lidar"] if r else (h or 0.0)) for r, (_, _, _, h) in zip(first, items)])
    for i, (key, poly, ground, h_osm) in enumerate(items):
        # pass 2: drop cells that belong to a taller neighbour (footprint / lidar misregistration)
        nb = [j for j in stree.query(poly, predicate="dwithin", distance=NEAR) if j != i]
        own_ref = min(hgt[i], h_osm) if h_osm else hgt[i]
        taller = [(items[j][1], float(hgt[j])) for j in nb if hgt[j] > own_ref + 8.0]
        res, why = tiers_for(poly, dsm, veg, tr, ground, h_osm, taller, inside[i]) if taller else (first[i], None)
        if why is None:
            why = "ok" if first[i] else "fallback_osm"
        stats[why] += 1
        if taller:
            stats["checked_vs_taller_neighbour"] += 1
        if res is None:
            continue
        if h_osm and abs(res["h_lidar"] - h_osm) > max(8.0, 0.35 * h_osm):
            stats["osm_lidar_disagree"] += 1
        out[key] = {"h_osm": h_osm, "h_lidar": round(res["h_lidar"], 2),
                    "tiers": [{"z": round(ground + z, 3), "wkt": (None if k == 0 else shapely.to_wkt(r, rounding_precision=2))}
                              for k, (z, r) in enumerate(zip(res["levels"], res["regions"]))]}
    (C.BUILD / "building_tiers.json").write_text(json.dumps(out))
    multi = sum(1 for v in out.values() if len(v["tiers"]) > 1)
    print("building tiers:", dict(stats), f"| {len(out)} footprints from lidar, {multi} multi-tier")


if __name__ == "__main__":
    main()
