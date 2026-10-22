"""OpenDRIVE + SUMO net + OSM + DEM + ortho  ->  tiled meshes for Blender (build/geometry.npz).

Layers are named "<CarlaLabel>__<Material>"; CarlaLabel is the CARLA semantic folder
(Road, SideWalk, RoadLine, Terrain, Building) that the asset will be imported into.
"""
import hashlib
import itertools
import json
import math
import re
from collections import defaultdict

import numpy as np
import rasterio
import shapely
from lxml import etree
from PIL import Image
from pyproj import Transformer
from rasterio.warp import Resampling, reproject
from rasterio.transform import from_bounds
from scipy.ndimage import map_coordinates, median_filter
from scipy.spatial import cKDTree
from shapely.geometry import Polygon, MultiPolygon, LineString
from shapely.ops import polygonize, linemerge


# results of the 1 mm grid union (polys_only / unary_union) by id: such a geometry is already polygonal, valid and
# on the grid, so polys_only of it is itself - skipping the repeat saves re-uniting whole-map geometries
_SNAPPED = {}


def _snapped(g):
    if shapely.get_num_coordinates(g) >= 500:    # small ones are cheap to redo; keep the cache small
        _SNAPPED[id(g)] = g
    return g


def polys_only(g):
    if _SNAPPED.get(id(g)) is g:
        return g
    g = shapely.make_valid(g)
    parts = [p for p in shapely.get_parts(g) if p.geom_type in ("Polygon", "MultiPolygon") and not p.is_empty]
    return _snapped(shapely.union_all(parts, grid_size=1e-3)) if parts else Polygon()


UNION_TILES = 16      # large unions: 16 x 16 tiles by centroid, then the tiles (whole-map unions of ~650k
UNION_TILED_FROM = 20000   # triangles took 60-160 s and fell back to the 5 mm buffer pass; tiled: ~30 s)


def robust_union(polys, grid=1e-2):
    """Union of many (possibly degenerate) triangles: grid-snapped union, falling back to repaired
    geometry and finally a tiny buffer when GEOS reports a topology exception. Large inputs are united
    tile by tile (same fallbacks per tile), then the tiles."""
    polys = np.asarray(polys, dtype=object)
    polys = polys[~shapely.is_empty(polys)] if len(polys) else polys
    if len(polys) >= UNION_TILED_FROM:
        c = shapely.centroid(polys)
        x, y = shapely.get_x(c), shapely.get_y(c)
        ix = np.minimum((UNION_TILES * (x - x.min()) / max(np.ptp(x), 1e-9)).astype(int), UNION_TILES - 1)
        iy = np.minimum((UNION_TILES * (y - y.min()) / max(np.ptp(y), 1e-9)).astype(int), UNION_TILES - 1)
        key = ix * UNION_TILES + iy
        parts = geom_par.union_groups([polys[key == k] for k in np.unique(key)], grid)   # tiles in parallel
        return geom_par.union_with_fallbacks(np.asarray(parts, dtype=object), grid)
    return geom_par.union_with_fallbacks(polys, grid)


def unary_union(geoms):
    """Robust union: polygonal parts only, snapped to a 1 mm grid."""
    if not isinstance(geoms, (list, tuple)):
        geoms = [geoms]
    parts = [polys_only(g) for g in geoms if g is not None and not g.is_empty]
    return _snapped(shapely.union_all(parts, grid_size=1e-3)) if parts else Polygon()

import config as C
import geom_par
from opendrive import OpenDrive, write_elevation

CURB = 0.15          # sidewalk height above road (m)
MARK_Z = 0.012       # paint lift above asphalt (m)
MARK_W = 0.12        # line width (m)
DASH, GAP = 3.05, 9.15   # US MUTCD 10 ft / 30 ft
TILE = 256.0         # mesh tile size (m) -> one UE static mesh per tile & layer
TERRAIN_STEP = 2.0   # m
TERRAIN_MARGIN = 120.0
ORTHO_PX = 4096      # power-of-two texture for UE mipmaps

ROAD_TYPES = {"driving", "restricted", "parking", "bidirectional", "biking", "bus", "taxi", "HOV",
              "entry", "exit", "onRamp", "offRamp", "connectingRamp", "shoulder", "stop", "border"}
WALK_TYPES = {"sidewalk", "walking", "curb", "median"}


# ----------------------------------------------------------------------------------- meshes
class Layer:
    def __init__(self):
        self.V, self.F, self.UV, self.T = [], [], [], []   # T: forced tile per face (or None)
        self.O = []                                        # O: owner road id per face (-1 = none)
        self.n = 0

    def add(self, V, F, UV, tile=None, owner=-1):
        V = np.asarray(V, np.float64); F = np.asarray(F, np.int64); UV = np.asarray(UV, np.float64)
        if len(F) == 0:
            return
        self.V.append(V); self.UV.append(UV); self.F.append(F + self.n)
        self.T.append(np.full(len(F), -1 if tile is None else tile, np.int64))
        self.O.append(np.full(len(F), owner, np.int64))
        self.n += len(V)

    def strip(self, A, B, uvA, uvB, up=True, owner=-1):
        """Quad strip between polylines A and B (n,3). Faces oriented to +Z if up."""
        n = len(A)
        if n < 2:
            return
        V = np.concatenate([A, B]); UV = np.concatenate([uvA, uvB])
        i = np.arange(n - 1)
        F = np.concatenate([np.stack([i, i + n, i + n + 1], 1), np.stack([i, i + n + 1, i + 1], 1)])
        if up is not None:
            e1 = V[F[:, 1]] - V[F[:, 0]]; e2 = V[F[:, 2]] - V[F[:, 0]]
            if np.sum(np.cross(e1, e2)[:, 2]) < 0 if up else np.sum(np.cross(e1, e2)[:, 2]) > 0:
                F = F[:, ::-1]
        self.add(V, F, UV, owner=owner)

    def wall(self, P, z0, z1, u0=0.0):
        """Vertical wall along polyline P (n,2) between per-point z0,z1 (outward = right side)."""
        P = np.asarray(P)
        n = len(P)
        if n < 2:
            return
        z0 = np.broadcast_to(z0, n); z1 = np.broadcast_to(z1, n)
        u = u0 + np.concatenate([[0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
        B = np.column_stack([P, z0]); Tp = np.column_stack([P, z1])
        V = np.concatenate([B, Tp]); UV = np.concatenate([np.column_stack([u, z0 - z0.min()]),
                                                       np.column_stack([u, z1 - z0.min()])])
        i = np.arange(n - 1)
        F = np.concatenate([np.stack([i, i + 1, i + n + 1], 1), np.stack([i, i + n + 1, i + n], 1)])
        self.add(V, F, UV)

    def drop_faces_inside(self, region, zref=None, tol=None):
        """Remove faces whose centroid lies inside `region` (and, if zref given, within tol of it)."""
        if not self.F or region.is_empty:
            return 0
        shapely.prepare(region)
        dropped, base = 0, 0
        for k in range(len(self.F)):
            c = self.V[k][self.F[k] - base].mean(1)        # F holds global indices
            base += len(self.V[k])
            inside = shapely.contains_xy(region, c[:, 0], c[:, 1])
            if zref is not None:
                inside &= np.abs(c[:, 2] - zref) < tol
            if inside.any():
                self.F[k] = self.F[k][~inside]; self.T[k] = self.T[k][~inside]; self.O[k] = self.O[k][~inside]
                dropped += int(inside.sum())
        return dropped

    def n_faces(self):
        return sum(len(f) for f in self.F)

    def arrays(self):
        if not self.V:
            return None
        return np.concatenate(self.V), np.concatenate(self.F), np.concatenate(self.UV), np.concatenate(self.T)


LAYERS = defaultdict(Layer)
PAVED_Z = None   # set in main(): height of the final road/sidewalk surface at (x, y)
CROSS_BANDS = []  # crosswalk footprints (filled by build_crosswalks)
DRIVE_OWNER = []  # road id of each polygon in drive_polys
DRIVE_P3, WALK_P3 = [], []          # 3D points of each drive / walk strip polygon (level tests)
JREF_BY = defaultdict(list)         # junction id -> 3D lane triangles of its connecting roads
LEVEL_TOL = 2.5                     # m: surfaces closer than this in height are on the same level


def z_near(p3, xy):
    """Height of a strip (its 3D points p3) near the plan position xy."""
    d = np.hypot(p3[:, 0] - xy[0], p3[:, 1] - xy[1])
    return float(p3[int(np.argmin(d)), 2])


def triangulate(poly, zfun, uvfun, seg=2.0):
    """Constrained Delaunay triangulation of a polygon (with holes) -> V, F, UV (upward)."""
    poly = shapely.segmentize(poly, seg)
    tris = shapely.get_parts(shapely.constrained_delaunay_triangles(poly))
    if len(tris) == 0:
        return None
    co = np.array([np.asarray(t.exterior.coords)[:3] for t in tris])          # (m,3,2)
    pts, inv = np.unique(np.round(co.reshape(-1, 2), 4), axis=0, return_inverse=True)
    F = inv.reshape(-1, 3)
    a, b, c = pts[F[:, 0]], pts[F[:, 1]], pts[F[:, 2]]
    cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
    F[cross < 0] = F[cross < 0][:, ::-1]
    F = F[np.abs(cross) > 1e-8]
    V = np.column_stack([pts, zfun(pts)])
    return V, F, uvfun(pts)


def planar_uv(P):
    return np.asarray(P)[:, :2].copy()      # metres; material scales


# ----------------------------------------------------------------------------------- rasters
TO_LL = Transformer.from_crs(C.MAP_PROJ, 4326, always_xy=True)
TO_MAP = Transformer.from_crs(4326, C.MAP_PROJ, always_xy=True)
TO_UTM = Transformer.from_crs(C.MAP_PROJ, C.UTM_EPSG, always_xy=True)


class Dem:
    def __init__(self, path, filt=3):
        with rasterio.open(path) as src:
            d = src.read(1).astype(np.float64); self.inv = ~src.transform; b = src.bounds
        d[d < -1000] = np.nan
        d = np.where(np.isnan(d), np.nanmedian(d), d)
        self.d = median_filter(d, size=filt) if filt > 1 else d
        self.utm_bounds = b

    def __call__(self, x, y):
        ux, uy = TO_UTM.transform(np.asarray(x, float), np.asarray(y, float))
        col, row = self.inv * (np.asarray(ux), np.asarray(uy))
        return map_coordinates(self.d, [np.asarray(row) - 0.5, np.asarray(col) - 0.5], order=1, mode="nearest")


# ----------------------------------------------------------------------------------- roads
def _other_road_strips(od):
    """Driving-lane strips of every non-junction road (plan polygon, 3D points, road id): used to
    find sidewalk lanes that SUMO put on a side facing ANOTHER carriageway (merges, gores, divided
    roads) -- a raised sidewalk there would sit in the middle of the asphalt."""
    polys, p3, owner = [], [], []
    for r in od.roads:
        if r.junction != "-1":
            continue
        for sec in r.sections:
            s = r.sample_s(sec.s, sec.e)
            if len(s) < 2:
                continue
            B, _ = r.borders(sec, s)
            for lane in sec.left + sec.right:
                if lane.type not in ROAD_TYPES:
                    continue
                ti, to = B[lane.id]
                if np.max(np.abs(to - ti)) < 0.05:
                    continue
                A = r.xyz(s, ti); Bo = r.xyz(s, to)
                polys.append(polys_only(Polygon(np.concatenate([A[:, :2], Bo[::-1, :2]]))))
                p3.append(np.concatenate([A, Bo])); owner.append(r.id)
    return polys, p3, owner


GORE_DIST = 0.8      # m: a sidewalk edge this close to another road's lanes faces that carriageway


def _neighbours3(mask):
    """Count of True in each sample's 3-window (itself + both neighbours); same length as mask."""
    return np.convolve(np.asarray(mask, int), [1, 1, 1])[1:-1]


def build_roads(od):
    drive_polys, walk_polys = [], []
    junction_drive = defaultdict(list)
    drive_pts, walk_pts = [], []
    o_polys, o_p3, o_owner = _other_road_strips(od)
    o_tree = shapely.STRtree(o_polys)
    stats = defaultdict(int)

    def facing_other_road(rid, P3):
        """Per sample: does this (outer sidewalk edge) point touch another road's lanes, same level?"""
        pts = shapely.points(P3[:, :2])
        out = np.zeros(len(P3), bool)
        pi, ti = o_tree.query(pts, predicate="dwithin", distance=GORE_DIST)
        for p, t in zip(pi, ti):
            if o_owner[t] != rid and abs(z_near(o_p3[t], P3[p, :2]) - P3[p, 2]) < 1.5:
                out[p] = True
        # close 1-sample gaps, ignore isolated hits
        return _neighbours3(out) >= 2

    for r in od.roads:
        in_junction = r.junction != "-1"
        for sec in r.sections:
            extra = [sec.s + o for l in sec.left + sec.right for o, _ in l.marks + l.widths]
            s = r.sample_s(sec.s, sec.e, extra)
            if len(s) < 2:
                continue
            B, _ = r.borders(sec, s)
            for lane in sec.left + sec.right:
                ti, to = B[lane.id]
                if np.max(np.abs(to - ti)) < 0.05:
                    continue
                walk = lane.type in WALK_TYPES
                if not walk and lane.type not in ROAD_TYPES:
                    continue
                if walk and in_junction:
                    continue        # pedestrian connectors: corners/crosswalks cover these
                dz = CURB if walk else 0.0
                A = r.xyz(s, ti, dz); Bo = r.xyz(s, to, dz)
                if walk and not in_junction:
                    gore = facing_other_road(r.id, r.xyz(s, to))
                    if gore.any():
                        # stretches facing another carriageway: flush asphalt (no kerb), rest stays sidewalk
                        for i0, i1 in _true_runs(gore):
                            Ag, Bg = r.xyz(s[i0:i1 + 1], ti[i0:i1 + 1]), r.xyz(s[i0:i1 + 1], to[i0:i1 + 1])
                            LAYERS["Road__Asphalt"].strip(Ag, Bg, planar_uv(Ag), planar_uv(Bg), owner=int(r.id))
                            gp = polys_only(Polygon(np.concatenate([Ag[:, :2], Bg[::-1, :2]])))
                            drive_polys.append(gp); DRIVE_OWNER.append(int(r.id)); DRIVE_P3.append(np.concatenate([Ag, Bg]))
                            drive_pts.append(np.concatenate([Ag, Bg]))
                            stats["sidewalk_m_to_asphalt"] += int(s[i1] - s[i0])
                        keep = ~gore
                        if keep.sum() < 2:
                            continue
                        # remaining sidewalk pieces are built separately below
                        for i0, i1 in _true_runs(keep):
                            sl = slice(i0, i1 + 1)
                            _sidewalk_strip(r, lane, s[sl], ti[sl], to[sl], walk_polys, walk_pts)
                        continue
                ring = np.concatenate([A[:, :2], Bo[::-1, :2]])
                poly = polys_only(Polygon(ring))
                pts = np.concatenate([A, Bo, r.xyz(s, (ti + to) / 2, dz)])
                if walk:
                    walk_polys.append(poly); walk_pts.append(pts); WALK_P3.append(pts)
                    LAYERS["SideWalk__Concrete"].strip(A, Bo, planar_uv(A), planar_uv(Bo))
                    # curb face on the road side, skirt on the outer side
                    up = 1 if lane.id > 0 else -1
                    inner_xy, outer_xy = A[:, :2], Bo[:, :2]
                    # wall() faces right of travel direction: choose order so it faces outward
                    c_in = inner_xy if up < 0 else inner_xy[::-1]
                    zc = A[:, 2] if up < 0 else A[::-1, 2]
                    LAYERS["SideWalk__Curb"].wall(c_in, zc - CURB - 0.03, zc)
                    c_out = outer_xy[::-1] if up < 0 else outer_xy
                    zo = Bo[::-1, 2] if up < 0 else Bo[:, 2]
                    LAYERS["SideWalk__Curb"].wall(c_out, zo - 0.4, zo)
                else:
                    drive_pts.append(pts)
                    if in_junction:
                        jname = od.junctions.get(r.junction, r.junction)
                        junction_drive[jname].append(poly)
                        # exact 3D lane surface of the connecting road -> junction height reference
                        n = len(A)
                        for i in range(n - 1):
                            JUNCTION_REF.append((A[i], Bo[i], Bo[i + 1]))
                            JUNCTION_REF.append((A[i], Bo[i + 1], A[i + 1]))
                            JREF_BY[jname].append((A[i], Bo[i], Bo[i + 1]))
                            JREF_BY[jname].append((A[i], Bo[i + 1], A[i + 1]))
                    else:
                        drive_polys.append(poly); DRIVE_OWNER.append(int(r.id)); DRIVE_P3.append(pts)
                        LAYERS["Road__Asphalt"].strip(A, Bo, planar_uv(A), planar_uv(Bo), owner=int(r.id))
    if stats:
        print("roads:", dict(stats))
    return drive_polys, walk_polys, junction_drive, np.concatenate(drive_pts), np.concatenate(walk_pts)


def _sidewalk_strip(r, lane, s, ti, to, walk_polys, walk_pts):
    """Raised sidewalk strip with the kerb face on the road side and a skirt on the outer side."""
    A = r.xyz(s, ti, CURB); Bo = r.xyz(s, to, CURB)
    walk_polys.append(polys_only(Polygon(np.concatenate([A[:, :2], Bo[::-1, :2]]))))
    pts = np.concatenate([A, Bo, r.xyz(s, (ti + to) / 2, CURB)])
    walk_pts.append(pts); WALK_P3.append(pts)
    LAYERS["SideWalk__Concrete"].strip(A, Bo, planar_uv(A), planar_uv(Bo))
    up = 1 if lane.id > 0 else -1
    inner_xy, outer_xy = A[:, :2], Bo[:, :2]
    c_in = inner_xy if up < 0 else inner_xy[::-1]
    zc = A[:, 2] if up < 0 else A[::-1, 2]
    LAYERS["SideWalk__Curb"].wall(c_in, zc - CURB - 0.03, zc)
    c_out = outer_xy[::-1] if up < 0 else outer_xy
    zo = Bo[::-1, 2] if up < 0 else Bo[:, 2]
    LAYERS["SideWalk__Curb"].wall(c_out, zo - 0.4, zo)
    # kerb returns at both strip ends (closes the kerb where a sidewalk stops)
    for k in (0, -1):
        E = np.array([A[k, :2], Bo[k, :2]]); zt = np.array([A[k, 2], Bo[k, 2]]); zb = np.full(2, zt.min() - CURB - 0.03)
        LAYERS["SideWalk__Curb"].wall(E, zb, zt)                       # both faces (end seen from either side)
        LAYERS["SideWalk__Curb"].wall(E[::-1], zb, zt[::-1])


def idw(tree, vals, P, k=6):
    d, i = tree.query(P, k=k)
    w = 1.0 / np.maximum(d, 0.05) ** 2
    return np.sum(vals[i] * w, 1) / np.sum(w, 1)


JUNCTION_REF = []   # 3D triangles of connecting-road lanes (exact OpenDRIVE surface)


class SurfaceRef:
    """Height = mean barycentric z of all reference triangles covering the point."""

    def __init__(self, tris, fallback):
        self.T = np.asarray(tris, np.float64)
        self.tree = shapely.STRtree(shapely.polygons(self.T[:, :, :2]))
        self.fallback = fallback

    def __call__(self, P):
        P = np.asarray(P, np.float64)
        z = self.fallback(P)
        pi, ti = self.tree.query(shapely.points(P), predicate="intersects")
        if len(pi) == 0:
            return z
        t = self.T[ti]
        x, y = P[pi, 0], P[pi, 1]
        (x1, y1, z1), (x2, y2, z2), (x3, y3, z3) = (t[:, 0].T, t[:, 1].T, t[:, 2].T)
        det = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3)
        ok = np.abs(det) > 1e-12
        det = np.where(ok, det, 1)
        l1 = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / det
        l2 = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / det
        zz = l1 * z1 + l2 * z2 + (1 - l1 - l2) * z3
        s = np.bincount(pi[ok], zz[ok], minlength=len(P)); c = np.bincount(pi[ok], minlength=len(P))
        return np.where(c > 0, s / np.maximum(c, 1), z)


def refined_triangulate(poly, zfun, step=2.5, seg=1.5):
    """Delaunay of boundary (seg m) + interior Steiner grid (step m), clipped to the polygon;
    uncovered slivers are filled with a constrained triangulation -> watertight & dense."""
    from scipy.spatial import Delaunay
    poly = shapely.segmentize(poly, seg)
    rings = [np.asarray(poly.exterior.coords)[:-1]] + [np.asarray(r.coords)[:-1] for r in poly.interiors]
    x0, y0, x1, y1 = poly.bounds
    gx, gy = np.meshgrid(np.arange(x0 + step / 2, x1, step), np.arange(y0 + step / 2, y1, step))
    G = np.column_stack([gx.ravel(), gy.ravel()])
    inner = poly.buffer(-0.8)
    G = G[shapely.contains_xy(inner, G[:, 0], G[:, 1])] if not inner.is_empty else G[:0]
    pts = np.unique(np.round(np.concatenate(rings + [G]), 4), axis=0)
    if len(pts) < 4 or len(G) == 0:
        return triangulate(poly, zfun, planar_uv, seg=seg)
    D = Delaunay(pts)
    F = D.simplices
    tp = shapely.polygons(pts[F])
    near = poly.buffer(0.01)
    shapely.prepare(near)
    keep = shapely.within(tp, near)
    F = F[keep]
    V = np.column_stack([pts, zfun(pts)])
    parts_V, parts_F, n = [V], [F], len(V)
    # Delaunay triangles do not overlap: their coverage union is far cheaper than a general union (the
    # difference snaps it to the same 0.1 mm grid); the general union when the coverage union is not valid
    try:
        covered = shapely.coverage_union_all(tp[keep])
        if not covered.is_valid:
            raise shapely.errors.GEOSException("invalid coverage union")
    except shapely.errors.GEOSException:      # sliver triangles: not a clean coverage
        covered = shapely.union_all(tp[keep], grid_size=1e-4)
    left = poly.difference(covered, grid_size=1e-4)
    for piece in shapely.get_parts(left):
        if piece.geom_type != "Polygon" or piece.area < 1e-4:
            continue
        res = triangulate(piece, zfun, planar_uv, seg=1e9)
        if res:
            parts_V.append(res[0]); parts_F.append(res[1] + n); n += len(res[0])
    V = np.concatenate(parts_V); F = np.concatenate(parts_F)
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
    F[cross < 0] = F[cross < 0][:, ::-1]
    return V, F, planar_uv(V)


def build_junctions(od, net_file, drive_polys, walk_polys, junction_drive, drive_pts, walk_pts):
    """Junction asphalt + sidewalk corner pads, processed per junction AT ITS OWN LEVEL: only strips
    and sidewalks within LEVEL_TOL of the junction height are subtracted / blended, so a junction
    under a bridge keeps its asphalt and a deck over a junction keeps its sidewalks."""
    dtree, wtree = cKDTree(drive_pts[:, :2]), cKDTree(walk_pts[:, :2])
    zwalk = lambda P: idw(wtree, walk_pts[:, 2], P, k=4)

    def zroad(P, zref=None):
        """Road height near P; with zref only surfaces on that level (multi-level aware)."""
        P = np.asarray(P, float)
        if zref is None:
            return idw(dtree, drive_pts[:, 2], P)
        d, i = dtree.query(P[:, :2], k=24)
        z = drive_pts[i, 2]
        ok = np.abs(z - np.broadcast_to(np.asarray(zref, float), (len(P),))[:, None]) < LEVEL_TOL
        w = np.where(ok, 1.0 / np.maximum(d, 0.05) ** 2, 0.0)
        s = w.sum(1)
        return np.where(s > 0, (w * z).sum(1) / np.maximum(s, 1e-12), z[:, 0])

    def diff(a, b):
        return polys_only(shapely.difference(polys_only(a), polys_only(b), grid_size=1e-3))

    # overlapping strips of different roads ON THE SAME LEVEL (clustered junctions in the source
    # network): keep the road with the lowest id, drop the other's faces lying inside it
    L = LAYERS["Road__Asphalt"]
    by_owner, p3_owner = defaultdict(list), defaultdict(list)
    for poly, own, p3 in zip(drive_polys, DRIVE_OWNER, DRIVE_P3):
        by_owner[own].append(poly); p3_owner[own].append(p3)
    own_ids = sorted(by_owner)
    own_poly = [unary_union(by_owner[o]).buffer(-0.05) for o in own_ids]
    own_p3 = [np.concatenate(p3_owner[o]) for o in own_ids]
    otree = shapely.STRtree(own_poly)
    dropped_ov, base = 0, 0
    for k in range(len(L.F)):
        c = L.V[k][L.F[k] - base].mean(1)
        base += len(L.V[k])
        o = L.O[k]
        if len(c) == 0 or o[0] < 0:
            continue
        pi, ti = otree.query(shapely.points(c[:, :2]), predicate="within")
        kill = np.zeros(len(c), bool)
        for p, t in zip(pi, ti):
            if own_ids[t] < o[p] and abs(z_near(own_p3[t], c[p, :2]) - c[p, 2]) < 1.0:
                kill[p] = True
        if kill.any():
            L.F[k], L.T[k], L.O[k] = L.F[k][~kill], L.T[k][~kill], L.O[k][~kill]
            dropped_ov += int(kill.sum())
    print(f"cleanup: dropped {dropped_ov} road-strip faces overlapping another road on the same level")

    dtr, wtr = shapely.STRtree(drive_polys), shapely.STRtree(walk_polys)
    walkareas = defaultdict(list)
    jlist = []   # (junction id, junction area, junction height or NaN)
    if net_file is not None:
        net = etree.parse(str(net_file)).getroot()
        # SUMO walking areas: the exact pedestrian corner polygons (rounded with --junctions.corner-detail),
        # which meet the sidewalk lane ends -> seamless corner pads with a continuous kerb
        for e in net.findall("edge"):
            if e.get("function") != "walkingarea":
                continue
            jid = e.get("id")[1:].rsplit("_w", 1)[0]
            for ln in e.findall("lane"):
                pts_ = [[float(v) for v in p.split(",")] for p in ln.get("shape", "").split()]
                if len(pts_) < 3:
                    continue
                pg = shapely.make_valid(Polygon([p[:2] for p in pts_])).buffer(0)
                if pg.is_empty or pg.area < 0.3:
                    continue
                walkareas[jid].append((pg, float(np.mean([p[2] for p in pts_ if len(p) > 2] or [np.nan]))))
        for j in net.findall("junction"):
            if j.get("type") == "internal" or not j.get("shape"):
                continue
            pts_ = [[float(v) for v in p.split(",")] for p in j.get("shape").split()]
            sh = np.array([p[:2] for p in pts_])
            if len(sh) < 3:
                continue
            J = polys_only(Polygon(sh))
            if J.area < 1.0:
                continue
            zJ = float(j.get("z")) if j.get("z") else float(np.mean([p[2] for p in pts_ if len(p) > 2] or [np.nan]))
            jlist.append((j.get("id"), J, zJ))
    else:
        # OdrForge: no SUMO net. A junction's area is the union of its connecting lanes, small gaps between them
        # closed (2 m); its corner sidewalks are connecting roads with sidewalk lanes, already among walk_polys.
        # Height: the mean of its connecting lanes' exact surface
        for jid, polys in junction_drive.items():
            J = polys_only(unary_union(polys).buffer(2.0, join_style=2).buffer(-2.0, join_style=2))
            if J.is_empty or J.area < 1.0:
                continue
            tri = JREF_BY.get(jid)
            zJ = float(np.mean([v[2] for t in tri for v in t])) if tri else float("nan")
            jlist.append((jid, J, zJ))
    patches, corners_all, d_walk = [], [], 0
    jdrive_u = unary_union([p for v in junction_drive.values() for p in v])
    for jid, J, zJ in jlist:
        cxy = np.array(J.representative_point().coords[0])
        Jq = J.buffer(3.0)
        di = [i for i in dtr.query(Jq) if np.isnan(zJ) or abs(z_near(DRIVE_P3[i], cxy) - zJ) < LEVEL_TOL]
        wi = [i for i in wtr.query(Jq) if np.isnan(zJ) or abs(z_near(WALK_P3[i], cxy) - zJ) < LEVEL_TOL]
        conn_list = junction_drive.get(jid, [])
        conn = unary_union(conn_list)
        lvl_strips = unary_union([drive_polys[i] for i in di])
        lvl_drive = unary_union([lvl_strips, conn])
        lvl_walk = unary_union([walk_polys[i] for i in wi])
        resid = J.difference(lvl_drive.buffer(0.02)).difference(lvl_walk.buffer(0.02))
        resid = resid.buffer(-0.15).buffer(0.15)
        fill, corners = [], []
        wa = [p for p, zz in walkareas.get(jid, []) if np.isnan(zJ) or np.isnan(zz) or abs(zz - zJ) < LEVEL_TOL]
        wa_u = Polygon()
        if wa:
            # corner pads = walking areas outside the driving lanes and the sidewalk strips, cleaned
            wa_u = diff(diff(unary_union(wa), lvl_drive.buffer(0.03)), lvl_walk.buffer(0.01))
            wa_u = polys_only(wa_u.buffer(-0.05).buffer(0.05).simplify(0.02))
            wa_u = unary_union([p for p in shapely.get_parts(wa_u) if p.area >= 0.3])
            corners.append(wa_u)
            resid = diff(resid, wa_u.buffer(0.02))
        for piece in shapely.get_parts(resid):
            if piece.area < 0.5:
                continue
            if wa:
                fill.append(piece)              # everything else in the junction shape is asphalt
            else:
                (corners if not lvl_walk.is_empty and piece.distance(lvl_walk) < 0.6 else fill).append(piece)
        raw = unary_union([conn] + fill).buffer(0.10)
        if raw.is_empty:
            continue
        inner = raw.buffer(-0.4)
        # sidewalk faces of short link roads poking into this junction (same level only)
        if not np.isnan(zJ):
            d_walk += LAYERS["SideWalk__Concrete"].drop_faces_inside(inner, zref=zJ + CURB, tol=LEVEL_TOL)
            d_walk += LAYERS["SideWalk__Curb"].drop_faces_inside(inner, zref=zJ, tol=LEVEL_TOL)
        patch = diff(diff(raw, lvl_strips), diff(lvl_walk, inner))
        if not wa_u.is_empty:
            patch = diff(patch, wa_u)
        patch = unary_union([p for p in shapely.get_parts(patch) if p.area > 0.05])
        corner_u = diff(diff(unary_union(corners), patch), lvl_strips)
        # heights: this junction's own connecting lanes; seams blended to adjacent same-level strips
        fb = (lambda P, z0=zJ: zroad(P, None if np.isnan(z0) else np.full(len(P), z0)))
        zj = SurfaceRef(JREF_BY.get(jid, []), fb) if JREF_BY.get(jid) else fb
        sp3 = np.concatenate([DRIVE_P3[i] for i in di]) if di else None
        stree = cKDTree(sp3[:, :2]) if sp3 is not None else None
        shapely.prepare(lvl_strips)

        def zpatch(P, zj=zj, sp3=sp3, stree=stree, strips=lvl_strips):
            P = np.asarray(P, float)
            base = zj(P)
            if stree is None or strips.is_empty:
                return base
            d = shapely.distance(strips, shapely.points(P[:, :2]))
            _, i = stree.query(P[:, :2])
            u = np.clip(d / 2.0, 0, 1); w = u * u * (3 - 2 * u)
            return sp3[i, 2] * (1 - w) + base * w
        for p in shapely.get_parts(patch):
            if p.area < 0.2:
                continue
            res = refined_triangulate(p, zpatch)
            if res:
                LAYERS["Road__Asphalt"].add(*res)
        if not patch.is_empty:
            patches.append(patch)
        zpad = lambda P, zp=zpatch: zp(P) + CURB
        edge = unary_union([lvl_drive, patch]).boundary
        for p in shapely.get_parts(corner_u):
            if p.area < 0.3 or p.geom_type != "Polygon":
                continue
            res = triangulate(p, zpad, planar_uv)
            if not res:
                continue
            LAYERS["SideWalk__Concrete"].add(*res)
            corners_all.append(p)
            ring = np.asarray(shapely.geometry.polygon.orient(p, -1.0).exterior.coords)
            ring = np.asarray(shapely.segmentize(LineString(ring), 0.5).coords)
            near = shapely.distance(shapely.points(ring), edge) < 0.4
            near = near | (_neighbours3(near) >= 2)     # bridge single-sample gaps in the kerb
            for run in _true_runs(near):
                P = ring[run[0]:run[1] + 1]
                zt = zpad(P)
                LAYERS["SideWalk__Curb"].wall(P, zt - CURB - 0.03, zt)
    print(f"cleanup: dropped {d_walk} sidewalk faces inside junction asphalt (same level)")
    road_patch = unary_union(patches)
    all_drive = unary_union(drive_polys + [jdrive_u])
    return all_drive, unary_union(walk_polys), road_patch, unary_union(corners_all), zroad, zwalk


# ----------------------------------------------------------------------------------- markings
def mark_strip(r, s, t, half_w, layer, dz=MARK_Z):
    A = r.xyz(s, t - half_w, dz); B = r.xyz(s, t + half_w, dz)
    v = s - s[0]
    LAYERS[layer].strip(A, B, np.column_stack([np.zeros_like(v), v]), np.column_stack([np.ones_like(v), v]))


def dash_intervals(a, b, phase0=0.0):
    out = []
    k = math.floor((a - phase0) / (DASH + GAP))
    while True:
        d0 = phase0 + k * (DASH + GAP); d1 = d0 + DASH
        if d0 >= b:
            break
        lo, hi = max(a, d0), min(b, d1)
        if hi - lo > 0.3:
            out.append((lo, hi))
        k += 1
    return out


def _runs(flag):
    out, start = [], None
    for i, f in enumerate(flag):
        if f and start is None:
            start = i
        if not f and start is not None:
            out.append((start, i - 1)); start = None
    if start is not None:
        out.append((start, len(flag) - 1))
    return out


def opposing_centre(od, max_gap=1.5):
    """For one-direction roads, flag stations where another road runs alongside in the
    opposite direction on the inner (left-of-traffic) border -> two-way street."""
    recs, pts, heads, owner = {}, [], [], []
    for r in od.roads:
        if r.junction != "-1":
            continue
        for sec in r.sections:
            rd = [l for l in sec.right if l.type in ROAD_TYPES]
            ld = [l for l in sec.left if l.type in ROAD_TYPES]
            if bool(rd) == bool(ld):
                continue
            s = np.arange(sec.s, sec.e, 1.0)
            s = np.append(s, sec.e) if len(s) == 0 or sec.e - s[-1] > 0.2 else s
            off = r.lane_offset(s)
            P = r.xyz(s, off)[:, :2]
            h = r.ref(s)[2] + (0 if rd else math.pi)
            recs[(r.id, sec.s)] = (s, len(pts), P, h)
            pts.extend(P); heads.extend(h); owner.extend([r.id] * len(s))
    pts, heads, owner = np.array(pts), np.array(heads), np.array(owner)
    tree = cKDTree(pts)
    out = {}
    for key, (s, i0, P, h) in recs.items():
        flag = np.zeros(len(s), bool)
        for k, nb in enumerate(tree.query_ball_point(P, max_gap)):
            nb = [j for j in nb if owner[j] != key[0]]
            flag[k] = any(math.cos(heads[j] - h[k]) < -0.8 for j in nb)
        out[key] = (s, flag)
    return out


def build_markings(od):
    centre = opposing_centre(od)
    tf_ = C.BUILD / "road_types.json"
    types = json.loads(tf_.read_text()) if tf_.exists() else {}
    for r in od.roads:
        if r.junction != "-1":
            continue
        for sec in r.sections:
            ldrive = [l for l in sec.left if l.type in ROAD_TYPES]
            rdrive = [l for l in sec.right if l.type in ROAD_TYPES]
            # centre line: US double yellow on two-way roads
            if ldrive and rdrive:
                s = r.sample_s(sec.s, sec.e)
                off = r.lane_offset(s)
                for sgn in (-1, 1):
                    mark_strip(r, s, off + sgn * 0.10, 0.05, "RoadLine__Yellow")
            elif (ldrive or rdrive) and (r.id, sec.s) in centre:
                # SUMO exports each direction as its own road: each draws its half of the
                # double yellow on its inner border where an opposing road runs alongside
                s, flag = centre[(r.id, sec.s)]
                sgn = -1 if rdrive else 1
                off = r.lane_offset(s)
                for lo, hi in _runs(flag):
                    if s[hi] - s[lo] < 1.0:
                        continue
                    sr = r.sample_s(s[lo], s[hi])
                    mark_strip(r, sr, r.lane_offset(sr) + sgn * 0.10, 0.05, "RoadLine__Yellow")
            # freeways / ramps: solid white right edge line, solid yellow left edge line (MUTCD)
            if types.get(r.id) in FREEWAY:
                s_e = r.sample_s(sec.s, sec.e, max_step=3.0)
                if len(s_e) >= 2:
                    Be, offe = r.borders(sec, s_e)
                    for side in (rdrive, ldrive):
                        if not side:
                            continue
                        sg = -1 if side is rdrive else 1
                        # left edge of travel = inner border of the innermost lane (unless opposing traffic)
                        if not (ldrive and rdrive):
                            mark_strip(r, s_e, Be[side[0].id][0] + sg * 0.25, MARK_W / 2 + 0.01, "RoadLine__Yellow")
                        mark_strip(r, s_e, Be[side[-1].id][1] - sg * 0.25, MARK_W / 2 + 0.01, "RoadLine__White")
            # lane dividers (outer border of each lane, if next lane outward is also a road lane)
            for side in (sec.left, sec.right):
                for i, lane in enumerate(side[:-1]):
                    nxt = side[i + 1]
                    if lane.type not in ROAD_TYPES or nxt.type not in ROAD_TYPES:
                        continue
                    marks = lane.marks or [(0.0, {"type": "broken"})]
                    for k, (o, m) in enumerate(marks):
                        a = sec.s + o
                        b = sec.s + marks[k + 1][0] if k + 1 < len(marks) else sec.e
                        typ = m.get("type", "broken")
                        if typ == "none" or b - a < 0.3:
                            continue
                        if lane.type != nxt.type:
                            typ = "solid"    # bike / bus lane separation
                        spans = dash_intervals(a, b) if "broken" in typ.split()[0] else [(a, b)]
                        for lo, hi in spans:
                            s = r.sample_s(lo, hi, max_step=2.0)
                            if len(s) < 2:
                                continue
                            B, _ = r.borders(sec, s)
                            mark_strip(r, s, B[lane.id][1], MARK_W / 2, "RoadLine__White")
            # stop bars where signals control this road end
            if r.signals is not None and len(r.signals):
                orients = {sg.get("orientation") for sg in r.signals.findall("signal")}
                for orient, lanes, (lo, hi) in (("+", rdrive, (sec.e - 1.0, sec.e - 0.55)),
                                                ("-", ldrive, (sec.s + 0.55, sec.s + 1.0))):
                    if orient not in orients or not lanes:
                        continue
                    if (orient == "+" and sec is not r.sections[-1]) or (orient == "-" and sec is not r.sections[0]):
                        continue
                    step = -0.5 if orient == "+" else 0.5     # move away from the junction
                    for _ in range(16):                        # keep 30 cm clear of any crosswalk
                        s = np.array([lo, hi])
                        B, _ = r.borders(sec, s)
                        t0 = B[lanes[0].id][0]; t1 = B[lanes[-1].id][1]
                        A = np.stack([r.xyz(np.full(8, s[k]), np.linspace(t0[k], t1[k], 8), MARK_Z) for k in range(2)])
                        bar = Polygon(np.concatenate([A[0, :, :2], A[1, ::-1, :2]])).buffer(0.3)
                        if not any(bar.intersects(q) for q in CROSS_BANDS):
                            break
                        lo, hi = lo + step, hi + step
                        if not (sec.s < lo < sec.e and sec.s < hi < sec.e):
                            break
                    else:
                        continue
                    if sec.s < lo < sec.e and sec.s < hi < sec.e:
                        LAYERS["RoadLine__White"].strip(A[0], A[1], np.zeros((8, 2)), np.ones((8, 2)))


def build_crosswalks(net_file, zroad):
    """Continental (ladder) crosswalks from SUMO crossing lanes (exact centreline + width).

    The OpenDRIVE crosswalk objects written by netconvert are anchored on the road's
    reference line, which is offset from the actual crossing, so the net is used instead.
    """
    n = 0
    placed = CROSS_BANDS
    placed.clear()
    net = etree.parse(str(net_file)).getroot()
    for e in net.iter("edge"):
        if e.get("function") != "crossing":
            continue
        ln = e.find("lane")
        P3c = np.array([[float(v) for v in p.split(",")] for p in ln.get("shape").split()])
        P = P3c[:, :2]
        zc_cross = float(P3c[:, 2].mean()) if P3c.shape[1] > 2 else None      # crossing level
        width = float(ln.get("width", 4.0))
        band = LineString(P).buffer(width / 2, cap_style="flat")
        if any(band.intersection(q).area > 0.10 * min(band.area, q.area) for q in placed):
            continue          # clustered junctions can produce overlapping crossings
        placed.append(band)
        for a, b in zip(P[:-1], P[1:]):
            d = b - a
            L = float(np.hypot(*d))
            if L < 2:
                continue
            ex = d / L                       # walking direction
            ey = np.array([-ex[1], ex[0]])   # traffic direction
            bar, gap, margin = 0.6, 0.6, 0.4
            for vb in np.arange(margin, L - margin - bar + 1e-6, bar + gap):
                vs = np.linspace(-width / 2 + 0.3, width / 2 - 0.3, 4)
                A = a + vb * ex + np.outer(vs, ey)
                B = a + (vb + bar) * ex + np.outer(vs, ey)
                zr_ = None if zc_cross is None else np.full(len(A), zc_cross)
                za, zb = zroad(A, zr_) + MARK_Z + 0.003, zroad(B, zr_) + MARK_Z + 0.003
                LAYERS["RoadLine__White"].strip(np.column_stack([A, za]), np.column_stack([B, zb]),
                                               np.column_stack([np.zeros(4), vs]), np.column_stack([np.ones(4), vs]))
            n += 1
    return n

def build_crosswalks_xodr(od, zroad):
    """Continental (ladder) crosswalks from the OpenDRIVE crosswalk objects (OdrForge: placed at the mapped crossing,
    outline across the carriageway): the outline's v extent is the walking line, its u extent the band width."""
    n = 0
    placed = CROSS_BANDS
    placed.clear()
    for r in od.roads:
        for ob in r.el.findall("objects/object"):
            if ob.get("type") != "crosswalk":
                continue
            corners = [(float(c.get("u")), float(c.get("v"))) for c in ob.findall("outline/cornerLocal")]
            if len(corners) < 3:
                continue
            s, t, hdg = float(ob.get("s")), float(ob.get("t")), float(ob.get("hdg", 0))
            x, y, h = (float(np.ravel(a)[0]) for a in r.ref(np.array([s])))
            o = r.xyz(np.array([s]), np.array([t]))[0]
            th = h + hdg
            ud, vd = np.array([np.cos(th), np.sin(th)]), np.array([-np.sin(th), np.cos(th)])
            us, vs_ = [c[0] for c in corners], [c[1] for c in corners]
            uc = 0.5 * (min(us) + max(us))
            P = np.array([o[:2] + uc * ud + min(vs_) * vd, o[:2] + uc * ud + max(vs_) * vd])
            width = max(us) - min(us)
            zc_cross = float(o[2])
            band = LineString(P).buffer(width / 2, cap_style="flat")
            if any(band.intersection(q).area > 0.10 * min(band.area, q.area) for q in placed):
                continue
            placed.append(band)
            a, b = P
            d = b - a
            L = float(np.hypot(*d))
            if L < 2:
                continue
            ex = d / L
            ey = np.array([-ex[1], ex[0]])
            bar, gap, margin = 0.6, 0.6, 0.4
            for vb in np.arange(margin, L - margin - bar + 1e-6, bar + gap):
                vv = np.linspace(-width / 2 + 0.3, width / 2 - 0.3, 4)
                A = a + vb * ex + np.outer(vv, ey)
                B = a + (vb + bar) * ex + np.outer(vv, ey)
                zr_ = np.full(len(A), zc_cross)
                za, zb = zroad(A, zr_) + MARK_Z + 0.003, zroad(B, zr_) + MARK_Z + 0.003
                LAYERS["RoadLine__White"].strip(np.column_stack([A, za]), np.column_stack([B, zb]),
                                               np.column_stack([np.zeros(4), vv]), np.column_stack([np.ones(4), vv]))
            n += 1
    return n


# ----------------------------------------------------------------------------------- terrain
LANE_UNDER = 0.35      # m the terrain is kept below a vehicle lane it would otherwise poke through
LANE_PUSH = 2.0        # m: terrain up to this far above a lane is pushed under it; higher (up to the headroom) is cut
LANE_HEADROOM = 4.5    # m kept free above a vehicle lane (tunnel / underpass ceiling height)
LANE_TERRAIN = {}      # terrain vertices pushed under / cut over vehicle lanes (stats)


def build_terrain(dem, extent, paved, paved_pts, tunnel_fp=None, roof_z=None, lanes=None):
    """Terrain conformed under ground-level pavement only: bridges keep the ground under them,
    tunnels keep the ground above them (and the ground is carved where it would cut into a
    tunnel, near its portals). With lanes (lane_pieces): no terrain inside the space a car occupies
    on any vehicle lane of any level - pushed under the lane, or cut out where it is higher up."""
    x0, y0, x1, y1 = extent
    xs = np.arange(x0, x1 + 1e-6, TERRAIN_STEP); ys = np.arange(y0, y1 + 1e-6, TERRAIN_STEP)
    GX, GY = np.meshgrid(xs, ys)
    P = np.column_stack([GX.ravel(), GY.ravel()])
    Z = dem(P[:, 0], P[:, 1])
    # only ground-level pavement shapes the terrain
    pz_pts = dem(paved_pts[:, 0], paved_pts[:, 1])
    gp = paved_pts[np.abs(paved_pts[:, 2] - pz_pts) < 2.0]
    tree = cKDTree(gp[:, :2])
    d, i = tree.query(P, k=4)
    znear = gp[i[:, 0], 2]
    zmin = gp[i, 2].min(1)
    inside = shapely.contains_xy(paved, P[:, 0], P[:, 1])
    if inside.any():
        pz = PAVED_Z(P[inside])
        lvl_ok = ~np.isnan(pz) & (np.abs(pz - Z[inside]) < 2.0)
        tmp = np.zeros(len(P), bool); tmp[np.flatnonzero(inside)[lvl_ok]] = True
        inside = tmp
    w = np.clip((d[:, 0] - 0.5) / 8.0, 0, 1) ** 1.5
    Zc = np.where(inside, zmin - 0.35, (znear - 0.03) * (1 - w) + Z * w)
    # far from ground-level roads keep the DEM untouched
    Zc = np.where((d[:, 0] > 9.0) & ~inside, Z, Zc)
    # inside paved areas never let the terrain come closer than 30 cm below the surface
    if inside.any():
        pz = PAVED_Z(P[inside])
        Zc[inside] = np.where(np.isnan(pz), Zc[inside], np.minimum(Zc[inside], pz - 0.30))
    cut_v = np.zeros(len(P), bool)
    if lanes is not None:
        ltree, _, lz = lanes
        vi, pk = ltree.query(shapely.points(P), predicate="dwithin", distance=0.5)
        dz = Zc[vi] - lz[pk]
        push = (dz > -LANE_UNDER) & (dz < LANE_PUSH)
        np.minimum.at(Zc, vi[push], lz[pk[push]] - LANE_UNDER)
        cut_v[vi[(dz >= LANE_PUSH) & (dz < LANE_HEADROOM)]] = True
        # the next grid row (up to one TERRAIN_STEP out) spans the triangles that slope over the lane edge: push
        # it under as well (no cut there - that would open holes beside the road)
        vo, po = ltree.query(shapely.points(P), predicate="dwithin", distance=TERRAIN_STEP)
        dzo = Zc[vo] - lz[po]
        own = np.zeros(len(P), bool)              # ground next to a lane of its own level (a street above a trench)
        own[vo[np.abs(dzo) < 0.5]] = True
        band = (dzo > -LANE_UNDER) & (dzo < LANE_HEADROOM) & ~cut_v[vo] & ~own[vo]
        np.minimum.at(Zc, vo[band], lz[po[band]] - LANE_UNDER)
        LANE_TERRAIN.update(pushed=int(len(np.unique(np.concatenate([vi[push], vo[band]])))), cut=int(cut_v.sum()))
    nx, ny = len(xs), len(ys)
    idx = np.arange(nx * ny).reshape(ny, nx)
    a, b, c, dd = idx[:-1, :-1].ravel(), idx[:-1, 1:].ravel(), idx[1:, 1:].ravel(), idx[1:, :-1].ravel()
    F = np.concatenate([np.stack([a, b, c], 1), np.stack([a, c, dd], 1)])
    # drop faces deep inside ground-level paved areas (never visible, saves ~40% triangles)
    cen = P[F].mean(1)
    deep = shapely.contains_xy(paved.buffer(-1.5), cen[:, 0], cen[:, 1]) & inside[F].all(1)
    # carve the ground where it would cut into a tunnel (near portals)
    if tunnel_fp is not None and not tunnel_fp.is_empty:
        intun = shapely.contains_xy(tunnel_fp.buffer(0.5), cen[:, 0], cen[:, 1])
        if intun.any():
            rz = roof_z(cen[intun])
            zc_ = Zc[F[intun]].max(1)
            carve = np.zeros(len(F), bool)
            carve[np.flatnonzero(intun)[~np.isnan(rz) & (zc_ < rz + 0.3)]] = True
            deep |= carve
    deep |= cut_v[F].any(1)
    if lanes is not None:
        # faces over a lane that still reach into the space above it: under a bridge the grid alternates between
        # the two levels (vertices conformed to the lower road next to ones under the upper deck) and the faces
        # between them slope through the underpass - drop them (the deck covers the gap)
        ltree, _, lz = lanes
        Vz = Zc[F]
        fi, pk = ltree.query(shapely.points(P[F].mean(1)), predicate="dwithin", distance=0.5)
        over = (Vz[fi].max(1) > lz[pk] + 0.15) & (Vz[fi].min(1) < lz[pk] + LANE_HEADROOM)
        drop = np.zeros(len(F), bool); drop[fi[over]] = True
        LANE_TERRAIN["faces_dropped"] = int((drop & ~deep).sum())
        deep |= drop
    F = F[~deep]
    V = np.column_stack([P, Zc])
    UV = np.column_stack([(P[:, 0] - x0) / (x1 - x0), (P[:, 1] - y0) / (y1 - y0)])
    LAYERS["Terrain__Ortho"].add(V, F, UV)
    return F.shape[0]


def build_ortho(extent):
    x0, y0, x1, y1 = extent
    with rasterio.open(C.ORTHO_UTM) as src:
        dst = np.zeros((3, ORTHO_PX, ORTHO_PX), np.uint8)
        reproject(src.read(), dst, src_transform=src.transform, src_crs=src.crs,
                  dst_transform=from_bounds(x0, y0, x1, y1, ORTHO_PX, ORTHO_PX),
                  dst_crs=rasterio.crs.CRS.from_proj4(C.MAP_PROJ), resampling=Resampling.lanczos)
    Image.fromarray(np.moveaxis(dst, 0, 2)).save(C.TEX / "T_LA_Ortho_D.jpg", quality=92)


# ----------------------------------------------------------------------------------- buildings
def parse_len(v):
    if not v:
        return None
    v = v.strip().lower().replace(",", ".")
    m = re.match(r"^([0-9.]+)\s*(m|ft|')?", v)
    if not m:
        return None
    x = float(m.group(1))
    return x * 0.3048 if m.group(2) in ("ft", "'") else x


def building_key(poly):
    """Stable id of a raw OSM building polygon (shared by 02c_building_tiers and build_buildings)."""
    c = poly.centroid
    return f"{c.x:.1f},{c.y:.1f},{abs(poly.area):.0f}"


def is_underground(tags):
    """Buildings mapped below ground (garages / concourses under plazas) are not extruded."""
    lay = tags.get("layer", "0").strip()
    return (tags.get("location") in ("underground", "subsurface") or tags.get("building") == "underground"
            or (lay.lstrip("-").isdigit() and int(lay) < 0 and tags.get("location") != "overground"))


def load_building_tiers():
    f = C.BUILD / "building_tiers.json"
    return json.loads(f.read_text()) if f.exists() else {}


def osm_polygons():
    root = etree.parse(str(C.OSM_FILE)).getroot()
    nodes = {}
    ids, lon, lat = [], [], []
    for n in root.iter("node"):
        ids.append(n.get("id")); lon.append(float(n.get("lon"))); lat.append(float(n.get("lat")))
    x, y = TO_MAP.transform(np.array(lon), np.array(lat))
    nodes = dict(zip(ids, zip(x, y)))
    ways = {}
    out = []
    for w in root.iter("way"):
        refs = [nd.get("ref") for nd in w.findall("nd")]
        tags = {t.get("k"): t.get("v") for t in w.findall("tag")}
        coords = [nodes[r] for r in refs if r in nodes]
        ways[w.get("id")] = (coords, tags)
        if ("building" in tags or "building:part" in tags) and len(coords) >= 4 and refs[0] == refs[-1]:
            out.append((Polygon(coords), tags))
    for rel in root.iter("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "multipolygon" or not ("building" in tags or "building:part" in tags):
            continue
        outer, inner = [], []
        for m in rel.findall("member"):
            if m.get("type") == "way" and m.get("ref") in ways and len(ways[m.get("ref")][0]) >= 2:
                (inner if m.get("role") == "inner" else outer).append(LineString(ways[m.get("ref")][0]))
        polys_o = list(polygonize(linemerge(outer))) if outer else []
        polys_i = list(polygonize(linemerge(inner))) if inner else []
        for po in polys_o:
            holes = [pi for pi in polys_i if po.contains(pi.representative_point())]
            g = po.difference(unary_union(holes)) if holes else po
            for part in shapely.get_parts(g):
                out.append((part, tags))
    return out


def facade_style(tags, h):
    mat = (tags.get("building:material", "") + tags.get("building:facade:material", "")).lower()
    # deterministic per-building variety (python's hash() is salted per process)
    k = int.from_bytes(hashlib.md5((tags.get("name", "") + tags.get("addr:housenumber", "")
                                    + tags.get("addr:street", "") + f"{h:.1f}").encode()).digest()[:4], "little")
    if "glass" in mat or h >= 80:              # modern high-rise towers
        return ("Glass", "GlassDark")[k % 2]
    if "brick" in mat:
        return "Brick"
    if h >= 40:                                # DTLA mid-rise: mix of 1960s glass and older stone
        return ("Glass", "GlassDark", "Office", "OfficeTan")[k % 4]
    if h >= 14:                                # historic core: masonry / terracotta office blocks
        return ("Office", "OfficeTan", "Brick")[k % 3]
    return ("Brick", "Stucco", "Stucco")[k % 3]


def tile_of(x, y, extent):
    nx = int(math.ceil((extent[2] - extent[0]) / TILE))
    return int((y - extent[1]) // TILE) * nx + int((x - extent[0]) // TILE)


def build_tiered(g, tiers, base, ground, tags, tile):
    """Stacked massing from lidar roof tiers (02c): footprint g up to tier 0, each nested tier region
    from the tier below up to its own roof; every tier's exposed roof is a flat cap."""
    zs = [t["z"] for t in tiers["tiers"]]
    regs = [g]
    for t in tiers["tiers"][1:]:
        r = shapely.make_valid(shapely.from_wkt(t["wkt"])).intersection(regs[-1]).buffer(-0.05).buffer(0.05)
        r = polys_only(r)
        r = shapely.union_all([p for p in shapely.get_parts(r) if p.area >= 20.0]) if not r.is_empty else r
        if r.is_empty:
            break
        regs.append(r)
    zs = zs[:len(regs)]
    if zs[0] - base < 0.5:
        return False
    style = facade_style(tags, zs[-1] - ground)
    L = LAYERS[f"Building__{style}"]
    for k, (reg, z) in enumerate(zip(regs, zs)):
        zb = base if k == 0 else zs[k - 1]
        for p in shapely.get_parts(reg):
            p = shapely.geometry.polygon.orient(p, 1.0)
            for ring in [p.exterior] + list(p.interiors):
                R = np.asarray(ring.coords)
                L.wall(R[:, :2], np.full(len(R), zb), np.full(len(R), z))
                L.T[-1][:] = tile
        own = reg.difference(regs[k + 1]) if k + 1 < len(regs) else reg
        for p in shapely.get_parts(polys_only(own)):
            if p.area < 0.5:
                continue
            res = triangulate(shapely.geometry.polygon.orient(p, 1.0), lambda P, z=z: np.full(len(P), z), lambda P: P, seg=1e9)
            if res:
                LAYERS["Building__Roof"].add(*res, tile=tile)
    return True


def lane_passage(g, base, top, lanes):
    """g minus every vehicle lane that runs through the building's height range (a road into a garage, a
    tunnel portal under a hillside building): the passage plus 0.3 m, with LANE_HEADROOM above the lane."""
    if lanes is None:
        return g, 0
    ltree, lpolys, lz = lanes
    k = ltree.query(g, predicate="intersects")
    k = k[(lz[k] + LANE_HEADROOM > base) & (lz[k] < top)]
    if not len(k):
        return g, 0
    return g.difference(unary_union(list(lpolys[k])).buffer(0.3)), len(k)


def build_buildings(dem, extent, paved, lanes=None):
    box = shapely.box(*extent)
    polys = osm_polygons()
    parts = [(p, t) for p, t in polys if "building:part" in t and t.get("building:part") != "no"]
    part_tree = shapely.STRtree([p for p, _ in parts]) if parts else None
    stats = defaultdict(int)
    tiers_all = load_building_tiers()
    # the paved area grown by 5 cm, once (buffering the whole map's paving per building took ~40 min)
    paved_off = paved.buffer(0.05)
    shapely.prepare(paved)
    for raw, tags in polys:
        if is_underground(tags):
            stats["underground_skipped"] += 1
            continue
        tiers = tiers_all.get(building_key(raw))
        poly = shapely.make_valid(raw).buffer(0)
        if poly.is_empty or not box.contains(poly.centroid) or poly.area < 4:
            continue
        is_part = "building:part" in tags and tags.get("building:part") != "no"
        if not is_part and part_tree is not None:
            hits = part_tree.query(poly, predicate="contains_properly")
            if len(hits):
                stats["outline_replaced_by_parts"] += 1
                continue
        if tags.get("building") in ("roof", "carport") and not is_part:
            pass
        lv = tags.get("building:levels")
        h = parse_len(tags.get("height")) or (float(lv) * 3.6 + 1.0 if lv and re.match(r"^[0-9.]+$", lv) else None)
        if h is None:
            h = 9.0; stats["default_height"] += 1
        minh = parse_len(tags.get("min_height")) or 0.0
        if not minh and tags.get("building:min_level", "").isdigit():
            minh = float(tags["building:min_level"]) * 3.6
        if tags.get("building") in ("roof", "carport", "canopy") and not minh:
            minh = max(0.0, h - 1.0)
        # keep buildings off the street (OSM footprints sometimes overlap the curb)
        poly = poly.difference(paved_off) if poly.intersects(paved) else poly
        pieces = []
        for g in shapely.get_parts(poly):
            if g.geom_type != "Polygon" or g.area < 4:
                continue
            ext = np.asarray(g.exterior.coords)
            ground = float(np.min(dem(ext[:, 0], ext[:, 1])))
            base = ground + minh - (0.0 if minh else 0.5)
            top = ground + h
            g2, nk = lane_passage(g, base, top, lanes)
            if nk:
                stats["lane_passages"] += 1
            pieces += [(p, ground, base, top) for p in shapely.get_parts(g2)]
        for g, ground, base, top in pieces:
            if g.geom_type != "Polygon" or g.area < 4:
                continue
            g = shapely.geometry.polygon.orient(g.simplify(0.15), 1.0)
            tile = tile_of(*g.centroid.coords[0], extent)
            if tiers and not minh:
                if build_tiered(g, tiers, base, ground, tags, tile):
                    stats["lidar_tiered" if len(tiers["tiers"]) > 1 else "lidar_height"] += 1
                    continue
            if top - base < 0.5:
                continue
            style = facade_style(tags, h)
            L = LAYERS[f"Building__{style}"]
            for ring in [g.exterior] + list(g.interiors):
                R = np.asarray(ring.coords)
                L_before = L.n
                L.wall(R[:, :2], np.full(len(R), base), np.full(len(R), top))
                L.T[-1][:] = tile
            res = triangulate(g, lambda P: np.full(len(P), top), lambda P: P, seg=1e9)
            if res:
                LAYERS["Building__Roof"].add(*res, tile=tile)
                if minh > 0:   # underside of floating parts / canopies
                    V, F, UV = res
                    LAYERS["Building__Roof"].add(np.column_stack([V[:, :2], np.full(len(V), base)]), F[:, ::-1], UV, tile=tile)
            stats[style] += 1
    return dict(stats)


# ----------------------------------------------------------------------------------- mesh repair
def weld(V, F, UV, T, tol=1.5e-3, ztol=0.03):
    """Merge vertices closer than tol in plan view and ztol in height (adjacent surface pieces
    meet with mm-level height differences), averaging their height; drop collapsed / zero-area /
    duplicate faces."""
    pairs = cKDTree(V[:, :2]).query_pairs(tol, output_type="ndarray")
    if len(pairs):
        pairs = pairs[np.abs(V[pairs[:, 0], 2] - V[pairs[:, 1], 2]) < ztol]
    n = len(V)
    parent = list(range(n))                        # union-find (scipy.csgraph is blocked here)

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for a, b in pairs.tolist():
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[max(ra, rb)] = min(ra, rb)
    roots = np.array([find(i) for i in range(n)])
    uniq, lab = np.unique(roots, return_inverse=True)
    lab = lab.ravel()
    first = uniq                                   # root = lowest index in its cluster
    V2, UV2 = V[first].copy(), UV[first]
    V2[:, 2] = np.bincount(lab, V[:, 2]) / np.bincount(lab)      # merged height = mean
    F2 = lab[F]
    ok = (F2[:, 0] != F2[:, 1]) & (F2[:, 1] != F2[:, 2]) & (F2[:, 0] != F2[:, 2])
    F2, T2 = F2[ok], T[ok]
    e1 = V2[F2[:, 1]] - V2[F2[:, 0]]; e2 = V2[F2[:, 2]] - V2[F2[:, 0]]
    area = np.linalg.norm(np.cross(e1, e2), axis=1) / 2
    ok = area > 1e-6
    F2, T2 = F2[ok], T2[ok]
    _, keep = np.unique(np.sort(F2, 1), axis=0, return_index=True)
    keep = np.sort(keep)
    return V2, F2[keep], UV2, T2[keep]


def fix_t_junctions(V, F, T, tol=5e-3, maxdz=0.03, iters=8):
    """Split boundary edges at vertices of neighbouring pieces lying on them (T-junctions),
    so adjacent pieces share vertices -> no hairline cracks / sparkles between them."""
    total = 0
    for _ in range(iters):
        E = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
        fid = np.tile(np.arange(len(F)), 3); slot = np.repeat([0, 1, 2], len(F))
        key = np.sort(E, 1)
        _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
        bnd = cnt[inv.ravel()] == 1
        bE, bf, bs = E[bnd], fid[bnd], slot[bnd]
        bv = np.unique(bE)
        tree = cKDTree(V[bv, :2])
        # every boundary edge against the boundary vertices near it, all edges at once (a loop over ~100k edges
        # with one ball query each took ~30 s): v on edge a-b strictly inside, within tol of it, within maxdz of
        # the edge's height there
        pa, pb = V[bE[:, 0]], V[bE[:, 1]]
        d2 = pb[:, :2] - pa[:, :2]
        L2 = np.einsum("ij,ij->i", d2, d2)
        live = np.nonzero(L2 >= 1e-8)[0]
        hits = tree.query_ball_point((pa[live, :2] + pb[live, :2]) / 2, np.sqrt(L2[live]) / 2 + tol)
        cnt_e = np.fromiter(map(len, hits), int, len(hits))
        ei = np.repeat(live, cnt_e)                                   # edge of each candidate, edge order
        v = bv[np.fromiter(itertools.chain.from_iterable(hits), int, int(cnt_e.sum()))]
        a, b = bE[ei, 0], bE[ei, 1]
        w = V[v, :2] - pa[ei, :2]
        t = np.einsum("ij,ij->i", w, d2[ei]) / L2[ei]
        r = w - t[:, None] * d2[ei]
        ok = (v != a) & (v != b) & (t > 1e-3) & (t < 1 - 1e-3) & (np.sqrt(np.einsum("ij,ij->i", r, r)) <= tol)
        ok &= np.abs(V[v, 2] - (pa[ei, 2] + t * (pb[ei, 2] - pa[ei, 2]))) <= maxdz
        ins = defaultdict(list)
        for f, k, tt, vv in zip(bf[ei[ok]].tolist(), bs[ei[ok]].tolist(), t[ok].tolist(), v[ok].tolist()):
            ins[f].append((k, tt, vv))
        if not ins:
            break
        drop, newF, newT = [], [], []
        for f, lst in ins.items():
            k = max({x[0] for x in lst}, key=lambda kk: sum(1 for x in lst if x[0] == kk))
            pts = sorted({(t, v) for kk, t, v in lst if kk == k})
            a, b, opp = F[f, k], F[f, (k + 1) % 3], F[f, (k + 2) % 3]
            chain = [a] + [v for _, v in pts] + [b]
            for i in range(len(chain) - 1):
                newF.append((chain[i], chain[i + 1], opp)); newT.append(T[f])
            drop.append(f); total += len(pts)
        keep = np.ones(len(F), bool); keep[drop] = False
        F = np.concatenate([F[keep], np.array(newF, F.dtype)])
        T = np.concatenate([T[keep], np.array(newT, T.dtype)])
    return F, T, total


class MeshZ:
    """Height of a triangle mesh below/above points (barycentric); NaN where no triangle."""

    def __init__(self, V, F):
        self.V, self.F = V, F
        self.tree = shapely.STRtree(shapely.polygons(V[F][:, :, :2]))

    def __call__(self, P, how="max", zref=None, tol=np.inf):
        """how: 'max' | 'min' | 'near' (surface closest to zref, within tol: multi-level aware)."""
        P = np.asarray(P, float)
        out = np.full(len(P), np.nan)
        pi, ti = self.tree.query(shapely.points(P[:, :2]), predicate="intersects")
        if len(pi) == 0:
            return out
        t = self.V[self.F[ti]]
        (x1, y1, z1), (x2, y2, z2), (x3, y3, z3) = (t[:, 0].T, t[:, 1].T, t[:, 2].T)
        det = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3)
        okd = np.abs(det) > 1e-12
        det = np.where(okd, det, 1)
        l1 = ((y2 - y3) * (P[pi, 0] - x3) + (x3 - x2) * (P[pi, 1] - y3)) / det
        l2 = ((y3 - y1) * (P[pi, 0] - x3) + (x1 - x3) * (P[pi, 1] - y3)) / det
        z = l1 * z1 + l2 * z2 + (1 - l1 - l2) * z3
        pi, z = pi[okd], z[okd]
        if how == "near":
            zr = np.broadcast_to(np.asarray(zref, float), (len(P),))[pi]
            dz = np.abs(z - zr)
            keep = dz <= tol
            pi, z, dz = pi[keep], z[keep], dz[keep]
            best = np.full(len(P), np.inf)
            np.minimum.at(best, pi, dz)
            sel = dz <= best[pi] + 1e-9
            out[pi[sel]] = z[sel]
            return out
        if how == "max":
            acc = np.full(len(P), -np.inf); np.maximum.at(acc, pi, z)
        else:
            acc = np.full(len(P), np.inf); np.minimum.at(acc, pi, z)
        return np.where(np.isfinite(acc), acc, np.nan)


def finalize_surfaces():
    """Hole filling, weld + T-junction repair of asphalt and sidewalk tops, upward orientation;
    paint projected onto the final asphalt."""
    stats = {}
    # 1. fill small enclosed holes in the asphalt (gaps between strips/patches) that no sidewalk covers
    L = LAYERS["Road__Asphalt"]
    V, F, UV, T = L.arrays()
    tri = shapely.polygons(V[F][:, :, :2])
    road_u = robust_union(tri[shapely.area(tri) > 1e-7], grid=1e-3)   # as the sidewalk below (GEOS can fail on one pass)
    WV, WF, _, _ = LAYERS["SideWalk__Concrete"].arrays()
    walk_u = robust_union(shapely.polygons(WV[WF][:, :, :2]), grid=1e-3)
    vt = cKDTree(V[:, :2])
    zfill = lambda P: idw(vt, V[:, 2], P, k=6)
    filled = 0
    tri_tree = shapely.STRtree(tri)
    for part in shapely.get_parts(road_u):
        for ring in part.interiors:
            h = Polygon(ring)
            if h.area > 60:
                continue
            # the grid-snapped union can report phantom holes: re-test against an exact local union
            near = tri[tri_tree.query(h.buffer(0.05), predicate="intersects")]
            near = near[shapely.area(near) > 1e-9]
            try:
                covered = shapely.union_all(near) if len(near) else Polygon()
                h = polys_only(h.difference(covered))
            except shapely.errors.GEOSException:
                continue
            for piece in shapely.get_parts(polys_only(h.difference(walk_u))):
                if piece.area < 0.002:
                    continue
                res = triangulate(piece, zfill, planar_uv, seg=1e9)
                if res:
                    L.add(*res); filled += 1
    stats["holes_filled"] = filled
    for name in ("Road__Asphalt", "SideWalk__Concrete"):
        L = LAYERS[name]
        V, F, UV, T = L.arrays()
        n0 = len(F)
        zt = 0.03          # merge only mm-level height differences (never across a curb)
        V, F, UV, T = weld(V, F, UV, T, ztol=zt)
        F, T, ins = fix_t_junctions(V, F, T, maxdz=zt)
        V, F, UV, T = weld(V, F, UV, T, ztol=zt)
        # 2. top surfaces must face up (welding/stitching can invert slivers)
        nz = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])[:, 2]
        F = np.where((nz < 0)[:, None], F[:, ::-1], F)
        L.V, L.F, L.UV, L.T, L.n = [V], [F], [UV], [T], len(V)
        L.O = [np.full(len(F), -1)]
        stats[name] = (n0, len(F), ins, int((nz < 0).sum()))
    RV, RF = LAYERS["Road__Asphalt"].V[0], LAYERS["Road__Asphalt"].F[0]
    road_z = MeshZ(RV, RF)
    dropped = 0
    for name in ("RoadLine__White", "RoadLine__Yellow"):
        L = LAYERS[name]
        res = L.arrays()
        if res is None:
            continue
        V, F, UV, T = res
        z = road_z(V[:, :2], how="near", zref=V[:, 2], tol=1.0)     # this paint's own level
        off = np.isnan(z)
        V = V.copy()
        V[~off, 2] = z[~off] + MARK_Z            # exactly MARK_Z above the final asphalt
        bad = off[F].any(1)                      # paint not fully on asphalt -> remove
        dropped += int(bad.sum())
        # on curved junction asphalt the surface between paint vertices can come up to the paint:
        # lift faces whose centre is < 5 mm above the asphalt (no z-fighting at distance)
        for _ in range(2):
            c = V[F].mean(1)
            lift = c[:, 2] - road_z(c[:, :2], how="near", zref=c[:, 2], tol=1.0)
            low = (~bad) & ~np.isnan(lift) & (lift < 0.005)
            if not low.any():
                break
            for f, dl in zip(F[low], 0.006 - lift[low]):
                V[f, 2] += dl
        L.V, L.F, L.UV, L.T, L.n = [V], [F[~bad]], [UV], [T[~bad]], len(V)
        L.O = [np.full(int((~bad).sum()), -1)]
    stats["paint_faces_dropped_off_asphalt"] = dropped
    return stats, road_z


# ----------------------------------------------------------------------------------- structures
DECK_T = 1.4          # bridge deck thickness (m)
PARAPET_H, PARAPET_W = 0.85, 0.35
JERSEY_H, JERSEY_W = 0.81, 0.45
TUNNEL_H = 5.6        # clear height
TILE_H = 2.8          # tunnel wall tile band height
LIGHT_SPACING = 10.0  # m between tunnel ceiling lights
TUNNEL_LIGHTS = []    # [x, y, z, heading] -> build/tunnel_lights.json (UE point lights)
PIER_SPACING = 25.0
FREEWAY = {"motorway", "motorway_link", "trunk", "trunk_link"}


def oriented_strip(layer, A, B, D, uv_scale=1.5):
    """Quad strip between polylines A,B (n,3); faces turned so their normal points along D (n,3)."""
    n = len(A)
    if n < 2:
        return
    V = np.concatenate([A, B])
    along = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(A, axis=0), axis=1))])
    across = np.linalg.norm(B - A, axis=1)
    UV = np.concatenate([np.column_stack([along, np.zeros(n)]), np.column_stack([along, across])]) / uv_scale
    i = np.arange(n - 1)
    F = np.concatenate([np.stack([i, i + n, i + n + 1], 1), np.stack([i, i + n + 1, i + 1], 1)])
    nrm = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    Dm = np.concatenate([(D[:-1] + D[1:]) / 2, (D[:-1] + D[1:]) / 2])
    flip = np.sum(nrm * Dm, 1) < 0
    F[flip] = F[flip][:, ::-1]
    LAYERS[layer].add(V, F, UV)


def box(layer, c, ex, ey, hx, hy, z0, z1):
    """Oriented box: centre c (2,), unit axes ex, ey, half sizes hx, hy, from z0 to z1."""
    corners = [c + sx * hx * ex + sy * hy * ey for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    for k in range(4):
        a, b = corners[k], corners[(k + 1) % 4]
        out = (a + b) / 2 - c
        A = np.array([[*a, z0], [*b, z0]]); B = np.array([[*a, z1], [*b, z1]])
        oriented_strip(layer, A, B, np.array([[*out, 0], [*out, 0]]))
    top = np.array([[*p, z1] for p in corners])
    oriented_strip(layer, top[[0, 1]], top[[3, 2]], np.array([[0, 0, 1], [0, 0, 1]]))


def wall_band(layer, P_bot, P_top_z, D, height_extra=0.0):
    oriented_strip(layer, P_bot, np.column_stack([P_bot[:, :2], P_top_z]), D)


LANE_PIECE = 4.0      # m: vehicle-lane pieces for the same-level tests (their z is the piece mean)


def lane_pieces(od, step=LANE_PIECE):
    """Every vehicle lane (ROAD_TYPES, all levels) of the OpenDRIVE as (STRtree of plan polygons <= step m long,
    their polygons, their mean z). What a car drives through, independent of how the surfaces were meshed."""
    polys, zs = [], []
    for r in od.roads:
        for sec in r.sections:
            lanes = [l for l in sec.left + sec.right if l.type in ROAD_TYPES]
            if not lanes or sec.e - sec.s < 1e-3:
                continue
            n = max(2, int(math.ceil(sec.e - sec.s)) + 1)        # 1 m stations
            s = np.linspace(sec.s, sec.e, n)
            B, _ = r.borders(sec, s)
            k = max(1, int(round(step)))
            for l in lanes:
                ti, to = B[l.id]
                A, O = r.xyz(s, ti), r.xyz(s, to)
                for i in range(0, n - 1, k):
                    j = min(i + k, n - 1)
                    p = shapely.make_valid(Polygon(np.vstack([A[i:j + 1, :2], O[i:j + 1, :2][::-1]])))
                    if not p.is_empty and p.area > 1e-3:
                        polys.append(p); zs.append(float(np.mean((A[i:j + 1, 2] + O[i:j + 1, 2]) / 2)))
    polys = np.array(polys, dtype=object)
    return shapely.STRtree(polys), polys, np.array(zs)


def structure_footprints(od, runs, kinds=("bridge", "tunnel")):
    polys = []
    for r in od.roads:
        for s0, s1, lv in runs.get(r.id, []):
            if lv not in kinds:
                continue
            for sec in r.sections:
                a, b = max(sec.s, s0), min(sec.e, s1)
                lanes = sec.left + sec.right
                if b - a < 0.5 or not lanes:
                    continue
                s = r.sample_s(a, b, max_step=3.0)
                if len(s) < 2:
                    continue
                B, _ = r.borders(sec, s)
                tl = np.max([np.maximum(*B[l.id]) for l in lanes], 0)
                tr = np.min([np.minimum(*B[l.id]) for l in lanes], 0)
                polys.append(polys_only(Polygon(np.concatenate([r.xyz(s, tl)[:, :2], r.xyz(s, tr)[::-1, :2]]))))
    return unary_union(polys) if polys else Polygon()


TUNNEL_ROOF = 0.6     # roof slab thickness over the ceiling (m)
TUNNEL_MIN_H = 3.0    # lowest clear height a tunnel is squeezed to under another level


def headroom(lanes, P, z):
    """Height from z up to the lowest vehicle lane of another level (> 2 m higher) over the plan points P; inf if none."""
    out = np.full(len(P), np.inf)
    if lanes is None:
        return out
    tree, _, lz = lanes
    vi, pk = tree.query(shapely.points(P), predicate="intersects")
    dz = lz[pk] - z[vi]
    up = dz > 2.0
    np.minimum.at(out, vi[up], dz[up])
    return out


def build_structures(od, dem, top_z, paved_all, lane_idx=None):
    """Bridge decks / fascia / parapets / piers, tunnel shells, freeway Jersey barriers."""
    runs = json.loads((C.BUILD / "road_levels.json").read_text())
    types = json.loads((C.BUILD / "road_types.json").read_text())
    # pavement at ground level (piers must not stand on it), from the finished surfaces themselves:
    # road / sidewalk triangles within 2 m of the DEM. (Subtracting structure footprints instead
    # would also remove the roads running UNDER a bridge - exactly where piers must not go.)
    gp = []
    for name in ("Road__Asphalt", "SideWalk__Concrete"):
        res = LAYERS[name].arrays()
        if res is None:
            continue
        V, F = res[0], res[1]
        c = V[F].mean(1)
        ground = np.abs(c[:, 2] - dem(c[:, 0], c[:, 1])) < 2.0
        gp.append(shapely.polygons(V[F[ground]][:, :, :2]))
    # road and sidewalk triangles in one tiled union (two unions, then their union, took ~40 s)
    ground_paved = robust_union(np.concatenate(gp)) if gp else Polygon()   # polygonal, on the 1 cm grid
    shapely.prepare(ground_paved)
    st = defaultdict(int)
    struct_polys = []
    for r in od.roads:
        rr = runs.get(r.id)
        if not rr:
            continue
        hw = types.get(r.id, "unknown")
        for s0, s1, lv in rr:
            freeway_ground = hw in FREEWAY and lv in ("grade", "struct") and r.junction == "-1"
            if lv not in ("bridge", "tunnel") and not freeway_ground:
                continue
            for sec in r.sections:
                a, b = max(sec.s, s0), min(sec.e, s1)
                if b - a < 1.0:
                    continue
                s = r.sample_s(a, b, max_step=3.0)
                if len(s) < 2:
                    continue
                B, off = r.borders(sec, s)
                lanes = sec.left + sec.right
                if not lanes:
                    continue
                tl = np.max([np.maximum(*B[l.id]) for l in lanes], 0)
                tr = np.min([np.minimum(*B[l.id]) for l in lanes], 0)
                left_walk = any(l.type in WALK_TYPES and np.allclose(np.maximum(*B[l.id]), tl) for l in sec.left)
                right_walk = any(l.type in WALK_TYPES and np.allclose(np.minimum(*B[l.id]), tr) for l in sec.right)
                zc = r.z(s)
                _, _, h = r.ref(s)
                nl = np.column_stack([-np.sin(h), np.cos(h), np.zeros_like(h)])     # left normal
                PL, PR = r.xyz(s, tl), r.xyz(s, tr)
                zl = zc + (CURB if left_walk else 0.0); zr = zc + (CURB if right_walk else 0.0)
                PL[:, 2], PR[:, 2] = zl, zr
                # an edge touching another carriageway at the same height gets no fascia/parapet
                probeL, probeR = r.xyz(s, tl + 0.6)[:, :2], r.xyz(s, tr - 0.6)[:, :2]
                adjL = ~np.isnan(top_z(probeL, zl))
                adjR = ~np.isnan(top_z(probeR, zr))
                if lv in ("bridge", "tunnel"):
                    struct_polys.append(polys_only(Polygon(np.concatenate([PL[:, :2], PR[::-1, :2]]))))
                if lv == "bridge":
                    st["bridge_runs"] += 1
                    dz = DECK_T
                    Ub, Ur = PL.copy(), PR.copy(); Ub[:, 2] = zc - dz; Ur[:, 2] = zc - dz
                    oriented_strip("Bridge__BridgeConc", Ub, Ur, np.tile([0, 0, -1.0], (len(s), 1)))   # underside
                    for P, ztop, adj, sgn in ((PL, zl, adjL, 1.0), (PR, zr, adjR, -1.0)):
                        for i0, i1 in _true_runs(~adj):
                            sl = slice(i0, i1 + 1)
                            Pb = P[sl].copy(); Pb[:, 2] = zc[sl] - dz
                            wall_band("Bridge__BridgeConc", Pb, ztop[sl], sgn * nl[sl])            # fascia
                            # parapet: outer face, top, inner face
                            outer = P[sl].copy()
                            wall_band("GuardRail__Struct", outer, ztop[sl] + PARAPET_H, sgn * nl[sl])
                            inner = outer.copy(); inner[:, :2] -= sgn * nl[sl, :2] * PARAPET_W
                            wall_band("GuardRail__Struct", inner, ztop[sl] + PARAPET_H, -sgn * nl[sl])
                            t_out = np.column_stack([outer[:, :2], ztop[sl] + PARAPET_H])
                            t_in = np.column_stack([inner[:, :2], ztop[sl] + PARAPET_H])
                            oriented_strip("GuardRail__Struct", t_out, t_in, np.tile([0, 0, 1.0], (i1 - i0 + 1, 1)))
                            st["parapet_m"] += int(s[i1] - s[i0])
                    # piers
                    for sp in np.arange(a + 6.0, b - 6.0, PIER_SPACING):
                        for shift in (0.0, 4.0, -4.0, 8.0, -8.0):
                            sq = np.array([sp + shift])
                            if not (a + 3 < sq[0] < b - 3):
                                continue
                            Bq, _ = r.borders(sec, sq)
                            tlq = max(max(Bq[l.id][0][0], Bq[l.id][1][0]) for l in lanes)
                            trq = min(min(Bq[l.id][0][0], Bq[l.id][1][0]) for l in lanes)
                            w = tlq - trq
                            xq, yq, hq = r.ref(sq)
                            ex = np.array([-math.sin(hq[0]), math.cos(hq[0])])        # across the deck
                            ey = np.array([math.cos(hq[0]), math.sin(hq[0])])         # along the road
                            cmid = np.array([xq[0], yq[0]]) + ex * (tlq + trq) / 2
                            cols = [cmid] if w < 14 else [cmid + ex * w / 4, cmid - ex * w / 4]
                            zdeck = float(r.z(sq)[0]) - dz
                            g = float(dem(np.array([c_[0] for c_ in cols]), np.array([c_[1] for c_ in cols])).min())
                            if zdeck - g < 2.5:
                                break                                            # too low: abutment
                            if any(shapely.contains_xy(ground_paved, c_[0], c_[1]) for c_ in cols):
                                continue                                         # would stand in a lane below
                            box("Bridge__PierConc", cmid, ex, ey, max(0.5, w / 2 - 0.5), 0.8, zdeck - 1.0, zdeck)  # cap
                            for c_ in cols:
                                box("Bridge__PierConc", c_, ex, ey, 0.65, 0.65, g - 0.3, zdeck - 1.0)
                            st["piers"] += 1
                            break
                elif lv == "tunnel":
                    st["tunnel_runs"] += 1
                    # clear height: TUNNEL_H, less where a road of the level above runs closer than ceiling + roof
                    Pm = r.xyz(s, (tl + tr) / 2)
                    hr = np.minimum.reduce([headroom(lane_idx, Q[:, :2], zc) for Q in (PL, PR, Pm)])
                    TH = np.clip(hr - TUNNEL_ROOF - 0.05, TUNNEL_MIN_H, TUNNEL_H)
                    st["tunnel_lowered_m"] += int(np.sum(TH < TUNNEL_H - 1e-6) * np.mean(np.diff(s)))
                    for P, adj, sgn in ((PL, adjL, 1.0), (PR, adjR, -1.0)):
                        for i0, i1 in _true_runs(~adj):
                            sl = slice(i0, i1 + 1)
                            # glazed tile up to TILE_H, grimy concrete above (faces inwards)
                            wall_band("Wall__TunnelTile", P[sl], P[sl, 2] + TILE_H, -sgn * nl[sl])
                            Pu = P[sl].copy(); Pu[:, 2] += TILE_H
                            wall_band("Wall__TunnelCeil", Pu, zc[sl] + TH[sl], -sgn * nl[sl])
                    Cl, Cr = PL.copy(), PR.copy(); Cl[:, 2] = zc + TH; Cr[:, 2] = zc + TH
                    oriented_strip("Wall__TunnelCeil", Cl, Cr, np.tile([0, 0, -1.0], (len(s), 1)))   # ceiling
                    # ceiling light fittings every LIGHT_SPACING m on the tunnel axis (emissive; UE adds lights)
                    run = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(Pm[:, :2], axis=0).T))])
                    for d_ in np.arange(LIGHT_SPACING / 2, run[-1], LIGHT_SPACING):
                        k = int(np.clip(np.searchsorted(run, d_), 0, len(s) - 1))
                        ex = np.array([-math.sin(h[k]), math.cos(h[k])]); ey = np.array([math.cos(h[k]), math.sin(h[k])])
                        z1 = zc[k] + TH[k] - 0.02; z0 = z1 - 0.12
                        box("Wall__TunnelLight", Pm[k, :2], ex, ey, 0.22, 0.75, z0, z1)
                        bot = np.array([[*(Pm[k, :2] + sx * 0.22 * ex + sy * 0.75 * ey), z0] for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
                        oriented_strip("Wall__TunnelLight", bot[[0, 1]], bot[[3, 2]], np.array([[0, 0, -1.0], [0, 0, -1.0]]))
                        TUNNEL_LIGHTS.append([float(Pm[k, 0]), float(Pm[k, 1]), float(z0 - 0.05), float(h[k])])
                        st["tunnel_lights"] += 1
                    Tl, Tr = Cl.copy(), Cr.copy(); Tl[:, 2] += TUNNEL_ROOF; Tr[:, 2] += TUNNEL_ROOF
                    oriented_strip("Wall__Struct", Tl, Tr, np.tile([0, 0, 1.0], (len(s), 1)))    # roof slab
                else:   # freeway at ground level / in a trench: Jersey barriers on exposed edges
                    for P, ztop, adj, sgn in ((PL, zl, adjL, 1.0), (PR, zr, adjR, -1.0)):
                        for i0, i1 in _true_runs(~adj):
                            sl = slice(i0, i1 + 1)
                            # entirely outside the carriageway edge (inner face 5 cm out), not on the outer lane
                            inner = P[sl].copy(); inner[:, :2] += sgn * nl[sl, :2] * 0.05; inner[:, 2] -= 0.05
                            outer = inner.copy(); outer[:, :2] += sgn * nl[sl, :2] * JERSEY_W
                            wall_band("GuardRail__Struct", outer, ztop[sl] + JERSEY_H, sgn * nl[sl])
                            wall_band("GuardRail__Struct", inner, ztop[sl] + JERSEY_H, -sgn * nl[sl])
                            oriented_strip("GuardRail__Struct", np.column_stack([outer[:, :2], ztop[sl] + JERSEY_H]),
                                           np.column_stack([inner[:, :2], ztop[sl] + JERSEY_H]),
                                           np.tile([0, 0, 1.0], (i1 - i0 + 1, 1)))
                            st["jersey_m"] += int(s[i1] - s[i0])
    (C.BUILD / "tunnel_lights.json").write_text(json.dumps(TUNNEL_LIGHTS))
    return dict(st), ground_paved


def _true_runs(mask):
    out, start = [], None
    for i, f in enumerate(mask):
        if f and start is None:
            start = i
        if (not f or i == len(mask) - 1) and start is not None:
            end = i if f else i - 1
            if end > start:
                out.append((start, end))
            start = None
    return out


# ----------------------------------------------------------------------------------- export
def export(extent):
    nx = int(math.ceil((extent[2] - extent[0]) / TILE))
    arrays, summary = {}, {}
    for name, L in sorted(LAYERS.items()):
        res = L.arrays()
        if res is None:
            continue
        V, F, UV, T = res
        cen = V[F].mean(1)
        auto = ((cen[:, 1] - extent[1]) // TILE).astype(int) * nx + ((cen[:, 0] - extent[0]) // TILE).astype(int)
        T = np.where(T >= 0, T, auto)
        for t in np.unique(T):
            f = F[T == t]
            used, inv = np.unique(f.ravel(), return_inverse=True)
            key = f"{name}|T{t:02d}"
            arrays[key + "|V"] = V[used].astype(np.float64)
            arrays[key + "|F"] = inv.reshape(-1, 3).astype(np.int32)
            arrays[key + "|UV"] = UV[used].astype(np.float32)
            summary[key] = int(len(f))
    np.savez_compressed(C.GEOM_FILE, **arrays)
    return summary


def preview(extent, path):
    x0, y0, x1, y1 = extent
    S = 2000 / max(x1 - x0, y1 - y0)
    W, H = int((x1 - x0) * S), int((y1 - y0) * S)
    img = Image.open(C.TEX / "T_LA_Ortho_D.jpg").resize((W, H)).convert("RGB")
    img = Image.blend(img, Image.new("RGB", (W, H), (0, 0, 0)), 0.55)
    from PIL import ImageDraw
    dr = ImageDraw.Draw(img)
    col = {"Road__Asphalt": (70, 70, 78), "SideWalk__Concrete": (175, 170, 160), "RoadLine__White": (255, 255, 255),
           "RoadLine__Yellow": (255, 200, 0), "Building__Roof": (120, 70, 160)}
    for name in ["Building__Roof", "SideWalk__Concrete", "Road__Asphalt", "RoadLine__White", "RoadLine__Yellow"]:
        res = LAYERS[name].arrays()
        if res is None:
            continue
        V, F, _, _ = res
        for f in F:
            p = [((V[k, 0] - x0) * S, (y1 - V[k, 1]) * S) for k in f]
            dr.polygon(p, fill=col[name])
    img.save(path)


def r_grade(r, s, ds=0.25):
    a, b = max(0.0, s - ds), min(r.length, s + ds)
    return float((r.z(np.array([b]))[0] - r.z(np.array([a]))[0]) / max(b - a, 1e-9))


def harmonize_junction_elevation(od, zjunc, step=2.0):
    """Rewrite the elevation profile of every connecting road so that all lanes crossing the
    same spot of a junction share ONE height (the junction surface). SUMO interpolates each
    connection independently, which on steep junctions gives up to ~1 m disagreement between
    overlapping lanes. End heights are kept exact so links to the incoming roads stay continuous.
    """
    n = 0
    for r in od.roads:
        if r.junction == "-1" or r.length < 0.5:
            continue
        sec = r.sections[0]
        lanes = [l for l in sec.left + sec.right if l.type in ROAD_TYPES]
        if not lanes:
            continue
        m = max(2, int(math.ceil(r.length / step)) + 1)
        s = np.linspace(0, r.length, m)
        B, _ = r.borders(sec, s)
        ti, to = B[lanes[0].id]
        P = r.xyz(s, (ti + to) / 2)
        z_orig = r.z(s)
        z_new = zjunc(P[:, :2], z_orig)          # same-level surfaces only (multi-level junctions)
        # blend: exact height AND grade at both ends (smoothstep weight has zero slope there),
        # junction surface in the middle
        # blend over the whole connection (half-length ramps, smoothstep) so the vertical curve
        # stays gentle; cap the pull towards the shared surface on non-planar junctions
        u = np.clip(np.minimum(s, r.length - s) / (r.length / 2 + 1e-9), 0, 1)
        w = u * u * (3 - 2 * u)
        z = z_orig + np.clip(z_new - z_orig, -0.10, 0.10) * w
        g = np.gradient(z, s) if m > 2 else np.full(m, (z[-1] - z[0]) / r.length)
        g[0], g[-1] = r_grade(r, 0.0), r_grade(r, r.length)     # keep the linked roads' grades
        write_elevation(r.el, s, z, g)
        n += 1
    od.tree.write(str(C.XODR_FILE), xml_declaration=True, encoding="UTF-8", pretty_print=True)
    return n


def main():
    import sys
    od = OpenDrive(C.XODR_FILE)
    print(f"roads {len(od.roads)}  junctions {len(od.junctions)}")
    drive_polys, walk_polys, jdrive, dpts, wpts = build_roads(od)
    net_file = C.NET_FILE if C.XODR_SOURCE == "sumo" else None   # OdrForge: no SUMO net
    all_drive, walk_u, road_patch, corner_u, zroad, zwalk = build_junctions(
        od, net_file, drive_polys, walk_polys, jdrive, dpts, wpts)
    # pull overlapping connecting lanes towards one shared junction surface (capped, C1). SUMO only: OdrForge
    # already gives each junction one surface, and its OpenDRIVE is used as written
    if (net_file is not None and "--no-harmonize" not in sys.argv
            and not getattr(main, "_done", False)):
        k = harmonize_junction_elevation(od, zroad)
        print(f"harmonized elevation of {k} connecting roads -> rebuilding from updated xodr")
        main._done = True
        LAYERS.clear(); JUNCTION_REF.clear(); DRIVE_OWNER.clear()
        DRIVE_P3.clear(); WALK_P3.clear(); JREF_BY.clear()
        return main()
    # first: stop bars are placed clear of them
    ncw = build_crosswalks(C.NET_FILE, zroad) if net_file is not None else build_crosswalks_xodr(od, zroad)
    build_markings(od)
    print(f"crosswalks {ncw}")
    fstats, road_z = finalize_surfaces()
    print("surface repair:", fstats)
    WV, WF = LAYERS["SideWalk__Concrete"].V[0], LAYERS["SideWalk__Concrete"].F[0]
    walk_z = MeshZ(WV, WF)
    global PAVED_Z
    PAVED_Z = lambda Q: np.fmin(road_z(Q, "min"), walk_z(Q, "min"))

    paved = unary_union([all_drive, walk_u, road_patch, corner_u]).buffer(0.05)
    rx0, ry0, rx1, ry1 = paved.bounds
    dem = Dem(C.DEM_UTM, filt=3)
    # terrain extent: road extent + margin, clamped to DEM coverage (in map frame)
    b = dem.utm_bounds
    inv = Transformer.from_crs(C.UTM_EPSG, C.MAP_PROJ, always_xy=True)
    cx, cy = inv.transform([b.left, b.right, b.left, b.right], [b.bottom, b.bottom, b.top, b.top])
    extent = (max(rx0 - TERRAIN_MARGIN, max(cx[0], cx[2]) + 2), max(ry0 - TERRAIN_MARGIN, max(cy[0], cy[1]) + 2),
              min(rx1 + TERRAIN_MARGIN, min(cx[1], cx[3]) - 2), min(ry1 + TERRAIN_MARGIN, min(cy[2], cy[3]) - 2))
    extent = tuple(float(round(v / TERRAIN_STEP) * TERRAIN_STEP) for v in extent)
    print("extent", extent)
    build_ortho(extent)
    # bridges / tunnels / freeway barriers
    # surface at the same level as zref (within 1 m) next to a structure edge, NaN if none
    top_z = lambda Q, zr: np.fmax(road_z(Q, "near", zref=zr, tol=1.0), walk_z(Q, "near", zref=zr, tol=1.0))
    lanes = lane_pieces(od)
    sstats, ground_paved = build_structures(od, dem, top_z, paved, lanes)
    print("structures:", sstats)
    runs = json.loads((C.BUILD / "road_levels.json").read_text())
    tunnel_fp = structure_footprints(od, runs, ("tunnel",))
    wall = LAYERS["Wall__Struct"].arrays()
    roof_z = MeshZ(wall[0], wall[1]) if wall is not None else (lambda Q, how="max": np.full(len(Q), np.nan))
    paved_pts = np.concatenate([dpts, wpts])
    nt = build_terrain(dem, extent, paved, paved_pts, tunnel_fp, lambda Q: roof_z(Q, "max"), lanes)
    print("terrain tris", nt, "| at vehicle lanes:", LANE_TERRAIN)
    bstats = build_buildings(Dem(C.DEM_UTM, filt=5), extent, ground_paved, lanes)
    print("buildings", bstats)
    summary = export(extent)
    meta = json.loads(C.META_FILE.read_text()) if C.META_FILE.exists() else {}
    meta.update(extent=extent, tile=TILE, ortho="T_LA_Ortho_D.jpg", map_proj=C.MAP_PROJ, lat0=C.LAT0, lon0=C.LON0,
                layers=summary, tris_total=int(sum(summary.values())))
    C.META_FILE.write_text(json.dumps(meta, indent=1))
    tot = defaultdict(int)
    for k, v in summary.items():
        tot[k.split("|")[0]] += v
    for k, v in sorted(tot.items()):
        print(f"  {k:24s} {v:8d} tris")
    print("total tris", sum(tot.values()))
    preview(extent, C.BUILD / "preview_top.png")


if __name__ == "__main__":
    main()







