"""Quality audit of the OpenDRIVE and the generated meshes. Prints a report + writes build/qa_report.json.

OpenDRIVE                                  Mesh
 A link continuity (xy, z, heading, grade)  F degenerate / downward-facing triangles
 B in-road grade & grade kinks              G cracks inside the road surface
 C lane widths & section width jumps        H overlapping road triangles (z-fighting)
 D lane link targets exist                  I height steps between adjacent road pieces
 E planView geometry continuity             J terrain poking through paved surfaces
                                            K paint floating / buried / off-road
                                            L curb height, M buildings on the street
"""
import json
import math
import sys
from collections import defaultdict

import numpy as np
import shapely
from scipy.spatial import cKDTree

import importlib

import config as C
from opendrive import OpenDrive, eval_geometry

GB = importlib.import_module("03_build_geometry")

ROAD_TYPES = {"driving", "restricted", "parking", "bidirectional", "biking", "bus", "taxi", "HOV",
              "entry", "exit", "onRamp", "offRamp", "connectingRamp", "shoulder", "stop", "border"}
REPORT = {}
SAME_LEVEL = 1.0      # m: surfaces further apart in height are on different levels (bridges, tunnels)
VERBOSE = "-v" in sys.argv


def section(name, items, fmt, limit=6):
    REPORT[name] = items
    flag = "OK " if not items else "!! "
    print(f"{flag}{name}: {len(items)}")
    for it in items[:limit if not VERBOSE else 1000]:
        print("      " + fmt(it))


# ----------------------------------------------------------------------------- OpenDRIVE
od = OpenDrive(C.XODR_FILE)
roads = od.by_id


def end_state(r, at_start):
    s = 0.0 if at_start else r.length
    x, y, h = r.ref(s)
    z = float(r.z(np.array([s]))[0])
    ds = 0.5
    g = float((r.z(np.array([min(r.length, s + ds)]))[0] - r.z(np.array([max(0, s - ds)]))[0])
              / max(1e-6, min(r.length, s + ds) - max(0, s - ds)))
    return np.array([float(x), float(y)]), z, float(h), g


def lane_offset_border(r, at_start):
    s = 0.0 if at_start else r.length
    sec = r.sections[0] if at_start else r.sections[-1]
    B, off = r.borders(sec, np.array([s]))
    return {k: (float(v[0][0]), float(v[1][0])) for k, v in B.items()}, float(off[0])


def lane_center(r, lane_id, at_start):
    s = 0.0 if at_start else r.length
    sec = r.sections[0] if at_start else r.sections[-1]
    B, _ = r.borders(sec, np.array([s]))
    if lane_id not in B:
        return None
    ti, to = B[lane_id]
    p = r.xyz(np.array([s]), (ti + to) / 2)[0]
    return p


link_issues = []
for r in od.roads:
    for which, el in (("succ", r.succ), ("pred", r.pred)):
        if el is None or el.get("elementType") != "road":
            continue
        o = roads.get(el.get("elementId"))
        if o is None:
            link_issues.append((r.id, which, "missing target road", el.get("elementId")))
            continue
        cp = el.get("contactPoint", "start")
        sec = r.sections[-1] if which == "succ" else r.sections[0]
        kind = "successor" if which == "succ" else "predecessor"
        for ln_el in r.el.find("lanes").findall("laneSection")[-1 if which == "succ" else 0].iter("lane"):
            lk = ln_el.find("link")
            tgt = lk.find(kind) if lk is not None else None
            if tgt is None:
                continue
            p1 = lane_center(r, int(ln_el.get("id")), which == "pred")
            p2 = lane_center(o, int(tgt.get("id")), cp == "start")
            if p1 is None or p2 is None:
                continue
            gap = float(np.hypot(*(p1[:2] - p2[:2]))); dz = float(p1[2] - p2[2])
            _, _, h1, g1 = end_state(r, which == "pred"); _, _, h2, g2 = end_state(o, cp == "start")
            same = (which == "succ" and cp == "start") or (which == "pred" and cp == "end")
            dh = (h2 - h1) if same else (h2 - h1 + math.pi)
            dh = abs((dh + math.pi) % (2 * math.pi) - math.pi)
            dg = abs(g2 - g1) if same else abs(g2 + g1)
            if gap > 0.05 or abs(dz) > 0.02 or dh > math.radians(3) or dg > 0.04:
                link_issues.append((r.id, ln_el.get("id"), which, o.id, tgt.get("id"), round(gap, 3), round(dz, 3),
                                    round(math.degrees(dh), 1), round(dg * 100, 1)))
section("A  lane-link discontinuities (road, lane, end, to road, to lane, xy gap m, z step m, heading deg, grade jump %)",
        link_issues, lambda t: str(t))

grade_max, kinks = [], []
for r in od.roads:
    s = np.arange(0, r.length, 0.5)
    if len(s) < 3:
        continue
    z = r.z(s)
    g = np.gradient(z, s)
    if np.abs(g).max() > 0.15:
        grade_max.append((r.id, r.name, r.junction != "-1", round(float(np.abs(g).max()) * 100, 1)))
    # grade change over 2 m (vertical kink) > 4 %
    k = np.abs(g[4:] - g[:-4])
    if len(k) and k.max() > 0.04:
        i = int(k.argmax())
        x, y, _ = r.ref(s[i])
        kinks.append((r.id, r.name, r.junction != "-1", round(float(k.max()) * 100, 1), round(float(x), 1), round(float(y), 1)))
section("B1 roads with grade > 15 % (id, name, in_junction, max grade %)", grade_max, str)
section("B2 vertical kinks: grade change > 4 % within 2 m (id, name, junction, jump %, x, y)", kinks, str)

narrow, jumps = [], []
for r in od.roads:
    for i, sec in enumerate(r.sections):
        s = np.linspace(sec.s, sec.e, max(2, int((sec.e - sec.s) / 0.5) + 1))
        for l in sec.left + sec.right:
            w = l.width(s - sec.s)
            if l.type in ROAD_TYPES and (w.min() < 2.2) and r.junction == "-1" and (sec.e - sec.s) > 1.0:
                narrow.append((r.id, l.id, l.type, round(float(w.min()), 2)))
            if w.min() < -1e-3:
                narrow.append((r.id, l.id, l.type, "NEGATIVE", round(float(w.min()), 3)))
        if i:   # width jump at section border for continuing lanes: the lane's predecessor link, else same id
            prev = r.sections[i - 1]
            sec_el = r.el.find("lanes").findall("laneSection")[i]
            pred = {int(e.get("id")): int(e.find("link/predecessor").get("id"))
                    for e in sec_el.iter("lane") if e.find("link/predecessor") is not None}
            for l in sec.left + sec.right:
                pid = pred.get(l.id, l.id)
                pl = next((p for p in prev.left + prev.right if p.id == pid), None)
                if pl is None:
                    continue
                w0 = float(pl.width(np.array([sec.s - prev.s]))[0]); w1 = float(l.width(np.array([0.0]))[0])
                if abs(w1 - w0) > 0.1 and min(w0, w1) > 0.01:
                    jumps.append((r.id, l.id, round(w0, 2), round(w1, 2)))
section("C1 narrow (<2.2 m) or negative driving lanes (road, lane, type, width)", narrow, str)
section("C2 lane width jumps at section borders (road, lane, before, after)", jumps, str)

badlinks = []
root = od.tree.getroot()
for rel in root.findall("road"):
    rid = rel.get("id")
    secs = rel.find("lanes").findall("laneSection")
    for k, sec in enumerate(secs):
        for ln in sec.iter("lane"):
            lk = ln.find("link")
            if lk is None:
                continue
            for kind in ("predecessor", "successor"):
                e = lk.find(kind)
                if e is None:
                    continue
                # links between sections of the same road point into the neighbouring section
                nb = k - 1 if kind == "predecessor" else k + 1
                if 0 <= nb < len(secs):
                    if e.get("id") not in {l.get("id") for l in secs[nb].iter("lane")}:
                        badlinks.append((rid, ln.get("id"), kind, "target lane missing in next section", e.get("id")))
                    continue
                link = rel.find("link").find(kind) if rel.find("link") is not None else None
                if link is None:
                    badlinks.append((rid, ln.get("id"), kind, "lane link without road link"))
                elif link.get("elementType") == "road":
                    tgt = root.find(f"road[@id='{link.get('elementId')}']")
                    ids = {l.get("id") for l in tgt.iter("lane")} if tgt is not None else set()
                    if e.get("id") not in ids:
                        badlinks.append((rid, ln.get("id"), kind, "target lane missing", e.get("id")))
section("D  broken lane links", badlinks, str)

geo = []
for r in od.roads:
    gs = r.el.find("planView").findall("geometry")
    for a, b in zip(gs[:-1], gs[1:]):
        L = float(a.get("length"))
        x, y = eval_geometry(a, np.array([max(L - 1e-4, 0.0), L]))
        hx = math.atan2(y[1] - y[0], x[1] - x[0]) if L > 1e-3 else float(a.get("hdg"))
        gap = math.hypot(x[1] - float(b.get("x")), y[1] - float(b.get("y")))
        dh = abs((float(b.get("hdg")) - hx + math.pi) % (2 * math.pi) - math.pi)
        if gap > 0.02 or dh > math.radians(2):
            geo.append((r.id, round(gap, 3), round(math.degrees(dh), 2)))
    if r.length < 0.5:
        geo.append((r.id, "very short road", round(r.length, 3)))
section("E  planView discontinuities (road, gap m, heading jump deg)", geo, str)

# ----------------------------------------------------------------------------- meshes
g = np.load(C.GEOM_FILE)


def layer(prefix):
    Vs, Fs, n = [], [], 0
    for k in g.files:
        if k.startswith(prefix) and k.endswith("|V"):
            V = g[k]; F = g[k[:-2] + "|F"]
            Vs.append(V); Fs.append(F + n); n += len(V)
    if not Vs:
        return np.zeros((0, 3)), np.zeros((0, 3), int)
    V = np.concatenate(Vs); F = np.concatenate(Fs)
    # weld coincident vertices with the builder's tolerance weld (rounding would split pairs that
    # straddle a rounding boundary and report false cracks)
    V2, F2, _, _ = GB.weld(V, F, np.zeros((len(V), 2)), np.zeros(len(F), int))
    return V2, F2


RV, RF = layer("Road__")
SV, SF = layer("SideWalk__Concrete")
CV, CF = layer("SideWalk__Curb")
MV, MF = layer("RoadLine__")
TV, TF = layer("Terrain__")
BV, BF = layer("Building__")


def tri_area_normal(V, F):
    e1 = V[F[:, 1]] - V[F[:, 0]]; e2 = V[F[:, 2]] - V[F[:, 0]]
    n = np.cross(e1, e2)
    a = np.linalg.norm(n, axis=1) / 2
    return a, n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)


deg, down = [], []
for name, V, F, top in (("Road", RV, RF, True), ("SideWalk", SV, SF, True), ("RoadLine", MV, MF, True),
                        ("Curb", CV, CF, False), ("Terrain", TV, TF, True), ("Building", BV, BF, False)):
    a, n = tri_area_normal(V, F)
    nd = int((a < 1e-5).sum())
    if nd:
        deg.append((name, nd, f"{nd / len(F) * 100:.2f}%"))
    if top:
        # top surfaces must never face down. (Near-vertical micro-slivers are legitimate: they
        # seal sub-3 cm height seams between adjacent pieces.)
        bad_n = (n[:, 2] < 0) & (a > 1e-5)
        nb = int(bad_n.sum())
        if nb:
            c = V[F[bad_n]].mean(1)[:3].round(1).tolist()
            down.append((name, nb, c))
section("F1 degenerate triangles (layer, count)", deg, str)
section("F2 surface triangles facing down/steep (layer, count, examples)", down, str)

# road surface polygon and boundary edges
ra, _ = tri_area_normal(RV, RF)
rpolys = shapely.polygons(RV[RF][:, :, :2])
road_u = shapely.union_all(rpolys[ra > 1e-6], grid_size=1e-3)
walk_u = shapely.union_all(shapely.polygons(SV[SF][:, :, :2]), grid_size=1e-3)
edges = np.sort(np.concatenate([RF[:, [0, 1]], RF[:, [1, 2]], RF[:, [2, 0]]]), axis=1)
ue, cnt = np.unique(edges, axis=0, return_counts=True)
bnd = ue[cnt == 1]
# a crack = an open edge running within 1 cm of another piece's open edge (exact test; a
# snapped union of ~50k triangles reports phantom interior boundaries)
segs = shapely.linestrings(np.stack([RV[bnd[:, 0], :2], RV[bnd[:, 1], :2]], 1))
stree = shapely.STRtree(segs)
si, sj = stree.query(segs, predicate="dwithin", distance=0.01)
keep = si < sj
share = (bnd[si[keep], 0][:, None] == bnd[sj[keep]]).any(1) | (bnd[si[keep], 1][:, None] == bnd[sj[keep]]).any(1)
ci, cj = si[keep][~share], sj[keep][~share]
# only edges running ALONGSIDE each other (>= 2 cm of overlap) form a crack; edges that merely
# come close near a corner of a densely segmented curb line do not
if len(ci):
    run = shapely.length(shapely.intersection(segs[cj], shapely.buffer(segs[ci], 0.01)))
    ci, cj = ci[run >= 0.02], cj[run >= 0.02]
cracks = []
for a_, b_ in zip(ci, cj):
    m_ = (RV[bnd[a_, 0]] + RV[bnd[a_, 1]]) / 2
    dz_ = abs(m_[2] - (RV[bnd[b_, 0], 2] + RV[bnd[b_, 1], 2]) / 2)
    if dz_ < SAME_LEVEL:                       # a deck edge above a lower road is not a crack
        cracks.append((round(float(m_[0]), 1), round(float(m_[1]), 1), round(float(dz_), 3)))
REPORT["G1_crack_dz_p95_cm"] = round(float(np.percentile([c[2] for c in cracks], 95)) * 100, 1) if cracks else 0.0
# holes = small interior rings that are not covered by sidewalk either (city blocks are big rings)
holes = []
for p in shapely.get_parts(road_u):
    for ring in p.interiors:
        h = shapely.Polygon(ring)
        if 0.01 < h.area < 50 and h.difference(walk_u).area > 0.01:
            holes.append((round(h.centroid.x, 1), round(h.centroid.y, 1), round(h.difference(walk_u).area, 3)))
section("G1 cracks: open edges within 1 cm of another piece (x, y, dz m)", cracks, str)
section("G2 uncovered holes in the road surface (x, y, area m2)", holes, str)

# exact pairwise overlap (a grid-snapped union would report phantom overlap/holes)
items = []
tree = shapely.STRtree(rpolys)
a_i, b_i = tree.query(rpolys, predicate="overlaps")
m = a_i < b_i
ai_, bi_ = a_i[m], b_i[m]
same = np.abs(RV[RF[ai_], 2].mean(1) - RV[RF[bi_], 2].mean(1)) < SAME_LEVEL   # stacked roads are not overlap
ai_, bi_ = ai_[same], bi_[same]
pa = shapely.area(shapely.intersection(rpolys[ai_], rpolys[bi_]))
over = float(pa.sum())
for i, j, ar in zip(ai_, bi_, pa):
    if ar > 0.05:
        c = rpolys[i].intersection(rpolys[j]).centroid
        dz = abs(RV[RF[i], 2].mean() - RV[RF[j], 2].mean())
        items.append((round(c.x, 1), round(c.y, 1), round(ar, 2), round(float(dz), 3)))
section(f"H  overlapping road triangles (x, y, overlap m2, dz) - total overlap {over:.1f} m2", items, str)

# height steps between coincident boundary vertices of different pieces (pre-weld positions)
raw = [g[k] for k in g.files if k.startswith("Road__") and k.endswith("|V")]
P = np.concatenate(raw)
t = cKDTree(P[:, :2])
pairs = t.query_pairs(0.02, output_type="ndarray")
dzs = np.abs(P[pairs[:, 0], 2] - P[pairs[:, 1], 2]) if len(pairs) else np.zeros(0)
steps = [(round(float(P[a, 0]), 1), round(float(P[a, 1]), 1), round(float(d), 3))
         for (a, b), d in zip(pairs, dzs) if 0.01 < d < SAME_LEVEL]
steps.sort(key=lambda s: -s[2])
section("I  height steps > 1 cm between coincident road vertices (x, y, dz)", steps, str)

# terrain poking through paved areas
paved = shapely.union_all([road_u, walk_u], grid_size=1e-3)
pin = paved.buffer(-0.05)
shapely.prepare(pin)
surf = np.concatenate([RV, SV])
dem_ = GB.Dem(C.DEM_UTM, filt=3)
surf = surf[np.abs(surf[:, 2] - dem_(surf[:, 0], surf[:, 1])) < 2.0]   # ground-level pavement only
st = cKDTree(surf[:, :2])
tin = shapely.contains_xy(pin, TV[:, 0], TV[:, 1])
d, i = st.query(TV[tin, :2], k=3)
zs = surf[i, 2].min(1)
poke = TV[tin][(TV[tin, 2] > zs - 0.05) & (d[:, 0] < 1.5)]
section("J  terrain vertices within 5 cm of / above paved surface (x, y, z)",
        [tuple(np.round(p, 2).tolist()) for p in poke], str)

# paint: height above asphalt and staying on asphalt
ma, mn = tri_area_normal(MV, MF)
mc = MV[MF].mean(1)
rt = cKDTree(RV[:, :2])
_, ri = rt.query(mc[:, :2], k=4)
# barycentric height of road under paint centroids
rtree = shapely.STRtree(rpolys)
RMZ = GB.MeshZ(RV, RF)
zroad = RMZ(mc, how="near", zref=mc[:, 2], tol=SAME_LEVEL)     # the asphalt at the paint's own level
lift = mc[:, 2] - zroad
off = np.isnan(lift)
bad = (~off) & ((lift < 0.003) | (lift > 0.04))
section("K1 paint triangles off the asphalt (x, y)", [tuple(np.round(c[:2], 1).tolist()) for c in mc[off]], str)
section("K2 paint buried (<3 mm) or floating (>4 cm) (x, y, lift m)",
        [(round(float(c[0]), 1), round(float(c[1]), 1), round(float(l), 3)) for c, l in zip(mc[bad], lift[bad])], str)

# curb height: sidewalk vertices at the road edge vs road surface 30 cm towards the road
sd, si = rt.query(SV[:, :2])
near = sd < 0.4
dirv = RV[si[near], :2] - SV[near, :2]
dirv /= np.maximum(np.linalg.norm(dirv, axis=1, keepdims=True), 1e-9)
probe = SV[near, :2] + dirv * 0.3
zr = RMZ(probe, how="near", zref=SV[near, 2] - 0.15, tol=SAME_LEVEL)   # road at the sidewalk's level
ch = SV[near, 2] - zr
valid = ~np.isnan(ch)
badc = [(round(float(p[0]), 1), round(float(p[1]), 1), round(float(c), 3))
        for p, c in zip(SV[near][valid], ch[valid]) if c < 0.10 or c > 0.20]
REPORT["L_curb_height_stats_cm"] = [round(float(np.percentile(ch[valid], q)) * 100, 1) for q in (1, 5, 50, 95, 99)]
print("   curb height percentiles 1/5/50/95/99 % (cm):", REPORT["L_curb_height_stats_cm"])
section("L  curb height outside 10-20 cm (x, y, height m)", badc, str)

# buildings intersecting the roadway
bpolys = shapely.union_all(shapely.polygons(BV[BF][:, :, :2])[tri_area_normal(BV, BF)[1][:, 2] > 0.9], grid_size=1e-3)
rc0 = RV[RF].mean(1)
ground_road = shapely.union_all(rpolys[(np.abs(rc0[:, 2] - dem_(rc0[:, 0], rc0[:, 1])) < 2.0) & (ra > 1e-6)], grid_size=1e-3)
ov = bpolys.intersection(ground_road.buffer(-0.2))      # roads in tunnels under buildings are fine
section("M  building footprint on the roadway (m2)", [round(ov.area, 2)] if ov.area > 0.5 else [], str)

# vertical clearance: wherever one road surface passes over another, the space between the lower
# road and the underside of the upper deck must be >= 4.3 m (US 14 ft)
DECK_T, MIN_CLEAR = GB.DECK_T, 4.3
dem_ = GB.Dem(C.DEM_UTM, filt=3)
hi = RV[:, 2] - dem_(RV[:, 0], RV[:, 1]) > 3.0                      # vertices on structures
clear_bad, pairs_checked = [], 0
if hi.any():
    lo_tree = cKDTree(RV[~hi, :2])
    lo_idx = np.flatnonzero(~hi)
    for k, nb in zip(np.flatnonzero(hi), lo_tree.query_ball_point(RV[hi, :2], 0.75)):
        if not nb:
            continue
        zl = RV[lo_idx[nb], 2]
        zl = zl[zl < RV[k, 2] - 1.0]
        if not len(zl):
            continue
        pairs_checked += 1
        c_ = RV[k, 2] - DECK_T - zl.max()
        if c_ < MIN_CLEAR:
            clear_bad.append((round(float(RV[k, 0]), 1), round(float(RV[k, 1]), 1), round(float(c_), 2)))
REPORT["N_clearance_points_checked"] = pairs_checked
print(f"   clearance checked at {pairs_checked} road-over-road points")
section("N  vertical clearance under bridges < 4.3 m (x, y, clearance m)", clear_bad, str)

# piers must not stand in lanes
SV_, SF_ = layer("Bridge__Pier")
if len(SF_):
    cen = SV_[SF_].mean(1)
    low = cen[:, 2] < dem_(cen[:, 0], cen[:, 1]) + 2.0              # pier feet
    rc = RV[RF].mean(1)
    ground_tri = np.abs(rc[:, 2] - dem_(rc[:, 0], rc[:, 1])) < 2.0      # ground-level asphalt only
    road_poly = shapely.union_all(rpolys[ground_tri & (ra > 1e-6)], grid_size=1e-3).buffer(-0.3)
    shapely.prepare(road_poly)
    feet_in = cen[low][shapely.contains_xy(road_poly, cen[low, 0], cen[low, 1])]
    lanes_hit = [tuple(np.round(p[:2], 1).tolist()) for p in feet_in]
else:
    lanes_hit = []
section("O  bridge piers standing on a roadway (x, y)", lanes_hit, str)

# nothing but road surface and paint in the space a car occupies on a driving lane (clearance.py)
import clearance
probes = clearance.lane_probes(od)
arrays = {k: g[k] for k in g.files}
fg = C.BUILD / "furniture_geom.npz"
if fg.exists():
    f_ = np.load(fg)
    arrays.update({k: f_[k] for k in f_.files})
obst = clearance.obstructions(probes, clearance.layer_triangles(arrays))
anyhit = np.any(list(obst.values()), 0) if obst else np.zeros(len(probes), bool)
REPORT["P_probes"] = int(len(probes))
REPORT["P_by_layer"] = {k: int(v.sum()) for k, v in obst.items() if v.any()}
print(f"   {len(probes)} driving-lane probes; by layer:", REPORT["P_by_layer"])
section("P  obstructions in driving lanes (x, y, z, in_junction)",
        [(round(float(p[0]), 1), round(float(p[1]), 1), round(float(p[2]), 1), bool(p[4])) for p in probes[anyhit]], str)

json.dump(REPORT, open(C.BUILD / "qa_report.json", "w"), indent=1, default=str)
print("\nreport:", C.BUILD / "qa_report.json")

