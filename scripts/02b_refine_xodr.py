"""Refine the netconvert OpenDRIVE (runs after 02, before 03).

1. Normal roads: replace line->line polyline corners (0.3..35 deg) by tangent arcs
   (tangent <= 3 m: road length changes < 1 cm, s-offsets stay valid).
2. Connecting (junction) roads are REGENERATED: a cubic (paramPoly3) from the exact end pose
   of the incoming lane to the exact start pose of the outgoing lane; lane widths taper from
   incoming to outgoing width. Removes the gaps/kinks netconvert leaves at junction entries.
   Connections reversing direction (> 150 deg, indirect U-turns through median links) are removed.
3. Elevation: a robust road-surface field from the lidar DEM - samples every 2 m on all road
   centre lines, 9 m median (rejects lidar artefacts: overhead structures, pits, cliffs at
   structure edges), 4 m mean, evaluated piecewise-linearly -> height is a function of position
   only, so both directions of a street and linked roads agree exactly.
4. Connecting roads: ends pinned to the linked roads, robust field in between
   (03_build_geometry later harmonises overlapping connecting lanes to one junction surface).
"""
import json
from collections import defaultdict
import math

import numpy as np
import rasterio
from lxml import etree
from pyproj import Transformer
from scipy.ndimage import map_coordinates, median_filter
from scipy.spatial import Delaunay, cKDTree

import config as C
from opendrive import OpenDrive, write_elevation

STEP = 2.0
TANGENT = 3.0
MEDIAN_R, MEAN_R = 9.0, 8.0


# ----------------------------------------------------------------------------- 1. corners
def road_half_width(road):
    """Largest |t| reached by any lane border of the road (laneOffset + summed widths)."""
    lanes = road.find("lanes")
    off = max([abs(float(o.get("a"))) for o in lanes.findall("laneOffset")] + [0.0])
    worst = 0.0
    for sec in lanes.findall("laneSection"):
        for side in ("left", "right"):
            se = sec.find(side)
            if se is None:
                continue
            tot = sum(max(float(w.get("a")) for w in ln.findall("width")) for ln in se.findall("lane") if ln.findall("width"))
            worst = max(worst, tot)
    return off + worst


def smooth_corners(tree):
    n = 0
    for road in tree.getroot().findall("road"):
        if road.get("junction", "-1") != "-1":
            continue
        pv = road.find("planView")
        changed = True
        while changed:
            changed = False
            gs = pv.findall("geometry")
            for a, b in zip(gs[:-1], gs[1:]):
                if a[0].tag != "line" or b[0].tag != "line":
                    continue
                ha, hb = float(a.get("hdg")), float(b.get("hdg"))
                th = (hb - ha + math.pi) % (2 * math.pi) - math.pi
                La, Lb = float(a.get("length")), float(b.get("length"))
                if not (math.radians(0.3) < abs(th) < math.radians(35)) or min(La, Lb) < 0.3:
                    continue
                # the arc radius must exceed the widest lane offset, or outer lanes fold over
                R_need = road_half_width(road) + 3.0
                T = max(min(TANGENT, La / 2.5, Lb / 2.5), R_need * math.tan(abs(th) / 2))
                if T > min(La, Lb) / 2.2:
                    continue                      # not enough room: keep the (small) kink
                R = T / math.tan(abs(th) / 2)
                arc_len = R * abs(th)
                sa = float(a.get("s"))
                a.set("length", f"{La - T:.8f}")
                x0 = float(a.get("x")) + (La - T) * math.cos(ha); y0 = float(a.get("y")) + (La - T) * math.sin(ha)
                arc = etree.Element("geometry", s=f"{sa + La - T:.8f}", x=f"{x0:.8f}", y=f"{y0:.8f}",
                                    hdg=f"{ha:.8f}", length=f"{arc_len:.8f}")
                etree.SubElement(arc, "arc", curvature=f"{math.copysign(1 / R, th):.10f}")
                a.addnext(arc)
                b.set("x", f"{float(b.get('x')) + T * math.cos(hb):.8f}")
                b.set("y", f"{float(b.get('y')) + T * math.sin(hb):.8f}")
                b.set("length", f"{Lb - T:.8f}")
                delta = arc_len - 2 * T
                old_b_s = float(b.get("s"))
                b.set("s", f"{sa + La - T + arc_len:.8f}")
                for g in pv.findall("geometry"):
                    if g is not b and float(g.get("s")) > old_b_s + 1e-6:
                        g.set("s", f"{float(g.get('s')) + delta:.8f}")
                road.set("length", f"{float(road.get('length')) + delta:.8f}")
                n += 1
                changed = True
                break
    return n


# ----------------------------------------------------------------------------- 2. junctions
def lane_pose(r, lane_id, at_start):
    """(x, y, heading, inner-border t-side point, width) of a lane at a road end, in travel direction."""
    s = 0.0 if at_start else r.length
    sec = r.sections[0] if at_start else r.sections[-1]
    B, _ = r.borders(sec, np.array([s]))
    if lane_id not in B:
        return None
    ti, to = float(B[lane_id][0][0]), float(B[lane_id][1][0])
    x, y, h = r.ref(s)
    return float(x), float(y), float(h), ti, abs(to - ti)


def hermite_ppoly3(p0, h0, p1, h1):
    """paramPoly3 (normalized) from pose p0/h0 to p1/h1; returns coeffs, arc length."""
    c, s = math.cos(-h0), math.sin(-h0)
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    u1, v1 = dx * c - dy * s, dx * s + dy * c
    dh = (h1 - h0 + math.pi) % (2 * math.pi) - math.pi
    chord = math.hypot(u1, v1)
    p = np.linspace(0, 1, 400)
    best = None
    # tangent magnitude chosen to maximise the minimum turning radius of the curve, so lanes
    # offset from the reference line do not fold (cusps) in tight urban turns
    for f in (0.5, 0.65, 0.8, 0.95, 1.1, 1.25, 1.4, 1.6):
        k = chord * f
        cU = 3 * u1 - 2 * k - k * math.cos(dh); dU = -2 * u1 + k + k * math.cos(dh)
        cV = 3 * v1 - k * math.sin(dh);         dV = -2 * v1 + k * math.sin(dh)
        du = k + 2 * cU * p + 3 * dU * p ** 2;  dv = 2 * cV * p + 3 * dV * p ** 2
        ddu = 2 * cU + 6 * dU * p;              ddv = 2 * cV + 6 * dV * p
        sp = np.hypot(du, dv)
        if sp.min() < 1e-6 * max(chord, 1e-3):
            continue
        kap = np.abs(du * ddv - dv * ddu) / sp ** 3
        rmin = 1.0 / max(kap.max(), 1e-9)
        if best is None or rmin > best[0]:
            best = (rmin, k, cU, dU, cV, dV)
    rmin, k, cU, dU, cV, dV = best
    U = k * p + cU * p ** 2 + dU * p ** 3; V = cV * p ** 2 + dV * p ** 3
    L = float(np.sum(np.hypot(np.diff(U), np.diff(V))))
    return (0.0, k, cU, dU, 0.0, 0.0, cV, dV), L, dh, rmin


def remap_straight_lanes(a, a_start, b, b_start, lanes):
    """Straight-through connection: re-target the connecting lanes' successors so that each lane
    keeps its lateral position (SUMO may shift every lane by one when a lane opens on one side,
    which forces a sharp sideways jog inside the junction). Returns metres of jog removed."""
    def center(road, lid, at_start, rev):
        pz = lane_pose(road, lid, at_start)
        if pz is None:
            return None, None
        x, y, h, ti, w = pz
        tc = ti - w / 2 if lid < 0 else ti + w / 2
        return np.array([x - tc * math.sin(h), y + tc * math.cos(h)]), (h + math.pi if rev else h)
    links = []
    for ln in lanes:
        lk = ln.find("link")
        p_, s_ = (lk.find("predecessor"), lk.find("successor")) if lk is not None else (None, None)
        if p_ is None or s_ is None:
            return 0.0
        links.append((ln, int(p_.get("id")), s_))
    pin0, ha = center(a, links[0][1], a_start, a_start)
    if pin0 is None:
        return 0.0
    _, hb = center(b, int(links[0][2].get("id")), b_start, not b_start)
    if hb is None or abs((hb - ha + math.pi) % (2 * math.pi) - math.pi) > math.radians(20):
        return 0.0
    nrm = np.array([-math.sin(ha), math.cos(ha)])
    sec = b.sections[0] if b_start else b.sections[-1]
    cands = sorted([l.id for l in sec.left + sec.right if l.type in ("driving", "restricted", "bus", "taxi", "HOV")
                    and (l.id < 0) == b_start], key=abs)
    if not cands:
        return 0.0
    lat_in = [float(center(a, pid, a_start, a_start)[0] @ nrm) for _, pid, _ in links]

    def cost(ids):
        return sum(abs(float(center(b, t, b_start, False)[0] @ nrm) - li) for t, li in zip(ids, lat_in))
    cur = [int(s_.get("id")) for _, _, s_ in links]
    if any(c not in cands for c in cur):
        return 0.0
    base = [cands.index(c) for c in cur]
    best_ids, best_cost = cur, cost(cur)
    for m in range(-3, 4):
        idx = [i + m for i in base]
        if m == 0 or min(idx) < 0 or max(idx) >= len(cands):
            continue
        ids = [cands[i] for i in idx]
        cst = cost(ids)
        if cst < best_cost:
            best_ids, best_cost = ids, cst
    gain = cost(cur) - best_cost
    if gain > 1.0 * len(links):
        for (_, _, s_), t in zip(links, best_ids):
            s_.set("id", str(t))
        return gain / len(links)
    return 0.0


def regenerate_junctions(tree):
    od = OpenDrive.__new__(OpenDrive)
    root = tree.getroot()
    from opendrive import Road
    roads = {r.get("id"): r for r in root.findall("road")}
    objs = {rid: Road(el) for rid, el in roads.items()}
    regen, removed, skipped = 0, set(), 0
    dropped_lanes, remapped, infeasible, tight, split = [], [], [], [], []
    next_id = [max(int(r) for r in roads if r.isdigit()) + 1]

    def split_per_lane(rid, el, lanes):
        """Replace a multi-lane connecting road by one single-lane connecting road per lane
        (each gets its own curve, so wide connections through a jog cannot fold)."""
        import copy
        j = root.find(f"junction[@id='{el.get('junction')}']")
        conns = [c for c in j.findall("connection") if c.get("connectingRoad") == rid]
        cid = [max([int(c.get("id")) for c in j.findall("connection") if c.get("id", "").isdigit()] + [0]) + 1]
        out = []
        for ln in lanes:
            lid = ln.get("id")
            nel = copy.deepcopy(el)
            nid = str(next_id[0]); next_id[0] += 1
            nel.set("id", nid); nel.set("name", f"{el.get('name')}_lane{lid}")
            sec = nel.find("lanes").find("laneSection")
            for side in sec:
                for l2 in side.findall("lane"):
                    if l2.get("id") not in ("0", lid):
                        side.remove(l2)
            for l2 in sec.iter("lane"):
                if l2.get("id") == lid:
                    l2.set("id", "-1")
            el.addnext(nel)
            for c in conns:
                for ll in c.findall("laneLink"):
                    if ll.get("to") == lid:
                        nc = etree.SubElement(j, "connection", id=str(cid[0]), incomingRoad=c.get("incomingRoad"),
                                              connectingRoad=nid, contactPoint=c.get("contactPoint", "start"))
                        etree.SubElement(nc, "laneLink", **{"from": ll.get("from"), "to": "-1"})
                        cid[0] += 1
            out.append((nid, nel))
        return out

    queue = list(roads.items())
    while queue:
        rid, el = queue.pop(0)
        if el.get("junction", "-1") == "-1":
            continue
        link = el.find("link")
        pe, se = link.find("predecessor"), link.find("successor")
        if pe is None or se is None or pe.get("elementType") != "road" or se.get("elementType") != "road":
            skipped += 1; continue
        a, b = objs.get(pe.get("elementId")), objs.get(se.get("elementId"))
        if a is None or b is None:
            skipped += 1; continue
        sec_el = el.find("lanes").find("laneSection")
        lanes = [ln for ln in sec_el.iter("lane") if ln.get("id") != "0"]
        if any(int(ln.get("id")) > 0 for ln in lanes):
            skipped += 1; continue
        # vehicle connections: drop the sidewalk lane netconvert appends on the outside. In tight
        # turns a 3 m sidewalk band on the inside cannot follow the curve without folding, and
        # CARLA walkers use the sidewalk meshes / navmesh, not junction sidewalk lanes.
        if any(ln.get("type") in ("driving", "restricted", "bus", "taxi", "HOV", "bidirectional") for ln in lanes):
            for ln in [l for l in lanes if l.get("type") in ("sidewalk", "walking")]:
                ln.getparent().remove(ln)
                dropped_lanes.append((rid, ln.get("id")))
            lanes = [ln for ln in lanes if ln.get("type") not in ("sidewalk", "walking")]
        if not lanes:
            skipped += 1; continue
        lanes.sort(key=lambda ln: -int(ln.get("id")))          # -1, -2, ...
        # incoming lane at the contact end, traversed in its driving direction
        a_start = pe.get("contactPoint") == "start"
        b_start = se.get("contactPoint", "start") == "start"
        jog = remap_straight_lanes(a, a_start, b, b_start, lanes)
        if jog:
            remapped.append((rid, round(jog, 2)))
        first = lanes[0]
        lk = first.find("link")
        pin, sout = (lk.find("predecessor"), lk.find("successor")) if lk is not None else (None, None)
        if pin is None or sout is None:
            skipped += 1; continue
        A = lane_pose(a, int(pin.get("id")), a_start)
        Bp = lane_pose(b, int(sout.get("id")), b_start)
        if A is None or Bp is None:
            skipped += 1; continue
        # the connecting road's reference line = inner border of its lane -1, which must
        # coincide with the inner border of the linked lanes (their t_inner from their ref line)
        def border_point(pose, rev):
            x, y, h, ti, _ = pose
            px, py = x - ti * math.sin(h), y + ti * math.cos(h)
            return (px, py), (h + math.pi if rev else h)
        if (int(pin.get("id")) > 0) != a_start or (int(sout.get("id")) > 0) != (not b_start):
            skipped += 1; continue                 # unusual direction conventions: leave as is
        # reference line through the MIDDLE of the lane group (laneOffset = half width), so
        # outer lanes of wide connections stay close to it and do not fold in curves
        last = lanes[-1]
        llk = last.find("link")
        lin, lout = (llk.find("predecessor"), llk.find("successor")) if llk is not None else (None, None)
        A2 = lane_pose(a, int(lin.get("id")), a_start) if lin is not None else None
        B2 = lane_pose(b, int(lout.get("id")), b_start) if lout is not None else None
        if A2 is None or B2 is None:
            skipped += 1; continue

        def mid_point(p_in, p_out):
            x, y, h, ti, _ = p_in
            _, _, _, ti2, w2 = p_out
            tm = (ti + (ti2 - w2)) / 2             # middle between inner border of first and outer of last
            return (x - tm * math.sin(h), y + tm * math.cos(h)), h, ti - tm
        p0, h0, half0 = mid_point(A, A2)
        p1, h1, half1 = mid_point(Bp, B2)
        coeff, L, dh, rmin = hermite_ppoly3(p0, h0, p1, h1)
        if abs(dh) > math.radians(150) or L < 0.05:
            removed.add(rid); continue
        band = max(abs(half0), abs(half1))
        if rmin < band + 0.1:
            if abs(dh) > math.radians(45):
                # turn that geometrically cannot fit (outgoing lane starts beside the incoming one):
                # the lanes would fold -> drop this movement rather than ship a cusp
                infeasible.append((rid, round(rmin, 1), round(band, 1)))
                removed.add(rid); continue
            if len(lanes) == 1 and rmin < 0.3:
                # single-lane cusp (lane change squeezed into a tiny junction): drop the movement
                infeasible.append((rid, round(rmin, 1), round(band, 1)))
                removed.add(rid); continue
            if len(lanes) > 1:
                for nid, nel in split_per_lane(rid, el, lanes):
                    roads[nid] = nel
                    queue.append((nid, nel))
                split.append((rid, len(lanes)))
                removed.add(rid); continue
            tight.append((rid, round(rmin, 1), round(band, 1)))
        old_L = float(el.get("length"))
        pv = el.find("planView")
        for g in list(pv):
            pv.remove(g)
        g = etree.SubElement(pv, "geometry", s="0", x=f"{p0[0]:.8f}", y=f"{p0[1]:.8f}", hdg=f"{h0:.8f}", length=f"{L:.8f}")
        etree.SubElement(g, "paramPoly3", aU="0", bU=f"{coeff[1]:.8f}", cU=f"{coeff[2]:.8f}", dU=f"{coeff[3]:.8f}",
                         aV="0", bV="0", cV=f"{coeff[6]:.8f}", dV=f"{coeff[7]:.8f}", pRange="normalized")
        el.set("length", f"{L:.8f}")
        lanes_el = el.find("lanes")
        for lo in lanes_el.findall("laneOffset"):
            lanes_el.remove(lo)
        dho = half1 - half0                        # laneOffset tapers like the widths (smoothstep)
        lanes_el.insert(0, etree.Element("laneOffset", s="0", a=f"{half0:.6f}", b="0",
                                         c=f"{3 * dho / L ** 2:.8f}", d=f"{-2 * dho / L ** 3:.8f}"))
        # widths: taper from linked incoming to linked outgoing lane width (smoothstep cubic)
        for ln in lanes:
            lk = ln.find("link")
            p_, s_ = (lk.find("predecessor"), lk.find("successor")) if lk is not None else (None, None)
            wa = lane_pose(a, int(p_.get("id")), a_start)[4] if p_ is not None and lane_pose(a, int(p_.get("id")), a_start) else None
            wb = lane_pose(b, int(s_.get("id")), b_start)[4] if s_ is not None and lane_pose(b, int(s_.get("id")), b_start) else None
            ws = ln.findall("width")
            w_old = float(ws[0].get("a")) if ws else 3.5
            wa = w_old if wa is None else wa; wb = w_old if wb is None else wb
            for w in ws:
                ln.remove(w)
            dw = wb - wa
            w = etree.Element("width", sOffset="0", a=f"{wa:.6f}", b="0", c=f"{3 * dw / L ** 2:.8f}", d=f"{-2 * dw / L ** 3:.8f}")
            ln.insert(1 if ln.find("link") is not None else 0, w)
        # scale s of objects/signals/marks onto the new length
        f = L / old_L if old_L > 1e-6 else 1.0
        for e in el.iter("object", "signal", "roadMark", "speed"):
            if e.get("s") is not None:
                e.set("s", f"{float(e.get('s')) * f:.6f}")
            if e.get("sOffset") is not None and e.tag != "width":
                e.set("sOffset", f"{float(e.get('sOffset')) * f:.6f}")
        regen += 1
    # remove reversing connections and anything that depends on them
    changed = True
    while changed:
        changed = False
        for rid, el in list(roads.items()):
            if rid in removed:
                continue
            lk = el.find("link")
            for k in ("predecessor", "successor"):
                e = lk.find(k) if lk is not None else None
                if e is not None and e.get("elementType") == "road" and e.get("elementId") in removed:
                    if el.get("junction", "-1") != "-1":
                        removed.add(rid); changed = True
                    else:
                        lk.remove(e); changed = True     # normal road: drop the dangling link
    for rid in removed:
        root.remove(roads[rid])
    gone = set(dropped_lanes)
    for j in root.findall("junction"):
        for c in j.findall("connection"):
            if c.get("connectingRoad") in removed:
                j.remove(c); continue
            for ll in c.findall("laneLink"):
                if (c.get("connectingRoad"), ll.get("to")) in gone:
                    c.remove(ll)
            if not c.findall("laneLink"):
                j.remove(c)
    print(f"junctions: removed {len(dropped_lanes)} sidewalk lanes from vehicle connections")
    print(f"junctions: re-mapped {len(remapped)} straight connections to keep lane position {remapped[:6]}")
    print(f"junctions: dropped {len(infeasible)} geometrically infeasible turns (rmin, half band) {infeasible}")
    print(f"junctions: split {len(split)} wide connections into per-lane roads {split}")
    if tight:
        print(f"junctions: {len(tight)} tight single-lane connections kept {tight[:6]}")
    return regen, len(removed), skipped


# ----------------------------------------------------------------------------- levels
def road_level_classifier():
    """-> f(road, s, P) giving 'grade' | 'bridge' | 'tunnel' per sample, from the source OSM ways.
    SUMO concatenates a bridge and its approaches into one edge, so this is decided per sample by
    the nearest source way (net.xml origId) rather than per road."""
    import shapely
    from levels import way_level, DRIVABLE
    osm = etree.parse(str(C.BUILD / "area_ele.osm")).getroot()
    tf = Transformer.from_crs(4326, C.MAP_PROJ, always_xy=True)
    ids, lon, lat = [], [], []
    for n in osm.iter("node"):
        ids.append(n.get("id")); lon.append(float(n.get("lon"))); lat.append(float(n.get("lat")))
    x, y = tf.transform(np.array(lon), np.array(lat))
    xy = dict(zip(ids, zip(x, y)))
    ways = {}
    for w in osm.iter("way"):
        tags = {t.get("k"): t.get("v") for t in w.findall("tag")}
        if tags.get("highway") not in DRIVABLE:
            continue
        pts = [xy[nd.get("ref")] for nd in w.findall("nd") if nd.get("ref") in xy]
        if len(pts) >= 2:
            ways[w.get("id")] = (way_level(tags), shapely.LineString(pts))
    net = etree.parse(str(C.NET_FILE)).getroot()
    orig = {}
    for e in net.iter("edge"):
        ids_ = set()
        for p in e.iter("param"):
            if p.get("key") == "origId":
                ids_.update(p.get("value").split())
        orig[e.get("id")] = [i for i in ids_ if i in ways]

    hw_rank = ["motorway", "trunk", "motorway_link", "trunk_link", "primary", "primary_link", "secondary",
               "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential", "living_street", "service"]
    way_hw = {}
    for w in osm.iter("way"):
        t = w.find("tag[@k='highway']")
        if t is not None:
            way_hw[w.get("id")] = t.get("v")

    def highway(r):
        sid = r.el.find("userData[@code='sumoId']")
        hws = [way_hw.get(i) for i in orig.get(sid.get("value"), [])] if sid is not None else []
        hws = [h for h in hws if h in hw_rank]
        return min(hws, key=hw_rank.index) if hws else "unknown"
    classify_hw = highway

    def classify(r, P):
        sid = r.el.find("userData[@code='sumoId']")
        cand = orig.get(sid.get("value"), []) if sid is not None else []
        levels_ = [ways[i][0] for i in cand]
        if not cand or all(l == "grade" for l in levels_):
            return np.array(["grade"] * len(P))
        if all(l == levels_[0] for l in levels_):
            return np.array([levels_[0]] * len(P))
        lines = [ways[i][1] for i in cand]
        d = np.stack([shapely.distance(ln, shapely.points(P)) for ln in lines])
        lab = np.array(levels_)[np.argmin(d, 0)]
        # mode filter (5 samples) removes flicker at abutments
        out = lab.copy()
        for i in range(len(lab)):
            win = lab[max(0, i - 2):i + 3]
            vals, cnt = np.unique(win, return_counts=True)
            out[i] = vals[np.argmax(cnt)]
        return out
    classify.highway = classify_hw
    return classify


# ----------------------------------------------------------------------------- 3./4. elevation
class Dem:
    def __init__(self):
        with rasterio.open(C.DEM_UTM) as src:
            d = src.read(1).astype(np.float64); self.inv = ~src.transform
        d[d < -1000] = np.nan
        self.d = median_filter(np.where(np.isnan(d), np.nanmedian(d), d), size=3)
        self.tf = Transformer.from_crs(C.MAP_PROJ, C.UTM_EPSG, always_xy=True)

    def __call__(self, P):
        ux, uy = self.tf.transform(P[:, 0], P[:, 1])
        col, row = self.inv * (np.asarray(ux), np.asarray(uy))
        return map_coordinates(self.d, [np.asarray(row) - 0.5, np.asarray(col) - 0.5], order=1, mode="nearest")


class RoadField:
    """Robust, smooth road-surface height field from DEM samples on the road network."""

    def __init__(self, P, z, H):
        """P: sample xy, z: lidar height, H: road heading at the sample. Filtering only uses
        neighbours on the same street axis (|cos dheading| > 0.95): cross streets near a junction
        would otherwise bias steep streets into terraces."""
        t = cKDTree(P)
        c, s = np.cos(H), np.sin(H)

        def robust_line(i, nb, zz, trim):
            """Height at sample i from a robust linear fit z = a + b*u along the street axis
            (u = along-axis offset): unbiased at one-sided ends, outliers (lidar artefacts) trimmed."""
            nb = np.asarray(nb)
            nb = nb[np.abs(c[i] * c[nb] + s[i] * s[nb]) > 0.95]
            u = (P[nb, 0] - P[i, 0]) * c[i] + (P[nb, 1] - P[i, 1]) * s[i]
            v = zz[nb]
            keep = np.ones(len(nb), bool)
            for _ in range(3):
                if keep.sum() < 3 or np.ptp(u[keep]) < 1.0:
                    return float(np.median(v[keep])) if keep.any() else float(zz[i])
                b, a = np.polyfit(u[keep], v[keep], 1)
                res = np.abs(v - (a + b * u))
                new = res < max(trim, 2.5 * np.median(res[keep]))
                if (new == keep).all():
                    break
                keep = new
            return float(a)

        zm = np.array([robust_line(i, nb, z, 0.15) for i, nb in enumerate(t.query_ball_point(P, MEDIAN_R))])
        zf = np.array([robust_line(i, nb, zm, 0.05) for i, nb in enumerate(t.query_ball_point(P, MEAN_R))])
        self.P, self.z, self.tri, self.tree = P, zf, Delaunay(P), t
        self.raw_dev = np.abs(z - zf)

    def __call__(self, Q):
        Q = np.asarray(Q, float)
        simp = self.tri.find_simplex(Q)
        out = self.z[self.tree.query(Q)[1]]                  # nearest outside the hull
        ok = simp >= 0
        if ok.any():
            T = self.tri.transform[simp[ok]]
            b = np.einsum("ijk,ik->ij", T[:, :2], Q[ok] - T[:, 2])
            w = np.column_stack([b, 1 - b.sum(1)])
            out[ok] = np.sum(self.z[self.tri.simplices[simp[ok]]] * w, 1)
        return out


MIN_SEP = 5.7            # 4.3 m clearance + 1.4 m deck
MAX_ADD_G = 0.08
END_KEEP = 25.0          # m: corrections vanish towards chain ends (junction links stay continuous)
RAMP = 60.0              # m: corrections fade in from ground-level (lidar-measured) sections
STRUCT_MAX_G = 0.15      # untagged structure runs steeper than this are level-solve artefacts
STRUCT_MAX_DEV = 12.0    # m: ... or further than this from the measured ground


def road_chains(roads):
    """Maximal chains of non-junction roads joined end to end, either by direct road links or through
    pass-through junctions (exactly two road ends meet: SUMO turns every OSM node into a junction, so
    one bridge / tunnel is many short roads with tiny junctions between them).
    Each chain: [(road id, +1 forward | -1 reversed, gap before this road in m), ...]."""
    by = {r.id: r for r in roads}
    adj = {}
    for r in roads:
        if r.junction != "-1":
            continue
        for end, e in (("start", r.pred), ("end", r.succ)):
            if (e is not None and e.get("elementType") == "road" and e.get("elementId") in by
                    and by[e.get("elementId")].junction == "-1"):
                adj[(r.id, end)] = (e.get("elementId"), e.get("contactPoint", "start"), 0.0)
    inc, glen = defaultdict(set), defaultdict(list)
    for r in roads:
        if r.junction == "-1":
            continue
        glen[r.junction].append(r.length)
        for e in (r.pred, r.succ):
            if e is not None and e.get("elementType") == "road" and e.get("elementId") in by:
                inc[r.junction].add((e.get("elementId"), e.get("contactPoint", "start")))
    for j, ends in inc.items():
        if len(ends) == 2 and len({x[0] for x in ends}) == 2:
            (ra, ea), (rb, eb) = sorted(ends)
            gap = float(np.median(glen[j]))
            adj[(ra, ea)] = (rb, eb, gap); adj[(rb, eb)] = (ra, ea, gap)
    seen, chains = set(), []
    for r in roads:
        if r.junction != "-1" or r.id in seen:
            continue
        rid, d, guard = r.id, 1, {r.id}          # walk back to the chain head
        while True:
            nxt = adj.get((rid, "start" if d > 0 else "end"))
            if not nxt or nxt[0] in guard or nxt[0] in seen:
                break
            rid, d = nxt[0], (-1 if nxt[1] == "start" else 1)
            guard.add(rid)
        chain, gap = [], 0.0
        while rid not in seen:
            seen.add(rid); chain.append((rid, d, gap))
            nxt = adj.get((rid, "end" if d > 0 else "start"))
            if not nxt or nxt[0] in seen:
                break
            rid, d, gap = nxt[0], (1 if nxt[1] == "start" else -1), nxt[2]
        chains.append(chain)
    for r in roads:                              # junction connectors: one-road chains
        if r.junction != "-1":
            chains.append([(r.id, 1, 0.0)])
    return chains


def smoothstep(x):
    x = np.clip(x, 0, 1)
    return x * x * (3 - 2 * x)


def repin_connectors(roads, Pz, S, changed):
    """C1 cubic (height and grade) for every connecting road touching a changed road end."""
    by = {r.id: r for r in roads}
    n = 0
    for r in roads:
        if r.junction == "-1":
            continue
        links = [e for e in (r.pred, r.succ) if e is not None and e.get("elementType") == "road"]
        if not any(e.get("elementId") in changed for e in links):
            continue

        def end_state(link, is_pred):
            rid = link.get("elementId") if link is not None else None
            if rid not in Pz or by[rid].junction != "-1":
                return None
            z, s = Pz[rid], S[rid]
            g0 = (z[1] - z[0]) / max(s[1] - s[0], 1e-9); g1 = (z[-1] - z[-2]) / max(s[-1] - s[-2], 1e-9)
            at_start = link.get("contactPoint", "start") == "start"
            if is_pred:
                return (z[0], -g0) if at_start else (z[-1], g1)
            return (z[0], g0) if at_start else (z[-1], -g1)
        zc0 = r.z(np.array([0.0, r.length]))
        A = end_state(r.pred, True) or (zc0[0], 0.0)
        B = end_state(r.succ, False) or (zc0[1], 0.0)
        s = np.linspace(0, r.length, max(2, int(math.ceil(r.length / 1.0)) + 1))
        L_ = max(r.length, 1e-9); u = s / L_
        zh = (2*u**3 - 3*u**2 + 1) * A[0] + (u**3 - 2*u**2 + u) * L_ * A[1] + (-2*u**3 + 3*u**2) * B[0] + (u**3 - u**2) * L_ * B[1]
        dzh = (6*u**2 - 6*u) / L_ * A[0] + (3*u**2 - 4*u + 1) * A[1] + (-6*u**2 + 6*u) / L_ * B[0] + (3*u**2 - 2*u) * B[1]
        write_elevation(r.el, s, zh, dzh)
        n += 1
    return n


def enforce_clearance(od, runs, iters=25, step=2.0):
    """Grade separation on the final OpenDRIVE profiles. Wherever two unlinked roads cross in plan,
    the upper must clear the lower by MIN_SEP. Only bridge / tunnel / structure sections move (the
    ground-level lidar profiles are measurements). The correction is a raised-cosine bump applied
    along the whole chain of linked roads (continuous across road splits) whose length keeps the
    added grade <= MAX_ADD_G, faded in smoothly from ground-level sections (RAMP) and to zero at
    chain ends (END_KEEP), so junction connections are untouched."""
    import shapely
    roads = [r for r in od.roads]
    by_id = {r.id: r for r in roads}
    S, Pz, lv = {}, {}, {}
    for r in roads:
        s = np.linspace(0, r.length, max(2, int(math.ceil(r.length / step)) + 1))
        S[r.id] = s
        Pz[r.id] = r.z(s)
        rr = runs.get(r.id, [[0, r.length, "grade"]])
        lv[r.id] = np.array([next((x[2] for x in rr if x[0] - 1e-6 <= v <= x[1] + 1e-6), rr[-1][2]) for v in s])
    # chain coordinate c(s) per road; per chain: length, members, mobility (fade-in from grade)
    chain_of, cpos, CH, mob = {}, {}, [], {}
    for ci, chain in enumerate(road_chains(roads)):
        off, cs, gr, members = 0.0, [], [], []
        for rid, d, gap in chain:
            off += gap
            L = by_id[rid].length
            c = off + (S[rid] if d > 0 else L - S[rid])
            chain_of[rid] = ci; cpos[rid] = c
            cs.append(c); gr.append(lv[rid] == "grade"); members.append(rid)
            off += L
        C = np.concatenate(cs); G = np.concatenate(gr)
        dist = np.full(len(C), 1e9)
        if G.any():
            cg = np.sort(C[G])
            if len(cg) == 1:
                dist = np.abs(C - cg[0])
            else:
                j = np.clip(np.searchsorted(cg, C), 1, len(cg) - 1)
                dist = np.minimum(np.abs(C - cg[j - 1]), np.abs(C - cg[j]))
            dist[G] = 0.0
        m = smoothstep(dist / RAMP)                                   # fade in from ground level
        k = 0
        for rid in members:
            n = len(S[rid]); mob[rid] = m[k:k + n]; k += n
        CH.append((off, members))
    linked = defaultdict(set)
    for r in roads:
        for e in (r.pred, r.succ):
            if e is not None and e.get("elementType") == "road":
                linked[r.id].add(e.get("elementId")); linked[e.get("elementId")].add(r.id)
    jun = {r.id: r.junction for r in roads}
    segs, owner, idx = [], [], []
    for r in roads:
        x, y, _ = r.ref(S[r.id])
        P = np.column_stack([x, y])
        for k in range(len(P) - 1):
            segs.append(shapely.LineString(P[k:k + 2])); owner.append(r.id); idx.append(k)
    segs = np.array(segs, dtype=object)
    tree = shapely.STRtree(list(segs))
    owner, idx = np.array(owner), np.array(idx)
    a_all, b_all = tree.query(list(segs), predicate="crosses")
    keep = (a_all < b_all) & (owner[a_all] != owner[b_all])
    pairs = []
    for a, b in zip(a_all[keep], b_all[keep]):
        ra, rb = owner[a], owner[b]
        if rb in linked[ra] or (jun[ra] != "-1" and jun[ra] == jun[rb]) or chain_of[ra] == chain_of[rb]:
            continue
        p = shapely.intersection(segs[a], segs[b])
        if p.geom_type != "Point":
            continue
        tt = []
        for sg in (a, b):
            q0, q1 = np.array(segs[sg].coords[0]), np.array(segs[sg].coords[1])
            tt.append(float(np.clip(np.dot(np.array(p.coords[0]) - q0, q1 - q0) / max(np.dot(q1 - q0, q1 - q0), 1e-9), 0, 1)))
        pairs.append((ra, idx[a], tt[0], rb, idx[b], tt[1]))
    stats = defaultdict(int)
    stats["crossings"] = len(pairs)

    def at(rid, k, t):
        return (Pz[rid][k] + t * (Pz[rid][k + 1] - Pz[rid][k]),
                cpos[rid][k] + t * (cpos[rid][k + 1] - cpos[rid][k]),
                mob[rid][k] + t * (mob[rid][k + 1] - mob[rid][k]))

    def low(za, zb, ka, kb, ra, rb):
        # < 1 m apart = same level: an unconnected crossing in the source data (missing OSM node /
        # wrong layer), not a grade separation to enforce -> counted, never bumped apart
        return 1.0 <= abs(za - zb) < MIN_SEP - 0.05

    stats["low_clearance_before"] = sum(low(at(ra, ka, ta)[0], at(rb, kb, tb)[0], ka, kb, ra, rb)
                                        for ra, ka, ta, rb, kb, tb in pairs)
    unfix = set()
    for it in range(iters):
        moved = 0
        for pi, (ra, ka, ta, rb, kb, tb) in enumerate(pairs):
            za, ca, ma = at(ra, ka, ta); zb, cb, mb = at(rb, kb, tb)
            if not low(za, zb, ka, kb, ra, rb):
                continue
            need = MIN_SEP - abs(za - zb)
            up, lo = ((ra, ca, ma), (rb, cb, mb)) if za >= zb else ((rb, cb, mb), (ra, ca, ma))
            cand = []
            for (rid, cc, mm), sign in ((up, 1.0), (lo, -1.0)):
                Lc, members = CH[chain_of[rid]]
                eff = mm * float(smoothstep(min(cc, Lc - cc) / END_KEEP))
                if eff > 0.05:
                    cand.append((rid, cc, sign, eff))
            if not cand:
                unfix.add(pi); continue
            share = need / len(cand)
            for rid, cc, sign, eff in cand:
                delta = float(np.clip(sign * share / eff, -3 * need, 3 * need))   # undo the local fade
                R = max(45.0, math.pi / 2 * abs(delta) / MAX_ADD_G)
                Lc, members = CH[chain_of[rid]]
                for m_id in members:
                    c = cpos[m_id]
                    d = np.abs(c - cc)
                    if d.min() >= R:
                        continue
                    w = 0.5 * (1 + np.cos(np.pi * np.minimum(d, R) / R))
                    w *= smoothstep(np.minimum(c, Lc - c) / END_KEEP) * mob[m_id]   # zero height AND slope change at chain ends
                    Pz[m_id] = Pz[m_id] + delta * w
            moved += 1
        stats["fix_steps"] += moved
        if not moved:
            break
    stats["low_clearance_after"] = sum(low(at(ra, ka, ta)[0], at(rb, kb, tb)[0], ka, kb, ra, rb)
                                       for ra, ka, ta, rb, kb, tb in pairs)
    stats["unfixable_ground_pairs"] = len(unfix)
    stats["same_level_unconnected"] = sum(abs(at(ra, ka, ta)[0] - at(rb, kb, tb)[0]) < 1.0
                                          and not (lv[ra][ka] == "grade" and lv[rb][kb] == "grade")
                                          for ra, ka, ta, rb, kb, tb in pairs)
    changed = set()
    for rid in by_id:
        if not np.allclose(Pz[rid], by_id[rid].z(S[rid])):
            s = S[rid]; z = Pz[rid]
            g = np.gradient(z, s) if len(s) > 2 else np.full(len(s), (z[-1] - z[0]) / max(s[-1], 1e-9))
            write_elevation(by_id[rid].el, s, z, g)
            changed.add(rid)
    stats["roads_adjusted"] = len(changed)
    stats["connectors_repinned"] = repin_connectors(roads, Pz, S, changed)
    stats["chains_multi_road"] = sum(1 for (Lc, members) in CH if len(members) > 1)
    return dict(stats)


def main():
    tree = etree.parse(str(C.XODR_FILE))
    k = smooth_corners(tree)
    print(f"planView: replaced {k} polyline corners by tangent arcs")
    reg, rem, skip = regenerate_junctions(tree)
    print(f"junctions: regenerated {reg} connecting roads, removed {rem} reversing, left {skip} as-is")
    tree.write(str(C.XODR_FILE), xml_declaration=True, encoding="UTF-8", pretty_print=True)

    od = OpenDrive(C.XODR_FILE)
    dem = Dem()
    classify = road_level_classifier()
    samples, heads, lev = [], [], {}
    for r in od.roads:
        if r.junction != "-1":
            # junction interiors are interpolated from the surrounding approaches (lidar inside
            # intersections often holds structure/opening artefacts)
            continue
        s = np.linspace(0, r.length, max(2, int(math.ceil(r.length / STEP)) + 1))
        h = r.ref(s)[2]
        Pr = r.xyz(s, r.lane_offset(s))[:, :2]
        L = classify(r, Pr)
        lev[r.id] = (s, L)
        g_ = L == "grade"                     # only ground-level roads feed the lidar field
        samples.append(Pr[g_]); heads.append(h[g_])
    P = np.concatenate(samples)
    field = RoadField(P, dem(P), np.concatenate(heads))
    print(f"elevation field: {len(P)} ground-level samples, lidar artefacts removed: "
          f"{int((field.raw_dev > 0.25).sum())} samples deviated > 25 cm (max {field.raw_dev.max():.2f} m)")

    from scipy.ndimage import gaussian_filter1d
    # the level-optimised node profile (levels.py) wins where the ground field disagrees by > 1 m
    # (structures OSM does not tag: approach spans, decks over the road, ...) -- but only while it is
    # physically plausible for a road: grades <= STRUCT_MAX_G and within STRUCT_MAX_DEV of the
    # measured ground. Beyond that the disagreement is an artefact of the network-wide level solve
    # (e.g. a street lifted over a crossing road) and the measured ground wins. Decided per
    # contiguous run, so no level jumps appear inside a road.
    pre = {}
    n_keep = n_rej = 0
    for r in od.roads:
        if r.junction != "-1":
            continue
        s, L = lev[r.id]
        Pxy = r.xyz(s, r.lane_offset(s))[:, :2]
        z = field(Pxy)
        zo_raw = r.z(s)
        cand = (L == "grade") & (np.abs(z - zo_raw) > 1.0)
        idx = np.flatnonzero(cand)
        for run in np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1):
            if not len(run):
                continue
            i0, i1 = max(run[0] - 1, 0), min(run[-1] + 1, len(s) - 1)
            g = np.abs(np.diff(zo_raw[i0:i1 + 1]) / np.maximum(np.diff(s[i0:i1 + 1]), 1e-6))
            if (len(g) and g.max() > STRUCT_MAX_G) or np.abs(zo_raw[run] - z[run]).max() > STRUCT_MAX_DEV:
                cand[run] = False; n_rej += len(run)
            else:
                n_keep += len(run)
        pre[r.id] = (s, L, z, zo_raw, cand)
    print(f"untagged structures: {n_keep} samples kept, {n_rej} rejected as implausible "
          f"(grade > {STRUCT_MAX_G:.0%} or > {STRUCT_MAX_DEV:.0f} m off the ground -> measured ground used)")
    ends, runs, n_lv = {}, {}, {"bridge": 0, "tunnel": 0}
    for r in od.roads:
        if r.junction != "-1":
            continue
        s, L, z, zo_raw, cand = pre[r.id]
        L = np.where(cand, "struct", L)
        ng = L != "grade"
        if ng.any():
            # bridges / tunnels keep netconvert's profile (lidar deck heights / portal grades),
            # smoothed; transitions to ground-level samples are blended over ~6 m
            zo = gaussian_filter1d(r.z(s), 2.0, mode="nearest")
            z = np.where(ng, zo, z)
            trans = np.flatnonzero(np.diff(ng.astype(int)) != 0)
            if len(trans):
                dist = np.min(np.abs(np.arange(len(s))[:, None] - trans[None, :]), 1)
                zs = gaussian_filter1d(z, 1.5, mode="nearest")
                z = np.where(dist <= 3, zs, z)
            for lv in ("bridge", "tunnel"):
                n_lv[lv] += int((L == lv).any())
        g = np.gradient(z, s) if len(s) > 2 else np.full(len(s), (z[-1] - z[0]) / max(r.length, 1e-9))
        write_elevation(r.el, s, z, g)
        ends[r.id] = (z[0], g[0], z[-1], g[-1])
        rr, start = [], 0
        for i in range(1, len(s) + 1):
            if i == len(s) or L[i] != L[start]:
                rr.append([round(float(s[start] if start == 0 else (s[start - 1] + s[start]) / 2), 3),
                           round(float(s[-1] if i == len(s) else (s[i - 1] + s[i]) / 2), 3), str(L[start])])
                start = i
        runs[r.id] = rr
    print(f"levels: {n_lv['bridge']} roads with bridge sections, {n_lv['tunnel']} with tunnel sections")
    nj = 0
    for r in od.roads:
        if r.junction == "-1":
            continue

        def end_state(link, is_pred):
            """(z, grade along this connecting road's direction) at the linked road's contact point."""
            if link is None or link.get("elementType") != "road" or link.get("elementId") not in ends:
                return None
            z0_, g0_, z1_, g1_ = ends[link.get("elementId")]
            at_start = link.get("contactPoint", "start") == "start"
            if is_pred:      # we leave the linked road: its end (forward) or its start (reverse)
                return (z0_, -g0_) if at_start else (z1_, g1_)
            return (z0_, g0_) if at_start else (z1_, -g1_)

        s = np.linspace(0, r.length, max(2, int(math.ceil(r.length / 1.0)) + 1))
        sec = r.sections[0]
        lanes = [l for l in sec.left + sec.right if l.type != "none"]
        if lanes:
            Bd, _ = r.borders(sec, s)
            tc = (Bd[lanes[0].id][0] + Bd[lanes[-1].id][1]) / 2
        else:
            tc = r.lane_offset(s)
        zf = field(r.xyz(s, tc)[:, :2])                 # lidar road surface under the lane group
        A = end_state(r.pred, True) or (zf[0], 0.0)
        B = end_state(r.succ, False) or (zf[-1], 0.0)
        # C1 cubic between the linked roads (height AND grade) ...
        L_ = max(r.length, 1e-9); u = s / L_
        h00, h10, h01, h11 = 2*u**3 - 3*u**2 + 1, u**3 - 2*u**2 + u, -2*u**3 + 3*u**2, u**3 - u**2
        zh = h00 * A[0] + h10 * L_ * A[1] + h01 * B[0] + h11 * L_ * B[1]
        # (the lidar field is NOT used inside junctions: its interpolation triangles there can
        # span to other streets, and intersection lidar holds artefacts. With street heights from
        # the robust axis fit, lanes crossing a junction agree to a few cm; 03 averages them.)
        dzh =(6 * u**2 - 6 * u) / L_ * A[0] + (3 * u**2 - 4 * u + 1) * A[1] + (-6 * u**2 + 6 * u) / L_ * B[0] + (3 * u**2 - 2 * u) * B[1]
        write_elevation(r.el, s, zh, dzh)
        nj += 1
    od.tree.write(str(C.XODR_FILE), xml_declaration=True, encoding="UTF-8", pretty_print=True)
    print(f"elevation: {len(ends)} roads from the field, {nj} connecting roads pinned to linked ends")
    # levels of connecting roads: from the linked road ends (a ramp junction on a viaduct is 'bridge')
    for r in od.roads:
        if r.junction == "-1":
            continue
        lv = set()
        for link, at_end_of_link in ((r.pred, None), (r.succ, None)):
            if link is None or link.get("elementType") != "road" or link.get("elementId") not in runs:
                continue
            rr = runs[link.get("elementId")]
            lv.add(rr[0][2] if link.get("contactPoint", "start") == "start" else rr[-1][2])
        level = "bridge" if "bridge" in lv else ("tunnel" if "tunnel" in lv else "grade")
        runs[r.id] = [[0.0, round(r.length, 3), level]]
    (C.BUILD / "road_levels.json").write_text(json.dumps(runs))
    # final grade separation on the finished profiles (later smoothing cannot undo it)
    od2 = OpenDrive(C.XODR_FILE)
    cs = enforce_clearance(od2, runs)
    od2.tree.write(str(C.XODR_FILE), xml_declaration=True, encoding="UTF-8", pretty_print=True)
    print(f"clearance: {cs}")
    (C.BUILD / "road_types.json").write_text(json.dumps(
        {r.id: classify.highway(r) for r in od.roads if r.junction == "-1"}))


if __name__ == "__main__":
    main()




