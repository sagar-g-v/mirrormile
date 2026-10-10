"""Street furniture: traffic-light mast arms, stop-sign posts, streetlights (runs after 03 / 02d).

Positions come from the final OpenDRIVE, so they agree with what CARLA simulates:
  * traffic lights: one mast-arm pole per signalised approach (OpenDRIVE dynamic <signal>s grouped
    by road + orientation), standing on the sidewalk at the curb, arm over the approach lanes,
    one 3-aspect head per approach lane facing the drivers
  * stop signs: the OpenDRIVE type-206 signals (02d), post on the sidewalk edge
  * streetlights (none are mapped in OSM here): LA cobra-head poles on the sidewalk 0.6 m from the
    curb every 35 m along streets with sidewalks (both sides, staggered, on wide streets),
    every 60 m on freeway shoulders; never inside buildings, near junction mouths or in tunnels

Outputs
  build/street_furniture.json  placements (map frame) for the UE import (CARLA blueprints)
  build/furniture_geom.npz     low-poly preview meshes for Blender (layers Pole__Metal,
                               Pole__LampHead, TrafficLight__SignalHead, TrafficSign__StopSign)
"""
import json
import math
from collections import defaultdict

import numpy as np
import shapely
import shapely.affinity
from scipy.spatial import cKDTree

import config as C
import importlib
from opendrive import OpenDrive

GB = importlib.import_module("03_build_geometry")

DRIVING = {"driving", "entry", "exit", "onRamp", "offRamp", "connectingRamp", "bidirectional"}
WALK = {"sidewalk", "walking"}
CURB = 0.15
LIGHT_SPACING = {"street": 35.0, "freeway": 60.0}
STREET_TYPES = {"trunk", "primary", "secondary", "tertiary", "unclassified", "residential", "living_street",
                "trunk_link", "primary_link", "secondary_link", "tertiary_link"}
FREEWAY_TYPES = {"motorway", "motorway_link"}
END_CLEAR = 12.0        # m kept free of streetlights at road ends (junction mouths)


class Mesh:
    def __init__(self):
        self.V, self.F, self.UV = [], [], []
        self.n = 0

    def add(self, V, F, UV=None):
        V = np.asarray(V, float)
        self.V.append(V); self.F.append(np.asarray(F) + self.n)
        self.UV.append(np.asarray(UV, float) if UV is not None else V[:, :2] * 0.5)
        self.n += len(V)

    def arrays(self):
        return (np.concatenate(self.V).astype(np.float32), np.concatenate(self.F).astype(np.int32),
                np.concatenate(self.UV).astype(np.float32))


MESH = defaultdict(Mesh)        # (layer, tile) -> Mesh
EXTENT = None


def tile(x, y):
    return GB.tile_of(x, y, EXTENT)


def cylinder(layer, p, h, r0, r1=None, seg=8):
    r1 = r0 if r1 is None else r1
    a = np.linspace(0, 2 * np.pi, seg, endpoint=False)
    ring = np.column_stack([np.cos(a), np.sin(a)])
    V = np.concatenate([np.column_stack([p[0] + r0 * ring[:, 0], p[1] + r0 * ring[:, 1], np.full(seg, p[2])]),
                        np.column_stack([p[0] + r1 * ring[:, 0], p[1] + r1 * ring[:, 1], np.full(seg, p[2] + h)]),
                        [[p[0], p[1], p[2] + h]]])
    F = []
    for k in range(seg):
        k1 = (k + 1) % seg
        F += [[k, k1, seg + k1], [k, seg + k1, seg + k], [seg + k, seg + k1, 2 * seg]]
    UV = np.column_stack([np.tile(np.arange(seg) / seg, 2).tolist() + [0.5], [0] * seg + [1] * seg + [1]])
    MESH[(layer, tile(p[0], p[1]))].add(V, F, UV)


def box(layer, c, ex, ey, hx, hy, z0, z1):
    """Oriented closed box (centre c xy, unit axes ex/ey, half sizes, z range)."""
    c, ex, ey = np.asarray(c, float), np.asarray(ex, float), np.asarray(ey, float)
    corners = [c + sx * hx * ex + sy * hy * ey for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    V = np.array([[*q, z0] for q in corners] + [[*q, z1] for q in corners])
    F = [[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7]]
    for k in range(4):
        k1 = (k + 1) % 4
        F += [[k, k1, 4 + k1], [k, 4 + k1, 4 + k]]
    MESH[(layer, tile(c[0], c[1]))].add(V, F)


def arm(layer, p0, p1, r=0.07):
    """Horizontal tube approximated by a thin box from p0 to p1 (3D points at the same z)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1[:2] - p0[:2]; L = float(np.hypot(*d))
    if L < 1e-3:
        return
    ex = d / L; ey = np.array([-ex[1], ex[0]])
    box(layer, (p0[:2] + p1[:2]) / 2, ex, ey, L / 2, r, p0[2] - r, p0[2] + r)


def octagon(layer, c, normal_h, z, r=0.38, th=0.03):
    """Stop-sign plate: vertical octagon facing heading normal_h (radians)."""
    n = np.array([math.cos(normal_h), math.sin(normal_h)]); u = np.array([-n[1], n[0]])
    a = np.pi / 8 + np.arange(8) * np.pi / 4
    pts = [(c[0] + math.cos(t) * r * u[0], c[1] + math.cos(t) * r * u[1], z + math.sin(t) * r) for t in a]
    front = np.array(pts) + np.array([n[0], n[1], 0]) * th
    back = np.array(pts)
    V = np.concatenate([front, back, [[c[0] + n[0] * th, c[1] + n[1] * th, z], [c[0], c[1], z]]])
    F = []
    for k in range(8):
        k1 = (k + 1) % 8
        F += [[16, k, k1], [17, 8 + k1, 8 + k], [k, 8 + k, 8 + k1], [k, 8 + k1, k1]]
    UV = np.column_stack([np.r_[np.cos(a), np.cos(a), 0, 0] * 0.5 + 0.5, np.r_[np.sin(a), np.sin(a), 0, 0] * 0.5 + 0.5])
    MESH[(layer, tile(c[0], c[1]))].add(V, F, UV)


def side_borders(r, s, side):
    """(t at the outer edge of the driving lanes, t at the curb/sidewalk inner edge or None, lanes)
    on one side ('right' = travel +s, 'left' = travel -s) of road r at station s."""
    sec = next((q for q in r.sections if q.s - 1e-6 <= s <= q.e + 1e-6), r.sections[-1])
    lanes = sec.right if side == "right" else sec.left
    if not lanes:
        return None, None, []
    B, off = r.borders(sec, np.array([s]))
    drv = [l for l in lanes if l.type in DRIVING]
    if not drv:
        return None, None, []
    edge = (min if side == "right" else max)(B[l.id][1][0] for l in drv)
    walk = [l for l in lanes if l.type in WALK]
    curb = (B[walk[0].id][0][0] if walk else None)
    centres = [(B[l.id][0][0] + B[l.id][1][0]) / 2 for l in drv]
    return edge, curb, centres


def traffic_lights(od, out):
    groups = defaultdict(list)
    for r in od.roads:
        if r.signals is None:
            continue
        for sg in r.signals.findall("signal"):
            if sg.get("dynamic") == "yes":
                groups[(r.id, sg.get("orientation", "+"))].append(float(sg.get("s")))
    for (rid, orient), ss in groups.items():
        r = od.by_id[rid]
        side = "right" if orient == "+" else "left"
        s = float(np.median(ss))
        edge, curb, centres = side_borders(r, s, side)
        if edge is None:
            continue
        sgn = -1.0 if side == "right" else 1.0                 # outward direction in t
        t_pole = (curb if curb is not None else edge) + sgn * 0.8
        x, y, h = (v[0] for v in r.ref(np.array([s])))
        travel = h if side == "right" else h + math.pi        # driving direction of the approach
        P = r.xyz(np.array([s]), np.array([t_pole]))[0]
        zb = P[2] + (CURB if curb is not None else 0.0)
        top = zb + 6.6
        cylinder("Pole__Metal", (P[0], P[1], zb), 6.9, 0.16, 0.11)
        heads = []
        t_far = min(centres, key=lambda t: abs(t)) if centres else edge
        A = r.xyz(np.array([s]), np.array([t_far - sgn * 0.6]))[0]
        arm("Pole__Metal", (P[0], P[1], top), (A[0], A[1], top))
        face = travel + math.pi                                # heads look at the oncoming drivers
        fx, fy = math.cos(face), math.sin(face)
        for tc in centres:
            Hc = r.xyz(np.array([s]), np.array([tc]))[0]
            box("TrafficLight__SignalHead", (Hc[0], Hc[1]), (fx, fy), (-fy, fx), 0.17, 0.2, top - 1.15, top - 0.08)
            heads.append([float(Hc[0]), float(Hc[1]), float(top - 0.6)])
        out["traffic_lights"].append(dict(road=rid, orientation=orient, s=round(s, 2), x=float(P[0]), y=float(P[1]),
                                          z=float(zb), heading=math.degrees(face), arm=float(np.hypot(A[0] - P[0], A[1] - P[1])),
                                          heads=heads))


def stop_signs(od, out):
    for r in od.roads:
        if r.signals is None:
            continue
        for sg in r.signals.findall("signal"):
            if sg.get("type") != "206":
                continue
            s, t = float(sg.get("s")), float(sg.get("t"))
            side = "right" if sg.get("orientation") == "+" else "left"
            sgn = -1.0 if side == "right" else 1.0
            _, curb, _ = side_borders(r, s, side)
            tt = (curb + sgn * 0.5) if curb is not None else t + sgn * 0.5
            P = r.xyz(np.array([s]), np.array([tt]))[0]
            zb = P[2] + (CURB if curb is not None else 0.0)
            h = r.ref(np.array([s]))[2][0]
            face = (h if side == "right" else h + math.pi) + math.pi
            cylinder("Pole__Metal", (P[0], P[1], zb), 2.5, 0.04)
            octagon("TrafficSign__StopSign", (P[0], P[1]), face, zb + 2.1)
            out["stop_signs"].append(dict(id=sg.get("id"), road=r.id, x=float(P[0]), y=float(P[1]), z=float(zb),
                                          heading=math.degrees(face)))


def street_lights(od, out, types, levels, blocked):
    placed = [[p["x"], p["y"]] for p in out["traffic_lights"] + out["stop_signs"] + out["signs"]]
    tree_pts = list(placed)
    for r in od.roads:
        if r.junction != "-1":
            continue
        cls = types.get(r.id, "")
        kind = "freeway" if cls in FREEWAY_TYPES else ("street" if cls in STREET_TYPES else None)
        if kind is None or r.length < 2 * END_CLEAR + 5:
            continue
        runs = levels.get(r.id, [[0, r.length, "grade"]])
        sp = LIGHT_SPACING[kind]
        wide = None
        for side in ("right", "left"):
            for s in np.arange(END_CLEAR + (sp / 2 if side == "left" else 0.0), r.length - END_CLEAR, sp):
                lv = next((x[2] for x in runs if x[0] - 1e-6 <= s <= x[1] + 1e-6), "grade")
                if lv == "tunnel":
                    continue
                edge, curb, _ = side_borders(r, s, side)
                if edge is None:
                    continue
                if kind == "street":
                    if curb is None:
                        continue                                # no sidewalk on this side
                    if wide is None:
                        e2, _, _ = side_borders(r, s, "left" if side == "right" else "right")
                        wide = e2 is not None and abs(edge - e2) > 14.0
                    if side == "left" and not wide:
                        continue                                # narrow streets: one side only
                sgn = -1.0 if side == "right" else 1.0
                t = (curb + sgn * 0.6) if curb is not None else edge + sgn * 1.4
                P = r.xyz(np.array([s]), np.array([t]))[0]
                if shapely.contains_xy(blocked, P[0], P[1]):
                    continue
                if tree_pts and cKDTree(np.array(tree_pts)).query(P[:2])[0] < 8.0:
                    continue
                zb = P[2] + (CURB if curb is not None else 0.0)
                h = r.ref(np.array([s]))[2][0]
                inward = h + (math.pi / 2 if side == "right" else -math.pi / 2)   # towards the road
                H = 12.0 if kind == "freeway" else 9.0
                L = 2.6 if kind == "freeway" else 2.2
                cylinder("Pole__Metal", (P[0], P[1], zb), H, 0.13, 0.07)
                ux, uy = math.cos(inward), math.sin(inward)
                E = (P[0] + ux * L, P[1] + uy * L, zb + H)
                arm("Pole__Metal", (P[0], P[1], zb + H), E, r=0.05)
                box("Pole__LampHead", (E[0], E[1]), (ux, uy), (-uy, ux), 0.35, 0.16, zb + H - 0.18, zb + H - 0.02)
                out["street_lights"].append(dict(road=r.id, kind=kind, x=float(P[0]), y=float(P[1]), z=float(zb),
                                                 height=H, heading=math.degrees(inward)))
                tree_pts.append([P[0], P[1]])


# ------------------------------------------------------------------ lane movements at approaches
def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def approach_movements(od):
    """{(incoming road, 'end'|'start'): {lane id: set('L'|'S'|'R')}} from the junction connections."""
    root = od.tree.getroot()
    mov = defaultdict(lambda: defaultdict(set))
    for j in root.findall("junction"):
        for c in j.findall("connection"):
            I, cr = od.by_id.get(c.get("incomingRoad")), od.by_id.get(c.get("connectingRoad"))
            if I is None or cr is None or I.junction != "-1":
                continue
            h = cr.ref(np.array([0.0, cr.length]))[2]
            dh = (h[1] - h[0]) if c.get("contactPoint", "start") == "start" else (h[0] - h[1])
            if abs(dh) > 2.6:
                continue                                    # U-turn
            m = "L" if dh > 0.6 else ("R" if dh < -0.6 else "S")
            for ll in c.findall("laneLink"):
                lid = int(ll.get("from"))
                end = "end" if lid < 0 else "start"
                mov[(I.id, end)][lid].add(m)
    return mov


def junction_legs(od):
    """junction id -> [(road, 'start'|'end')] of the non-junction roads meeting it."""
    legs = defaultdict(list)
    for r in od.roads:
        if r.junction != "-1":
            continue
        for end, e in (("start", r.pred), ("end", r.succ)):
            if e is not None and e.get("elementType") == "junction":
                legs[e.get("elementId")].append((r, end))
    return legs


def travel_at(r, end):
    """(station near the junction, unit travel direction INTO the junction, lanes into it)."""
    s = r.length if end == "end" else 0.0
    h = r.ref(np.array([s]))[2][0]
    sec = r.sections[-1] if end == "end" else r.sections[0]
    lanes = [l for l in (sec.right if end == "end" else sec.left) if l.type in DRIVING]
    th = h if end == "end" else h + math.pi
    return s, th, lanes


def outbound_lanes(r, end):
    sec = r.sections[0] if end == "start" else r.sections[-1]
    return [l for l in (sec.right if end == "start" else sec.left) if l.type in DRIVING]


# ------------------------------------------------------------------ pavement legends (paint)
def arrow_shape(moves):
    """Arrow polygon in local metres (x = driver's right, y = forward), lane-centred."""
    from shapely.geometry import LineString, Polygon
    parts = []

    def head(tip, d, L=1.25, W=0.95):
        d = np.asarray(d, float) / np.linalg.norm(d); n = np.array([-d[1], d[0]])
        base = np.asarray(tip) - d * L
        return Polygon([tuple(tip), tuple(base + n * W / 2), tuple(base - n * W / 2)])
    shaft_top = 2.4 if ("L" in moves or "R" in moves) else 3.4
    parts.append(LineString([(0, 0), (0, shaft_top + (0.2 if "S" not in moves else 0))]).buffer(0.13, cap_style="flat"))
    if "S" in moves:
        parts.append(LineString([(0, 0), (0, 3.6)]).buffer(0.13, cap_style="flat")); parts.append(head((0, 4.85), (0, 1)))
    for m, sx in (("L", -1), ("R", 1)):
        if m in moves:
            a = (0, 2.0); b = (sx * 0.75, 2.75)
            parts.append(LineString([a, b]).buffer(0.13, cap_style="flat"))
            parts.append(head((sx * 1.55, 3.55), (sx, 1)))
    return shapely.union_all(parts)


GLYPHS = {"O": [[(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)]], "N": [[(0, 0), (0, 1), (1, 0), (1, 1)]],
          "L": [[(0, 1), (0, 0), (1, 0)]], "Y": [[(0, 1), (0.5, 0.55), (1, 1)], [(0.5, 0.55), (0.5, 0)]],
          "S": [[(1, 1), (0, 1), (0, 0.52), (1, 0.48), (1, 0), (0, 0)]], "T": [[(0, 1), (1, 1)], [(0.5, 1), (0.5, 0)]],
          "P": [[(0, 0), (0, 1), (1, 1), (1, 0.5), (0, 0.5)]]}


def word_shape(word, letter_w=0.55, letter_h=2.4, gap=0.2, stroke=0.13):
    """Elongated stroke letters side by side across the lane (MUTCD-style word marking)."""
    from shapely.geometry import LineString
    total = len(word) * letter_w + (len(word) - 1) * gap
    parts = []
    for i, ch in enumerate(word):
        x0 = -total / 2 + i * (letter_w + gap)
        for pl in GLYPHS[ch]:
            parts.append(LineString([(x0 + u * letter_w, v * letter_h) for u, v in pl]).buffer(stroke / 2, cap_style="square", join_style="mitre"))
    return shapely.union_all(parts)


def paint_shape(r, s, t, th, shape, road_z, layer="RoadLine__White"):
    """Place a local-frame legend at (s, t) of road r, travel heading th; drape onto the asphalt."""
    P0 = r.xyz(np.array([s]), np.array([t]))[0]
    f = np.array([math.cos(th), math.sin(th)]); rt = np.array([math.sin(th), -math.cos(th)])
    g = shapely.affinity.affine_transform(shape, [rt[0], f[0], rt[1], f[1], P0[0], P0[1]])
    zref = float(P0[2])
    ok = [True]

    def zf(Q):
        z = road_z(Q[:, :2], how="near", zref=np.full(len(Q), zref), tol=1.0)
        if np.isnan(z).any():
            ok[0] = False
            z = np.where(np.isnan(z), zref, z)
        return z + GB.MARK_Z
    for part in shapely.get_parts(g):
        res = GB.triangulate(part, zf, lambda Q: Q[:, :2] * 0.5, seg=0.5)
        if res is None:
            continue
        if not ok[0]:
            return False
        V, F, UV = res
        MESH[(layer, tile(P0[0], P0[1]))].add(V, F, UV)
    return True


def pavement_legends(od, out, mov, road_z, stop_roads):
    n_arrow = n_only = n_stop = 0
    for (rid, end), lanes in mov.items():
        r = od.by_id[rid]
        if r.length < 30:
            continue
        s_end, th, lanes_in = travel_at(r, end)
        if not lanes_in:
            continue
        allm = set().union(*lanes.values()) if lanes else set()
        sec = r.sections[-1] if end == "end" else r.sections[0]
        sgn = -1.0 if end == "end" else 1.0                     # towards the road interior
        for l in lanes_in:
            m = lanes.get(l.id, set())
            restricted = m and (m != allm or (len(lanes_in) == 1 and m in ({"L"}, {"R"})))
            if (rid, end) in stop_roads:
                s = s_end + sgn * 7.0
                B, _ = r.borders(sec, np.array([s]))
                tc = (B[l.id][0][0] + B[l.id][1][0]) / 2
                if paint_shape(r, s, tc, th, word_shape("STOP"), road_z):
                    n_stop += 1
                continue
            if not restricted or m == {"L", "S", "R"}:
                continue
            key = "".join(sorted(m))
            s_a = s_end + sgn * 16.0
            B, _ = r.borders(sec, np.array([s_a]))
            tc = (B[l.id][0][0] + B[l.id][1][0]) / 2
            if paint_shape(r, s_a, tc, th, arrow_shape(m), road_z):
                n_arrow += 1
                out["pavement_arrows"].append(dict(road=rid, lane=l.id, moves=key))
                if m in ({"L"}, {"R"}):
                    s_o = s_end + sgn * 8.0
                    B, _ = r.borders(sec, np.array([s_o]))
                    tc = (B[l.id][0][0] + B[l.id][1][0]) / 2
                    n_only += bool(paint_shape(r, s_o, tc, th, word_shape("ONLY"), road_z))
    return n_arrow, n_only, n_stop


# ------------------------------------------------------------------ regulatory signs (exported)
ATLAS_COLS, ATLAS_ROWS = 4, 2
TILE_UV = {"ONEWAY_R": (0, 85, 256, 170), "ONEWAY_L": (0, 85, 256, 170), "NO_LEFT": (8, 8, 248, 248),
           "NO_RIGHT": (8, 8, 248, 248), "DO_NOT_ENTER": (8, 8, 248, 248), "BACK": (0, 0, 256, 256)}
TILE_IDX = {"ONEWAY_R": 0, "ONEWAY_L": 1, "NO_LEFT": 2, "NO_RIGHT": 3, "DO_NOT_ENTER": 4, "BACK": 5}


def atlas_uv(name, uu, vv):
    """uu, vv in [0,1] across the tile's sign area (vv = 0 bottom) -> atlas UV."""
    i = TILE_IDX[name]; x0, y0, x1, y1 = TILE_UV[name]
    px = (i % ATLAS_COLS) * 256 + x0 + uu * (x1 - x0)
    py = (i // ATLAS_COLS) * 256 + y1 - vv * (y1 - y0)
    return px / (256 * ATLAS_COLS), 1 - py / (256 * ATLAS_ROWS)


def sign_plate(c, normal_h, zc, w, h, name, th=0.025):
    """Plate box: front face (towards normal_h) shows the atlas tile, other faces the galvanised back."""
    n = np.array([math.cos(normal_h), math.sin(normal_h)]); u = np.array([-n[1], n[0]])   # u = viewer's right
    c = np.asarray(c, float)
    corners = lambda off: [c + off * n - u * w / 2, c + off * n + u * w / 2]
    f0, f1 = corners(th / 2); b0, b1 = corners(-th / 2)
    V = np.array([[*f0, zc - h / 2], [*f1, zc - h / 2], [*f1, zc + h / 2], [*f0, zc + h / 2],
                  [*b1, zc - h / 2], [*b0, zc - h / 2], [*b0, zc + h / 2], [*b1, zc + h / 2]])
    k = tile(c[0], c[1])
    front_uv = [atlas_uv(name, 0, 0), atlas_uv(name, 1, 0), atlas_uv(name, 1, 1), atlas_uv(name, 0, 1)]
    back_uv = [atlas_uv("BACK", 0, 0), atlas_uv("BACK", 1, 0), atlas_uv("BACK", 1, 1), atlas_uv("BACK", 0, 1)]
    MESH[("TrafficSign__SignAtlas", k)].add(V[:4], [[0, 1, 2], [0, 2, 3]], front_uv)
    MESH[("TrafficSign__SignAtlas", k)].add(V[4:], [[0, 1, 2], [0, 2, 3]], back_uv)
    rim = [(0, 1, 4, 5), (1, 2, 7, 4), (2, 3, 6, 7), (3, 0, 5, 6)]
    for a, b, bb, aa in rim:
        MESH[("TrafficSign__SignAtlas", k)].add(V[[a, b, bb, aa]], [[0, 2, 1], [0, 3, 2]], back_uv)


def post_with_sign(P, zb, normal_h, name, w, h, top=2.6):
    cylinder("Pole__SignPost", (P[0], P[1], zb), top, 0.035)
    back = np.array([math.cos(normal_h), math.sin(normal_h)]) * -0.05
    sign_plate((P[0] + back[0], P[1] + back[1]), normal_h, zb + top - h / 2 - 0.05, w, h, name)


def curb_point(r, s, side_sign, lanes_side):
    """Point on the sidewalk just behind the curb on one side (side_sign -1 = right of +s)."""
    side = "right" if side_sign < 0 else "left"
    edge, curb, _ = side_borders(r, s, side)
    if edge is None:
        sec = next((q for q in r.sections if q.s - 1e-6 <= s <= q.e + 1e-6), r.sections[-1])
        B, off = r.borders(sec, np.array([s]))
        ls = sec.right if side == "right" else sec.left
        edge = (min if side == "right" else max)([B[l.id][1][0] for l in ls] or [off[0]])
    t = (curb + side_sign * 0.5) if curb is not None else edge + side_sign * 0.8
    P = r.xyz(np.array([s]), np.array([t]))[0]
    return P, P[2] + (CURB if curb is not None else 0.0)


def dual_carriageways(od):
    """One-way roads that are one carriageway of a divided street: an opposite-direction one-way
    road (same name when named) runs parallel within 35 m."""
    info = []
    for r in od.roads:
        if r.junction != "-1" or r.length < 5:
            continue
        sec = r.sections[len(r.sections) // 2]
        rt = any(l.type in DRIVING for l in sec.right); lt = any(l.type in DRIVING for l in sec.left)
        if rt == lt:
            continue
        x, y, h = (v[0] for v in r.ref(np.array([r.length / 2])))
        info.append((r.id, r.name, x, y, h if rt else h + math.pi))
    if not info:
        return set()
    P = np.array([[i[2], i[3]] for i in info])
    tree = cKDTree(P)
    dual = set()
    for k, (rid, name, x, y, u) in enumerate(info):
        for j in tree.query_ball_point([x, y], 35.0):
            if j == k:
                continue
            rid2, name2, _, _, u2 = info[j]
            if math.cos(u - u2) < -0.8 and (not name or not name2 or name == name2):
                dual.add(rid); break
    return dual


def regulatory_signs(od, out, mov, blocked, occupied, types):
    legs = junction_legs(od)
    dual = dual_carriageways(od)
    occ = list(occupied)

    def free(P):
        if shapely.contains_xy(blocked, P[0], P[1]):
            return False
        if occ and cKDTree(np.array(occ)).query(P[:2])[0] < 2.0:
            return False
        occ.append([P[0], P[1]])
        return True
    cnt = defaultdict(int)
    for j, lst in legs.items():
        if len(lst) < 3:
            continue
        cen = np.mean([r.xyz(np.array([r.length if e == "end" else 0.0]), np.zeros(1))[0, :2] for r, e in lst], 0)
        for r, end in lst:
            s_end, th, lanes_in = travel_at(r, end)
            if not lanes_in or r.length < 25:
                continue
            allm = set().union(*mov.get((r.id, end), {}).values()) if mov.get((r.id, end)) else set()
            if not allm:
                continue
            for m, name, lo, hi in (("L", "NO_LEFT", 45, 135), ("R", "NO_RIGHT", -135, -45)):
                if m in allm:
                    continue
                exists = False
                for o, oe in lst:
                    if o.id == r.id or not outbound_lanes(o, oe):
                        continue
                    so = min(15.0, o.length) if oe == "start" else max(o.length - 15.0, 0)
                    q = o.xyz(np.array([so]), np.zeros(1))[0, :2]
                    a = math.degrees(wrap(math.atan2(q[1] - cen[1], q[0] - cen[0]) - th))
                    if lo < a < hi:
                        exists = True; break
                if not exists:
                    continue
                s = s_end - 6.0 if end == "end" else 6.0
                P, zb = curb_point(r, s, -1.0 if end == "end" else 1.0, lanes_in)
                if free(P):
                    post_with_sign(P, zb, th + math.pi, name, 0.76, 0.76)
                    out["signs"].append(dict(type=name, x=float(P[0]), y=float(P[1]), z=float(zb), heading=math.degrees(th + math.pi)))
                    cnt[name] += 1
        # one-way streets: ONE WAY at the mouth where traffic leaves the junction, DO NOT ENTER where it arrives
        for o, oe in lst:
            sec = o.sections[0] if oe == "start" else o.sections[-1]
            right = [l for l in sec.right if l.type in DRIVING]; left = [l for l in sec.left if l.type in DRIVING]
            if bool(right) == bool(left) or o.length < 20:
                continue
            ocls = types.get(o.id, "")
            if ocls == "motorway":
                continue                                   # freeway mainline: no street signs
            is_ramp = ocls.endswith("_link") and ocls.startswith(("motorway", "trunk"))
            if o.id in dual and not is_ramp:
                continue                                   # carriageway of a divided street
            travel_pos = bool(right)                      # traffic travels +s
            s = 4.0 if oe == "start" else o.length - 4.0
            h = o.ref(np.array([s]))[2][0]
            u = h if travel_pos else h + math.pi           # travel direction
            leaving_junction = (oe == "start") == travel_pos
            if leaving_junction and not is_ramp:
                for side_sign in (-1.0, 1.0):
                    P, zb = curb_point(o, s, side_sign, None)
                    nh = h + (math.pi / 2 if side_sign > 0 else -math.pi / 2)   # facing away from the street axis
                    vright = np.array([-math.sin(nh), math.cos(nh)])             # viewer's right when facing the plate
                    name = "ONEWAY_R" if np.dot(vright, [math.cos(u), math.sin(u)]) > 0 else "ONEWAY_L"
                    if free(P):
                        post_with_sign(P, zb, nh, name, 0.91, 0.30, top=2.9)
                        out["signs"].append(dict(type="ONE_WAY", x=float(P[0]), y=float(P[1]), z=float(zb), heading=math.degrees(nh)))
                        cnt["ONE_WAY"] += 1
            elif not leaving_junction:
                # a wrong-way driver would enter travelling against u: sign on their right, facing them
                wrong = u + math.pi
                side_sign = 1.0 if travel_pos else -1.0      # their right = left of the legal travel
                P, zb = curb_point(o, s, side_sign, None)
                if free(P):
                    post_with_sign(P, zb, wrong + math.pi, "DO_NOT_ENTER", 0.76, 0.76)
                    out["signs"].append(dict(type="DO_NOT_ENTER", x=float(P[0]), y=float(P[1]), z=float(zb), heading=math.degrees(wrong + math.pi)))
                    cnt["DO_NOT_ENTER"] += 1
    return dict(cnt)


def main():
    global EXTENT
    meta = json.loads(C.META_FILE.read_text())
    od = OpenDrive(C.XODR_FILE)
    if "extent" in meta:
        EXTENT = tuple(meta["extent"])
    else:                                    # 03 not run yet: tile grid from the road network bounds
        xs = np.concatenate([r.X for r in od.roads]); ys = np.concatenate([r.Y for r in od.roads])
        EXTENT = (float(xs.min()) - 50, float(ys.min()) - 50, float(xs.max()) + 50, float(ys.max()) + 50)
    types = json.loads((C.BUILD / "road_types.json").read_text())
    levels = json.loads((C.BUILD / "road_levels.json").read_text())
    bld = [shapely.make_valid(p) for p, t in GB.osm_polygons() if not GB.is_underground(t)]
    blocked = shapely.union_all([b for b in bld if not b.is_empty])
    shapely.prepare(blocked)
    out = {"traffic_lights": [], "stop_signs": [], "street_lights": [], "signs": [], "pavement_arrows": [],
           "note": "map frame (OpenDRIVE x, y; UE: y = -y, yaw = -heading), metres / degrees"}
    traffic_lights(od, out)
    stop_signs(od, out)
    g0 = np.load(C.GEOM_FILE)
    Vs, Fs, n = [], [], 0
    for k in g0.files:
        if k.startswith("Road__Asphalt") and k.endswith("|V"):
            Vs.append(g0[k]); Fs.append(g0[k[:-2] + "|F"] + n); n += len(g0[k])
    road_z = GB.MeshZ(np.concatenate(Vs), np.concatenate(Fs))
    mov = approach_movements(od)
    stop_roads = set()
    for r in od.roads:
        if r.signals is not None:
            for sg in r.signals.findall("signal"):
                if sg.get("type") == "206":
                    stop_roads.add((r.id, sg.get("id").rsplit("_", 1)[-1]))
    n_arrow, n_only, n_stop = pavement_legends(od, out, mov, road_z, stop_roads)
    occupied = [[p["x"], p["y"]] for p in out["traffic_lights"] + out["stop_signs"]]
    sign_counts = regulatory_signs(od, out, mov, blocked, occupied, types)
    street_lights(od, out, types, levels, blocked)
    (C.BUILD / "street_furniture.json").write_text(json.dumps(out))
    g = {}
    for (layer, t), m in MESH.items():
        V, F, UV = m.arrays()
        g[f"{layer}|{t}|V"], g[f"{layer}|{t}|F"], g[f"{layer}|{t}|UV"] = V, F, UV
    np.savez_compressed(C.BUILD / "furniture_geom.npz", **g)
    print(f"street furniture: {len(out['traffic_lights'])} traffic-light poles "
          f"({sum(len(p['heads']) for p in out['traffic_lights'])} heads), {len(out['stop_signs'])} stop signs, "
          f"{len(out['street_lights'])} streetlights "
          f"({sum(p['kind'] == 'freeway' for p in out['street_lights'])} freeway)")
    print(f"pavement legends: {n_arrow} turn arrows, {n_only} ONLY, {n_stop} STOP | regulatory signs: {sign_counts}")


if __name__ == "__main__":
    main()
