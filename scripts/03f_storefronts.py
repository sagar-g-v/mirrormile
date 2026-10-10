"""Storefronts from real shop data (runs after 03 / 03d).

OpenStreetMap shops / restaurants / cafes / bars / banks / hotels ... (Overpass, cached in
data/pois.json) are matched to the wall of their building that faces the sidewalk. Each gets, on
the ground floor:
  * display glazing with a door (interior lit at night, colour temperature by business type)
  * a fascia sign board with the business's real name, themed by type (restaurant, fast food,
    cafe, bar / nightclub, bank, pharmacy / clinic, hotel, theatre, fashion, grocery, office ...)
  * a tube-light strip under the sign, an awning on cafes / restaurants / bakeries
Street-facing walls of commercial buildings without a mapped business get lit lobby / display
glazing in some bays, so ground floors read brighter than the floors above, as on real streets.

Outputs
  build/storefront_geom.npz   meshes (Building__ShopGlass / ShopSign / ShopTube / Awning layers)
  build/storefronts.json      placements (map frame) for lights in Blender / Unreal
  textures/T_ShopSigns_D.png  sign atlas (one tile per business name)
  textures/T_Shopfront_D.png, T_Shopfront_E.png   glazing atlas: day look / night interior light
  textures/T_Awnings_D.png    awning fabrics
"""
import hashlib
import json
import math
from collections import defaultdict

import numpy as np
import requests
import shapely
from PIL import Image, ImageDraw, ImageFilter, ImageFont
from pyproj import Transformer

import config as C
import importlib

GB = importlib.import_module("03_build_geometry")

# theme: sign background, text, glazing light (rgb), tube light (rgb), awning, glazing width (m)
THEMES = {
    "restaurant": ((120, 18, 22), (250, 236, 205), (255, 178, 110), (255, 200, 140), True, 8.0),
    "fast_food":  ((205, 30, 25), (255, 214, 40), (255, 236, 205), (255, 246, 230), False, 7.0),
    "cafe":       ((54, 38, 28), (240, 222, 190), (255, 190, 125), (255, 205, 150), True, 6.0),
    "bar":        ((14, 10, 18), (255, 70, 190), (190, 80, 255), (255, 60, 170), False, 6.0),
    "bank":       ((18, 38, 92), (245, 245, 250), (226, 235, 255), (230, 238, 255), False, 10.0),
    "health":     ((248, 248, 246), (20, 140, 70), (230, 245, 255), (235, 250, 245), False, 7.0),
    "hotel":      ((16, 16, 18), (226, 186, 92), (255, 196, 130), (255, 210, 150), False, 12.0),
    "theatre":    ((110, 10, 14), (255, 220, 60), (255, 225, 160), (255, 236, 180), False, 12.0),
    "fashion":    ((245, 243, 238), (25, 25, 25), (255, 238, 215), (255, 246, 232), False, 8.0),
    "grocery":    ((250, 250, 250), (200, 24, 30), (232, 240, 255), (240, 246, 255), False, 12.0),
    "culture":    ((236, 234, 228), (30, 30, 30), (255, 236, 210), (255, 240, 220), False, 9.0),
    "office":     ((28, 30, 34), (235, 235, 235), (255, 214, 165), (255, 226, 190), False, 6.0),
    "shop":       ((40, 44, 52), (245, 245, 245), (255, 226, 190), (255, 236, 210), False, 7.0),
    "generic":    (None, None, (255, 214, 168), (255, 226, 190), False, 0.0),
}
GLASS_TILES = list(THEMES)                     # one glazing tile per theme
LIT_GENERIC = 0.55                             # share of generic ground-floor bays lit at night
BAY = 8.0                                      # m, generic frontage bay
GROUND_H, SIGN_H = 3.3, 0.8                    # glazing height, sign board height (m)
OFFSET = 0.04                                  # m in front of the facade (no z-fighting)
GENERIC_NAMES = False                          # True: signs show the business type instead of real names
                                               # (real names are OSM data and may be trademarks)


def theme_of(t):
    a, s = t.get("amenity", ""), t.get("shop", "")
    if a in ("restaurant", "food_court", "biergarten"):
        return "restaurant"
    if a in ("fast_food", "ice_cream"):
        return "fast_food"
    if a == "cafe" or s in ("bakery", "coffee", "pastry", "tea"):
        return "cafe"
    if a in ("bar", "pub", "nightclub", "casino"):
        return "bar"
    if a in ("bank", "bureau_de_change"):
        return "bank"
    if a in ("pharmacy", "clinic", "dentist", "doctors", "veterinary") or "healthcare" in t or s in ("chemist", "optician", "medical_supply"):
        return "health"
    if t.get("tourism") in ("hotel", "hostel", "motel", "guest_house"):
        return "hotel"
    if a in ("theatre", "cinema", "arts_centre"):
        return "theatre"
    if s in ("clothes", "jewelry", "shoes", "cosmetics", "beauty", "boutique", "fashion_accessories", "bag", "watches", "perfumery", "hairdresser", "gift"):
        return "fashion"
    if s in ("convenience", "supermarket", "grocery", "greengrocer", "deli", "alcohol", "beverages", "department_store", "mall"):
        return "grocery"
    if t.get("tourism") in ("museum", "gallery") or a in ("library", "post_office"):
        return "culture"
    if "office" in t:
        return "office"
    return "shop"


def fetch_pois():
    f = C.DATA / "pois.json"
    if f.exists():
        return json.loads(f.read_text())
    s, w, n, e = C.outer_bbox()
    bb = f"{s},{w},{n},{e}"
    q = f"""[out:json][timeout:120];
(
  nwr["shop"]({bb});
  nwr["amenity"~"^(restaurant|cafe|fast_food|bar|pub|biergarten|ice_cream|food_court|nightclub|bank|bureau_de_change|pharmacy|clinic|dentist|doctors|veterinary|cinema|theatre|arts_centre|casino|car_rental|fuel|marketplace|post_office|library)$"]({bb});
  nwr["tourism"~"^(hotel|hostel|motel|guest_house|museum|gallery)$"]({bb});
  nwr["office"]["name"]({bb});
  nwr["craft"]({bb});
  nwr["healthcare"]({bb});
);
out center tags;"""
    for url in ("https://overpass-api.de/api/interpreter", "https://overpass.kumi.systems/api/interpreter"):
        try:
            r = requests.post(url, data={"data": q}, timeout=180, headers={"User-Agent": "mirrormile/1.0"})
            r.raise_for_status()
            els = r.json()["elements"]
            f.write_text(json.dumps(els))
            return els
        except Exception as ex:  # noqa: BLE001
            print("overpass failed:", url, ex)
    return []


# ------------------------------------------------------------------ textures
def _font(sz, narrow=False):
    for f in (("ARIALNB.TTF", "arialbd.ttf") if narrow else ("arialbd.ttf", "ARIALNB.TTF")):
        try:
            return ImageFont.truetype(f, sz)
        except OSError:
            continue
    return ImageFont.load_default()


SIGN_W, SIGN_HPX, SIGN_COLS = 512, 128, 8


def sign_atlas(entries):
    """entries: [(name, theme)] unique -> atlas (4096 wide), tile index per entry."""
    rows = max(1, math.ceil(len(entries) / SIGN_COLS))
    rows_p2 = 1 << (rows - 1).bit_length()
    img = Image.new("RGB", (SIGN_W * SIGN_COLS, SIGN_HPX * rows_p2), (30, 30, 30))
    for i, (name, th) in enumerate(entries):
        bg, fg = THEMES[th][0], THEMES[th][1]
        tile = Image.new("RGB", (SIGN_W, SIGN_HPX), bg)
        d = ImageDraw.Draw(tile)
        d.rectangle([2, 2, SIGN_W - 3, SIGN_HPX - 3], outline=tuple(int(c * 0.6) for c in bg), width=4)
        text = name.upper() if th in ("fast_food", "grocery", "theatre", "bank", "hotel") else name
        sz = 84
        while sz > 22:
            f = _font(sz, narrow=len(text) > 14)
            if d.textlength(text, font=f) <= SIGN_W - 40:
                break
            sz -= 4
        tw = d.textlength(text, font=f)
        pos = ((SIGN_W - tw) / 2, (SIGN_HPX - sz) / 2 - sz * 0.12)
        if th == "bar":                                        # neon: glow halo
            glow = Image.new("RGB", (SIGN_W, SIGN_HPX), (0, 0, 0))
            ImageDraw.Draw(glow).text(pos, text, fill=fg, font=f)
            tile = Image.fromarray(np.clip(np.asarray(tile, np.float32) + np.asarray(glow.filter(ImageFilter.GaussianBlur(7)), np.float32) * 0.9, 0, 255).astype(np.uint8))
            d = ImageDraw.Draw(tile)
        d.text(pos, text, fill=fg, font=f)
        img.paste(tile, ((i % SIGN_COLS) * SIGN_W, (i // SIGN_COLS) * SIGN_HPX))
    img.save(C.TEX / "T_ShopSigns_D.png")
    return img.size


def glazing_atlas():
    """One 512 px tile per theme: aluminium frame + door; day = dark glass, night = lit interior."""
    T = 512
    n = len(GLASS_TILES)
    day = Image.new("RGB", (T * n, T), (40, 44, 50)); night = Image.new("RGB", (T * n, T), (0, 0, 0))
    rs = np.random.default_rng(5)
    for i, th in enumerate(GLASS_TILES):
        light = np.array(THEMES[th][2], np.float32)
        D = np.zeros((T, T, 3), np.float32); E = np.zeros((T, T, 3), np.float32)
        D[:] = (62, 70, 78)                                     # glass by day
        yy = np.linspace(0, 1, T)[:, None, None]
        E[:] = light * (0.55 + 0.45 * (1 - yy))                 # brighter towards the ceiling
        for _ in range(9):                                      # shelves / displays / people: darker blocks
            x0 = rs.integers(20, T - 120); w = rs.integers(40, 140); y0 = rs.integers(int(T * 0.45), T - 60); h = rs.integers(30, 160)
            E[y0:y0 + h, x0:x0 + w] *= rs.uniform(0.25, 0.7)
        for y in (18, 46):                                      # tube lights on the ceiling
            E[y:y + 7, 30:T - 30] = np.minimum(255, light * 1.6)
        if th == "generic":
            E *= 0.75
        # frame: mullions, transom, door
        F = np.zeros((T, T), bool)
        F[:14, :] = F[-14:, :] = True; F[:, :14] = F[:, -14:] = True
        F[int(T * 0.22):int(T * 0.22) + 10, :] = True          # transom
        for x in (int(T * 0.33), int(T * 0.66)):
            F[:, x - 6:x + 6] = True
        F[int(T * 0.22):, int(T * 0.42):int(T * 0.44)] = True; F[int(T * 0.22):, int(T * 0.56):int(T * 0.58)] = True  # door
        D[F] = (34, 35, 38); E[F] = 0
        day.paste(Image.fromarray(D.clip(0, 255).astype(np.uint8)), (i * T, 0))
        night.paste(Image.fromarray(E.clip(0, 255).astype(np.uint8)), (i * T, 0))
    day.save(C.TEX / "T_Shopfront_D.png"); night.save(C.TEX / "T_Shopfront_E.png")


AWNING_COLORS = [((140, 20, 26), (235, 225, 205)), ((24, 70, 40), (235, 228, 210)), ((20, 30, 70), (230, 230, 230)),
                 ((60, 40, 30), (210, 190, 160))]


def awning_atlas():
    T = 256
    img = Image.new("RGB", (T * len(AWNING_COLORS), T))
    for i, (a, b) in enumerate(AWNING_COLORS):
        tile = Image.new("RGB", (T, T), a); d = ImageDraw.Draw(tile)
        for x in range(0, T, 64):
            d.rectangle([x, 0, x + 31, T], fill=b)
        img.paste(tile, (i * T, 0))
    img.save(C.TEX / "T_Awnings_D.png")


# ------------------------------------------------------------------ geometry
class Mesh:
    def __init__(self):
        self.V, self.F, self.UV, self.n = [], [], [], 0

    def quad(self, P, uv):
        self.V.append(np.asarray(P, float)); self.UV.append(np.asarray(uv, float))
        self.F.append(np.array([[0, 1, 2], [0, 2, 3]]) + self.n); self.n += 4

    def arrays(self):
        return (np.concatenate(self.V).astype(np.float32), np.concatenate(self.F).astype(np.int32),
                np.concatenate(self.UV).astype(np.float32))


MESH = defaultdict(Mesh)
EXTENT = None


def wall_quad(layer, a, b, z0, z1, n, uv_rect, off=OFFSET):
    """Vertical quad on the wall segment a->b (outward normal n), z0..z1, UV rect (u0, v0, u1, v1)."""
    a = np.asarray(a) + n * off; b = np.asarray(b) + n * off
    u0, v0, u1, v1 = uv_rect
    P = [(a[0], a[1], z0), (b[0], b[1], z0), (b[0], b[1], z1), (a[0], a[1], z1)]
    MESH[(layer, GB.tile_of(*((a + b) / 2), EXTENT))].quad(P, [(u0, v0), (u1, v0), (u1, v1), (u0, v1)])


def box_on_wall(layer, a, b, z0, z1, n, depth, uv_front, uv_side):
    """Sign board / tube: a box protruding `depth` from the wall; front face gets uv_front."""
    f_a, f_b = np.asarray(a) + n * (OFFSET + depth), np.asarray(b) + n * (OFFSET + depth)
    w_a, w_b = np.asarray(a) + n * OFFSET, np.asarray(b) + n * OFFSET
    k = (layer, GB.tile_of(*((f_a + f_b) / 2), EXTENT))
    u0, v0, u1, v1 = uv_front
    MESH[k].quad([(*f_a, z0), (*f_b, z0), (*f_b, z1), (*f_a, z1)], [(u0, v0), (u1, v0), (u1, v1), (u0, v1)])
    s0, t0, s1, t1 = uv_side
    for P in ([(*w_a, z1), (*w_b, z1), (*f_b, z1), (*f_a, z1)],          # top
              [(*f_a, z0), (*f_b, z0), (*w_b, z0), (*w_a, z0)],          # bottom
              [(*w_a, z0), (*f_a, z0), (*f_a, z1), (*w_a, z1)],          # ends
              [(*f_b, z0), (*w_b, z0), (*w_b, z1), (*f_b, z1)]):
        MESH[k].quad(P, [(s0, t0), (s1, t0), (s1, t1), (s0, t1)])


def awning(a, b, z_top, n, depth, idx):
    a, b = np.asarray(a), np.asarray(b)
    hi_a, hi_b = a + n * OFFSET, b + n * OFFSET
    lo_a, lo_b = a + n * (OFFSET + depth), b + n * (OFFSET + depth)
    u0, u1 = idx / len(AWNING_COLORS), (idx + 1) / len(AWNING_COLORS)
    L = float(np.linalg.norm(b - a))
    k = ("Building__Awning", GB.tile_of(*((a + b) / 2), EXTENT))
    MESH[k].quad([(*lo_a, z_top - 0.9), (*lo_b, z_top - 0.9), (*hi_b, z_top), (*hi_a, z_top)],
                 [(u0, 0), (u0 + (u1 - u0) * min(L / 2, 8), 0), (u0 + (u1 - u0) * min(L / 2, 8), 1), (u0, 1)])
    MESH[k].quad([(*lo_a, z_top - 1.15), (*lo_b, z_top - 1.15), (*lo_b, z_top - 0.9), (*lo_a, z_top - 0.9)],
                 [(u0, 0), (u1, 0), (u1, 0.25), (u0, 0.25)])


def free_span(occ, L, c, w, gap=0.3, min_w=2.4):
    """Free interval of width <= w on [gap, L - gap] closest to centre c, avoiding occupied ones."""
    edges = sorted(occ)
    free, cur = [], gap
    for o0, o1 in edges:
        if o0 - gap > cur:
            free.append((cur, o0 - gap))
        cur = max(cur, o1 + gap)
    if L - gap > cur:
        free.append((cur, L - gap))
    best = None
    for f0, f1 in free:
        ww = min(w, f1 - f0)
        if ww < min_w:
            continue
        cc = float(np.clip(c, f0 + ww / 2, f1 - ww / 2))
        if best is None or abs(cc - c) < best[0]:
            best = (abs(cc - c), cc - ww / 2, cc + ww / 2)
    return None if best is None else (best[1], best[2])


def glass_uv(th, frac0=0.0, frac1=1.0):
    i = GLASS_TILES.index(th); n = len(GLASS_TILES)
    return ((i + frac0) / n, 0.0, (i + frac1) / n, 1.0)


def main():
    global EXTENT
    meta = json.loads(C.META_FILE.read_text())
    if "extent" in meta:
        EXTENT = tuple(meta["extent"])
    else:                                   # 03 still running: bounds of the existing geometry
        g0 = np.load(C.GEOM_FILE)
        V = np.concatenate([g0[k] for k in g0.files if k.startswith("Terrain__Ortho") and k.endswith("|V")])
        EXTENT = (float(V[:, 0].min()), float(V[:, 1].min()), float(V[:, 0].max()), float(V[:, 1].max()))
    box = shapely.box(*EXTENT)
    tf = Transformer.from_crs(4326, C.MAP_PROJ, always_xy=True)
    pois = []
    for x in fetch_pois():
        t = x.get("tags", {})
        lat, lon = (x["lat"], x["lon"]) if "lat" in x else (x["center"]["lat"], x["center"]["lon"])
        if t.get("level") and t["level"].lstrip("-").split(";")[0].isdigit() and int(t["level"].split(";")[0]) >= 1:
            continue                                     # upper-floor business: no shopfront
        if t.get("shop") == "vacant":
            continue
        px, py = tf.transform(lon, lat)
        pois.append((px, py, t, theme_of(t)))
    print(f"businesses: {len(pois)} at street level")
    # street surface (sidewalk + road) as triangle index: point-on-pavement tests, no polygon union
    g = np.load(C.GEOM_FILE)

    def mesh_of(prefixes):
        Vs, Fs, n = [], [], 0
        for k in g.files:
            if k.startswith(prefixes) and k.endswith("|V"):
                Vs.append(g[k]); Fs.append(g[k[:-2] + "|F"] + n); n += len(g[k])
        return GB.MeshZ(np.concatenate(Vs), np.concatenate(Fs))
    paved_z = mesh_of(("SideWalk__Concrete", "Road__Asphalt"))
    walk_z = mesh_of(("SideWalk__Concrete",))

    def on_paved(P):
        return ~np.isnan(paved_z(np.atleast_2d(P), how="max"))
    dem = GB.Dem(C.DEM_UTM, filt=5)
    tiers = GB.load_building_tiers()
    buildings = []
    for raw, tags in GB.osm_polygons():
        if GB.is_underground(tags) or ("building:part" in tags and tags.get("building:part") != "no"):
            continue
        poly = shapely.make_valid(raw).buffer(0)
        if poly.is_empty or not box.contains(poly.centroid) or poly.area < 20:
            continue
        t = tiers.get(GB.building_key(raw))
        h = (t["tiers"][0]["z"] - float(np.min(dem(*np.asarray(poly.exterior.coords if poly.geom_type == "Polygon" else max(shapely.get_parts(poly), key=lambda q: q.area).exterior.coords)[:, :2].T)))) if t else (GB.parse_len(tags.get("height")) or 9.0)
        if h < GROUND_H + SIGN_H + 0.6:
            continue
        for p in shapely.get_parts(poly):
            if p.geom_type == "Polygon" and p.area >= 20:
                p = shapely.geometry.polygon.orient(p.simplify(0.15), 1.0)
                commercial = tags.get("building") in ("commercial", "retail", "office", "hotel", "yes", "mixed_use", "civic") and h >= 7
                buildings.append((p, tags, commercial))
    btree = shapely.STRtree([b[0] for b in buildings])
    # frontage edges: wall segments whose outside (1 m out) is street surface
    front = {}
    E_a, E_b, E_bi = [], [], []
    for bi, (p, tags, com) in enumerate(buildings):
        R = np.asarray(p.exterior.coords)
        for a, b in zip(R[:-1], R[1:]):
            if np.hypot(*(b - a)) >= 2.5:
                E_a.append(a); E_b.append(b); E_bi.append(bi)
    E_a, E_b = np.array(E_a), np.array(E_b)
    D = E_b - E_a; Ls = np.hypot(D[:, 0], D[:, 1])
    N = np.column_stack([D[:, 1], -D[:, 0]]) / Ls[:, None]   # CCW ring: outward = right of travel
    M = (E_a + E_b) / 2
    # facing the street: pavement within 6 m outside (buildings are often set back behind a forecourt),
    # building (not road) just inside
    out_hit = on_paved(M + N * 1.2) | on_paved(M + N * 3.0) | on_paved(M + N * 6.0)
    street = out_hit & ~on_paved(M - N * 0.6)
    for a, b, nrm, L, bi, ok in zip(E_a, E_b, N, Ls, E_bi, street):
        if ok:
            front.setdefault(bi, []).append([a, b, nrm, float(L), []])   # [] = occupied intervals
    names, name_idx = [], {}
    shops = []
    stats = defaultdict(int)
    for px, py, t, th in pois:
        pt = shapely.Point(px, py)
        inside = set(btree.query(pt, predicate="within"))           # business inside this building
        cands = set(btree.query(pt.buffer(25.0))) | inside
        best = None
        for bi in cands:
            for si, (a, b, nrm, L, occ) in enumerate(front.get(bi, [])):
                ab = b - a; u = float(np.clip(np.dot([px - a[0], py - a[1]], ab) / (L * L), 0, 1))
                q = a + ab * u
                dd = float(np.hypot(px - q[0], py - q[1])) - (1000.0 if bi in inside else 0.0)
                if (bi in inside or dd <= 25.0) and (best is None or dd < best[0]):
                    best = (dd, bi, si, u)
        if best is None:
            stats["no_frontage"] += 1; continue
        _, bi, si, u = best
        a, b, nrm, L, occ = front[bi][si]
        w = min(THEMES[th][5], L - 0.6)
        if w < 2.4:
            stats["frontage_too_short"] += 1; continue
        c = float(np.clip(u * L, w / 2 + 0.3, L - w / 2 - 0.3))
        span = free_span(occ, L, c, w)                       # nearest free stretch, never overlapping
        if span is None:
            stats["frontage_full"] += 1; continue
        s0, s1 = span
        occ.append((s0, s1))
        name = (None if GENERIC_NAMES else t.get("name")) or (t.get("amenity") or t.get("shop") or t.get("tourism") or th).replace("_", " ").upper()
        key = (name, th)
        if key not in name_idx:
            name_idx[key] = len(names); names.append(key)
        shops.append((bi, si, s0, s1, th, name_idx[key], name, t))
        stats[th] += 1
    # textures
    aw, ah = sign_atlas(names) if names else (SIGN_W * SIGN_COLS, SIGN_HPX)
    glazing_atlas(); awning_atlas()
    rows = ah // SIGN_HPX
    side_uv = (0.0, 0.0, 1.0 / (SIGN_COLS * 4), 1.0 / (rows * 4))
    out = {"shops": [], "generic": []}

    def place(a, b, nrm, s0, s1, th, z0, sign_idx=None, name=None):
        d = (b - a) / np.linalg.norm(b - a)
        pa, pb = a + d * s0, a + d * s1
        wall_quad("Building__ShopGlass", pa, pb, z0, z0 + GROUND_H, nrm, glass_uv(th, 0, 1))
        rec = dict(theme=th, x=float((pa[0] + pb[0]) / 2), y=float((pa[1] + pb[1]) / 2), z=float(z0),
                   heading=math.degrees(math.atan2(nrm[1], nrm[0])), width=float(s1 - s0))
        if sign_idx is not None:
            col, row = sign_idx % SIGN_COLS, sign_idx // SIGN_COLS
            uv = (col / SIGN_COLS, 1 - (row + 1) / rows, (col + 1) / SIGN_COLS, 1 - row / rows)
            box_on_wall("Building__ShopSign", pa + d * 0.1, pb - d * 0.1, z0 + GROUND_H + 0.12, z0 + GROUND_H + 0.12 + SIGN_H,
                        nrm, 0.14, uv, side_uv)
            box_on_wall("Building__ShopTube", pa + d * 0.25, pb - d * 0.25, z0 + GROUND_H + 0.02, z0 + GROUND_H + 0.07,
                        nrm, 0.22, (0, 0, 1, 1), (0, 0, 1, 1))
            if THEMES[th][4]:
                awning(pa, pb, z0 + GROUND_H - 0.05, nrm, 1.4, int(hashlib.md5(name.encode()).digest()[0]) % len(AWNING_COLORS))
            rec["name"] = name
        return rec

    for bi, si, s0, s1, th, idx, name, t in shops:
        a, b, nrm, L, occ = front[bi][si]
        mid = a + (b - a) / L * (s0 + s1) / 2 + nrm * 1.0
        z0 = walk_z(np.array([mid]), how="near", zref=dem(np.array([mid[0]]), np.array([mid[1]])), tol=3.0)[0]
        if np.isnan(z0):
            z0 = float(dem(np.array([mid[0]]), np.array([mid[1]]))[0])
        out["shops"].append(place(a, b, nrm, s0, s1, th, float(z0), idx, name))
    rs = np.random.default_rng(23)
    for bi, segs in front.items():
        if not buildings[bi][2]:
            continue
        for a, b, nrm, L, occ in segs:
            for s0 in np.arange(0.4, L - BAY + 0.01, BAY):
                s1 = s0 + BAY - 0.4
                if any(s0 < o1 and o0 < s1 for o0, o1 in occ) or rs.random() > LIT_GENERIC:
                    continue
                mid = a + (b - a) / L * (s0 + s1) / 2 + nrm * 1.0
                z0 = walk_z(np.array([mid]), how="near", zref=dem(np.array([mid[0]]), np.array([mid[1]])), tol=3.0)[0]
                if np.isnan(z0):
                    z0 = float(dem(np.array([mid[0]]), np.array([mid[1]]))[0])
                out["generic"].append(place(a, b, nrm, s0, s1, "generic", float(z0)))
    geo = {}
    for (layer, tile), m in MESH.items():
        V, F, UV = m.arrays()
        geo[f"{layer}|{tile}|V"], geo[f"{layer}|{tile}|F"], geo[f"{layer}|{tile}|UV"] = V, F, UV
    np.savez_compressed(C.BUILD / "storefront_geom.npz", **geo)
    (C.BUILD / "storefronts.json").write_text(json.dumps(out))
    print(f"storefronts: {len(out['shops'])} named businesses ({len(names)} sign designs), "
          f"{len(out['generic'])} generic lit ground-floor bays | by theme: {dict(stats)}")


if __name__ == "__main__":
    main()
