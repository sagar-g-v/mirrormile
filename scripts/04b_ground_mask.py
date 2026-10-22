"""Ground cover mask for the terrain (runs after 03 / 03b / 04).

The terrain used to show the aerial orthophoto. In downtown that is mostly rooftops, tower shadows
and parked cars, which reads as flat grey mud at street level. Instead the terrain gets tiling
photo materials blended by this mask (same UV as the orthophoto: 0..1 over the map extent):

  R  lawn        OSM parks / grass / gardens / pitches + NAIP NDVI vegetation
  G  dry ground  default for open land (dry beige soil, typical LA verges / lots), brownfields
  B  pavers      courtyards and plazas in built-up blocks, pedestrian areas, squares
  A  asphalt     parking lots

Output: textures/T_LA_Ground_M.png (RGBA, weights sum to 255) + build/ground_mask_debug.png
"""
import importlib
import json

import numpy as np
import shapely
from lxml import etree
from PIL import Image
from rasterio.features import rasterize
from rasterio.transform import Affine, from_bounds
from scipy.ndimage import gaussian_filter, uniform_filter
from shapely.geometry import LineString, Polygon
from shapely.ops import linemerge, polygonize

import config as C

GB = importlib.import_module("03_build_geometry")
VG = importlib.import_module("03b_vegetation")

PX = GB.ORTHO_PX
LAWN, DRY, PAVERS, ASPHALT = range(4)
NDVI_LAWN = 0.20            # smoothed NDVI above this (and not under a building) is lawn
BUILT_UP = 0.30             # building cover in a 60 m window above this -> courtyards are paved

TAG_CLASS = [               # (key, values, class): later rules win
    ("landuse", {"brownfield", "construction", "railway", "industrial"}, DRY),
    ("natural", {"scrub", "sand"}, DRY),
    ("surface", {"sand", "unpaved", "dirt", "gravel"}, DRY),
    ("landuse", {"grass", "recreation_ground", "forest", "meadow", "cemetery", "village_green"}, LAWN),
    ("leisure", {"park", "garden", "pitch", "dog_park", "playground", "golf_course", "stadium"}, LAWN),
    ("surface", {"grass", "artificial_turf"}, LAWN),
    ("highway", {"pedestrian", "footway"}, PAVERS),
    ("place", {"square"}, PAVERS),
    ("leisure", {"outdoor_seating"}, PAVERS),
    ("amenity", {"fountain"}, PAVERS),
    ("surface", {"paving_stones", "concrete", "concrete:plates", "paved", "bricks"}, PAVERS),
    ("amenity", {"parking"}, ASPHALT),
    ("surface", {"asphalt"}, ASPHALT),
]
AREA_KEYS = {"landuse", "leisure", "natural", "place", "amenity"}


def area_polygons():
    """Closed ways and multipolygons that describe ground cover: [(polygon, class)]."""
    root = etree.parse(str(C.OSM_FILE)).getroot()
    ids, lon, lat = [], [], []
    for n in root.iter("node"):
        ids.append(n.get("id")); lon.append(float(n.get("lon"))); lat.append(float(n.get("lat")))
    x, y = GB.TO_MAP.transform(np.array(lon), np.array(lat))
    nodes = dict(zip(ids, zip(x, y)))
    ways, out = {}, []

    def cls_of(tags):
        if "building" in tags or "building:part" in tags or GB.is_underground(tags):
            return None
        is_area = tags.get("area") == "yes" or AREA_KEYS & tags.keys()
        if not is_area:
            return None             # a surface= on a plain street line is not ground cover
        c = None
        for k, vals, v in TAG_CLASS:
            if tags.get(k) in vals:
                c = v
        return c

    for w in root.iter("way"):
        refs = [nd.get("ref") for nd in w.findall("nd")]
        tags = {t.get("k"): t.get("v") for t in w.findall("tag")}
        coords = [nodes[r] for r in refs if r in nodes]
        ways[w.get("id")] = coords
        c = cls_of(tags)
        if c is not None and len(coords) >= 4 and refs[0] == refs[-1]:
            p = shapely.make_valid(Polygon(coords))
            if not p.is_empty:
                out.append((p, c))
    for rel in root.iter("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        c = cls_of(tags)
        if tags.get("type") != "multipolygon" or c is None:
            continue
        outer, inner = [], []
        for m in rel.findall("member"):
            if m.get("type") == "way" and len(ways.get(m.get("ref"), ())) >= 2:
                (inner if m.get("role") == "inner" else outer).append(LineString(ways[m.get("ref")]))
        if not outer:
            continue
        po = shapely.union_all(list(polygonize(linemerge(outer))))
        if inner:
            po = po.difference(shapely.union_all(list(polygonize(linemerge(inner)))))
        if not po.is_empty:
            out.append((shapely.make_valid(po), c))
    return out


def main():
    meta = json.loads(C.META_FILE.read_text())
    x0, y0, x1, y1 = meta["extent"]
    tr = from_bounds(x0, y0, x1, y1, PX, PX)            # row 0 = north, like T_LA_Ortho_D
    px_m = (x1 - x0) / PX
    # built-up blocks: building cover around each pixel
    bld = VG.building_polygons()
    bmask = rasterize([(bld, 1)], out_shape=(PX, PX), transform=tr).astype(np.float32)
    cover = uniform_filter(bmask, size=max(3, int(60 / px_m)))
    cls = np.where(cover > BUILT_UP, PAVERS, DRY).astype(np.uint8)
    # real vegetation (NAIP near-infrared) where no building covers the ground
    ndvi = gaussian_filter(VG.ndvi_grid(tr, PX, PX), 1.2)
    cls[(ndvi > NDVI_LAWN) & (bmask == 0)] = LAWN
    # mapped ground cover overrides (smallest polygons last so they win over enclosing areas)
    polys = sorted(area_polygons(), key=lambda pc: -pc[0].area)
    tagged = np.zeros((PX, PX), bool)
    for p, c in polys:
        # only the pixel window around the polygon (one pixel margin), with the window's own transform: the same
        # pixel centres as the whole raster (381 whole-raster passes took ~25 s)
        bx0, by0, bx1, by1 = p.bounds
        c0, c1 = max(0, int((bx0 - tr.c) / tr.a) - 1), min(PX, int((bx1 - tr.c) / tr.a) + 2)
        r0, r1 = max(0, int((by1 - tr.f) / tr.e) - 1), min(PX, int((by0 - tr.f) / tr.e) + 2)
        if c0 >= c1 or r0 >= r1:
            continue
        win = (slice(r0, r1), slice(c0, c1))
        m = rasterize([(p, 1)], out_shape=(r1 - r0, c1 - c0), transform=tr * Affine.translation(c0, r0)).astype(bool)
        if c == PAVERS:              # planters with real vegetation stay green inside plazas
            m &= ~((ndvi[win] > NDVI_LAWN + 0.05) & (bmask[win] == 0))
        cls[win][m] = c; tagged[win] |= m
    # soft edges: blend the one-hot classes over ~1 m, with a little noise so borders are not straight
    rng = np.random.default_rng(3)
    jitter = gaussian_filter(rng.standard_normal((PX, PX)).astype(np.float32), 3.0)
    jitter *= 0.35 / (jitter.std() + 1e-6)
    W = []
    for k in range(4):
        w = gaussian_filter((cls == k).astype(np.float32), 1.0 / px_m)
        W.append(np.clip(w + (jitter if k in (LAWN, DRY) else 0.0) * w * (1 - w) * 4, 0, None))
    W = np.stack(W, -1)
    W /= np.maximum(W.sum(-1, keepdims=True), 1e-6)
    img = np.round(W * 255).astype(np.uint8)
    img[..., 3] = 255 - img[..., :3].sum(-1).clip(0, 255).astype(np.uint8)   # exact sum
    Image.fromarray(img, "RGBA").save(C.TEX / "T_LA_Ground_M.png")
    pal = np.array([(96, 150, 70), (196, 178, 140), (150, 150, 150), (60, 60, 64)], np.uint8)
    dbg = pal[cls]
    dbg[bmask > 0] = (230, 230, 230)
    Image.fromarray(dbg).resize((2048, 2048), Image.NEAREST).save(C.BUILD / "ground_mask_debug.png")
    share = np.bincount(cls[bmask == 0].ravel(), minlength=4) / max(1, (bmask == 0).sum())
    print(f"ground mask {PX}px ({px_m:.2f} m/px): open ground lawn {share[0]:.0%}, dry {share[1]:.0%}, "
          f"pavers {share[2]:.0%}, parking {share[3]:.0%}; {len(polys)} OSM areas")


if __name__ == "__main__":
    main()
