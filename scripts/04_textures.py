import math
"""Prepare textures.

* PBR road/sidewalk/curb sets from ambientCG (CC0)  -> textures/T_<name>_{D,N,R}.png/jpg
* Procedural facade atlases (window grid = 3.6 m floors, 3.0 m bays) and roof gravel
* Sidewalk slab joints (1.5 m, US style) baked into the concrete albedo/normal
* Lane paint wear texture
Normal maps are OpenGL convention (Blender). The UE import flips green (DirectX).
"""
import io
import zipfile

import numpy as np
import requests
from PIL import Image, ImageFilter

import config as C

SRC = C.TEX / "src"
SETS = {  # name: (ambientCG id, resolution, output px)
    "Asphalt": ("Asphalt031", "2K", 2048),
    "Sidewalk": ("Concrete047A", "1K", 1024),
    "Curb": ("Concrete034", "1K", 1024),
    "BridgeConc": ("Concrete036", "2K", 1024),     # weathered, water-stained: bridge fascia / underside / piers
    "TunnelTile": ("Tiles010", "1K", 1024),        # white glazed tile, tunnel lower walls (2nd St tunnel)
    "TunnelCeil": ("Concrete028", "1K", 1024),     # board-formed concrete, grimy: tunnel upper walls / ceiling
}
rng = np.random.default_rng(7)


def fetch(asset, res):
    d = SRC / asset
    if d.exists() and any(d.iterdir()):
        return d
    d.mkdir(parents=True, exist_ok=True)
    url = f"https://ambientcg.com/get?file={asset}_{res}-JPG.zip"
    r = requests.get(url, timeout=300, headers={"User-Agent": "osm2carla/1.0"})
    r.raise_for_status()
    with zipfile.ZipFile(io.BytesIO(r.content)) as z:
        for n in z.namelist():
            # images only; ignore anything else in the archive
            if n.lower().endswith((".jpg", ".png")) and "/" not in n and ".." not in n:
                (d / n).write_bytes(z.read(n))
    print(f"{asset}: {len(r.content)/1e6:.1f} MB")
    return d


def find(d, key):
    m = [p for p in d.iterdir() if key in p.name]
    return Image.open(m[0]) if m else None


def pbr_sets():
    for name, (asset, res, px) in SETS.items():
        d = fetch(asset, res)
        col = find(d, "_Color").convert("RGB").resize((px, px), Image.LANCZOS)
        nrm = find(d, "_NormalGL").convert("RGB").resize((px, px), Image.LANCZOS)
        rough = find(d, "_Roughness").convert("L").resize((px, px), Image.LANCZOS)
        ao = find(d, "_AmbientOcclusion")
        if ao is not None:  # bake AO into albedo (UE/FBX has no AO slot by default)
            a = np.asarray(ao.convert("L").resize((px, px)), np.float32) / 255
            col = Image.fromarray((np.asarray(col, np.float32) * (0.55 + 0.45 * a[..., None])).clip(0, 255).astype(np.uint8))
        if name == "Sidewalk":   # neutral grey concrete (source is yellowish) + slab joints
            c = np.asarray(col, np.float32); g = c.mean(2, keepdims=True)
            col = Image.fromarray((g * 0.75 + c * 0.25).clip(0, 255).astype(np.uint8))
            col, nrm = slab_joints(col, nrm)
        if name == "Asphalt":   # weathered asphalt: ~0.10 linear albedo
            c = np.asarray(col, np.float32) * 0.70 + np.array([6, 5, 4])
            col = Image.fromarray(c.clip(0, 255).astype(np.uint8))
        if name == "TunnelCeil":   # soot / exhaust grime
            col = Image.fromarray((np.asarray(col, np.float32) * 0.45).clip(0, 255).astype(np.uint8))
        if name == "TunnelTile":   # traffic film on old glazed tile
            c = np.asarray(col, np.float32)
            col = Image.fromarray((c * 0.78 + np.array([8, 6, 0])).clip(0, 255).astype(np.uint8))
        col.save(C.TEX / f"T_{name}_D.jpg", quality=93)
        nrm.save(C.TEX / f"T_{name}_N.png")
        rough.save(C.TEX / f"T_{name}_R.jpg", quality=93)


def slab_joints(col, nrm):
    """Dark tooled joint along the tile border -> 1 texture tile = one 1.5 m slab."""
    c = np.asarray(col, np.float32).copy(); n = np.asarray(nrm, np.float32).copy()
    px = c.shape[0]; w = max(2, px // 220)
    for sl in (np.s_[:w, :], np.s_[-w:, :], np.s_[:, :w], np.s_[:, -w:]):
        c[sl] *= 0.45
    # normal: groove walls
    n[:w, :, 1] = 90; n[-w:, :, 1] = 165; n[:, :w, 0] = 90; n[:, -w:, 0] = 165
    return Image.fromarray(c.clip(0, 255).astype(np.uint8)), Image.fromarray(n.clip(0, 255).astype(np.uint8))


def noise(h, w, scale, octaves=4):
    out = np.zeros((h, w), np.float32)
    for o in range(octaves):
        s = max(1, int(scale / 2 ** o))
        small = rng.random((h // s + 2, w // s + 2)).astype(np.float32)
        out += np.asarray(Image.fromarray(small).resize((w + s * 2, h + s * 2), Image.BICUBIC))[:h, :w] / 2 ** o
    return (out - out.min()) / (np.ptp(out) + 1e-6)


def facade(name, bay_px, floor_px, wall, glass, win_w, win_h, sill, mullion=False, floors=4, bays=4):
    """Facade tile covering `bays` x `floors` (3.0 m x 3.6 m each). RGB + roughness."""
    W, H = bay_px * bays, floor_px * floors
    img = np.zeros((H, W, 3), np.float32) + np.array(wall, np.float32)
    img *= (0.88 + 0.24 * noise(H, W, 64))[..., None]
    rough = np.full((H, W), 0.85, np.float32)
    for fy in range(floors):
        for bx in range(bays):
            x0 = bx * bay_px + (bay_px - win_w) // 2
            y0 = fy * floor_px + floor_px - sill - win_h
            g = np.array(glass, np.float32) * (0.75 + 0.5 * rng.random())
            grad = np.linspace(1.15, 0.8, win_h)[:, None, None]       # sky reflection gradient
            img[y0:y0 + win_h, x0:x0 + win_w] = g * grad
            rough[y0:y0 + win_h, x0:x0 + win_w] = 0.08
            if mullion:
                img[y0:y0 + win_h, x0 + win_w // 2 - 1:x0 + win_w // 2 + 2] = np.array(wall) * 0.7
            # frame / sill
            img[y0 + win_h:y0 + win_h + 3, x0 - 2:x0 + win_w + 2] = np.array(wall) * 1.15
    Image.fromarray(img.clip(0, 255).astype(np.uint8)).save(C.TEX / f"T_Facade{name}_D.jpg", quality=92)
    Image.fromarray((rough * 255).astype(np.uint8)).save(C.TEX / f"T_Facade{name}_R.jpg", quality=92)


def facades():
    # 1 bay = 3.0 m, 1 floor = 3.6 m; tile = 4 bays x 4 floors = 12 m x 14.4 m (UV metres / tile size)
    bp, fp = 128, 154
    # Glass curtain wall: almost all glass, thin spandrel bands
    facade("Glass", bp, fp, wall=(78, 86, 94), glass=(82, 100, 114), win_w=bp - 6, win_h=fp - 26, sill=8, mullion=True)
    facade("Office", bp, fp, wall=(170, 165, 155), glass=(55, 70, 85), win_w=86, win_h=96, sill=26)
    facade("Brick", bp, fp, wall=(140, 75, 55), glass=(45, 55, 65), win_w=58, win_h=94, sill=30)
    facade("Stucco", bp, fp, wall=(205, 190, 165), glass=(50, 60, 70), win_w=70, win_h=80, sill=34)
    facade("GlassDark", bp, fp, wall=(40, 42, 46), glass=(38, 44, 52), win_w=bp - 6, win_h=fp - 30, sill=10, mullion=True)
    facade("OfficeTan", bp, fp, wall=(196, 172, 135), glass=(48, 58, 66), win_w=70, win_h=104, sill=24)
    # brick courses
    b = np.asarray(Image.open(C.TEX / "T_FacadeBrick_D.jpg"), np.float32)
    for y in range(0, b.shape[0], 6):
        b[y:y + 1] *= 0.7
    Image.fromarray(b.clip(0, 255).astype(np.uint8)).save(C.TEX / "T_FacadeBrick_D.jpg", quality=92)
    photo_facades()


# photographed facades (ambientCG, CC0) replace the procedural ones; daytime variants only.
# tile size in 05 UV_SCALE = floors in the texture x 3.6 m (square textures)
PHOTO_FACADES = {"Glass": "Facade006", "GlassDark": "Facade001", "Office": "Facade019A",
                 "OfficeTan": "Facade020A", "Brick": "Facade018A"}


def photo_facades():
    for name, asset in PHOTO_FACADES.items():
        d = fetch(asset, "2K")
        col = find(d, "_Color").convert("RGB")
        ao = find(d, "_AmbientOcclusion")
        if ao is not None:
            a = np.asarray(ao.convert("L").resize(col.size), np.float32) / 255
            col = Image.fromarray((np.asarray(col, np.float32) * (0.6 + 0.4 * a[..., None])).clip(0, 255).astype(np.uint8))
        col.save(C.TEX / f"T_Facade{name}_D.jpg", quality=92)
        find(d, "_Roughness").convert("L").resize(col.size).save(C.TEX / f"T_Facade{name}_R.jpg", quality=92)


def paint():
    """White paint with subtle wear; R channel = albedo, roughness separate."""
    n = noise(256, 64, 16)
    a = 0.82 + 0.18 * n
    Image.fromarray((a * 255).astype(np.uint8)).save(C.TEX / "T_Paint_Mask.jpg", quality=92)
    Image.new("L", (4, 4), 255).save(C.TEX / "T_WhiteLinear.png")   # default roughness map in UE


def vegetation_textures():
    """RGBA foliage cards + bark + lawn (procedural)."""
    from PIL import ImageDraw
    # palm frond: a central rib with many narrow leaflets fanning out (Washingtonia-like), alpha cut-out
    W, H = 512, 1024
    im = Image.new("RGBA", (W, H), (0, 0, 0, 0)); d = ImageDraw.Draw(im)
    for i in range(70):
        t = i / 70
        y = int(H * (0.05 + 0.9 * t))
        L = int(W * 0.48 * math.sin(math.pi * min(1, 0.15 + t)) )
        g = int(110 + 50 * rng.random()); col = (int(60 + 30 * rng.random()), g, int(35 + 20 * rng.random()), 255)
        for side in (-1, 1):
            d.line([(W // 2, y), (W // 2 + side * L, y - int(0.18 * L))], fill=col, width=6)
    d.line([(W // 2, 0), (W // 2, H)], fill=(120, 110, 60, 255), width=8)
    im.save(C.TEX / "T_PalmFrond_D.png")
    # leaf cluster: overlapping elliptical leaves of varied greens on alpha
    S = 512
    im = Image.new("RGBA", (S, S), (0, 0, 0, 0)); d = ImageDraw.Draw(im)
    for _ in range(900):
        x, y = rng.normal(S / 2, S / 5, 2)
        if math.hypot(x - S / 2, y - S / 2) > S * 0.47:
            continue
        a, b = rng.uniform(10, 22), rng.uniform(5, 10)
        g = int(rng.uniform(70, 140)); col = (int(g * rng.uniform(0.35, 0.6)), g, int(g * rng.uniform(0.2, 0.4)), 255)
        d.ellipse([x - a, y - b, x + a, y + b], fill=col)
    im.save(C.TEX / "T_Leaves_D.png")
    # bark
    n = noise(512, 512, 32)
    v = (0.55 + 0.45 * np.sin(np.linspace(0, 40 * math.pi, 512))[None, :] * 0.3 + 0.4 * n)
    bark = np.stack([95 * v, 80 * v, 62 * v], -1)
    Image.fromarray(bark.clip(0, 255).astype(np.uint8)).save(C.TEX / "T_Bark_D.jpg", quality=92)
    # lawn: fine green noise with darker clumps (1 tile = 2 m)
    n1, n2 = noise(1024, 1024, 8), noise(1024, 1024, 128)
    g = 0.65 + 0.25 * n1 + 0.25 * n2
    lawn = np.stack([62 * g, 112 * g, 40 * g], -1)
    Image.fromarray(lawn.clip(0, 255).astype(np.uint8)).save(C.TEX / "T_Grass_D.jpg", quality=92)


ROOF_STYLES = {   # name: (base RGB, grain, seam spacing px or 0, seam darkening)
    "RoofWhite": ((214, 214, 208), 0.10, 128, 0.88),   # white TPO / cool-roof membrane (very common in LA)
    "RoofLight": ((178, 174, 166), 0.14, 0, 1.0),      # light concrete deck
    "RoofGray":  ((128, 126, 122), 0.22, 0, 1.0),      # gravel ballast
    "RoofDark":  ((70, 68, 66), 0.16, 256, 0.8),       # tar / modified bitumen
}


def roof_textures():
    """Procedural roof materials (1 tile = 10 m). Aerial imagery is not used on roofs: NAIP is not a
    true orthophoto, tall buildings lean by tens of metres, so their roofs would show the street."""
    from PIL import ImageDraw
    S = 1024
    for name, (rgb, grain, seam, dark) in ROOF_STYLES.items():
        n = 0.6 * noise(S, S, 4) + 0.4 * noise(S, S, 64)
        img = np.array(rgb, np.float32)[None, None, :] * (1 - grain + 2 * grain * n[..., None])
        if seam:
            for x in range(0, S, seam):
                img[:, x:x + 2] *= dark
        im = Image.fromarray(img.clip(0, 255).astype(np.uint8))
        d = ImageDraw.Draw(im)
        for _ in range(6):        # stains / patches
            x, y, r = rng.uniform(0, S), rng.uniform(0, S), rng.uniform(20, 90)
            d.ellipse([x - r, y - r * 0.6, x + r, y + r * 0.6], fill=tuple(int(c * 0.9) for c in rgb))
        im = im.filter(ImageFilter.GaussianBlur(1.2))
        im.save(C.TEX / f"T_{name}_D.jpg", quality=92)


# regulatory sign atlas (MUTCD look), 4 x 2 tiles of 256 px; 03e maps plates onto these tiles
SIGN_TILES = ["ONEWAY_R", "ONEWAY_L", "NO_LEFT", "NO_RIGHT", "DO_NOT_ENTER", "BACK", "SPARE1", "SPARE2"]


def sign_atlas():
    from PIL import ImageDraw, ImageFont
    T = 256
    img = Image.new("RGB", (4 * T, 2 * T), (140, 142, 145))
    d = ImageDraw.Draw(img)

    def font(sz):
        for f in ("arialbd.ttf", "ARIALNB.TTF", "DejaVuSans-Bold.ttf"):
            try:
                return ImageFont.truetype(f, sz)
            except OSError:
                continue
        return ImageFont.load_default()

    def origin(name):
        i = SIGN_TILES.index(name)
        return (i % 4) * T, (i // 4) * T

    # ONE WAY: black panel, white arrow, black "ONE WAY" on the arrow (tile holds a 3:1 plate in its middle)
    for name, right in (("ONEWAY_R", True), ("ONEWAY_L", False)):
        ox, oy = origin(name)
        d.rectangle([ox, oy + 85, ox + T - 1, oy + 170], fill=(15, 15, 15))
        xa, xb = (ox + 20, ox + T - 50) if right else (ox + T - 20, ox + 50)
        d.rectangle([min(xa, xb), oy + 108, max(xa, xb), oy + 147], fill=(245, 245, 245))
        tip = (xb + 30) if right else (xb - 30)
        d.polygon([(xb, oy + 95), (xb, oy + 160), (tip, oy + 127)], fill=(245, 245, 245))
        f = font(30)
        tw = d.textlength("ONE WAY", font=f)
        d.text((ox + (T - tw) / 2 - (12 if right else -12), oy + 111), "ONE WAY", fill=(15, 15, 15), font=f)
    # NO LEFT / NO RIGHT TURN: white square, black bent arrow, red circle + slash
    for name, left in (("NO_LEFT", True), ("NO_RIGHT", False)):
        ox, oy = origin(name)
        d.rectangle([ox + 8, oy + 8, ox + T - 9, oy + T - 9], fill=(245, 245, 245), outline=(15, 15, 15), width=6)
        cx, cy = ox + T / 2, oy + T / 2
        d.line([(cx, cy + 70), (cx, cy - 10)], fill=(15, 15, 15), width=22)
        tx = cx - 60 if left else cx + 60
        d.line([(cx, cy - 10), (tx + (10 if left else -10), cy - 10)], fill=(15, 15, 15), width=22)
        d.polygon([(tx - (25 if left else -25), cy - 10), (tx + (15 if left else -15), cy - 45),
                   (tx + (15 if left else -15), cy + 25)], fill=(15, 15, 15))
        d.ellipse([cx - 95, cy - 95, cx + 95, cy + 95], outline=(200, 20, 20), width=16)
        d.line([(cx - 67, cy - 67), (cx + 67, cy + 67)], fill=(200, 20, 20), width=16)
    # DO NOT ENTER: red disc, white bar, white text
    ox, oy = origin("DO_NOT_ENTER")
    d.rectangle([ox + 8, oy + 8, ox + T - 9, oy + T - 9], fill=(245, 245, 245))
    d.ellipse([ox + 20, oy + 20, ox + T - 21, oy + T - 21], fill=(195, 18, 25))
    d.rectangle([ox + 45, oy + 112, ox + T - 46, oy + 144], fill=(245, 245, 245))
    f = font(30)
    for txt, y in (("DO NOT", oy + 60), ("ENTER", oy + 160)):
        tw = d.textlength(txt, font=f); d.text((ox + (T - tw) / 2, y), txt, fill=(245, 245, 245), font=f)
    # BACK: galvanised aluminium
    ox, oy = origin("BACK")
    n = noise(T, T, 32)
    back = (150 + 25 * n).astype(np.uint8)
    img.paste(Image.fromarray(np.dstack([back, back, back + 3])), (ox, oy))
    img.save(C.TEX / "T_Signs_D.png")


if __name__ == "__main__":
    sign_atlas()
    pbr_sets()
    facades()
    paint()
    vegetation_textures()
    roof_textures()
    print(sorted(p.name for p in C.TEX.glob("T_*")))


