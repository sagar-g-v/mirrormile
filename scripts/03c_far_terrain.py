"""Surrounding terrain (to the horizon): hills and mountains around the detailed map.

Two bands, both USGS 3DEP DEM + NAIP imagery:
  near band  (|x|,|y| <= NEAR):  5 m DEM, ~1.6 m/px imagery   -> layer Terrain__Near
  outer band (to FAR_RADIUS):   50 m DEM, ~17 m/px imagery    -> layer Terrain__Far
Grid spacing grows from 15 m at the detailed-map edge to ~600 m at the horizon. Earth curvature
(with standard refraction) is applied so distant mountains sit at the right elevation angle in a
flat-world engine. The inner edge is a skirt tucked 0.6 m under the detailed terrain.
Output: build/far_geom.npz, textures/T_LA_Near_D.jpg, textures/T_LA_Far_D.jpg
"""
import hashlib
import io
import json
import time

import numpy as np
import rasterio
import requests
from PIL import Image
from pyproj import Transformer
from rasterio.transform import from_bounds
from rasterio.warp import Resampling, reproject
from scipy.ndimage import map_coordinates

import config as C

FAR_RADIUS = 35000.0       # m: includes the San Gabriel front range (~25-30 km north-east)
NEAR = 3200.0              # m: half-size of the high-detail surrounding band
R_EARTH, REFRACTION = 6371000.0, 0.13
IMG_PX = 4096
DEM_SVC = "https://elevation.nationalmap.gov/arcgis/rest/services/3DEPElevation/ImageServer/exportImage"
NAIP_SVC = "https://imagery.nationalmap.gov/arcgis/rest/services/USGSNAIPPlus/ImageServer/exportImage"
UA = {"User-Agent": "osm2carla/1.0"}
MAP_CRS = rasterio.crs.CRS.from_proj4(C.MAP_PROJ)


CACHE = C.DATA / "cache" / "far_terrain"   # one file per exact request: a rebuild reads the tiles from disk


def fetch(svc, bbox, size, extra):
    x0, y0, x1, y1 = bbox
    p = dict(bbox=f"{x0},{y0},{x1},{y1}", bboxSR=C.UTM_EPSG, imageSR=C.UTM_EPSG, size=f"{size[0]},{size[1]}",
             format="tiff", f="image", compression="None", **extra)
    key = hashlib.sha256(json.dumps([svc, sorted((k, str(v)) for k, v in p.items())]).encode()).hexdigest()[:32]
    cached = CACHE / f"{key}.tif"
    if cached.exists():
        content = cached.read_bytes()
    else:
        for attempt in range(4):        # the image services answer 502 / 504 under load: retry with a pause
            r = requests.get(svc, params=p, headers=UA, timeout=600)
            if r.status_code not in (502, 503, 504) or attempt == 3:
                break
            time.sleep(30 * (attempt + 1))
        r.raise_for_status()
        content = r.content
        with rasterio.open(io.BytesIO(content)):      # a readable image only
            pass
        CACHE.mkdir(parents=True, exist_ok=True)
        tmp = cached.with_suffix(".part")
        tmp.write_bytes(content)
        tmp.replace(cached)
    with rasterio.open(io.BytesIO(content)) as src:
        return src.read()


def mosaic(svc, bbox, nt, tp, bands, extra):
    """nt x nt tiles of tp px (single large requests exceed the service limit)."""
    out = np.zeros((bands, nt * tp, nt * tp), np.float32)
    tw = (bbox[2] - bbox[0]) / nt
    for j in range(nt):
        for i in range(nt):
            sub = (bbox[0] + i * tw, bbox[3] - (j + 1) * tw, bbox[0] + (i + 1) * tw, bbox[3] - j * tw)
            a = fetch(svc, sub, (tp, tp), extra)
            out[:, j * tp:(j + 1) * tp, i * tp:(i + 1) * tp] = a[:bands, :tp, :tp]
    return out, from_bounds(*bbox, nt * tp, nt * tp)


def to_texture(img, tr, half, sea_mask=None, sea_tr=None):
    dst = np.zeros((3, IMG_PX, IMG_PX), np.float32)
    reproject(img, dst, src_transform=tr, src_crs=f"EPSG:{C.UTM_EPSG}",
              dst_transform=from_bounds(-half, -half, half, half, IMG_PX, IMG_PX), dst_crs=MAP_CRS,
              resampling=Resampling.lanczos)
    rgb = np.moveaxis(dst, 0, 2).clip(0, 255).astype(np.uint8).copy()
    water = rgb.sum(2) == 0
    if sea_mask is not None:
        sm = np.zeros((1, IMG_PX, IMG_PX), np.float32)
        reproject(sea_mask[None].astype(np.float32), sm, src_transform=sea_tr, src_crs=f"EPSG:{C.UTM_EPSG}",
                  dst_transform=from_bounds(-half, -half, half, half, IMG_PX, IMG_PX), dst_crs=MAP_CRS,
                  resampling=Resampling.bilinear)
        water |= sm[0] > 0.5
    rgb[water] = (38, 66, 88)                        # NAIP coverage over the ocean is patchy
    return rgb


def axis(inner_half):
    """Uniform 15 m over the detailed span, slow growth through the near band, fast beyond."""
    xs = list(np.arange(-inner_half, inner_half + 1e-6, 15.0))
    out, x, s = [], inner_half, 15.0
    while x < FAR_RADIUS:
        s *= 1.035 if x < NEAR else 1.09
        x = min(FAR_RADIUS, x + s); out.append(x)
    out = np.array(out)
    return np.concatenate([-out[::-1], np.array(xs), out])


def sampler(dem, tr):
    inv = ~tr
    return lambda ux, uy: map_coordinates(dem, [np.asarray((inv * (ux, uy))[1]) - 0.5,
                                                np.asarray((inv * (ux, uy))[0]) - 0.5], order=1, mode="nearest")


def main():
    meta = json.loads(C.META_FILE.read_text())
    ex = meta["extent"]
    to_utm = Transformer.from_crs(C.MAP_PROJ, C.UTM_EPSG, always_xy=True)
    cx, cy = to_utm.transform(0.0, 0.0)

    # outer DEM (50 m) and near DEM (5 m)
    pad = FAR_RADIUS * 1.05
    bb_far = (cx - pad, cy - pad, cx + pad, cy + pad)
    far_dem, far_tr = mosaic(DEM_SVC, bb_far, 2, 735, 1, dict(pixelType="F32", noData=-9999, interpolation="RSP_BilinearInterpolation"))
    far_dem = far_dem[0].astype(np.float64)
    sea = far_dem < -100
    far_dem[sea] = 0.0
    npad = NEAR * 1.08
    bb_near = (cx - npad, cy - npad, cx + npad, cy + npad)
    near_dem, near_tr = mosaic(DEM_SVC, bb_near, 2, 700, 1, dict(pixelType="F32", noData=-9999, interpolation="RSP_BilinearInterpolation"))
    near_dem = near_dem[0].astype(np.float64)
    near_dem[near_dem < -100] = 0.0
    print(f"DEMs: far {far_dem.shape} ({(bb_far[2] - bb_far[0]) / far_dem.shape[1]:.0f} m), "
          f"near {near_dem.shape} ({(bb_near[2] - bb_near[0]) / near_dem.shape[1]:.1f} m)")
    zf, zn = sampler(far_dem, far_tr), sampler(near_dem, near_tr)

    hx = max(abs(ex[0]), abs(ex[2])); hy = max(abs(ex[1]), abs(ex[3]))
    xs, ys = axis(hx), axis(hy)
    GX, GY = np.meshgrid(xs, ys)
    X, Y = GX.ravel(), GY.ravel()
    ux, uy = to_utm.transform(X, Y)
    ux, uy = np.asarray(ux), np.asarray(uy)
    cheb = np.maximum(np.abs(X), np.abs(Y))
    w = np.clip((cheb - (NEAR - 400)) / 400, 0, 1)        # near -> far DEM blend at the band edge
    Z = zn(ux, uy) * (1 - w) + zf(ux, uy) * w
    d = np.hypot(X, Y)
    Z = Z - d * d / (2 * R_EARTH) * (1 - REFRACTION)      # earth curvature + refraction
    inside = (X >= ex[0] - 1e-6) & (X <= ex[2] + 1e-6) & (Y >= ex[1] - 1e-6) & (Y <= ex[3] + 1e-6)
    Z[inside] -= 0.6                                       # skirt under the detailed terrain
    nx, ny = len(xs), len(ys)
    idx = np.arange(nx * ny).reshape(ny, nx)
    a, b, c, dd = idx[:-1, :-1].ravel(), idx[:-1, 1:].ravel(), idx[1:, 1:].ravel(), idx[1:, :-1].ravel()
    F = np.concatenate([np.stack([a, b, c], 1), np.stack([a, c, dd], 1)])
    P = np.column_stack([X, Y])
    cen = P[F].mean(1)
    hole = (cen[:, 0] > ex[0] + 4) & (cen[:, 0] < ex[2] - 4) & (cen[:, 1] > ex[1] + 4) & (cen[:, 1] < ex[3] - 4)
    F, cen = F[~hole], cen[~hole]
    V = np.column_stack([X, Y, Z])

    # textures
    # 8 x 8 tiles of 500 px (same pixels as 4 x 4 x 1000): an 18 km tile timed out at the service (504)
    img, itr = mosaic(NAIP_SVC, bb_far, 8, 500, 3, dict(bandIds="0,1,2", pixelType="U8", interpolation="RSP_BilinearInterpolation"))
    Image.fromarray(to_texture(img, itr, FAR_RADIUS, sea, far_tr)).save(C.TEX / "T_LA_Far_D.jpg", quality=90)
    img, itr = mosaic(NAIP_SVC, bb_near, 4, 1000, 3, dict(bandIds="0,1,2", pixelType="U8", interpolation="RSP_BilinearInterpolation"))
    Image.fromarray(to_texture(img, itr, NEAR)).save(C.TEX / "T_LA_Near_D.jpg", quality=90)

    near_face = np.max(np.abs(cen), axis=1) <= NEAR
    arrays = {}
    for name, sel, half in (("Terrain__Near", near_face, NEAR), ("Terrain__Far", ~near_face, FAR_RADIUS)):
        UV = np.column_stack([(X + half) / (2 * half), (Y + half) / (2 * half)])
        nb = 4 if name.endswith("Near") else 8
        bx = np.clip(((cen[sel, 0] + half) / (2 * half) * nb).astype(int), 0, nb - 1)
        by = np.clip(((cen[sel, 1] + half) / (2 * half) * nb).astype(int), 0, nb - 1)
        Fs = F[sel]
        for t in np.unique(by * nb + bx):
            f = Fs[(by * nb + bx) == t]
            used, invf = np.unique(f.ravel(), return_inverse=True)
            key = f"{name}|{'N' if name.endswith('Near') else 'F'}{t:02d}"
            arrays[key + "|V"] = V[used]; arrays[key + "|F"] = invf.reshape(-1, 3).astype(np.int32)
            arrays[key + "|UV"] = UV[used].astype(np.float32)
        print(f"{name}: {int(sel.sum())} tris")
    np.savez_compressed(C.BUILD / "far_geom.npz", **arrays)


if __name__ == "__main__":
    main()
