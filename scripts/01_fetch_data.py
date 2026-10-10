"""Download open data for the configured area.

* OpenStreetMap (Overpass API)                       -> data/area.osm      (ODbL)
* USGS 3DEP 1 m bare-earth DEM (lidar derived)       -> data/dem_utm.tif   (public domain)
* USGS NAIP orthoimagery (0.6 m)                     -> data/ortho_utm.tif (public domain)
"""
import io
import sys
import time

import numpy as np
import rasterio
import requests
from pyproj import Transformer
from rasterio.transform import from_origin
from rasterio.warp import Resampling, calculate_default_transform, reproject

import config as C

UA = {"User-Agent": "osm2carla-pipeline/1.0 (research use)"}
OVERPASS = ["https://overpass-api.de/api/interpreter",
            "https://overpass.kumi.systems/api/interpreter"]
DEM_SVC = "https://elevation.nationalmap.gov/arcgis/rest/services/3DEPElevation/ImageServer/exportImage"
NAIP_SVC = "https://imagery.nationalmap.gov/arcgis/rest/services/USGSNAIPPlus/ImageServer/exportImage"


def fetch_osm():
    s, w, n, e = C.outer_bbox()
    # Everything in the box (roads, buildings, landuse) plus complete ways crossing it.
    q = f"""[out:xml][timeout:180];
(
  way({s},{w},{n},{e});
  relation["type"="multipolygon"]({s},{w},{n},{e});
  relation["type"="restriction"]({s},{w},{n},{e});
);
(._;>;);
out meta;"""
    for url in OVERPASS:
        try:
            r = requests.post(url, data={"data": q}, headers=UA, timeout=300)
            r.raise_for_status()
            C.OSM_FILE.write_bytes(r.content)
            print(f"OSM: {len(r.content)/1e6:.1f} MB from {url}")
            return
        except Exception as ex:  # try next mirror
            print("Overpass failed:", url, ex)
            time.sleep(3)
    sys.exit("Could not download OSM data")


def utm_box():
    s, w, n, e = C.outer_bbox()
    t = Transformer.from_crs(4326, C.UTM_EPSG, always_xy=True)
    xs, ys = t.transform([w, e, w, e], [s, s, n, n])
    return min(xs), min(ys), max(xs), max(ys)


def export_image(svc, res, extra, tiles=1):
    """Call an ArcGIS ImageServer exportImage in UTM, tiling to stay under size limits."""
    x0, y0, x1, y1 = utm_box()
    x0, y0 = np.floor([x0, y0]); x1, y1 = np.ceil([x1, y1])
    W, H = int((x1 - x0) / res), int((y1 - y0) / res)
    out = None
    tw, th = int(np.ceil(W / tiles)), int(np.ceil(H / tiles))
    for ty in range(tiles):
        for tx in range(tiles):
            cx0, cy1 = x0 + tx * tw * res, y1 - ty * th * res
            cw, ch = min(tw, W - tx * tw), min(th, H - ty * th)
            params = dict(bbox=f"{cx0},{cy1 - ch*res},{cx0 + cw*res},{cy1}", bboxSR=C.UTM_EPSG,
                          imageSR=C.UTM_EPSG, size=f"{cw},{ch}", format="tiff", f="image", **extra)
            r = requests.get(svc, params=params, headers=UA, timeout=300)
            r.raise_for_status()
            if not r.content.startswith((b"II", b"MM")):
                sys.exit(f"Unexpected response from {svc}: {r.content[:300]!r}")
            with rasterio.open(io.BytesIO(r.content)) as src:
                arr = src.read()
            if out is None:
                out = np.zeros((arr.shape[0], H, W), arr.dtype)
            out[:, ty*th:ty*th+ch, tx*tw:tx*tw+cw] = arr[:, :ch, :cw]
    return out, from_origin(x0, y1, res, res)


def save(path, arr, transform, nodata=None):
    prof = dict(driver="GTiff", width=arr.shape[2], height=arr.shape[1], count=arr.shape[0],
                dtype=arr.dtype, crs=f"EPSG:{C.UTM_EPSG}", transform=transform,
                compress="deflate", nodata=nodata)
    with rasterio.open(path, "w", **prof) as dst:
        dst.write(arr)


def fetch_dem():
    arr, tr = export_image(DEM_SVC, C.DEM_RES, dict(pixelType="F32", noData=-9999,
                           interpolation="RSP_BilinearInterpolation", compression="None"))
    arr = arr[:1].astype(np.float32)
    print(f"DEM: {arr.shape[2]}x{arr.shape[1]} px, z {np.nanmin(arr):.1f}..{np.nanmax(arr):.1f} m")
    save(C.DEM_UTM, arr, tr, nodata=-9999)
    # WGS84 copy for netconvert --heightmap.geotiff
    with rasterio.open(C.DEM_UTM) as src:
        t, w, h = calculate_default_transform(src.crs, "EPSG:4326", src.width, src.height, *src.bounds)
        dst = np.zeros((1, h, w), np.float32)
        reproject(arr, dst, src_transform=src.transform, src_crs=src.crs, dst_transform=t,
                  dst_crs="EPSG:4326", resampling=Resampling.bilinear, src_nodata=-9999, dst_nodata=-9999)
        with rasterio.open(C.DEM_WGS, "w", driver="GTiff", width=w, height=h, count=1, dtype="float32",
                           crs="EPSG:4326", transform=t, nodata=-9999) as d:
            d.write(dst)


def fetch_ortho():
    arr, tr = export_image(NAIP_SVC, C.ORTHO_RES, dict(bandIds="0,1,2", pixelType="U8",
                           interpolation="RSP_CubicConvolution", compression="None"), tiles=2)
    print(f"Ortho: {arr.shape[2]}x{arr.shape[1]} px, bands={arr.shape[0]}")
    save(C.ORTHO_UTM, arr[:3], tr)


def fetch_nir():
    """NAIP band 4 (near infrared) + red, for NDVI vegetation detection."""
    arr, tr = export_image(NAIP_SVC, C.ORTHO_RES, dict(bandIds="0,3", pixelType="U8",
                           interpolation="RSP_BilinearInterpolation", compression="None"), tiles=2)
    print(f"Red+NIR: {arr.shape[2]}x{arr.shape[1]} px")
    save(C.DATA / "rednir_utm.tif", arr[:2], tr)


if __name__ == "__main__":
    what = set(sys.argv[1:]) or {"osm", "dem", "ortho", "nir"}
    if "osm" in what: fetch_osm()
    if "dem" in what: fetch_dem()
    if "ortho" in what: fetch_ortho()
    if "nir" in what: fetch_nir()
