"""Roof materials (post-process of build/geometry.npz, runs after 03).

NAIP aerial imagery is not a true orthophoto: buildings lean away from the camera by roughly
height x 0.3, so a 100 m tower's roof shows up ~30 m off its footprint and an ortho-textured roof
would wear the street / the neighbours. Roofs therefore get procedural roof materials:
  * each roof (one connected roof surface = one building / building part) gets ONE material
  * low buildings (< 20 m, small lean): the style closest to the roof's real colour, sampled from
    the imagery inside its footprint (white membrane / light concrete / gray gravel / dark tar)
  * taller buildings: deterministic light or gray (the imagery at their footprint is unreliable)
UVs become planar metres (material tile 10 m).
"""
import hashlib
import json

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree

import config as C
import importlib
GB = importlib.import_module("03_build_geometry")

STYLES = {"RoofWhite": (214, 214, 208), "RoofLight": (178, 174, 166), "RoofGray": (128, 126, 122), "RoofDark": (70, 68, 66)}
LOW_H = 20.0


def components(F, n):
    parent = np.arange(n)

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]; i = parent[i]
        return i
    for a, b, c in F:
        ra, rb, rc = find(a), find(b), find(c)
        r = min(ra, rb, rc)
        parent[ra] = parent[rb] = parent[rc] = r
    roots = np.array([find(i) for i in range(n)])
    return roots[F[:, 0]]


def main():
    g = dict(np.load(C.GEOM_FILE))
    meta = json.loads(C.META_FILE.read_text())
    x0, y0, x1, y1 = meta["extent"]
    ortho = np.asarray(Image.open(C.TEX / meta["ortho"]).convert("RGB"), np.float32)
    H, W = ortho.shape[:2]
    dem = GB.Dem(C.DEM_UTM, filt=5)
    names = list(STYLES)
    cols = np.array([STYLES[k] for k in names], np.float32)
    out, counts = {}, {k: 0 for k in names}
    for key in [k for k in g if k.startswith("Building__Roof|") and k.endswith("|V")]:
        base = key[:-2]
        tile = base.split("|")[1]
        V, F = g.pop(base + "|V"), g.pop(base + "|F"); g.pop(base + "|UV")
        comp = components(F, len(V))
        per = {k: [] for k in names}
        for c in np.unique(comp):
            f = F[comp == c]
            P = V[np.unique(f.ravel())]
            cx, cy = P[:, 0].mean(), P[:, 1].mean()
            h = float(P[:, 2].max() - dem(np.array([cx]), np.array([cy]))[0])
            key_h = int.from_bytes(hashlib.md5(f"{cx:.1f},{cy:.1f}".encode()).digest()[:4], "little")
            if h < LOW_H:
                # median colour of the imagery inside the roof's bounding box (shrunk 25 %)
                ax0, ax1 = np.percentile(P[:, 0], [25, 75]); ay0, ay1 = np.percentile(P[:, 1], [25, 75])
                c0 = int((ax0 - x0) / (x1 - x0) * W); c1 = max(c0 + 1, int((ax1 - x0) / (x1 - x0) * W))
                r0 = int((y1 - ay1) / (y1 - y0) * H); r1 = max(r0 + 1, int((y1 - ay0) / (y1 - y0) * H))
                patch = ortho[max(r0, 0):min(r1, H), max(c0, 0):min(c1, W)].reshape(-1, 3)
                rgb = np.median(patch, 0) if len(patch) else cols[1]
                style = names[int(np.argmin(np.linalg.norm(cols - rgb, axis=1)))]
            else:
                style = ("RoofLight", "RoofGray", "RoofLight", "RoofWhite")[key_h % 4]
            per[style].append(f)
            counts[style] += 1
        for style, fs in per.items():
            if not fs:
                continue
            f = np.concatenate(fs)
            used, inv = np.unique(f.ravel(), return_inverse=True)
            k = f"Building__{style}|{tile}"
            out[k + "|V"] = V[used]
            out[k + "|F"] = inv.reshape(-1, 3).astype(np.int32)
            out[k + "|UV"] = V[used, :2].astype(np.float32)          # planar metres
    g.update(out)
    np.savez_compressed(C.GEOM_FILE, **g)
    print("roof materials per building:", counts)


if __name__ == "__main__":
    main()
