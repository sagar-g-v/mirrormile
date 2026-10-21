"""Lane clearance: does any non-road mesh stand in the space a car occupies on a driving lane?

Probes every 2 m along each driving lane of the OpenDRIVE that a car fits in (width >= MIN_W; narrower taper ends
are left out), two segments per probe:
  across: horizontal, perpendicular to the lane, at z + 0.8 m, lane centre -1 m .. +1 m   (walls, barriers, buildings)
  up:     vertical at the lane centre, z + 0.2 m .. z + 1.8 m                             (terrain, decks, ceilings)
A hit on any triangle of a layer other than the road surface and its paint is an obstruction.
"""
import collections

import numpy as np

SKIP = ("Road__", "RoadLine__")
CELL = 4.0
MIN_W = 2.2
STEP = 2.0


def lane_probes(od, types=("driving",)):
    """(x, y, z, heading, in_junction) of lane centres every STEP m, lanes of the given types and >= MIN_W wide."""
    out = []
    for r in od.roads:
        for sec in r.sections:
            lanes = [l for l in sec.left + sec.right if l.type in types]
            if not lanes or sec.e - sec.s < 1e-3:
                continue
            s = np.arange(sec.s + STEP / 2, sec.e, STEP)
            if not len(s):
                continue
            B, _ = r.borders(sec, s)
            _, _, h = r.ref(s)
            for l in lanes:
                ti, to = B[l.id]
                ok = np.abs(to - ti) >= MIN_W
                if not ok.any():
                    continue
                P = r.xyz(s[ok], ((ti + to) / 2)[ok])
                out.append(np.column_stack([P, h[ok], np.full(ok.sum(), r.junction != "-1")]))
    return np.concatenate(out) if out else np.zeros((0, 5))


def layer_triangles(arrays):
    """{layer group: (m, 3, 3) triangles} from geometry.npz-style arrays (keys 'Layer__Kind|...|V' / '|F')."""
    tris = collections.defaultdict(list)
    for k in arrays.files if hasattr(arrays, "files") else arrays:
        if not k.endswith("|V"):
            continue
        lay = k.split("|")[0]
        if lay.startswith(SKIP):
            continue
        tris[lay.split("__")[0]].append(arrays[k][arrays[k[:-2] + "|F"]])
    return {k: np.concatenate(v) for k, v in tris.items()}


def seg_hits(P0, P1, T):
    """Moller-Trumbore, all segments (n, 3) x all triangles (m, 3, 3) -> (n,) any hit."""
    if len(T) == 0 or len(P0) == 0:
        return np.zeros(len(P0), bool)
    D = P1 - P0
    e1 = T[:, 1] - T[:, 0]; e2 = T[:, 2] - T[:, 0]
    h = np.cross(D[:, None, :], e2[None, :, :])
    a = np.einsum("mk,nmk->nm", e1, h)
    ok = np.abs(a) > 1e-9
    f = np.where(ok, 1.0 / np.where(ok, a, 1.0), 0.0)
    s = P0[:, None, :] - T[None, :, 0]
    u = f * np.einsum("nmk,nmk->nm", s, h)
    q = np.cross(s, e1[None, :, :])
    v = f * np.einsum("nk,nmk->nm", D, q)
    t = f * np.einsum("mk,nmk->nm", e2, q)
    return (ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t >= 0) & (t <= 1)).any(1)


def _grid(T):
    lo = np.floor(T[:, :, :2].min(1) / CELL).astype(int); hi = np.floor(T[:, :, :2].max(1) / CELL).astype(int)
    cells = collections.defaultdict(list)
    for i in range(len(T)):
        for cx in range(lo[i, 0], hi[i, 0] + 1):
            for cy in range(lo[i, 1], hi[i, 1] + 1):
                cells[(cx, cy)].append(i)
    return {k: np.array(v) for k, v in cells.items()}


def obstructions(probes, tris):
    """{layer: boolean mask over probes} - probes whose car envelope a layer's triangles cut."""
    x, y, z, hd = probes[:, 0], probes[:, 1], probes[:, 2], probes[:, 3]
    c = np.column_stack([x, y]); nrm = np.column_stack([-np.sin(hd), np.cos(hd)])
    A0 = np.column_stack([c - nrm, z + 0.8]); A1 = np.column_stack([c + nrm, z + 0.8])
    U0 = np.column_stack([c, z + 0.2]); U1 = np.column_stack([c, z + 1.8])
    order = collections.defaultdict(list)
    for i, k in enumerate(map(tuple, np.floor(c / CELL).astype(int))):
        order[k].append(i)
    out = {}
    for lay, T in tris.items():
        cells = _grid(T)
        hit = np.zeros(len(probes), bool)
        for k, idx in order.items():
            cand = [cells[(k[0] + dx, k[1] + dy)] for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (k[0] + dx, k[1] + dy) in cells]
            if not cand:
                continue
            tt = T[np.unique(np.concatenate(cand))]
            idx = np.array(idx)
            for j in range(0, len(idx), 64):
                b = idx[j:j + 64]
                hit[b] |= seg_hits(A0[b], A1[b], tt) | seg_hits(U0[b], U1[b], tt)
        out[lay] = hit
    return out
