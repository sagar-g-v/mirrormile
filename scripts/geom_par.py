"""Process-parallel helpers for 03_build_geometry.py. Geometry crosses processes as WKB (exact doubles), so a
parallel result is the same as the serial one. The pool starts on first use and is reused."""
import os
from concurrent.futures import ProcessPoolExecutor

import numpy as np
import shapely

_POOL = None
WORKERS = max(1, min(16, (os.cpu_count() or 2) - 2))


def union_with_fallbacks(polys, grid):
    """Grid-snapped union, falling back to repaired geometry and finally a tiny buffer when GEOS reports a
    topology exception."""
    try:
        return shapely.union_all(polys, grid_size=grid)
    except shapely.errors.GEOSException:
        pass
    good = shapely.make_valid(polys)
    try:
        return shapely.union_all(good, grid_size=grid)
    except shapely.errors.GEOSException:
        return shapely.union_all(shapely.buffer(good, 0.005)).buffer(-0.005)


def _union_wkb(args):
    wkb, grid = args
    return shapely.to_wkb(union_with_fallbacks(shapely.from_wkb(wkb), grid))


def pool():
    global _POOL
    if _POOL is None and WORKERS > 1:
        _POOL = ProcessPoolExecutor(max_workers=WORKERS)
    return _POOL


def union_groups(groups, grid):
    """union_with_fallbacks of each group (arrays of geometries), in parallel when there are workers."""
    p = pool()
    if p is None or len(groups) < 2:
        return [union_with_fallbacks(g, grid) for g in groups]
    jobs = [(shapely.to_wkb(np.asarray(g, dtype=object)), grid) for g in groups]
    return [shapely.from_wkb(w) for w in p.map(_union_wkb, jobs, chunksize=max(1, len(jobs) // (4 * WORKERS)))]
