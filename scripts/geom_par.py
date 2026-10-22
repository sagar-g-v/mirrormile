"""Thread-parallel geometry helpers for 03_build_geometry.py and qa_check.py. Shapely 2 releases the GIL inside
GEOS, so threads run unions and buffers on all cores; a parallel result is the same object a serial call returns
(measured: 16 tile unions of the LA road triangles 23.4 s serial, 2.5 s on 16 threads, byte-identical)."""
import os
from concurrent.futures import ThreadPoolExecutor

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


class _Done:
    def __init__(self, value):
        self.value = value

    def result(self):
        return self.value


def pool():
    global _POOL
    if _POOL is None and WORKERS > 1:
        _POOL = ThreadPoolExecutor(max_workers=WORKERS)
    return _POOL


def submit(fn, *args, **kw):
    """fn(*args, **kw) on the pool (or now, without one); .result() gives its value"""
    p = pool()
    return p.submit(fn, *args, **kw) if p is not None else _Done(fn(*args, **kw))


def union_groups(groups, grid):
    """union_with_fallbacks of each group (arrays of geometries), in parallel when there are workers."""
    p = pool()
    if p is None or len(groups) < 2:
        return [union_with_fallbacks(g, grid) for g in groups]
    return list(p.map(lambda g: union_with_fallbacks(g, grid), groups))
