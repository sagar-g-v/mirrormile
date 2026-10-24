"""Check a Traffic Manager path before tm.set_path: CARLA 0.10's LocalizationStage::ImportPath extends a vehicle's
waypoint buffer while its last waypoint is within the horizon of the first, choosing at each fork the branch whose
junction exit is closest to the next path point, with no loop guard. Where a ring of short roads between clustered
junctions (two divided roads crossing) lies within the horizon and the next path point is not reachable on the
chosen branch, the extension never ends and the client runs out of memory (the MemoryError of tm.set_path runs).

TMGraph rebuilds the TM's InMemoryMap from the carla.Map (InMemoryMap.cpp SetUp: 5 m waypoints of lanes wider than
1 m, by segment (road, lane, section), segment ends linked through the topology - past segments without waypoints -,
a dead end borrows the next waypoints of a same-direction neighbour the lane marking allows changing to);
TMGraph.check(start, path) replays ImportPath tick by tick (front waypoint dropped as the vehicle drives) and reports
'ok', 'removed' (dead end), 'endless' (an extension that never ends), 'endless-walk' (the fork walk never ends) or
'circling' (the vehicle keeps going without reaching the next point). Measured on LA (OdrForge): 8 of ~965 random
routes endless - with 60 vehicles a crash in about a third of the runs."""
import math
from collections import defaultdict

import carla

RES, MIN_W = 5.0, 1.0


class TMGraph:
    def __init__(self, m):
        seg = lambda w: (w.road_id, w.lane_id, w.section_id)
        succ_seg = defaultdict(list)
        for a, b in m.get_topology():
            if seg(a) != seg(b):
                succ_seg[seg(a)].append(seg(b))
        segs = defaultdict(list)
        for w in m.generate_waypoints(RES):
            if w.lane_width > MIN_W:
                segs[seg(w)].append(w)
        self.nodes, self.loc, self.nxt, self.isj, self.road = [], [], [], [], []
        first, last = {}, {}
        for k, ws in segs.items():
            ws.sort(key=lambda w: w.s)
            if ws[0].lane_id > 0:
                ws.reverse()
            ids = []
            for w in ws:
                ids.append(len(self.nodes)); self.nodes.append(w); self.nxt.append([])
                l = w.transform.location
                self.loc.append((l.x, l.y, l.z)); self.isj.append(w.is_junction); self.road.append(w.road_id)
            for a, b in zip(ids, ids[1:]):
                self.nxt[a] = [b]
            first[k], last[k] = ids[0], ids[-1]

        def successors(k, seen):
            out = []
            for s in succ_seg.get(k, []):
                if s in first:
                    out.append(first[s])
                elif s not in seen:
                    seen.add(s); out += successors(s, seen)
            return out
        for k in segs:
            self.nxt[last[k]] = successors(k, set())
        self.grid = defaultdict(list)
        for i, (x, y, z) in enumerate(self.loc):
            self.grid[(int(x // 10), int(y // 10))].append(i)
        LC = carla.LaneChange
        for i, w in enumerate(self.nodes):
            if not self.nxt[i] and not w.is_junction:
                lc = w.lane_change
                for side, nb in (("R", w.get_right_lane()), ("L", w.get_left_lane())):
                    ok = (side == "R" and lc in (LC.Right, LC.Both)) or (side == "L" and lc in (LC.Left, LC.Both))
                    if ok and nb is not None and nb.lane_type == carla.LaneType.Driving and nb.lane_id * w.lane_id > 0:
                        l = nb.transform.location
                        j = self.near((l.x, l.y, l.z))
                        if self.nxt[j]:
                            self.nxt[i] = list(self.nxt[j]); break

    def near(self, p):
        cx, cy = int(p[0] // 10), int(p[1] // 10)
        best, bi = math.inf, -1
        for r in range(200):
            for i in range(cx - r, cx + r + 1):
                for j in range(cy - r, cy + r + 1):
                    if max(abs(i - cx), abs(j - cy)) != r:
                        continue
                    for k in self.grid.get((i, j), ()):
                        q = self.loc[k]
                        dd = (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2 + (q[2] - p[2]) ** 2
                        if dd < best:
                            best, bi = dd, k
            if bi >= 0 and best < (r * 10) ** 2:
                return bi
        return bi

    def ambiguous(self, i, r=1.0):
        """another TM waypoint within r of waypoint i (branches starting / lanes merging at one spot): the TM finds a
        path point's waypoint by nearest location, so such a point may stand for the other branch - left out"""
        p = self.loc[i]
        cx, cy = int(p[0] // 10), int(p[1] // 10)
        for a in (cx - 1, cx, cx + 1):
            for b in (cy - 1, cy, cy + 1):
                for k in self.grid.get((a, b), ()):
                    if k != i and self.d2(i, k) < r * r:
                        return True
        return False

    def d2(self, a, b):
        p, q = self.loc[a], self.loc[b]
        return (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2

    def check(self, start, path, horizon=20.0, max_ext=2000, max_ticks=100000, rejoin=None):
        """start, path: (x, y, z) tuples (CARLA frame). ImportPath replayed; see the module doc. rejoin=n: 'ok' once n
        path points are consumed (a restart that has found its way back onto the path)"""
        n0 = len(path)
        nxt, isj, h2 = self.nxt, self.isj, horizon * horizon
        buf = [self.near(start)]
        path = list(path)
        if not path:
            return "ok"
        imported = self.near(path[0])
        ticks = 0
        while True:
            if not path or (rejoin and n0 - len(path) >= rejoin):
                return "ok"
            if len(buf) > 1:
                buf.pop(0)
            elif ticks and not nxt[buf[0]]:
                return "removed"
            ticks += 1
            if ticks > max_ticks:
                return "circling"
            ext, grew = 0, False
            while path and self.d2(buf[-1], buf[0]) <= h2:
                grew = True
                ext += 1
                if ext > max_ext:
                    return "endless"
                cand = nxt[buf[-1]]
                sel = 0
                if len(cand) > 1:
                    best = math.inf
                    for k, c in enumerate(cand):
                        j, g = c, 0
                        for cond in (lambda j: not isj[j], lambda j: isj[j], lambda j: self.d2(c, j) < 50.0):
                            while cond(j):
                                g += 1
                                if not nxt[j] or g > max_ext:
                                    return "endless-walk"
                                j = nxt[j][0]
                        if self.road[j] == self.road[imported]:
                            sel = k; break
                        dd = self.d2(j, imported)
                        if dd < best:
                            best, sel = dd, k
                elif not cand:
                    return "removed"
                c = cand[sel]
                if self.d2(c, imported) < 30.0:
                    path.pop(0)
                    if imported in nxt[c]:
                        buf.append(c)
                    buf.append(imported)
                    if path:
                        imported = self.near(path[0])
                else:
                    buf.append(c)
            if not grew and len(buf) <= 1:
                return "stalled"

    def _alive(self):
        """waypoints from which the TM's graph goes on forever (dead ends and what only leads into them pruned)"""
        if getattr(self, "_alive_set", None) is None:
            n = len(self.nodes)
            prev = [[] for _ in range(n)]
            for a, bs in enumerate(self.nxt):
                for b in bs:
                    prev[b].append(a)
            out = [len(set(bs)) for bs in self.nxt]
            dead = [i for i in range(n) if out[i] == 0]
            gone = set(dead)
            while dead:
                b = dead.pop()
                for a in set(prev[b]):
                    if a in gone:
                        continue
                    out[a] -= 1
                    if out[a] == 0:
                        gone.add(a); dead.append(a)
            self._alive_set = set(range(n)) - gone
        return self._alive_set

    def route(self, start, length, rng, every=2):
        """a random path of `length` m in the TM's own graph from the waypoint nearest start: alive successors only;
        a point every `every` waypoints (~10 m) outside junctions and on the first waypoint after each junction (the
        TM picks a fork's branch by the road after the junction: with a point there it takes the route's branch).
        Measured on LA: every TM waypoint (every=1) is no safer - where branches start at one spot the TM's
        nearest-waypoint lookup can take a path point for the other branch - and leaving such points out sends the
        greedy choice into dead ends; the check (safe) is what keeps a path out."""
        alive = self._alive()
        cur = self.near(start)
        if cur not in alive:
            return []
        pts, d, k = [], 0.0, 0
        while d < length:
            nx = [j for j in self.nxt[cur] if j in alive]
            if not nx:
                break
            j = rng.choice(nx)
            was_j = self.isj[cur]
            d += math.sqrt(self.d2(cur, j)); cur = j; k += 1
            if (every == 1 or (not self.isj[cur] and (was_j or k % every == 0))):
                pts.append(self.loc[cur])
        return pts

    def robust(self, path, every=3, horizons=(30.0, 130.0)):
        """'ok' unless a vehicle that drifts into a same-direction neighbour lane at a path point (swerving, pushed in a
        collision: the TM then restarts the import from where it is) makes ImportPath loop without end. Measured on LA:
        12 of 60 routes that replay cleanly on their own lanes were fragile so, and seed 23 drifted into one of them."""
        for k in range(0, len(path) - 1, every):
            w = self.nodes[self.near(path[k])]
            for nb in (w.get_left_lane(), w.get_right_lane()):
                if nb is None or nb.lane_type != carla.LaneType.Driving or nb.lane_id * w.lane_id <= 0:
                    continue
                l = nb.transform.location
                for h in horizons:
                    v = self.check((l.x, l.y, l.z), path[k + 1:], horizon=h, rejoin=5)
                    if v in ("endless", "endless-walk"):
                        return "fragile"
        return "ok"

    def safe(self, start, path):
        """'ok' at the standstill, city and highway horizons (the TM's: 15 m at least, 2 s ahead, above 60 km/h 4 s -
        130 m at ~117 km/h), else the first failure"""
        for h in (15.0, 30.0, 60.0, 130.0):
            r = self.check(start, path, horizon=h)
            if r != "ok":
                return r
        return self.robust(path)
