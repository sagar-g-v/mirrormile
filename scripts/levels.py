"""Grade-separated road levels: elevation of every drivable OSM node (ground, bridge, tunnel).

Bare-earth DEMs have bridges removed and see the surface above tunnels, and OSM bridge / tunnel
tags rarely cover a structure exactly. Elevation is therefore solved per road CHAIN (maximal path
through nodes where exactly two road segments meet, across tag changes) by a second-order Viterbi:
  candidates per node: ground DEM, every lidar bridge-deck level (class 17) nearby, and a
  structure interpolation (between the run's boundary nodes for bridge / tunnel runs);
  node cost by tag (a 'ground' node may still take a deck height, at a price, when the DEM there
  is the street *under* the structure); transition cost = grade change^2 + grade above 10 %.
Bridge / tunnel ways are first densified to <= DENSIFY m so curved ramps keep their profile.
"""
import math
from collections import defaultdict

import numpy as np
from lxml import etree
from scipy.spatial import cKDTree

DENSIFY = 8.0
DECK_R = 2.5             # m: lidar search radius around a node
LEVEL_GAP = 1.5          # m: vertical gap separating stacked decks
DRIVABLE = {"motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary",
            "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential", "living_street", "service"}
# node cost of each candidate kind, by the node's tag level
COST = {  # level: (ground DEM, lidar deck, structure interpolation, robust approach line)
    "grade": (0.0, 1.5, 3.0, 0.8),
    "bridge": (4.0, 0.0, 2.5, 3.0),
    "tunnel": (8.0, 6.0, 0.0, 6.0),
}
ROBUST_R, ROBUST_TRIM = 25.0, 0.5
W_CURV, W_STEEP, MAX_G = 300.0, 3000.0, 0.10


def way_level(tags):
    if tags.get("bridge") not in (None, "no"):
        return "bridge"
    if tags.get("tunnel") not in (None, "no") or tags.get("covered") == "yes":
        return "tunnel"                    # incl. building_passage: the road runs under a structure
    try:
        if float(tags.get("layer", 0)) < 0:
            return "tunnel"
    except ValueError:
        pass
    return "grade"


def densify(root, xy_of, to_ll, next_id):
    """Insert nodes so bridge/tunnel ways have <= DENSIFY m spacing. xy_of: node id -> (x, y) metres."""
    added = 0
    anchor = [root.findall("node")[-1]]           # new nodes go right after the existing ones

    def put(el):
        anchor[0].addnext(el); anchor[0] = el
    for w in list(root.iter("way")):
        tags = {t.get("k"): t.get("v") for t in w.findall("tag")}
        if tags.get("highway") not in DRIVABLE or way_level(tags) == "grade":
            continue
        nds = w.findall("nd")
        for a, b in zip(nds[:-1], nds[1:]):
            pa, pb = xy_of.get(a.get("ref")), xy_of.get(b.get("ref"))
            if pa is None or pb is None:
                continue
            L = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
            k = int(L // DENSIFY)
            prev = a
            for i in range(1, k + 1):
                t = i / (k + 1)
                x, y = pa[0] + t * (pb[0] - pa[0]), pa[1] + t * (pb[1] - pa[1])
                lon, lat = to_ll(x, y)
                nid = str(next_id[0]); next_id[0] += 1
                put(etree.Element("node", id=nid, lat=f"{lat:.8f}", lon=f"{lon:.8f}", version="1", visible="true"))
                nd = etree.Element("nd", ref=nid)
                prev.addnext(nd); prev = nd
                xy_of[nid] = (x, y)
                added += 1
    return added


def deck_levels(tree, deck_z, P):
    out = []
    for nb in tree.query_ball_point(P, DECK_R):
        z = np.sort(deck_z[nb]) if nb else np.zeros(0)
        levels, start = [], 0
        for i in range(1, len(z) + 1):
            if i == len(z) or z[i] - z[i - 1] > LEVEL_GAP:
                if i - start >= 3:
                    levels.append(float(np.median(z[start:i])))
                start = i
        out.append(levels)
    return out


def viterbi2(s, opts):
    """Second-order Viterbi. opts[i] = [(z, cost)]. State = (option at i, option at i-1)."""
    n = len(s)
    if n == 1:
        return np.array([min(opts[0], key=lambda o: o[1])[0]])
    # init with pairs (1, 0)
    ds = max(s[1] - s[0], 0.5)
    K0, K1 = len(opts[0]), len(opts[1])
    cost = np.zeros((K1, K0))
    for a in range(K1):
        for b in range(K0):
            g = (opts[1][a][0] - opts[0][b][0]) / ds
            cost[a, b] = opts[1][a][1] + opts[0][b][1] + W_STEEP * max(0.0, abs(g) - MAX_G) ** 2
    backs = []
    for i in range(2, n):
        ds1, ds0 = max(s[i] - s[i - 1], 0.5), max(s[i - 1] - s[i - 2], 0.5)
        Kc, Kp, Kpp = len(opts[i]), len(opts[i - 1]), len(opts[i - 2])
        zc = np.array([o[0] for o in opts[i]]); cc = np.array([o[1] for o in opts[i]])
        zp = np.array([o[0] for o in opts[i - 1]]); zpp = np.array([o[0] for o in opts[i - 2]])
        g1 = (zc[:, None] - zp[None, :]) / ds1                     # (Kc, Kp)
        g0 = (zp[:, None] - zpp[None, :]) / ds0                    # (Kp, Kpp)
        # total[c, p, pp] = cost[p, pp] + curvature(c,p,pp) + steep(c,p) + node(c)
        curv = W_CURV * (g1[:, :, None] - g0[None, :, :]) ** 2
        steep = W_STEEP * np.maximum(0.0, np.abs(g1) - MAX_G) ** 2
        tot = cost[None, :, :] + curv + steep[:, :, None] + cc[:, None, None]
        arg = np.argmin(tot, axis=2)                               # best pp for (c, p)
        cost = np.take_along_axis(tot, arg[:, :, None], 2)[:, :, 0]
        backs.append(arg)
    a, b = np.unravel_index(np.argmin(cost), cost.shape)
    idx = [0] * n
    idx[n - 1], idx[n - 2] = int(a), int(b)
    for i in range(n - 1, 1, -1):
        idx[i - 2] = int(backs[i - 2][idx[i], idx[i - 1]])
    return np.array([opts[i][idx[i]][0] for i in range(n)])


def assign_levels(root, xy_of, dem_z, deck_xy, deck_z):
    """Return ({node_id: z}, stats, {way_id: level}) for every node of a drivable way."""
    seg_level = {}                     # (a, b) sorted node pair -> level
    way_levels = {}
    adj = defaultdict(set)
    for w in root.iter("way"):
        tags = {t.get("k"): t.get("v") for t in w.findall("tag")}
        if tags.get("highway") not in DRIVABLE:
            continue
        lv = way_level(tags)
        way_levels[w.get("id")] = lv
        nodes = [nd.get("ref") for nd in w.findall("nd") if nd.get("ref") in xy_of]
        for a, b in zip(nodes[:-1], nodes[1:]):
            if a == b:
                continue
            key = (a, b) if a < b else (b, a)
            # a segment shared by ways of different levels keeps the structural one
            if seg_level.get(key, "grade") == "grade":
                seg_level[key] = lv
            adj[a].add(b); adj[b].add(a)
    nodes = sorted(adj)
    P = np.array([xy_of[n] for n in nodes])
    zg = dict(zip(nodes, dem_z(P[:, 0], P[:, 1])))
    node_lv = {}
    for n in nodes:
        lvls = {seg_level[(n, m) if n < m else (m, n)] for m in adj[n]}
        node_lv[n] = "bridge" if "bridge" in lvls else ("tunnel" if "tunnel" in lvls else "grade")

    # chains: maximal paths through degree-2 nodes
    visited, chains_ = set(), []
    for start in nodes:
        if len(adj[start]) == 2:
            continue
        for nb in adj[start]:
            key = (start, nb) if start < nb else (nb, start)
            if key in visited:
                continue
            seq = [start, nb]; visited.add(key)
            while len(adj[seq[-1]]) == 2:
                nxt = [m for m in adj[seq[-1]] if m != seq[-2]]
                if not nxt:
                    break
                k2 = (seq[-1], nxt[0]) if seq[-1] < nxt[0] else (nxt[0], seq[-1])
                if k2 in visited:
                    break
                visited.add(k2); seq.append(nxt[0])
            chains_.append(seq)
    for start in nodes:                 # pure loops (all degree 2)
        for nb in adj[start]:
            key = (start, nb) if start < nb else (nb, start)
            if key not in visited:
                seq = [start, nb]; visited.add(key)
                while len(adj[seq[-1]]) == 2:
                    nxt = [m for m in adj[seq[-1]] if m != seq[-2]]
                    k2 = (seq[-1], nxt[0]) if seq[-1] < nxt[0] else (nxt[0], seq[-1])
                    if not nxt or k2 in visited:
                        break
                    visited.add(k2); seq.append(nxt[0])
                chains_.append(seq)

    harmonic = structure_harmonic(nodes, adj, seg_level, xy_of, zg)
    tree = cKDTree(deck_xy) if len(deck_xy) else None
    acc = defaultdict(list)
    on_deck = {}
    stats = defaultdict(int)
    for seq in chains_:
        Q = np.array([xy_of[n] for n in seq])
        s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(Q, axis=0).T))])
        lv = [node_lv[n] for n in seq]
        dem = np.array([zg[n] for n in seq])
        # robust approach line: DEM sampled every 2 m along the chain, trimmed local line fits
        sd = np.arange(0, s[-1] + 1e-6, 2.0) if s[-1] > 2 else s
        Qd = np.column_stack([np.interp(sd, s, Q[:, 0]), np.interp(sd, s, Q[:, 1])])
        zd = dem_z(Qd[:, 0], Qd[:, 1])
        robust = np.empty(len(seq))
        for k in range(len(seq)):
            m = np.abs(sd - s[k]) <= ROBUST_R
            u, v = sd[m] - s[k], zd[m]
            keep = np.ones(len(u), bool)
            val = dem[k]
            for _ in range(4):
                if keep.sum() < 4 or np.ptp(u[keep]) < 4:
                    break
                b_, a_ = np.polyfit(u[keep], v[keep], 1)
                val = a_
                res = np.abs(v - (a_ + b_ * u))
                new = res < max(ROBUST_TRIM, 2.5 * np.median(res[keep]))
                if (new == keep).all():
                    break
                keep = new
            robust[k] = val
        # structure interpolation: across each bridge/tunnel run between its boundary nodes
        interp = dem.copy()
        i = 0
        while i < len(seq):
            if lv[i] == "grade":
                i += 1; continue
            j = i
            while j + 1 < len(seq) and lv[j + 1] != "grade":
                j += 1
            a, b = max(i - 1, 0), min(j + 1, len(seq) - 1)
            t = (s[i:j + 1] - s[a]) / max(s[b] - s[a], 1e-6)
            interp[i:j + 1] = dem[a] + (dem[b] - dem[a]) * t
            i = j + 1
        # structure networks (underground grids, interchange ramps): harmonic solution between portals
        for k, n in enumerate(seq):
            if n in harmonic:
                interp[k] = harmonic[n]
        decks = deck_levels(tree, deck_z, Q) if tree is not None else [[] for _ in seq]
        opts = []
        for k in range(len(seq)):
            cg, cd, ci, cr = COST[lv[k]]
            o = [(dem[k], cg), (robust[k], cr)] + [(z, cd) for z in decks[k]]
            if lv[k] != "grade":
                o.append((interp[k], ci))
            opts.append(o)
        z = viterbi2(s, opts)
        for k in range(len(seq)):
            if decks[k] and min(abs(np.array(decks[k]) - z[k])) < 1e-6:
                on_deck[seq[k]] = True              # height confirmed by lidar deck points
        stats["structure_chains" if any(l != "grade" for l in lv) else "ground_chains"] += 1
        stats["nodes_on_deck"] += int(sum(1 for k in range(len(seq)) if decks[k] and min(abs(np.array(decks[k]) - z[k])) < 1e-6))
        stats["grade_tagged_nodes_lifted"] += int(sum(1 for k in range(len(seq)) if lv[k] == "grade" and z[k] - dem[k] > 1.0))
        for n, v in zip(seq, z):
            acc[n].append(float(v))
    zmap = {n: float(np.mean(v)) for n, v in acc.items()}
    stats["chains"] = len(chains_)
    stats.update(enforce_crossings(root, xy_of, zmap, chains_, on_deck, node_lv))
    return zmap, dict(stats), way_levels


def structure_harmonic(nodes, adj, seg_level, xy_of, zg, iters=3000):
    """Heights for nodes of bridge / tunnel networks: harmonic interpolation (graph Laplace,
    edge weight 1/length) with the nodes touching ground-level roads (portals, abutments) fixed
    at the DEM. Underground junctions (all incident roads in tunnels) thus get heights between
    their portals instead of the surface above them."""
    lv = lambda a, b: seg_level[(a, b) if a < b else (b, a)]
    struct = {n for n in nodes if any(lv(n, m) != "grade" for m in adj[n])}
    boundary = {n for n in struct if any(lv(n, m) == "grade" for m in adj[n])}
    free = [n for n in struct if n not in boundary]
    if not free:
        return {}
    idx = {n: i for i, n in enumerate(free)}
    z = np.array([zg[n] for n in free], float)
    nb, w, fixed_term, wsum = [], [], np.zeros(len(free)), np.zeros(len(free))
    for i, n in enumerate(free):
        ii, ww = [], []
        for m in adj[n]:
            if lv(n, m) == "grade":
                continue
            d = max(math.hypot(xy_of[n][0] - xy_of[m][0], xy_of[n][1] - xy_of[m][1]), 0.5)
            if m in idx:
                ii.append(idx[m]); ww.append(1.0 / d)
            elif m in boundary:
                fixed_term[i] += zg[m] / d
            else:
                continue
            wsum[i] += 1.0 / d
        nb.append(np.array(ii, int)); w.append(np.array(ww))
    # components without any portal in the map: keep 8 m below the surface
    has_b = wsum > np.array([sum(w[i]) for i in range(len(free))]) + 1e-12
    reach = has_b.copy()
    for _ in range(len(free)):
        new = reach.copy()
        for i in range(len(free)):
            if not reach[i] and any(reach[nb[i]]):
                new[i] = True
        if (new == reach).all():
            break
        reach = new
    z[~reach] -= 8.0
    for _ in range(iters):                      # Jacobi iterations
        zn = z.copy()
        for i in range(len(free)):
            if not reach[i] or wsum[i] <= 0:
                continue
            zn[i] = (fixed_term[i] + (w[i] * z[nb[i]]).sum()) / wsum[i]
        if np.max(np.abs(zn - z)) < 1e-4:
            z = zn; break
        z = zn
    out = {n: float(z[i]) for n, i in idx.items()}
    out.update({n: float(zg[n]) for n in boundary})
    return out


MIN_SEP = 5.7            # m: 4.3 m clearance + 1.4 m deck between crossing roads
BUMP_R = 45.0            # m: minimum half-length of the smooth correction along a chain
MAX_ADD_G = 0.08         # max grade the correction may add
PORTAL_KEEP = 30.0       # m: corrections taper to zero towards fixed ground nodes (portals, abutments)


def enforce_crossings(root, xy_of, zmap, chains_, on_deck, node_lv, iters=4):
    """Roads crossing in plan without a shared node are grade-separated: the upper one (by OSM
    layer, else by current height) must clear the lower one by MIN_SEP. Violations are fixed on the
    road whose heights are NOT lidar-confirmed, with a raised-cosine bump along its chain."""
    import shapely
    layer_of = {}
    for w in root.iter("way"):
        tags = {t.get("k"): t.get("v") for t in w.findall("tag")}
        if tags.get("highway") not in DRIVABLE:
            continue
        try:
            lay = float(tags.get("layer", 0))
        except ValueError:
            lay = 0.0
        nodes = [nd.get("ref") for nd in w.findall("nd") if nd.get("ref") in xy_of]
        for a, b in zip(nodes[:-1], nodes[1:]):
            layer_of[(a, b) if a < b else (b, a)] = lay
    segs = [k for k in layer_of if k[0] in zmap and k[1] in zmap]
    lines = shapely.linestrings([[xy_of[a], xy_of[b]] for a, b in segs])
    tree = shapely.STRtree(lines)
    # chain lookup: node -> (chain index, arc position)
    where = defaultdict(list)
    chain_s = []
    for ci, seq in enumerate(chains_):
        Q = np.array([xy_of[n] for n in seq])
        s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(Q, axis=0).T))])
        chain_s.append(s)
        for n, sv in zip(seq, s):
            where[n].append((ci, sv))
    fixed, worst_after = 0, 0.0
    skipped = set()
    for _ in range(iters):
        ia, ib = tree.query(lines, predicate="crosses")
        changed = 0
        for i, j in zip(ia, ib):
            if i >= j:
                continue
            (a1, b1), (a2, b2) = segs[i], segs[j]
            if {a1, b1} & {a2, b2}:
                continue
            p = shapely.intersection(lines[i], lines[j])
            if p.geom_type != "Point":
                continue
            def z_at(a, b):
                pa, pb = np.array(xy_of[a]), np.array(xy_of[b])
                t = np.clip(np.dot(np.array(p.coords[0]) - pa, pb - pa) / max(np.dot(pb - pa, pb - pa), 1e-9), 0, 1)
                return zmap[a] + t * (zmap[b] - zmap[a]), t
            z1, t1 = z_at(a1, b1); z2, t2 = z_at(a2, b2)
            l1, l2 = layer_of[segs[i]], layer_of[segs[j]]
            if l1 == l2:
                up_first = z1 >= z2
            else:
                up_first = l1 > l2
            zu, zl = (z1, z2) if up_first else (z2, z1)
            need = MIN_SEP - (zu - zl)
            if need <= 0.05:
                continue
            up, lo = ((a1, b1), (a2, b2)) if up_first else ((a2, b2), (a1, b1))
            conf_up = all(on_deck.get(n, False) for n in up)
            conf_lo = all(on_deck.get(n, False) or node_lv.get(n) == "grade" for n in lo)
            # move the unconfirmed one; if both are free, split the correction
            if conf_up and conf_lo:
                continue                            # both measured: report, do not invent
            targets = []
            if not conf_up:
                targets.append((up, +need if conf_lo else +need / 2))
            if not conf_lo:
                targets.append((lo, -need if conf_up else -need / 2))
            applied = False
            for (na, nb), delta in targets:
                hits = {}
                for n in (na, nb):
                    for ci, sv in where.get(n, []):
                        hits.setdefault(ci, []).append(sv)
                # bump length grows with the correction so the added grade stays <= MAX_ADD_G
                R = max(BUMP_R, math.pi / 2 * abs(delta) / MAX_ADD_G)
                for ci, svs in hits.items():
                    sc = float(np.mean(svs))
                    s = chain_s[ci]
                    seq = chains_[ci]
                    sg = np.array([sv for n, sv in zip(seq, s) if node_lv.get(n) == "grade"])
                    # fixed ground nodes (portals / abutments) bound the bump: if the crossing is
                    # too close to one, the clearance cannot be reached without a cliff -> skip
                    if len(sg) and np.min(np.abs(sg - sc)) < PORTAL_KEEP:
                        continue
                    for n, sv in zip(seq, s):
                        d = abs(sv - sc)
                        if d >= R or node_lv.get(n) == "grade":
                            continue
                        taper = 1.0 if not len(sg) else float(np.clip(np.min(np.abs(sg - sv)) / PORTAL_KEEP, 0, 1))
                        zmap[n] += delta * 0.5 * (1 + math.cos(math.pi * d / R)) * taper
                    applied = True
            if applied:
                changed += 1
            else:
                skipped.add((segs[i], segs[j]))
        fixed += changed
        if not changed:
            break
    return {"crossing_fixes": fixed, "crossing_conflicts_left": len(skipped)}
