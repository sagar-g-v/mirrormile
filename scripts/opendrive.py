"""Minimal, exact OpenDRIVE 1.4-1.6 evaluator: reference line, elevation, lane borders, marks.

Supports line / arc / spiral / poly3 / paramPoly3 (normalized and arcLength), laneOffset,
lane width polynomials, roadMark records, junction membership, objects and signals.
"""
import math
from dataclasses import dataclass, field

import numpy as np
from lxml import etree

DENSE = 0.2  # m, internal reference-line resolution


def _poly(rec, ds):
    return rec[0] + rec[1] * ds + rec[2] * ds * ds + rec[3] * ds * ds * ds


def _abcd(e):
    return tuple(float(e.get(k, 0)) for k in ("a", "b", "c", "d"))


def _pick(records, s):
    """records: sorted list of (s0, payload). Return (s0, payload) active at s."""
    i = 0
    for j, r in enumerate(records):
        if r[0] <= s + 1e-9:
            i = j
        else:
            break
    return records[i]


def _arc_resample(s_local, xs, ys, L):
    """Given a densely sampled curve, return points at arc-length positions s_local."""
    d = np.hypot(np.diff(xs), np.diff(ys))
    cum = np.concatenate([[0], np.cumsum(d)])
    if cum[-1] > 0:
        cum *= L / cum[-1]
    return np.interp(s_local, cum, xs), np.interp(s_local, cum, ys)


def eval_geometry(g, ds):
    """g: geometry element. ds: local arc-length positions. returns x, y (absolute)."""
    x0, y0, h0, L = (float(g.get(k)) for k in ("x", "y", "hdg", "length"))
    kind = g[0]
    tag = etree.QName(kind).localname
    if tag == "line":
        u, v = ds, np.zeros_like(ds)
    elif tag == "arc":
        k = float(kind.get("curvature"))
        if abs(k) < 1e-12:
            u, v = ds, np.zeros_like(ds)
        else:
            u = np.sin(k * ds) / k
            v = (1 - np.cos(k * ds)) / k
    elif tag == "spiral":
        k0, k1 = float(kind.get("curvStart")), float(kind.get("curvEnd"))
        n = max(200, int(L / 0.02))
        t = np.linspace(0, L, n)
        th = k0 * t + 0.5 * (k1 - k0) / L * t * t
        cu = np.concatenate([[0], np.cumsum((np.cos(th[1:]) + np.cos(th[:-1])) * 0.5 * np.diff(t))])
        cv = np.concatenate([[0], np.cumsum((np.sin(th[1:]) + np.sin(th[:-1])) * 0.5 * np.diff(t))])
        u, v = np.interp(ds, t, cu), np.interp(ds, t, cv)
    elif tag == "poly3":
        a, b, c, d = _abcd(kind)
        uu = np.linspace(0, L * 1.5 + 1, max(400, int(L * 30)))
        vv = a + b * uu + c * uu ** 2 + d * uu ** 3
        cum = np.concatenate([[0], np.cumsum(np.hypot(np.diff(uu), np.diff(vv)))])
        u, v = np.interp(ds, cum, uu), np.interp(ds, cum, vv)
    elif tag == "paramPoly3":
        au, bu, cu_, du = (float(kind.get(k)) for k in ("aU", "bU", "cU", "dU"))
        av, bv, cv_, dv = (float(kind.get(k)) for k in ("aV", "bV", "cV", "dV"))
        pmax = 1.0 if kind.get("pRange", "normalized") == "normalized" else L
        p = np.linspace(0, pmax, max(400, int(L * 30)))
        uu = au + bu * p + cu_ * p ** 2 + du * p ** 3
        vv = av + bv * p + cv_ * p ** 2 + dv * p ** 3
        u, v = _arc_resample(ds, uu, vv, L)  # CARLA also re-parameterizes by arc length
    else:
        raise ValueError(f"unsupported geometry {tag}")
    c, s = math.cos(h0), math.sin(h0)
    return x0 + u * c - v * s, y0 + u * s + v * c


@dataclass
class Lane:
    id: int
    type: str
    widths: list            # [(sOffset, abcd)]
    marks: list             # [(sOffset, dict)]
    level: bool = True

    def width(self, ds):
        out = np.zeros_like(ds)
        for i, (s0, rec) in enumerate(self.widths):
            s1 = self.widths[i + 1][0] if i + 1 < len(self.widths) else 1e18
            m = (ds >= s0 - 1e-9) & (ds < s1)
            out[m] = _poly(rec, ds[m] - s0)
        return out


@dataclass
class LaneSection:
    s: float
    e: float = 0.0
    left: list = field(default_factory=list)    # ordered 1,2,3...
    right: list = field(default_factory=list)   # ordered -1,-2,...
    center_marks: list = field(default_factory=list)


class Road:
    def __init__(self, el):
        self.el = el
        self.id = el.get("id")
        self.name = el.get("name", "")
        self.length = float(el.get("length"))
        self.junction = el.get("junction", "-1")
        link = el.find("link")
        self.pred = link.find("predecessor") if link is not None else None
        self.succ = link.find("successor") if link is not None else None
        # reference line, densely sampled
        S, X, Y = [], [], []
        self.geom_s = []
        for g in el.find("planView").findall("geometry"):
            s0, L = float(g.get("s")), float(g.get("length"))
            self.geom_s.append(s0)
            if L <= 1e-9:
                continue
            n = max(2, int(math.ceil(L / DENSE)) + 1)
            ds = np.linspace(0, L, n)
            x, y = eval_geometry(g, ds)
            S.append(s0 + ds); X.append(x); Y.append(y)
        self.S = np.concatenate(S); self.X = np.concatenate(X); self.Y = np.concatenate(Y)
        o = np.argsort(self.S, kind="stable")
        self.S, self.X, self.Y = self.S[o], self.X[o], self.Y[o]
        keep = np.concatenate([[True], np.diff(self.S) > 1e-6])   # joints between geometries
        self.S, self.X, self.Y = self.S[keep], self.X[keep], self.Y[keep]
        dx = np.gradient(self.X, self.S, edge_order=2) if len(self.S) > 2 else np.full(len(self.S), self.X[-1] - self.X[0])
        dy = np.gradient(self.Y, self.S, edge_order=2) if len(self.S) > 2 else np.full(len(self.S), self.Y[-1] - self.Y[0])
        self.H = np.unwrap(np.arctan2(dy, dx))
        # elevation
        ep = el.find("elevationProfile")
        self.elev = sorted([(float(e.get("s")), _abcd(e)) for e in ep.findall("elevation")]) if ep is not None else []
        # superelevation (roll about the reference line, positive falls to the right)
        lp = el.find("lateralProfile")
        self.superelev = sorted([(float(e.get("s")), _abcd(e)) for e in lp.findall("superelevation")]) if lp is not None else []
        # lanes
        lanes = el.find("lanes")
        self.offsets = sorted([(float(e.get("s")), _abcd(e)) for e in lanes.findall("laneOffset")])
        self.sections = []
        for sec in lanes.findall("laneSection"):
            ls = LaneSection(float(sec.get("s")))
            for side, store in (("left", ls.left), ("right", ls.right)):
                se = sec.find(side)
                if se is None:
                    continue
                for ln in se.findall("lane"):
                    store.append(self._lane(ln))
                store.sort(key=lambda l: abs(l.id))
            c = sec.find("center")
            if c is not None and c.find("lane") is not None:
                ls.center_marks = self._marks(c.find("lane"))
            self.sections.append(ls)
        self.sections.sort(key=lambda s: s.s)
        for i, sec in enumerate(self.sections):
            sec.e = self.sections[i + 1].s if i + 1 < len(self.sections) else self.length
        self.objects = el.find("objects")
        self.signals = el.find("signals")
        self.fine = any(any(abs(v) > 1e-9 for v in rec[1:]) for _, rec in self.offsets) or any(
            any(abs(v) > 1e-9 for v in rec[1:]) for sec in self.sections for l in sec.left + sec.right for _, rec in l.widths)

    @staticmethod
    def _marks(ln):
        return sorted([(float(m.get("sOffset", 0)), dict(m.attrib)) for m in ln.findall("roadMark")], key=lambda r: r[0])

    def _lane(self, ln):
        widths = sorted([(float(w.get("sOffset", 0)), _abcd(w)) for w in ln.findall("width")], key=lambda r: r[0])
        return Lane(int(ln.get("id")), ln.get("type", "none"), widths or [(0.0, (0, 0, 0, 0))],
                    self._marks(ln), ln.get("level", "true") in ("true", "1"))

    # ---- evaluation ---------------------------------------------------------------
    def ref(self, s):
        s = np.clip(np.asarray(s, float), 0, self.length)
        return np.interp(s, self.S, self.X), np.interp(s, self.S, self.Y), np.interp(s, self.S, self.H)

    @staticmethod
    def _piecewise(records, s):
        s = np.asarray(s, float)
        if not records:
            return np.zeros_like(s)
        out = np.zeros_like(s)
        for i, (s0, rec) in enumerate(records):
            s1 = records[i + 1][0] if i + 1 < len(records) else 1e18
            m = (s >= s0 - 1e-9) & (s < s1) if i else (s < s1)
            out[m] = _poly(rec, s[m] - s0)
        return out

    def z(self, s):
        """Elevation of the reference line."""
        return self._piecewise(self.elev, s)

    def superelevation(self, s):
        """Roll angle [rad] about the reference line (OpenDRIVE lateralProfile/superelevation)."""
        return self._piecewise(self.superelev, s)

    def surface_z(self, s, t):
        """Road surface height at (s, t): elevation plus t * tan(superelevation)."""
        z = self.z(s)
        if not self.superelev:
            return z
        return z + np.asarray(t, float) * np.tan(self.superelevation(s))

    def lane_offset(self, s):
        s = np.asarray(s, float)
        if not self.offsets:
            return np.zeros_like(s)
        out = np.zeros_like(s)
        for i, (s0, rec) in enumerate(self.offsets):
            s1 = self.offsets[i + 1][0] if i + 1 < len(self.offsets) else 1e18
            m = (s >= s0 - 1e-9) & (s < s1) if i else (s < s1)
            out[m] = _poly(rec, s[m] - s0)
        return out

    def xyz(self, s, t, dz=0.0):
        x, y, h = self.ref(s)
        return np.stack([x - t * np.sin(h), y + t * np.cos(h), self.surface_z(s, t) + dz], -1)

    def borders(self, sec, s):
        """Return {lane_id: (t_inner, t_outer)} for lanes of a section at positions s."""
        ds = np.asarray(s) - sec.s
        off = self.lane_offset(s)
        out = {}
        t = off.copy()
        for l in sec.left:
            w = l.width(ds); out[l.id] = (t.copy(), t + w); t = t + w
        t = off.copy()
        for l in sec.right:
            w = l.width(ds); out[l.id] = (t.copy(), t - w); t = t - w
        return out, off

    def sample_s(self, s0, s1, extra=(), max_step=4.0, max_turn=math.radians(1.5)):
        """Adaptive stations: dense on curves, sparse on straights, forced breakpoints."""
        max_step = 1.0 if self.fine else max_step
        m = (self.S > s0 + 1e-6) & (self.S < s1 - 1e-6)
        S, H = self.S[m], self.H[m]
        keep = [s0]
        h_last = float(self.ref(s0)[2])
        for s, h in zip(S, H):
            if s - keep[-1] >= max_step or abs(h - h_last) >= max_turn:
                keep.append(float(s)); h_last = float(h)
        forced = [v for v in list(self.geom_s) + list(extra) if s0 + 0.05 < v < s1 - 0.05]
        keep = sorted(set(np.round(keep + forced + [s1], 4)))
        # drop slivers created by forced points
        out = [keep[0]]
        for v in keep[1:]:
            if v - out[-1] > 0.05 or v == keep[-1]:
                out.append(v)
        if len(out) > 2 and out[-1] - out[-2] < 0.05:
            out.pop(-2)
        return np.array(out)


def write_elevation(road_el, s, z, dz=None):
    """Replace a road's elevationProfile by C1-continuous cubic Hermite segments through (s, z).
    dz: optional slopes at the stations (default: finite differences)."""
    s = np.asarray(s, float); z = np.asarray(z, float)
    if dz is None:
        dz = np.gradient(z, s) if len(s) > 2 else np.full(len(s), (z[-1] - z[0]) / max(s[-1] - s[0], 1e-9))
    dz = np.asarray(dz, float)
    ep = road_el.find("elevationProfile")
    if ep is None:
        ep = etree.Element("elevationProfile")
        road_el.find("planView").addnext(ep)
    for e in list(ep):
        ep.remove(e)
    for i in range(len(s) - 1):
        h = s[i + 1] - s[i]
        c = (3 * (z[i + 1] - z[i]) / h - 2 * dz[i] - dz[i + 1]) / h
        d = (dz[i] + dz[i + 1] - 2 * (z[i + 1] - z[i]) / h) / (h * h)
        etree.SubElement(ep, "elevation", s=f"{s[i]:.6f}", a=f"{z[i]:.6f}", b=f"{dz[i]:.8f}",
                         c=f"{c:.8f}", d=f"{d:.8f}")


class OpenDrive:
    def __init__(self, path):
        self.tree = etree.parse(str(path))
        root = self.tree.getroot()
        self.header = root.find("header")
        self.roads = [Road(r) for r in root.findall("road")]
        self.by_id = {r.id: r for r in self.roads}
        self.junctions = {j.get("id"): j.get("name", j.get("id")) for j in root.findall("junction")}

