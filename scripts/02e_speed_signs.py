"""Speed limits for the OpenDRIVE (runs after 02b / 02d; idempotent).

1. Lane speeds: SUMO fills streets without an OSM maxspeed with its (European) type defaults
   (100 / 80 / 50 / 20 km/h). Those are replaced with California defaults by road class; mapped
   OSM limits (25 / 30 / 35 / 55 mph ...) are kept. Junction connectors inherit their incoming road.
2. Road speed: <type><speed max=.. unit="mph"/> on every road (OpenDRIVE road-level limit).
3. Speed-limit signs (country="OpenDRIVE" type="274", value in km/h): CARLA vehicles take their
   speed limit (and the Traffic Manager its target speed) from the speed-limit signs they pass.
   A sign is placed at the start of a road (per travel direction) where the limit differs from the
   road traffic comes from, where no sign was passed for SIGN_EVERY metres, and on entry roads.
"""
import json
import math
from collections import defaultdict

import numpy as np
from lxml import etree

import config as C
from opendrive import OpenDrive

DRIVING = {"driving", "entry", "exit", "onRamp", "offRamp", "connectingRamp", "bidirectional"}
SUMO_DEFAULTS = (27.78, 22.22, 13.89, 5.56)          # m/s: 100 / 80 / 50 / 20 km/h
CA_MPH = {"motorway": 65, "motorway_link": 45, "trunk": 50, "trunk_link": 35, "primary": 35, "secondary": 35,
          "tertiary": 30, "primary_link": 25, "secondary_link": 25, "tertiary_link": 25, "unclassified": 25,
          "residential": 25, "living_street": 15, "service": 15}
SIGN_EVERY = {"motorway": 1500.0, "default": 400.0}
MPH = 0.44704


def carla_kmh(mph):
    """CARLA has speed-limit sign models (and Traffic Manager behaviour) for 30, 40 ... 120 km/h only:
    the posted mph limit is shown on the nearest one (25 mph -> 40, 35 mph -> 60, 55 mph -> 90)."""
    return int(min(120, max(30, round(mph * 1.609344 / 10.0) * 10)))


def lane_speed_elems(r):
    out = []
    for sec in r.el.find("lanes").findall("laneSection"):
        for side in ("left", "right"):
            se = sec.find(side)
            if se is None:
                continue
            for ln in se.findall("lane"):
                if ln.get("type") in DRIVING:
                    out.extend((side, sp) for sp in ln.findall("speed"))
    return out


def is_default(v):
    return any(abs(v - d) < 0.02 for d in SUMO_DEFAULTS)


def main():
    od = OpenDrive(C.XODR_FILE)
    root = od.tree.getroot()
    types = json.loads((C.BUILD / "road_types.json").read_text())
    by = od.by_id
    # junction connections: connector -> (incoming road, outgoing road)
    conn_in = {}
    for j in root.findall("junction"):
        for c in j.findall("connection"):
            conn_in[c.get("connectingRoad")] = c.get("incomingRoad")
    cls = {}
    for r in od.roads:
        cls[r.id] = types.get(r.id) if r.junction == "-1" else types.get(conn_in.get(r.id, ""), None)
    # 1. lane speeds
    stats = defaultdict(int)
    for r in od.roads:
        c = cls.get(r.id) or "residential"
        new = CA_MPH.get(c, 25) * MPH
        for _, sp in lane_speed_elems(r):
            v = float(sp.get("max"))
            if is_default(v):
                sp.set("max", f"{new:.2f}"); stats["lane_speeds_set_ca_default"] += 1
            else:
                stats["lane_speeds_osm_kept"] += 1
    od = OpenDrive.__new__(OpenDrive)                      # re-read lane speeds from the edited tree
    od.tree = etree.ElementTree(root); od.roads = []
    from opendrive import Road
    od.roads = [Road(e) for e in root.findall("road")]; by = {r.id: r for r in od.roads}

    def side_mph(r, side):
        vals = [float(sp.get("max")) for sd, sp in lane_speed_elems(r) if sd == side]
        return int(round(np.median(vals) / MPH / 5.0) * 5) if vals else None

    # 2. road-level speed
    for r in od.roads:
        vals = [float(sp.get("max")) for _, sp in lane_speed_elems(r)]
        if not vals:
            continue
        mph = int(round(np.median(vals) / MPH / 5.0) * 5)
        for t in r.el.findall("type"):
            for old in t.findall("speed"):
                t.remove(old)
            etree.SubElement(t, "speed", max=str(mph), unit="mph")
    # 3. speed-limit signs: road-level travel graph (road, +1 travels +s on right lanes / -1 on left)
    nodes = {}
    for r in od.roads:
        if r.junction != "-1":
            continue
        for d, side in ((1, "right"), (-1, "left")):
            mph = side_mph(r, side)
            if mph:
                nodes[(r.id, d)] = mph
    preds = defaultdict(set)

    def node_at(rid, contact_is_start, entering):
        """travel node of road rid when entering (or leaving) it at the given end."""
        if entering:
            return (rid, 1) if contact_is_start else (rid, -1)
        return (rid, -1) if contact_is_start else (rid, 1)

    for c in od.roads:
        if c.junction == "-1" or c.pred is None or c.succ is None:
            continue
        if c.pred.get("elementType") != "road" or c.succ.get("elementType") != "road":
            continue
        a = node_at(c.pred.get("elementId"), c.pred.get("contactPoint") == "start", entering=False)
        b = node_at(c.succ.get("elementId"), c.succ.get("contactPoint") == "start", entering=True)
        if a in nodes and b in nodes:
            preds[b].add(a)
    for r in od.roads:                                      # direct road-road links
        if r.junction != "-1":
            continue
        for end, e in (("end", r.succ), ("start", r.pred)):
            if e is None or e.get("elementType") != "road" or by.get(e.get("elementId")) is None:
                continue
            if by[e.get("elementId")].junction != "-1":
                continue
            a = (r.id, 1) if end == "end" else (r.id, -1)
            b = node_at(e.get("elementId"), e.get("contactPoint") == "start", entering=True)
            if a in nodes and b in nodes:
                preds[b].add(a)
    L = {k: by[k[0]].length for k in nodes}
    D = {k: math.inf for k in nodes}
    sign = {k: False for k in nodes}
    for _ in range(30):
        changed = False
        for k in nodes:
            ps = preds.get(k, ())
            every = SIGN_EVERY["motorway"] if types.get(k[0]) == "motorway" else SIGN_EVERY["default"]
            d_in = min((0.0 if sign[p] else D[p]) + L[p] for p in ps) if ps else math.inf
            want = (not ps) or any(nodes[p] != nodes[k] for p in ps) or d_in >= every
            want = want and L[k] >= 30.0
            if want != sign[k] or (d_in != D[k] and not (math.isinf(d_in) and math.isinf(D[k]))):
                changed = True
            sign[k], D[k] = want, d_in
        if not changed:
            break
    # write signals
    for r in od.roads:
        if r.signals is not None:
            for sg in r.signals.findall("signal"):
                if sg.get("id", "").startswith("speed_"):
                    r.signals.remove(sg)
    for (rid, d), on in sign.items():
        if not on:
            continue
        r = by[rid]
        sec = r.sections[0] if d > 0 else r.sections[-1]
        lanes = [l for l in (sec.right if d > 0 else sec.left) if l.type in DRIVING]
        if not lanes:
            continue
        s = min(8.0, r.length / 3) if d > 0 else r.length - min(8.0, r.length / 3)
        B, _ = r.borders(sec, np.array([s]))
        t = (min(B[l.id][1][0] for l in lanes) - 0.5) if d > 0 else (max(B[l.id][1][0] for l in lanes) + 0.5)
        mph = nodes[(rid, d)]
        sigs = r.el.find("signals")
        if sigs is None:
            sigs = etree.Element("signals")
            anchor = r.el.find("objects") if r.el.find("objects") is not None else r.el.find("lanes")
            anchor.addnext(sigs)
        sg = etree.SubElement(sigs, "signal", s=f"{s:.3f}", t=f"{t:.3f}", id=f"speed_{rid}_{'p' if d > 0 else 'n'}",
                              name=f"SpeedLimit{mph}mph", dynamic="no", orientation="+" if d > 0 else "-",
                              zOffset="0", country="OpenDRIVE", type="274", subtype="-1",
                              value=str(carla_kmh(mph)), unit="km/h", text=f"{mph} mph",
                              hOffset="0", pitch="0", roll="0", height="0.9", width="0.6")
        ids = [l.id for l in lanes]
        etree.SubElement(sg, "validity", fromLane=str(min(ids, key=abs)), toLane=str(max(ids, key=abs)))
        stats["speed_signs"] += 1
    od.tree.write(str(C.XODR_FILE), xml_declaration=True, encoding="UTF-8", pretty_print=True)
    dist = defaultdict(int)
    for k, v in nodes.items():
        dist[v] += 1
    print("speeds:", dict(stats), "| travel directions by limit (mph):", dict(sorted(dist.items())))


if __name__ == "__main__":
    main()
