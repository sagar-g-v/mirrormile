"""Stop signs from OSM into the OpenDRIVE (runs after 02b; idempotent).

SUMO's OpenDRIVE export carries traffic lights (dynamic <signal>s + <controller>s) but no static
signs. CARLA's Traffic Manager and its sign generator use OpenDRIVE stop signals
(country="OpenDRIVE" type="206"), so every OSM highway=stop is mapped to junction approaches:

  * node ON an approach way (> 8 m from the junction centre): that approach only
  * node AT the junction: stop=all -> every approach; otherwise the approaches of the minor road
    class (all-way stop when every approach has the same class, the common LA residential case)
  * junctions with traffic lights are skipped (the lights win)

The sign sits 3 m before the road end on the right of the approaching traffic, just outside the
outermost driving lane, valid for all lanes heading into the junction.
"""
import json
from collections import defaultdict

import numpy as np
from lxml import etree
from pyproj import Transformer

import config as C
from opendrive import OpenDrive

DRIVING = {"driving", "entry", "exit", "onRamp", "offRamp", "connectingRamp", "bidirectional"}
RANK = {"motorway": 0, "trunk": 1, "primary": 2, "secondary": 3, "tertiary": 4, "unclassified": 5,
        "residential": 5, "living_street": 6, "service": 7}
SET_BACK = 3.0          # m before the road end
AT_JUNCTION = 8.0       # m: node this close to the junction centre = junction node
MATCH = 30.0            # m: max distance node -> junction centre


def osm_stops():
    root = etree.parse(str(C.OSM_FILE)).getroot()
    tf = Transformer.from_crs(4326, C.MAP_PROJ, always_xy=True)
    out = []
    for n in root.iter("node"):
        tags = {t.get("k"): t.get("v") for t in n.findall("tag")}
        if tags.get("highway") == "stop":
            x, y = tf.transform(float(n.get("lon")), float(n.get("lat")))
            out.append((n.get("id"), x, y, tags))
    return out


def approaches(od):
    """junction id -> [(road, 'end'|'start')] for non-junction roads meeting it."""
    out = defaultdict(list)
    for r in od.roads:
        if r.junction != "-1":
            continue
        for end, e in (("start", r.pred), ("end", r.succ)):
            if e is not None and e.get("elementType") == "junction":
                out[e.get("elementId")].append((r, end))
    return out


def driving_side(r, end):
    """Lanes travelling INTO the junction at this end: right lanes at 'end', left lanes at 'start'."""
    sec = r.sections[-1] if end == "end" else r.sections[0]
    lanes = sec.right if end == "end" else sec.left
    return sec, [l for l in lanes if l.type in DRIVING]


def add_stop(r, end, sid):
    sec, lanes = driving_side(r, end)
    if not lanes or r.length < 2 * SET_BACK:
        return False
    s = r.length - SET_BACK if end == "end" else SET_BACK
    B, _ = r.borders(sec, np.array([s]))
    if end == "end":
        t = min(B[l.id][1][0] for l in lanes) - 0.5
        orient, ids = "+", [l.id for l in lanes]
    else:
        t = max(B[l.id][1][0] for l in lanes) + 0.5
        orient, ids = "-", [l.id for l in lanes]
    sigs = r.el.find("signals")
    if sigs is None:
        sigs = etree.Element("signals")
        anchor = r.el.find("objects") if r.el.find("objects") is not None else r.el.find("lanes")
        anchor.addnext(sigs)
    sg = etree.SubElement(sigs, "signal", s=f"{s:.3f}", t=f"{t:.3f}", id=sid, name="Stop", dynamic="no",
                          orientation=orient, zOffset="0", country="OpenDRIVE", type="206", subtype="-1",
                          value="-1", text="", hOffset="0", pitch="0", roll="0", height="2.1", width="0.76")
    etree.SubElement(sg, "validity", fromLane=str(min(ids, key=abs)), toLane=str(max(ids, key=abs)))
    return True


def main():
    od = OpenDrive(C.XODR_FILE)
    types = json.loads((C.BUILD / "road_types.json").read_text())
    # idempotent: drop stop signs from an earlier run
    for r in od.roads:
        if r.signals is not None:
            for sg in r.signals.findall("signal"):
                if sg.get("id", "").startswith("stop_"):
                    r.signals.remove(sg)
    app = approaches(od)
    centre, has_tls = {}, set()
    for r in od.roads:
        if r.junction != "-1":
            P = r.xyz(np.array([0.0, r.length]), np.zeros(2))[:, :2]
            centre.setdefault(r.junction, []).extend(P.tolist())
    centre = {j: np.mean(v, 0) for j, v in centre.items()}
    for j, lst in app.items():
        for r, end in lst:
            if r.signals is not None and any(sg.get("dynamic") == "yes" for sg in r.signals.findall("signal")):
                has_tls.add(j)
    jids = list(centre)
    J = np.array([centre[j] for j in jids])
    done, stats = set(), defaultdict(int)
    for nid, x, y, tags in osm_stops():
        d = np.hypot(J[:, 0] - x, J[:, 1] - y)
        k = int(np.argmin(d))
        if d[k] > MATCH:
            stats["no_junction"] += 1; continue
        j = jids[k]
        if j in has_tls:
            stats["junction_has_lights"] += 1; continue
        cands = app.get(j, [])
        if d[k] > AT_JUNCTION:      # on an approach way: the approach whose end is nearest the node
            def end_dist(re):
                r, end = re
                P = r.xyz(np.array([r.length if end == "end" else 0.0]), np.zeros(1))[0, :2]
                return np.hypot(P[0] - x, P[1] - y)
            cands = sorted(cands, key=end_dist)[:1]
            stats["node_on_approach"] += 1
        elif tags.get("stop") != "all":
            ranks = [RANK.get(types.get(r.id, ""), 5) for r, _ in cands]
            if ranks and len(set(ranks)) > 1:
                worst = max(ranks)
                cands = [c for c, rk in zip(cands, ranks) if rk == worst]
                stats["minor_road_stop"] += 1
            else:
                stats["all_way_stop"] += 1
        else:
            stats["all_way_stop"] += 1
        for r, end in cands:
            key = (r.id, end)
            if key in done:
                continue
            if add_stop(r, end, f"stop_{nid}_{r.id}_{end}"):
                done.add(key); stats["signs"] += 1
    od.tree.write(str(C.XODR_FILE), xml_declaration=True, encoding="UTF-8", pretty_print=True)
    print("stop signs:", dict(stats))


if __name__ == "__main__":
    main()
