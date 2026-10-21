"""Road levels and highway classes for an OdrForge OpenDRIVE (the OdrForge-mode stand-in for what 02b writes).

03 (bridge decks, tunnels, freeway barriers), 02d and 03e read build/road_levels.json and build/road_types.json,
keyed by OpenDRIVE road id. 02b derives them from SUMO edge ids; OdrForge writes the source ways itself:
  - levels: the road's <bridge> / <tunnel> objects (s, length) -> runs of 'grade' | 'bridge' | 'tunnel';
    a connecting road without its own objects takes the level of the linked road ends (as in 02b)
  - types:  highest-ranked OSM highway class of the road's userData osmWays

    MIRRORMILE_XODR=odrforge python scripts/02g_odrforge_levels.py
"""
import json
from collections import Counter

from lxml import etree

import config as C

HW_RANK = ["motorway", "trunk", "motorway_link", "trunk_link", "primary", "primary_link", "secondary",
           "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential", "living_street", "service"]


def runs_of(length, spans):
    """[(s0, s1, level)] covering [0, length]; spans = [(s, len, level)] (overlaps: later wins)."""
    cuts = sorted({0.0, length, *[min(max(s, 0.0), length) for s, n, _ in spans],
                   *[min(max(s + n, 0.0), length) for s, n, _ in spans]})
    out = []
    for a, b in zip(cuts, cuts[1:]):
        if b - a < 1e-3:
            continue
        m = (a + b) / 2
        lv = "grade"
        for s, n, l in spans:
            if s <= m <= s + n:
                lv = l
        if out and out[-1][2] == lv:
            out[-1][1] = round(b, 3)
        else:
            out.append([round(a, 3), round(b, 3), lv])
    return out or [[0.0, round(length, 3), "grade"]]


def main():
    way_hw = {}
    for _, w in etree.iterparse(str(C.OSM_FILE), tag="way"):
        t = w.find("tag[@k='highway']")
        if t is not None:
            way_hw[w.get("id")] = t.get("v")
        w.clear()
    roads = etree.parse(str(C.XODR_FILE)).getroot().findall("road")
    runs, types, conn = {}, {}, []
    for r in roads:
        rid, length = r.get("id"), float(r.get("length"))
        spans = [(float(o.get("s")), float(o.get("length")), o.tag)
                 for o in r.iterfind("objects/*") if o.tag in ("bridge", "tunnel")]
        if r.get("junction") != "-1" and not spans:
            conn.append(r)
            continue
        runs[rid] = runs_of(length, spans)
        if r.get("junction") == "-1":
            u = r.find("userData[@code='osmWays']")
            hws = [way_hw.get(i) for i in u.get("value").split(",")] if u is not None else []
            hws = [h for h in hws if h in HW_RANK]
            types[rid] = min(hws, key=HW_RANK.index) if hws else "unknown"
    for r in conn:
        lv = set()
        for link in (r.find("link/predecessor"), r.find("link/successor")):
            if link is None or link.get("elementType") != "road" or link.get("elementId") not in runs:
                continue
            rr = runs[link.get("elementId")]
            lv.add(rr[0][2] if link.get("contactPoint", "start") == "start" else rr[-1][2])
        level = "bridge" if "bridge" in lv else ("tunnel" if "tunnel" in lv else "grade")
        runs[r.get("id")] = [[0.0, round(float(r.get("length")), 3), level]]
    (C.BUILD / "road_levels.json").write_text(json.dumps(runs))
    (C.BUILD / "road_types.json").write_text(json.dumps(types))
    print("levels:", dict(Counter(lv for v in runs.values() for _, _, lv in v)), "roads", len(runs))
    print("types:", dict(Counter(types.values()).most_common()))


if __name__ == "__main__":
    main()
