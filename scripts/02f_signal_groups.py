"""Traffic-light groups for CARLA (runs after 02b / 02d / 02e; idempotent).

SUMO's OpenDRIVE export writes the traffic-light <controller>s (which signal heads switch together)
but not the junction -> controller links. CARLA builds its traffic-light groups (the phase cycle
of one intersection) from <junction><controller id=.. type="0" sequence=../></junction>; without
them every traffic light is left without a group ("TrafficLightComponent doesn't have any Group
assigned") and never cycles. Each controller is linked to the junction its signal heads face:
a signal near the end of its road controls the junction at that end.
"""
from collections import defaultdict

from lxml import etree

import config as C
from opendrive import OpenDrive


MAX_ID = 32          # CARLA: "The max size of a signal id is 32" (longer ids are truncated, warned every tick)


def shorten_ids(root):
    """SUMO names signals / controllers after joined junction clusters (cluster_<id>_<id>..._#1more_9,
    up to ~60 chars). Give every id longer than MAX_ID a short unique one and update all references
    (<control signalId>, <signalReference id>, junction controller links)."""
    sig_map, ctl_map = {}, {}
    for sg in root.iter("signal"):
        sid = sg.get("id", "")
        if len(sid) > MAX_ID:
            sig_map[sid] = sig_map.get(sid) or f"sg{len(sig_map):05d}"
            sg.set("id", sig_map[sid])
    for ref in root.iter("signalReference"):
        if ref.get("id") in sig_map:
            ref.set("id", sig_map[ref.get("id")])
    for ctrl in root.findall("controller"):
        cid = ctrl.get("id", "")
        if len(cid) > MAX_ID:
            ctl_map[cid] = f"tc{len(ctl_map):04d}"
            ctrl.set("id", ctl_map[cid])
        if ctrl.get("name") and len(ctrl.get("name")) > MAX_ID:
            ctrl.set("name", ctrl.get("id"))
        for cs in ctrl.findall("control"):
            if cs.get("signalId") in sig_map:
                cs.set("signalId", sig_map[cs.get("signalId")])
    for j in root.findall("junction"):
        for c in j.findall("controller"):
            if c.get("id") in ctl_map:
                c.set("id", ctl_map[c.get("id")])
    return len(sig_map), len(ctl_map)


def main():
    od = OpenDrive(C.XODR_FILE)
    root = od.tree.getroot()
    n_sig, n_ctl = shorten_ids(root)
    od = OpenDrive.__new__(OpenDrive); od.tree = etree.ElementTree(root)     # re-read with the new ids
    from opendrive import Road
    od.roads = [Road(e) for e in root.findall("road")]
    sig_road = {}
    for r in od.roads:
        if r.signals is None:
            continue
        for sg in r.signals.findall("signal"):
            sig_road[sg.get("id")] = (r, float(sg.get("s", 0.0)))
    junctions = {j.get("id"): j for j in root.findall("junction")}
    for j in junctions.values():                       # idempotent: drop earlier links
        for c in j.findall("controller"):
            j.remove(c)
    links = defaultdict(list)
    unlinked = 0
    for ctrl in root.findall("controller"):
        votes = defaultdict(int)
        for cs in ctrl.findall("control"):
            hit = sig_road.get(cs.get("signalId"))
            if hit is None:
                continue
            r, s = hit
            link = r.succ if s > r.length / 2 else r.pred
            if link is not None and link.get("elementType") == "junction":
                votes[link.get("elementId")] += 1
        if not votes:
            unlinked += 1
            continue
        jid = max(votes, key=votes.get)
        if jid in junctions:
            links[jid].append(ctrl.get("id"))
    for jid, ctrls in links.items():
        j = junctions[jid]
        anchor = j.find("connection")
        for k, cid in enumerate(ctrls):
            el = etree.Element("controller", id=cid, type="0", sequence=str(k))
            if anchor is not None:
                anchor.addprevious(el)
            else:
                j.append(el)
    od.tree.write(str(C.XODR_FILE), xml_declaration=True, encoding="UTF-8", pretty_print=True)
    print(f"signal ids: {n_sig} signals and {n_ctl} controllers renamed to <= {MAX_ID} chars")
    print(f"signal groups: {sum(len(v) for v in links.values())} controllers linked to {len(links)} junctions"
          f"{f', {unlinked} without a junction' if unlinked else ''}")


if __name__ == "__main__":
    main()
