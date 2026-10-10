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


def main():
    od = OpenDrive(C.XODR_FILE)
    root = od.tree.getroot()
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
    print(f"signal groups: {sum(len(v) for v in links.values())} controllers linked to {len(links)} junctions"
          f"{f', {unlinked} without a junction' if unlinked else ''}")


if __name__ == "__main__":
    main()
