"""Compare OpenDRIVE files with CARLA's parser (LibCarla): topology, waypoints, junctions, landmarks,
lane continuity (position jumps / heading kinks when walking every driving lane at 1 m).

  venv38\\Scripts\\python scripts\\compare_xodr_carla.py A.xodr [B.xodr ...]
"""
import json
import math
import sys

import carla


def stats(path):
    xodr = open(path, encoding="utf-8").read()
    m = carla.Map("cmp", xodr)
    topo = m.get_topology()
    wps = m.generate_waypoints(2.0)
    drive = [w for w in wps if w.lane_type == carla.LaneType.Driving]
    side = [w for w in wps if w.lane_type == carla.LaneType.Sidewalk]
    junc = {w.get_junction().id for w in wps if w.is_junction}
    lm = m.get_all_landmarks()
    types = {}
    for l in lm:
        types[l.type] = types.get(l.type, 0) + 1
    # continuity: from every driving waypoint step 1 m along the lane (incl. into junctions)
    jumps = kinks = steps = 0
    worst = 0.0
    kink_deg = []
    for w in drive[::3]:
        cur = w
        for _ in range(30):
            nx = cur.next(1.0)
            if not nx:
                break
            for n in nx:
                d = n.transform.location.distance(cur.transform.location)
                dh = abs((n.transform.rotation.yaw - cur.transform.rotation.yaw + 180) % 360 - 180)
                steps += 1
                if d > 1.6:
                    jumps += 1
                    worst = max(worst, d)
                if dh > 12.0:  # > 12 deg per metre = radius < ~5 m or a kink
                    kinks += 1
                kink_deg.append(dh)
            cur = nx[0]
    kink_deg.sort()
    spawns = m.get_spawn_points()
    return {
        "file": path,
        "topology_segments": len(topo),
        "waypoints_2m": len(wps),
        "driving_wps": len(drive),
        "sidewalk_wps": len(side),
        "junctions_with_wps": len(junc),
        "spawn_points": len(spawns),
        "landmarks": len(lm),
        "landmark_types": dict(sorted(types.items(), key=lambda kv: -kv[1])[:8]),
        "steps_1m": steps,
        "jumps_gt_1.6m": jumps,
        "worst_jump_m": round(worst, 2),
        "kinks_gt_12deg_per_m": kinks,
        "heading_change_p99_deg_per_m": round(kink_deg[int(0.99 * len(kink_deg))], 2) if kink_deg else 0,
        "heading_change_p999_deg_per_m": round(kink_deg[int(0.999 * len(kink_deg))], 2) if kink_deg else 0,
    }


if __name__ == "__main__":
    res = [stats(p) for p in sys.argv[1:]]
    print(json.dumps(res, indent=1))
