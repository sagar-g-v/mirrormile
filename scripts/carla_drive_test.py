"""CARLA drive test: Traffic Manager vehicles driven in synchronous mode on a cooked map (--map, the imported world)
or on one .xodr in CARLA's OpenDRIVE standalone mode (generate_opendrive_world). Prints / writes JSON metrics.
usage (venv38, a CARLA server running): carla_drive_test.py [file.xodr] [--map LA_DTLA_ODR] [--vehicles 60]
       [--ticks 3000] [--seed 7] [--routes 3000] [--out result.json]   (run_drive_test.ps1 starts / stops the server)
--routes: every vehicle a random path of that length, made in the Traffic Manager's own graph and replayed through its
ImportPath logic before tm.set_path (tm_path_check.py) - stock CARLA 0.10's ImportPath can loop without end at rings
of short roads between clustered junctions (client MemoryError); the TM patch on sagar-g-v/carla branch
tm-importpath-guard bounds it.
Metrics: spawn points / successful spawns; per vehicle distance driven, longest still period (speed < 0.1 m/s, not
at a red light), collisions, lane invasions (solid / any), off-road ticks (no driving lane under the vehicle), fall
ticks (vehicle more than 1 m below the lane surface under it)."""
import argparse, json, math, random, time
import carla

ap = argparse.ArgumentParser()
ap.add_argument("xodr", nargs="?"); ap.add_argument("--map", help="load this cooked CARLA map (real meshes) instead of the bare OpenDRIVE world")
ap.add_argument("--vehicles", type=int, default=60); ap.add_argument("--ticks", type=int, default=3000)
ap.add_argument("--seed", type=int, default=7); ap.add_argument("--out"); ap.add_argument("--port", type=int, default=2000); ap.add_argument("--near", type=float, default=0.0)
ap.add_argument("--routes", type=float, default=0.0,
                help="give every vehicle a random route of this length (m) through lanes it can drive on forever "
                     "(no dead end, map edge or dropped lane ahead) - the Traffic Manager's custom path")
a = ap.parse_args()
random.seed(a.seed)
client = carla.Client("localhost", a.port); client.set_timeout(600.0)
t0 = time.time()
if a.map:
    # the server's current map, if that is the one asked for (no reload)
    world = client.get_world()
    if not world.get_map().name.endswith(a.map):
        world = client.load_world(a.map)
else:
    xodr = open(a.xodr, encoding="utf-8").read()
    params = carla.OpendriveGenerationParameters(vertex_distance=2.0, max_road_length=500.0, wall_height=0.0,
                                                 additional_width=0.6, smooth_junctions=True, enable_mesh_visibility=True)
    world = client.generate_opendrive_world(xodr, params)
# the new episode starts after the call returns: wait until the world id stops changing before spawning
last, same = None, 0
while same < 3:
    time.sleep(2.0)
    world = client.get_world()
    same = same + 1 if world.id == last else 0
    last = world.id
load_s = time.time() - t0
settings = world.get_settings(); settings.synchronous_mode = True; settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)
tm = client.get_trafficmanager(8000); tm.set_synchronous_mode(True); tm.set_random_device_seed(a.seed)
m = world.get_map()
sps = m.get_spawn_points(); random.shuffle(sps)
if a.near > 0: sps = [p for p in sps if math.hypot(p.location.x, p.location.y) < a.near]

key = lambda w: (w.road_id, w.section_id, w.lane_id)
alive = None
if a.routes > 0:
    # lane segments with an endless way on: prune every segment whose successors all end (repeat to a fixed point)
    succ = {}
    for s, e in m.get_topology():
        if s.lane_type != carla.LaneType.Driving:
            continue
        succ.setdefault(key(s), set()).update(key(n) for n in e.next(1.0) if n.lane_type == carla.LaneType.Driving)
    alive = set(succ)
    while True:
        dead = {k for k in alive if not (succ.get(k, set()) & alive)}
        if not dead:
            break
        alive -= dead
    n_all = len(succ)
    sps = [p for p in sps if key(m.get_waypoint(p.location, lane_type=carla.LaneType.Driving)) in alive]
    # and in the Traffic Manager's own graph (tm_path_check): routes are made and checked there
    import tm_path_check
    tmg = tm_path_check.TMGraph(m)
    xyz = lambda l: (l.x, l.y, l.z)
    sps = [p for p in sps if tmg.near(xyz(p.location)) in tmg._alive()]
    print(f"routes: {len(alive)} of {n_all} lane segments lead on forever; spawn points on them: {len(sps)}", flush=True)


def route(start, length, step=10.0, every=2, no_return=300.0):
    """random walk through alive lane segments, a location every `every` steps. Points only on lanes >= 2 m wide (the
    Traffic Manager leaves lanes <= 1 m out of its own map), and no segment again within `no_return` m: the TM's path
    import has no loop guard, so a point it cannot reach on a small cycle grows its waypoint buffer without end"""
    pts, cur, d, recent = [], start, 0.0, []
    while d < length:
        nx = [w for w in cur.next(step) if key(w) in alive]
        fresh = [w for w in nx if key(w) == key(cur) or key(w) not in recent]
        if not fresh:
            break
        cur = random.choice(fresh); d += step
        if not recent or recent[-1] != key(cur):
            recent.append(key(cur))
        recent = recent[-int(no_return / step):]
        if int(d / step) % every == 0 and cur.lane_width >= 2.0:
            pts.append(cur.transform.location)
    return pts
print('spawn z range', min(p.location.z for p in sps), max(p.location.z for p in sps), 'candidates', len(sps), flush=True)
bps = [b for b in world.get_blueprint_library().filter("vehicle.*") if int(b.get_attribute("number_of_wheels")) == 4]
vehicles = []
for sp in sps[: a.vehicles * 2]:
    if len(vehicles) >= a.vehicles: break
    sp.location.z += 0.5
    v = world.try_spawn_actor(random.choice(bps), sp)
    if v: vehicles.append(v)
def restore():
    try:
        st_ = world.get_settings(); st_.synchronous_mode = False; st_.fixed_delta_seconds = None; world.apply_settings(st_)
        tm.set_synchronous_mode(False)
    except Exception:
        pass
import atexit; atexit.register(restore)
world.tick()
vtypes = {str(v.id): [v.type_id, round(2 * v.bounding_box.extent.y, 2)] for v in vehicles}   # id -> blueprint, width m
for v in vehicles: v.set_autopilot(True, 8000)
route_len = []
route_rejected = {}
route_dump = []
if a.routes > 0:
    # every path is replayed through the Traffic Manager's own ImportPath logic first (tm_path_check): set_path only
    # with a path it can follow to the end - a ring of short roads between clustered junctions can make ImportPath
    # extend the buffer without end (the client's MemoryError)
    for v in vehicles:
        pts = []
        for attempt in range(10):
            cand = [carla.Location(*q) for q in tmg.route(xyz(v.get_location()), a.routes, random)]
            verdict = tmg.safe(xyz(v.get_location()), [xyz(p) for p in cand]) if cand else "empty"
            if verdict == "ok":
                pts = cand; break
            route_rejected[verdict] = route_rejected.get(verdict, 0) + 1
        route_len.append(sum(p.distance(q) for p, q in zip(pts, pts[1:])))   # m along the path points
        if pts:
            tm.auto_lane_change(v, False)     # stay on the path's lanes (a TM lane change restarts the import elsewhere)
            tm.set_path(v, pts)
            route_dump.append((v, pts))
    json.dump([dict(id=v.id, start=xyz(v.get_location()), path=[xyz(p) for p in pp]) for v, pp in route_dump],
              open(a.out + ".routes.json", "w"))         # what set_path got (offline replay of a crash)
    print(f"routes set: {sum(1 for x in route_len if x)} vehicles, length min {min(route_len):.0f} m; "
          f"rejected before set_path: {route_rejected}", flush=True)
col = {v.id: 0 for v in vehicles}; inv = {v.id: [0, 0] for v in vehicles}; sensors = []
import collections as _c
col_what, col_ex = _c.Counter(), _c.defaultdict(list)   # what the vehicles hit (static mesh: its semantic tags)
incidents, last_contact, solid_events = [], {}, []
offroad_events, offroad_last = [], {}   # one entry per new contact (same vehicle + object: > 1 s since the last event)
def on_col(e, i):
    col[i] += 1
    o = e.other_actor
    what = o.type_id if o is not None else "none"
    if o is not None and what.startswith("static"):
        what += str(sorted(set(o.semantic_tags)))
    col_what[what] += 1
    key = (i, o.id if o is not None else 0)
    if e.frame - last_contact.get(key, -10**9) > 20:
        p = e.transform.location
        w = m.get_waypoint(p, project_to_road=True, lane_type=carla.LaneType.Driving)
        lat = None
        if w is not None:
            c = w.transform.location; f = w.transform.get_right_vector()
            lat = round((p.x - c.x) * f.x + (p.y - c.y) * f.y, 2)     # + right of the lane centre
        incidents.append(dict(t=round(e.timestamp, 1), v=i, what=what, x=round(p.x, 1), y=round(p.y, 1), z=round(p.z, 1),
                              road=w.road_id if w else None, lane=w.lane_id if w else None,
                              junction=w.is_junction if w else None, half_width=round(w.lane_width / 2, 2) if w else None,
                              lateral=lat, speed=round(e.actor.get_velocity().length(), 1)))
    last_contact[key] = e.frame
    if len(col_ex[what]) < 5 and (not col_ex[what] or col_ex[what][-1][0] != i):
        p = e.transform.location; col_ex[what].append((i, round(p.x), round(p.y), round(p.z, 1)))
bl = world.get_blueprint_library()
for v in vehicles:
    c = world.spawn_actor(bl.find("sensor.other.collision"), carla.Transform(), attach_to=v)
    c.listen(lambda e, i=v.id: on_col(e, i)); sensors.append(c)
    l = world.spawn_actor(bl.find("sensor.other.lane_invasion"), carla.Transform(), attach_to=v)
    def on_inv(e, i=v.id):
        inv[i][0] += 1
        kinds = [f"{x.type}/{x.color}" for x in e.crossed_lane_markings]
        if any(str(x.type) in ("Solid", "SolidSolid", "SolidBroken", "BrokenSolid", "Curb", "Grass") for x in e.crossed_lane_markings):
            inv[i][1] += 1
            p = e.transform.location
            w = m.get_waypoint(p, project_to_road=True, lane_type=carla.LaneType.Driving)
            solid_events.append(dict(t=round(e.timestamp, 1), v=i, marks=kinds, x=round(p.x, 1), y=round(p.y, 1),
                                     road=w.road_id if w else None, lane=w.lane_id if w else None,
                                     junction=w.is_junction if w else None,
                                     lane_change=str(w.lane_change) if w else None))
    l.listen(on_inv); sensors.append(l)
st = {v.id: {"dist": 0.0, "still": 0, "still_max": 0, "offroad": 0, "fall": 0, "last": v.get_location()} for v in vehicles}
_w = m.generate_waypoints(10.0)
ext = (min(w.transform.location.x for w in _w), max(w.transform.location.x for w in _w),
       min(w.transform.location.y for w in _w), max(w.transform.location.y for w in _w))
t1 = time.time()
gone, gone_at = set(), {}
import collections
why, whyex = collections.Counter(), collections.defaultdict(list)
for k in range(a.ticks):
    world.tick()
    if k % 5: continue
    for v in vehicles:
        if v.id in gone: continue
        try:
            loc = v.get_location()
        except RuntimeError as ex:  # destroyed by the server (fell out of the world)
            gone.add(v.id); gone_at[v.id] = k
            ll = st[v.id]["last"]; wp = m.get_waypoint(ll)
            ahead, d, cur = None, 0.0, wp
            while cur is not None and d < 60.0:
                nx = cur.next(2.0)
                if not nx: ahead = d; break
                cur = nx[0]; d += 2.0
            reason = "dead end ahead (%.0f m)" % ahead if ahead is not None else ("stuck" if st[v.id]["still"] * 0.05 > 15 else "other")
            if ahead is not None and cur is not None and min(cur.transform.location.x - ext[0], ext[1] - cur.transform.location.x,
                                                             cur.transform.location.y - ext[2], ext[3] - cur.transform.location.y) < 60:
                reason = "map edge ahead (%.0f m)" % ahead   # the road leaves the clipped area
            why[reason.split(" (")[0]] += 1
            if len(whyex[reason.split(" (")[0]]) < 6: whyex[reason.split(" (")[0]].append((round(k * 0.05, 1), wp.road_id, wp.lane_id, round(ll.x), round(ll.y), reason))
            continue
        s = st[v.id]
        s["dist"] += loc.distance(s["last"]); s["last"] = loc
        sp = v.get_velocity().length()
        red = v.is_at_traffic_light() and v.get_traffic_light_state() == carla.TrafficLightState.Red
        s["still"] = s["still"] + 5 if (sp < 0.1 and not red) else 0
        s["still_max"] = max(s["still_max"], s["still"])
        wp = m.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Driving)
        if wp is None:
            s["offroad"] += 1
            if k - offroad_last.get(v.id, -10**9) > 40:      # one entry per vehicle per 2 s off the lanes
                n = m.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
                anyl = m.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Any)
                d = loc.distance(n.transform.location) if n else None
                offroad_events.append(dict(t=round(k * 0.05, 1), v=v.id, x=round(loc.x, 1), y=round(loc.y, 1), z=round(loc.z, 1),
                                           near_road=n.road_id if n else None, near_lane=n.lane_id if n else None,
                                           near_junction=n.is_junction if n else None, dist=round(d, 2) if d is not None else None,
                                           half_width=round(n.lane_width / 2, 2) if n else None, speed=round(sp, 1),
                                           on_lane_type=str(anyl.lane_type) if anyl else None))
            offroad_last[v.id] = k
        else:
            if loc.z < wp.transform.location.z - 1.0: s["fall"] += 1
sim_s = a.ticks * 0.05
res = {"file": a.map or a.xodr, "load_s": round(load_s, 1), "run_s": round(time.time() - t1, 1), "sim_s": sim_s,
       "spawn_points": len(m.get_spawn_points()), "vehicles": len(vehicles),
       "distance_km": round(sum(s["dist"] for s in st.values()) / 1000, 2),
       "mean_speed_kmh": round(sum(s["dist"] for s in st.values()) / max(1, len(vehicles)) / sim_s * 3.6, 1),
       "stuck_over_30s": sum(1 for s in st.values() if s["still_max"] * 0.05 > 30),
       "collisions": sum(col.values()), "vehicles_with_collision": sum(1 for c in col.values() if c),
       "lane_invasions": sum(i[0] for i in inv.values()), "solid_invasions": sum(i[1] for i in inv.values()),
       "offroad_samples": sum(s["offroad"] for s in st.values()), "vehicles_offroad": sum(1 for s in st.values() if s["offroad"]),
       "fall_samples": sum(s["fall"] for s in st.values()), "vehicles_fallen": sum(1 for s in st.values() if s["fall"]),
       "vehicles_destroyed": len(gone), "destroyed_at_s": sorted(round(x * 0.05, 1) for x in gone_at.values()),
       "removed_why": dict(why), "removed_examples": dict(whyex),
       "collisions_with": dict(col_what.most_common()), "collision_examples": dict(col_ex),
       "incidents": incidents, "solid_events": solid_events, "offroad_events": offroad_events,
       "vehicle_types": vtypes,
       "routes_m": a.routes, "route_lengths_min": min(route_len) if route_len else None, "routes_rejected": route_rejected,
       "alive_segments": len(alive) if alive is not None else None,
       "samples": (a.ticks // 5) * len(vehicles)}
for s in sensors: s.stop(); s.destroy()
client.apply_batch([carla.command.DestroyActor(v.id) for v in vehicles if v.id not in gone])
settings.synchronous_mode = False; world.apply_settings(settings)
print(json.dumps(res, indent=1))
if a.out: json.dump(res, open(a.out, "w"), indent=1)
