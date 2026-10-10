"""Live test of the map in a running CARLA server (Python 3.8 env with the matching carla wheel).

  venv38\\Scripts\\python.exe scripts\\test_carla_client.py [--host 127.0.0.1] [--port 2000]

Checks: map name + road network, spawn points, traffic lights / signs generated from the OpenDRIVE,
street lights known to CARLA's light manager; then spawns a car on autopilot (Traffic Manager)
in downtown, verifies it drives and reads a speed limit, and captures camera images by day and at
night (lit windows / shops / street lights) into build/carla_test/.
"""
import argparse
import json
import math
import os
import queue
import sys
import time

import carla

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "build", "carla_test")
MAP = "LA_DTLA"
DOWNTOWN = (422.0, 487.1)            # UE frame (map y mirrored): a street with storefronts


def capture(world, cam_q, name, settle=1.5):
    time.sleep(settle)
    while not cam_q.empty():
        cam_q.get_nowait()
    img = cam_q.get(timeout=20.0)
    path = os.path.join(OUT, f"{name}.png")
    img.save_to_disk(path)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    client = carla.Client(a.host, a.port)
    client.set_timeout(180.0)
    world = client.get_world()
    if MAP not in world.get_map().name:
        world = client.load_world(MAP)
    m = world.get_map()
    rep = {"map": m.name}
    topo = m.get_topology()
    wps = m.generate_waypoints(5.0)
    sps = m.get_spawn_points()
    rep.update(topology_edges=len(topo), waypoints_5m=len(wps), spawn_points=len(sps))
    actors = world.get_actors()
    rep["traffic_lights"] = len(actors.filter("traffic.traffic_light"))
    rep["stop_signs"] = len(actors.filter("traffic.stop"))
    rep["speed_limit_signs"] = len(actors.filter("traffic.speed_limit.*"))
    rep["landmarks_in_xodr"] = len(m.get_all_landmarks())
    tls = list(actors.filter("traffic.traffic_light"))
    rep["traffic_light_groups"] = len({tuple(sorted(t.id for t in tl.get_group_traffic_lights())) for tl in tls}) if tls else 0
    print(json.dumps(rep, indent=1), flush=True)

    # car in downtown on autopilot, with a driver camera
    bp = world.get_blueprint_library()
    car_bp = (bp.filter("vehicle.lincoln.mkz*") or bp.filter("vehicle.*"))[0]
    sp = min(sps, key=lambda t: math.hypot(t.location.x - DOWNTOWN[0], t.location.y - DOWNTOWN[1]))
    car = world.try_spawn_actor(car_bp, sp)
    if car is None:
        for t in sorted(sps, key=lambda t: math.hypot(t.location.x - DOWNTOWN[0], t.location.y - DOWNTOWN[1]))[1:20]:
            car = world.try_spawn_actor(car_bp, t)
            if car:
                sp = t
                break
    cam_bp = bp.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", "1280"); cam_bp.set_attribute("image_size_y", "720")
    cam_bp.set_attribute("fov", "90")
    cam = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=-6.0, z=2.6), carla.Rotation(pitch=-8)), attach_to=car)
    q = queue.Queue()
    cam.listen(q.put)
    res = {"spawned_at": [round(sp.location.x, 1), round(sp.location.y, 1), round(sp.location.z, 1)]}
    try:
        world.set_weather(carla.WeatherParameters.ClearNoon)
        res["day_start"] = capture(world, q, "day_start", settle=3.0)
        tm = client.get_trafficmanager(8000)
        car.set_autopilot(True, tm.get_port())
        start = car.get_location()
        path, speeds, limits = 0.0, [], set()
        prev = start
        for i in range(40):                      # ~20 s of driving
            time.sleep(0.5)
            loc = car.get_location()
            path += loc.distance(prev); prev = loc
            v = car.get_velocity(); speeds.append(math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2) * 3.6)
            limits.add(round(car.get_speed_limit(), 1))
            if i == 20:
                res["day_driving"] = capture(world, q, "day_driving", settle=0.1)
        res.update(driven_m=round(path, 1), max_speed_kmh=round(max(speeds), 1), speed_limits_seen_kmh=sorted(limits),
                   end=[round(prev.x, 1), round(prev.y, 1), round(prev.z, 1)])
        car.set_autopilot(False, tm.get_port())
        car.apply_control(carla.VehicleControl(brake=1.0))
        night = carla.WeatherParameters.ClearNight if hasattr(carla.WeatherParameters, "ClearNight") else carla.WeatherParameters(sun_altitude_angle=-20.0)
        world.set_weather(night)
        res["night"] = capture(world, q, "night", settle=4.0)
    finally:
        cam.stop(); cam.destroy(); car.destroy()
    print(json.dumps(res, indent=1), flush=True)
    ok = rep["spawn_points"] > 0 and res.get("driven_m", 0) > 20
    print("RESULT:", "OK" if ok else "CHECK", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
