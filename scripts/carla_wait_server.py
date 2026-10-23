"""Block until the CARLA server answers on --port (or --limit seconds pass); prints the map name."""
import argparse, sys, time
import carla
ap = argparse.ArgumentParser(); ap.add_argument("--port", type=int, default=2000); ap.add_argument("--limit", type=float, default=1800)
a = ap.parse_args()
t0 = time.time()
while time.time() - t0 < a.limit:
    try:
        c = carla.Client("localhost", a.port); c.set_timeout(10.0)
        print("ready after %.0f s, map %s" % (time.time() - t0, c.get_world().get_map().name), flush=True)
        sys.exit(0)
    except RuntimeError:
        time.sleep(15)
print("server not ready after %.0f s" % a.limit); sys.exit(1)
