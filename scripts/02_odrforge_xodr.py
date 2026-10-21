"""OSM -> OpenDRIVE with OdrForge (osm2xodr, carla profile), the alternative to 02_build_xodr.py + 02b..02f.

OdrForge writes what the SUMO path adds in later steps: real elevation from the area's terrain grids (lidar / DEM,
junction planes, crossfall), stop and give-way signs, speed-limit signs and traffic-light controllers in the
CARLA sign catalogue. Same map frame as the rest of the pipeline: origin = config LAT0/LON0, clipped to BBOX.

    MIRRORMILE_XODR=odrforge python scripts/02_odrforge_xodr.py
"""
import subprocess
import sys

import config as C


def main():
    exe = C.ODRFORGE_EXE
    if not exe.exists():
        sys.exit(f"OdrForge converter not found: {exe} (set ODRFORGE_EXE)")
    s, w, n, e = C.BBOX
    cmd = [str(exe), str(C.OSM_FILE), str(C.XODR_FILE), "--target", "carla", "--country", "US",
           "--origin", f"{C.LAT0},{C.LON0}", "--name", C.MAP_NAME, "--clip", f"{s},{w},{n},{e}"]
    for flag, name in (("--terrain", "ground.odrz"), ("--structures", "decks.odrz")):
        grid = C.ODRFORGE_TERRAIN / name
        if grid.exists():
            cmd += [flag, str(grid)]
        else:
            print(f"warning: {grid} missing - heights without it")
    print(" ".join(cmd))
    r = subprocess.run(cmd, capture_output=True, text=True)
    print(r.stdout[-3000:])
    if r.returncode not in (0, 1):  # 1: written, with validation findings (listed above)
        sys.exit(f"osm2xodr failed (code {r.returncode})\n{r.stderr[-2000:]}")
    if not C.XODR_FILE.exists():
        sys.exit("osm2xodr wrote no OpenDRIVE file")
    print(f"wrote {C.XODR_FILE}")


if __name__ == "__main__":
    main()
