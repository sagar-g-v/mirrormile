# MirrorMile

**Real streets, mirrored mile for mile into simulation.**

MirrorMile builds simulation-ready digital twins of real cities from open geodata: a lane-accurate
OpenDRIVE road network with measured elevation, plus matching 3D meshes for roads, markings,
sidewalks, bridges, tunnels, buildings, vegetation, street furniture and surrounding terrain,
packaged for driving simulators.

> Status: active development. Full documentation, benchmarks and screenshots will follow.

## Compatibility

| Target | Support |
|---|---|
| **CARLA** (UE5 `ue5-dev`; OpenDRIVE maps for CARLA 0.9.x) | OpenDRIVE + FBX per semantic label, import script for the Unreal Editor, spawn points, traffic lights / stop signs / speed limits as OpenDRIVE signals |
| **OpenDRIVE** | Lanes, junctions, elevation profiles, speed limits, traffic-light controllers, stop and speed-limit signals |
| **Unreal Engine 5** | FBX meshes with PBR materials, semantic folder layout (Road, RoadLine, SideWalk, Building, Vegetation, Terrain, Bridge, GuardRail, Wall, Pole, TrafficSign) |
| **Blender 5.x** | Full scene build, previews and FBX export |
| **SUMO 1.28** | Network conversion (`netconvert`) |

## Pipeline (overview)

1. **Fetch**: OpenStreetMap, USGS 3DEP elevation and lidar point cloud, NAIP aerial / near-infrared imagery
2. **Road network**: SUMO `netconvert` -> OpenDRIVE, junction regeneration, lidar-based elevation,
   bridge / tunnel levels and grade separation, stop signs, speed limits and signs
3. **Geometry**: road / sidewalk / junction surfaces, lane markings and pavement legends,
   bridges and tunnels, terrain, lidar-derived building massing, roofs, vegetation, street furniture
4. **Build**: textures, Blender scene, FBX export
5. **Simulator**: Unreal / CARLA import, validation of the lane graph with LibCarla

```powershell
.\run_all.ps1                 # full build; -SkipDownload reuses downloaded data
```

The map area and name are set in `scripts/config.py`.

## Requirements

- Python 3.12+ (`pip install -r requirements.txt`), Python 3.8 environment with the `carla` wheel for validation
- Blender 5.x, CARLA UE5 editor build (for the import step)

## Data and licensing

Generated maps contain third-party data with attribution and share-alike obligations; see
[NOTICE.md](NOTICE.md). The MirrorMile source code is proprietary; see [LICENSE](LICENSE).
