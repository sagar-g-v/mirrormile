# LA_DTLA map – data sources, licences and attribution

This map (OpenDRIVE road network, meshes, textures) is built by the scripts in this project from the
sources below. Ship this file with every distributed copy of the map.

## Required attribution

- **Map data © OpenStreetMap contributors**, available under the Open Database License (ODbL 1.0),
  https://www.openstreetmap.org/copyright. (Includes LA County building data imported into OSM.)
- Elevation and lidar: **U.S. Geological Survey, 3D Elevation Program** (3DEP 1 m DEM; lidar project
  CA_LosAngeles_1_B23). Public domain.
- Imagery: **USDA Farm Production and Conservation – NAIP**, via USGS The National Map. Public domain.
- Materials: **ambientCG.com** (Asphalt031, Concrete047A, Concrete034, Concrete036, Concrete028, Tiles010,
  Facade001/006/018A/018C/019A/019C/020A/020C),
  CC0 1.0. Attribution is not required but is given here.

## Business names on storefront signs

Shop, restaurant, hotel and other business names and categories come from OpenStreetMap (via the
Overpass API) and are covered by the ODbL attribution above. The names may be trademarks of their
owners; for commercial distribution consider `GENERIC_NAMES = True` in `scripts/03f_storefronts.py`,
which shows the business type instead.

## OpenStreetMap / ODbL

- The **OpenDRIVE file (`LA_DTLA.xodr`)** and any extracted OSM vectors (road centrelines, building footprints)
  are a **Derivative Database** of OSM. If you distribute or publicly use them, offer them under the
  **ODbL**, or offer the means to recreate them (this project's scripts and the OSM extract date).
- The rendered 3D meshes, textures and images are a **Produced Work**. You may license them under your
  own terms, but they must carry the OpenStreetMap attribution above.

## Not used

No Google Maps Platform content of any kind (3D Tiles, Street View, satellite, Roads, Solar) is used
in this pipeline. LA County LARIAC services are not used directly; county building data enters only
through OpenStreetMap.

## Tools (their licences do not cover outputs)

- SUMO netconvert (EPL-2.0)
- Blender (GPL; scenes and exports belong to the author)
- QGIS (GPL)
- Python libraries: numpy, scipy, shapely/GEOS, rasterio/GDAL, pyproj/PROJ, laspy, lazrs, Pillow, lxml,
  requests (BSD/MIT/Apache/LGPL)

`scripts/05_blender_build.py` uses the Blender Python API. If you distribute that script, use a
GPL-compatible licence for it.

## When packaging for CARLA / Unreal Engine

- Placed CARLA vegetation meshes are CARLA content: CARLA code is MIT, CARLA assets are CC-BY 4.0,
  so attribute "CARLA" if you redistribute them.
- Packaged Unreal Engine builds fall under the Unreal Engine EULA, which covers royalties and seats
  depending on revenue and use. Check the current terms.
