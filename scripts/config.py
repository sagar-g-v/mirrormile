"""Shared settings for the OSM -> SUMO -> OpenDRIVE -> Blender -> CARLA pipeline."""
from pathlib import Path

MAP_NAME = "LA_DTLA"

# Core area (WGS84): Downtown LA from the financial district / Pershing Square north-west to the
# Four Level Interchange (US-101 / I-110), incl. the Harbor Freeway, its street overpasses,
# Bunker Hill bridges and the 2nd / 3rd Street tunnels.   south, west, north, east
BBOX = (34.0440, -118.2700, 34.0640, -118.2450)
# Extra margin (deg) fetched around the core for terrain/imagery context.
MARGIN = 0.0015

# Download CRS for rasters (UTM 11N, WGS84).
UTM_EPSG = 32611
# Map CRS: transverse Mercator centred on the area, origin (0,0) = BBOX centre.
# CARLA reads +lat_0/+lon_0 from the OpenDRIVE geoReference for GNSS/geolocation,
# so the map frame must be a local tangent projection around that point.
LAT0 = round((BBOX[0] + BBOX[2]) / 2, 6)
LON0 = round((BBOX[1] + BBOX[3]) / 2, 6)
MAP_PROJ = (f"+proj=tmerc +lat_0={LAT0} +lon_0={LON0} +k=1 +x_0=0 +y_0=0 "
            "+ellps=WGS84 +datum=WGS84 +units=m +no_defs")

DEM_RES = 1.0      # m, USGS 3DEP 1 m (LA County 2016 lidar-derived)
ORTHO_RES = 0.5    # m, resampled from USGS NAIP (0.6 m)

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "data"
BUILD = ROOT / "build"
TEX = ROOT / "textures"
EXPORT = ROOT / "export" / MAP_NAME
for _d in (DATA, BUILD, TEX, EXPORT):
    _d.mkdir(parents=True, exist_ok=True)

OSM_FILE = DATA / "area.osm"
DEM_UTM = DATA / "dem_utm.tif"
DEM_WGS = DATA / "dem_wgs84.tif"
ORTHO_UTM = DATA / "ortho_utm.tif"
NET_FILE = BUILD / f"{MAP_NAME}.net.xml"
XODR_FILE = EXPORT / f"{MAP_NAME}.xodr"
GEOM_FILE = BUILD / "geometry.npz"     # meshes produced by 03, consumed by Blender
META_FILE = BUILD / "meta.json"


def outer_bbox():
    s, w, n, e = BBOX
    return (s - MARGIN, w - MARGIN, n + MARGIN, e + MARGIN)
