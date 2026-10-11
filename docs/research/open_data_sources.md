# Open data sources for improving OSM-derived OpenDRIVE (researched 2026-10-11)

Target bbox (DTLA): lon -118.284..-118.231, lat 34.031..34.077 (~25 km²). Counts below are live `returnCountOnly=true` queries run 2026-10-11; sizes are estimates. Nothing was downloaded.
**ArcGIS bbox query template** (works for every Esri layer listed): `<layer>/query?where=1=1&geometry=-118.284,34.031,-118.231,34.077&geometryType=esriGeometryEnvelope&inSR=4326&spatialRel=esriSpatialRelIntersects&outFields=*&outSR=4326&f=geojson`, then page with `resultOffset`/`resultRecordCount` (max 1000-2000 rows per request).
Abbreviations: LAHUB = `https://maps.lacity.org/lahub/rest/services`; LADOT_AGOL = `https://services1.arcgis.com/tp9wqSVX1AitKgjd/arcgis/rest/services`; BOE_AGOL = `https://services5.arcgis.com/7nsPwEMP38bSkCjy/arcgis/rest/services`.
Licence note (LA City): data.lacity.org (Socrata) items are **CC0 1.0 / "Public Domain"**. GeoHub/LAHub ArcGIS items carry only an "as-is" disclaimer with no named licence. Treat them as City open data under the CC0 policy and keep the City attribution.

## A. Downtown Los Angeles, ranked by expected accuracy gain

| # | Dataset (publisher) | Endpoint / format | Bbox count, size | Licence, updated | Accuracy | Improves |
|---|---|---|---|---|---|---|
| 1 | **BOE Sidewalks (Mapped Areas): Curbs, Sidewalks, Driveways, Parkways, Access Ramps, Alley Sidewalk** (City of LA Bureau of Engineering) | `BOE_AGOL/Sidewalks_Mapped_Areas/FeatureServer/{3 Curbs,6 Sidewalks,4 Driveways,5 Parkways,1 Access Ramps,2 Alley Sidewalk}`. Polygons with CALC_WIDTH/CALC_LENGTH. Esri JSON or GeoJSON | Curbs 43,742; Sidewalks 37,900; Parkways 9,315; Driveways 3,686; Ramps 4,035. About 60-100 MB GeoJSON in total | City open data (no named licence). Last edit 2026-08-28 | Planimetric, digitised from LARIAC orthos at about 0.1-0.3 m (not stated) | **Biggest win.** True road-edge, curb and sidewalk geometry. Gives the carriageway width (curb-to-curb), border/sidewalk lane widths, driveway gaps and curb-ramp positions |
| 2 | **USGS 3DEP lidar, CA_LosAngeles_B23** (USGS, flown with LA County as LARIAC7) | EPT: `https://s3-us-west-2.amazonaws.com/usgs-lidar-public/CA_LosAngeles_1_B23/ept.json` (also `_2_B23`). LAZ via PDAL `readers.ept` with `bounds` and a `Classification[2:2],[17:17]` filter | Full work unit 216.9 bn points. Bbox at QL1 density: roughly 0.3-1 bn points, about 2-6 GB LAZ, so stream it and thin with `resolution` | Public domain (USGS). Flown 2023-01-08 to 2024-01-07, report dated 2025-06-24 | QL1. NVA RMSEz 5.48 cm (VVA95 24.8 cm). EPSG 6340 + NAVD88 (GEOID18) | Road-surface elevation profile, superelevation (crossfall from ground points within curb polygons), **class 17 bridge deck** for overpasses and freeway stacks |
| 3 | **USGS 3DEP 1 m DEM, CA_LosAngeles_B23** | TNM API `https://tnmaccess.nationalmap.gov/api/v1/products?datasets=Digital%20Elevation%20Model%20(DEM)%201%20meter&bbox=-118.284,34.031,-118.231,34.077` returns tiles `.../1m/Projects/CA_LosAngeles_B23/TIFF/USGS_1M_11_x38y377…tif` and `x38y378`. GeoTIFF (read windows via /vsicurl/). Whole-US dynamic option: `https://elevation.nationalmap.gov/arcgis/rest/services/3DEPElevation/ImageServer/exportImage?bbox=…` | 2 tiles × about 355 MB (10 km tiles), only a few MB once windowed | Public domain. Published 2025-08-11 (older 2016 tiles also exist) | DEM NVA RMSEz 5.74 cm. Bare earth, hydro-flattened, 0.5 m source GSD | Fast elevation and grade for `<elevationProfile>`. **Bridges are removed from bare earth**, so use #2 class 17 or OSM `bridge`/`layer` tags on top |
| 4 | **FHWA HPMS 2024, California** (FHWA/Caltrans) | `https://geo.dot.gov/server/rest/services/Hosted/HPMS_FULL_CA_2024/FeatureServer/0`. Polylines with through_lanes, dir_through_lanes, lane_width, median_type/width, shoulder widths, turn_lanes_l/r, signal_type, number_signals, stop_signs, speed_limit, aadt | 2,532 segments, about 3 MB | Public domain (US federal). Data year 2024 | Route/milepost-based (LRS); align it to OSM | Lane count, **lane width**, median width and turn-bay counts for arterials and freeways. Good for checking OSM `lanes` |
| 5 | **LADOT Signal Locations / LA_City_Signals** | `LAHUB/LADOT/MapServer/30` (587 in bbox). `LADOT_AGOL/CurrentSignals/FeatureServer/0` (523; fields CAT, TYPE, HOR/VERT head counts F1-F7_CAR). `LADOT_AGOL/dot_signal_plans/FeatureServer/0` (532, links to signal plan PDFs) | Under 1 MB | City open data. Signal plans last edited 2018-10-31 | Intersection-node points | Which junctions get `<signal>` and controllers. HOR/VERT and head counts help pick signal types. Plan PDFs carry phasing (manual extraction) |
| 6 | **LADOT Crosswalks** | `LAHUB/LADOT/MapServer/4`. Points with ControlTypeDesc, MarkingTypeDesc (ladder/continental), color, Primary_St/Cross_Stre | 2,050, under 1 MB | City open data | Approach-point location | `crosswalk` objects and road-marking type per junction leg |
| 7 | **LADOT Stop and Yield Signs; Regulatory Signs** | `LAHUB/LADOT/MapServer/19` (1,367; SIGN_TYPE, FACE_DIREC). `LAHUB/LADOT/MapServer/18` (4,488; MUTCD-style DESCRIPTIO, STKNUMBER) | Under 2 MB | City open data | Points, with facing direction | `<signal>`/`<object>` signs, priority (stop/yield) at unsignalised junctions, no-turn and parking regulations |
| 8 | **BSL Street Lights** (Bureau of Street Lighting) | `LAHUB/Bureau_of_Street_Lighting/MapServer/0` (fields POSTDESC, LAMPA-F). Also Socrata `https://data.lacity.org/resource/9ei6-svt8` (map; use `within_box(...)`) | 13,551, about 4 MB | **CC0 1.0**. Socrata updated 2026-10-02 | Pole points | Streetlight poles as `<object type="pole">` with lamp count |
| 9 | **LADOT Speed Zone Surveys** | `LADOT_AGOL/speed_survey_status/FeatureServer/0` (BEGIN_, END_, SPEED, EXPDATE, ACTIVE) | 37 segments | City open data. **Stale (2015-16)** | Survey segment polylines | Posted speed limits where OSM `maxspeed` is missing. The old `SpeedLimits_posted` service has been removed (returns 400) |
| 10 | **LADOT Bikeways** | `LADOT_AGOL/LA_City_Bikeways/FeatureServer/0` (Class I-IV, ClassIV_Protection, One_Dir_BW, ST_WIDTH) or `LAHUB/LADOT/MapServer/0` | 401, under 1 MB | City open data. Last edit 2026-08-11 | Centreline | `biking` lanes, buffer/protection, one-way bike lanes |
| 11 | **BOE Streets (Centerline) and Intersections** | `LAHUB/Street_Information/MapServer/36` (2,844; PLANNING_STREET_STANDARD, Street_Designation such as "Avenue II", ST_SUBTYPE). `/9` Intersections (1,844). `/39` Streets (Width) (WIDTH, but only 43 in bbox). `/0` Bridges and Structures (178; clearances) | About 3 MB | City open data | Survey-grade official centreline | Cross-check of OSM topology. Mobility Plan street standard gives the default roadway/ROW width. Bridge vertical clearance |
| 12 | **LADOT Parking Meter Sensors / Inventory; Red Curb** | `LAHUB/LADOT/MapServer/16` (5,942 spaces). Socrata `s49e-q6j2` / `epux-nymu` (Public Domain, updated 2026-10-10). `LAHUB/LADOT/MapServer/29` Red Curb (36 polygons) | About 2 MB | Public Domain / CC0 | Per-space points | `parking` lanes and positions, no-stopping zones |
| 13 | **LA County CAMS Address Street Lines** (LA County eGIS) | `https://services.arcgis.com/RmCCgQtiZLDCtblq/arcgis/rest/services/eGIS_Addressing_ROAD_LINES/FeatureServer/0` (Elevation, Surface, DrivingDir, Accuracy) | 4,597 | County terms (attribute County). Updated 2026-10 | n/a | Has no lanes or width. Useful only for one-way direction and grade-separation `Elevation` codes |

Other notes for LA:
- **LARIAC.** The LARIAC7 QL1 lidar (late 2023 to Jan 2024) is the same USGS-partnered collection as #2. The public route to it is USGS/AWS. LARIAC orthophotos and the raw deliverables are licensed to participants only. LARIAC4 (2016) is `USGS_LPC_CA_LosAngeles_2016_LAS_2018` (75 bn points, also on EPT).
- **Road-surface class 11.** Neither LA lidar has ASPRS class 11 (road surface). Classes in B23: 1, 2, 4, 5, 6, 7, 9, 17, 18, 20, 22. Derive road-surface points as class 2 points inside the BOE curb or roadway polygons.
- **LADOT Downtown/Central City Parking Districts and DASH routes** are also on `LAHUB/LADOT/MapServer`. They are useful only as context.
- **Lane striping (lane lines).** No open lane-line dataset was found for LA. Lane positions must come from imagery (NAIP 0.6 m, public domain, via Planetary Computer/AWS COG) or from Mapillary (below).

## B. Global / pluggable sources (ranked)

1. **National lidar and DEMs (best elevation).** Rank by licence and quality:
   - NL **AHN4/5**: PDOK/GeoTiles COPC/LAZ, 0.5 m DTM. CC0. Class 26 covers civil structures such as bridges.
   - FR **IGN LiDAR HD**: COPC tiles at `data.geopf.fr`, 10+ pts/m². Licence Ouverte 2.0. Class 17 bridge.
   - UK **EA National LIDAR Programme**: 1 m DTM/DSM and point cloud. OGL v3.
   - DK **Danmarks Højdemodel**: dataforsyningen.dk, 0.4 m DTM plus point cloud. Free-data terms with attribution.
   - JP **GSI 5 m/1 m DEM**: GSI terms, CC BY 4.0-compatible. The PLATEAU CityGML road polygons (LOD1-3, some cities with lanes) are CC BY 4.0-compatible.
   - Also: DE state DGM1 (e.g. NRW, dl-de/zero-2.0), ES PNOA-LiDAR (CC BY 4.0), CH swissSURFACE3D (OGD).
   - US: 3DEP EPT on `s3://usgs-lidar-public` (public domain). Index: `https://usgs.entwine.io` / the `resources.geojson` boundary file.
2. **Copernicus DEM GLO-30** (ESA/Airbus). COGs at `s3://copernicus-dem-30m/` (no auth). Copernicus DEM licence: free worldwide with attribution. Absolute vertical accuracy under 4 m (LE90). It is a DSM, so it includes buildings and trees; use it only as a coarse fallback.
   - **FABDEM** (Univ. Bristol): bare-earth GLO-30, **CC BY-NC-SA 4.0, non-commercial only**.
   - NASADEM: public domain, 30 m.
3. **Overture Maps transportation/segment** (Overture Maps Foundation). GeoParquet on `s3://overturemaps-us-west-2/release/2026-09-23.1/theme=transportation/type=segment/` (monthly, latest 2026-09-23.1). Query the bbox with DuckDB on the `bbox` column. Properties: `speed_limits`, `width_rules` (edge-to-edge metres), `road_surface`, `road_flags` (bridge/tunnel…), `level_rules`, `access_restrictions`, `prohibited_transitions`, `destinations`.
   - **No lane count or lane layout.**
   - **ODbL 1.0**, because it is OSM-derived. Some segment sources (e.g. TomTom) are CDLA-Permissive-2.0; check `sources[].license` per feature.
   - Gain over raw OSM is small (same source). Its value is normalised GERS IDs and linear-referenced speed and width rules.
4. **Mapillary map features** (Meta). Traffic signs (about 1,500 classes) and points (crosswalk lines, street lamps, poles, lane-marking arrows…), triangulated from at least 2 images for signs and at least 3 for other points.
   - API: `https://graph.mapillary.com/map_features?bbox=minLon,minLat,maxLon,maxLat&fields=id,object_value,geometry,aligned_direction,first_seen_at,last_seen_at&access_token=MLY|…` (bbox kept small, max 2,000 results per call).
   - Vector tiles: `https://tiles.mapillary.com/maps/vtp/mly_map_feature_traffic_sign/2/{z}/{x}/{y}?access_token=…` (also `mly_map_feature_point`).
   - Needs a free client token. Imagery is CC BY-SA 4.0. Map features are offered under the Mapillary Terms for personal and commercial use with attribution; check the terms before redistributing.
   - Position accuracy is a few metres.
   - Improves: sign and signal-head positions and types, crosswalk marking presence, streetlight positions.
5. **National authoritative road and signal data:**
   - NL **NWB + WKD** (speed limits, lanes) and **NDW Verkeersborden** (every traffic sign, with facing). CC0.
   - NL **BGT** (road polygons split into carriageway/bike/foot at about 20 cm, which gives curbs). CC0, via PDOK.
   - DK **GeoDanmark** (Vejkant road edges, CC BY 4.0).
   - UK **OS Open Roads** (OGL; MasterMap Highways is not open).
   - US **FHWA HPMS** for all states (`geo.dot.gov/.../HPMS_FULL_<ST>_<YEAR>`, public domain; lanes, lane width, speed, signals).
   - US city open-data portals (NYC planimetrics, SF, Seattle "Traffic Signal Assemblies" etc.).
6. **Microsoft Road Detections** (github.com/microsoft/RoadDetections). Global road centrelines detected from Bing imagery 2020-22. ODbL. Gap-filling only, with no attributes.
   - Esri World Imagery and Community Maps are **not** open for derivative datasets.
7. **OpenAerialMap** (HOT). `https://api.openaerialmap.org/meta?bbox=…` gives COG URLs. Mostly CC BY 4.0. Patchy coverage of drone and aerial imagery, useful for digitising lane lines or markings.
   - US: **NAIP** (public domain, 0.3-0.6 m).

## Recommended plug-in order for the library

1. Curb/edge polygons (#A1, or BGT/PLATEAU) for widths.
2. Lidar EPT/COPC ground plus class 17 (or a national DTM), falling back to GLO-30.
3. HPMS / national lane and speed attributes.
4. City signals, signs and crosswalks, falling back to Mapillary.
5. Streetlights, parking and bike lanes.
