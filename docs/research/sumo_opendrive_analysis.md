# SUMO netconvert OSM -> OpenDRIVE: how it works and where it fails

Scope: an implementation-oriented reading of Eclipse SUMO `main` (fetched 2026-10) and the CARLA fork
(`carla-simulator/sumo`, branch `carla_osm2odr`, used by `carla.Osm2Odr`). It is meant to guide the
design of a native OSM -> OpenDRIVE generator for CARLA.

Primary sources read in full or grepped:
- `src/netimport/NIImporter_OpenStreetMap.cpp` (NodesHandler, EdgesHandler, `insertEdge`, `reconstructLayerElevation`)
- `src/netwrite/NWWriter_OpenDrive.cpp/.h`, `src/netwrite/NWFrame.cpp` (options)
- `src/netbuild/NBFrame.cpp`, `NBNodeCont.cpp` (`createClusterId`, `joinNodeCluster`), `NBNode.cpp` (`bezierControlPoints`)
- `data/typemap/osmNetconvert.typ.xml`
- CARLA fork: `src/netwrite/NWWriter_OpenDrive.cpp`, `src/OSM2ODR.cpp`
- Docs: sumo.dlr.de `Networks/Import/OpenStreetMap`, `Networks/Export` (OpenDRIVE section)

Line numbers below refer to the files as fetched (`NWWriter_OpenDrive.cpp` = 1432 lines in SUMO main).

---

## 1. Pipeline

```
OSM XML
  -> NIImporter_OpenStreetMap
       NodesHandler : lat/lon/ele + a few node tags -> NIOSMNode
       EdgesHandler : way tags -> Edge (type id "highway.X", lanes, widths, speeds, sidewalk/cycleway flags)
       insertEdge() : every way is split at nodes shared with other ways; each split piece becomes
                      1 or 2 *directed* NBEdges ("id#k" and "-id#k"), sidewalks/bike lanes are added
                      as extra SUMO lanes (index 0 = outermost)
       RelationHandler: turn restrictions, PT routes
  -> NBNetBuilder::compute()   (netbuild)
       geometry.remove / edges.join, junctions.join (clusters within 10 m), roundabouts.guess,
       ramps.guess, sidewalks.guess, crossings.guess, tls.guess-signals / tls.join,
       node shapes (NBNodeShapeComputer, radius 4 m), edge cut-back to node shape,
       lane-to-lane connections, internal (junction) lanes, TLS programs (NBOwnTLDef)
  -> NWWriter_OpenDrive::writeNetwork()
       one <road> per directed NBEdge, one <junction> per NBNode with connections,
       one connecting <road> per (incoming edge, outgoing edge, group of parallel lanes),
       <controller> per traffic-light definition, signals/objects from TLS + optional loaded shapes
```

The writer works on the *SUMO simulation model*, not on OSM. Everything OpenDRIVE can express but
SUMO cannot (variable widths, lane sections, bidirectional roads, superelevation, curbs, stop lines,
sign semantics) is lost before the writer runs. This is the central design flaw.

CARLA's `OSM2ODR::ConvertOSMToOpenDRIVE` hard-codes: `--geometry.remove --ramps.guess --edges.join
--junctions.join --roundabouts.guess --tls.group-signals true --sidewalks.guess true --osm.sidewalks true
--osm.crossings true --default.lanewidth <s.default_lane_width> --default.sidewalk-width <..>
--keep-edges.by-type highway.<types>` plus optional `--osm.layer-elevation`, `--offset.x/y`,
`--offset.disable-normalization`. Note: no `--osm.turn-lanes`, no `--osm.elevation`, no `--lefthand`,
no `--osm.bike-access`, no `--type-files` (so the default European typemap is always used).

### 1.1 OSM tags actually consumed

Node tags (NodesHandler, line ~1662 whitelist — everything else is dropped unless `--osm.all-attributes`):
| Tag | Effect |
|---|---|
| `highway=traffic_signals`, `crossing=traffic_signals` | node `tlsControlled` -> `NBOwnTLDef` on that node |
| `highway=crossing` | `pedestrianCrossing` (used only with `--osm.crossings`) |
| `ele` | node z, only with `--osm.elevation` (off in CARLA) |
| `railway=*`, `railway:signal*`, `crossing:barrier/light`, `public_transport`, `highway=bus_stop`, `name` | rail / PT only |

**Ignored node tags:** `highway=stop`, `highway=give_way`, `stop=all|minor`, `direction=*`,
`traffic_signals:direction`, `traffic_sign=*` (only via polyconvert POIs, see 2.8), `kerb`,
`crossing=marked/zebra/uncontrolled` distinction, `traffic_calming`, `highway=turning_circle`, `barrier`.

Way tags (EdgesHandler whitelist, lines 1913-1956):
| Tag | Effect |
|---|---|
| `highway` | type id -> typemap row (lanes, speed, priority, oneway, permissions, width) |
| `lanes`, `lanes:forward/backward` | lane counts (list `a;b` -> minimum) |
| `oneway`, `junction=roundabout/circular` | direction; roundabout priority |
| `maxspeed`, `maxspeed:forward/backward`, `maxspeed:type`, `zone:maxspeed` | edge speed (country symbols like `DE:urban` mapped by table) |
| `width` | split evenly over *tagged* lanes (excludes sidewalks/bike lanes) |
| `width:lanes(:forward/backward)` | per-lane width; silently dropped if count mismatches |
| `placement` | lateral shift, **only for explicit oneways** (else counted & skipped) |
| `sidewalk`, `sidewalk:left/right/both` | adds a pedestrian *lane* (only with `--osm.sidewalks`) |
| `cycleway`, `cycleway:left/right/both`, `oneway:bicycle`, `segregated` | bike lane (only with `--osm.bike-access`) |
| `busway*`, `bus/psv(:lanes)`, `vehicle:lanes`, `bicycle:lanes` | lane permissions |
| `turn:lanes(:forward/backward/both_ways)` | turn signs -> connection guessing (only with `--osm.turn-lanes`) |
| `change`, `change:lanes(:fwd/bwd)` | SUMO lane-change prohibition (**not written to xodr road marks**) |
| `layer` | elevation heuristic only with `--osm.layer-elevation` |
| `access=no`, `foot`, `bicycle`, `emergency`, `service` | permissions / type |
| `parking:*`, `parking:lane:*` | only for `--parking-output` (never in xodr) |
| `name`, `ref` | road `name` |

**Ignored way tags relevant to driving maps:** `bridge`, `tunnel`, `incline`, `surface`, `smoothness`,
`lit`, `divider` (read only via `--osm.all-attributes`), `maxspeed:lanes`, `minspeed`, `access:lanes`,
`destination*`, `overtaking`, `lane_markings=no`, `sidewalk:*:width`, `sidewalk:*:kerb`,
`cycleway:*:width`, `parking:*:orientation/width`, `width:carriageway`, `est_width`,
`placement:start/end/forward/backward`, `turn:lanes` without the option, `area:highway` (polygons),
`junction=yes/intersection` areas, `highway=*` areas, `kerb`, `embankment`, `cutting`.

### 1.2 Typemap defaults (osmNetconvert.typ.xml)
motorway 2 lanes / 39.44 m/s (142 km/h); trunk, primary, secondary 27.78 m/s (100 km/h); tertiary
22.22 (80 km/h); residential/unclassified 13.89 (50 km/h); living_street 2.78; service 5.56 and
**service does not allow passenger cars** (`allow="delivery pedestrian bicycle"`). Primary defaults to
2 lanes per direction. No width given -> `SUMO_const_laneWidth` 3.2 m (CARLA overrides with
`default_lane_width`, 4.0 m by default). These are rural-German defaults; the docs themselves call the
typemap values "ad hoc". `osmNetconvertUrbanDe` exists but CARLA never loads it.

---

## 2. How the OpenDRIVE is written (NWWriter_OpenDrive)

### 2.1 Header, units, georeference
`revMajor=1 revMinor=4`, `north/south/east/west` = converted boundary, `<geoReference>` = proj string
in CDATA, optional `<offset>` (negated netOffset). Units: m, rad, speeds in m/s with no `unit`
attribute (valid default). Elevation `z` in metres from projected nodes.

### 2.2 IDs
- Numeric ids via `StringBijection`: nodes 1..N, edges and connecting roads from `nodes.size()*10`
  upward (line 76-77). Ids depend on map iteration order -> not stable across runs/edits.
- `<junction name="cluster_123_456_789_1011_#3more">`: name = SUMO node id. `createClusterId`
  joins up to `max-join-ids` (default 4) OSM node ids (~10 digits each) -> typically 50-65 chars.
- `<controller id=tl->getID()>` and signal ids `tl->getID() + "_" + tlIndex` are **raw SUMO strings**
  (`cluster_...`, `joinedS_...`, `GS_cluster_...`) -> routinely > 32 chars. CARLA's fork converts signal
  ids to ints (`getintID`) and numbers controllers itself; upstream SUMO does not.

### 2.3 Reference line / planView (normal roads)
- One road per **directed** SUMO edge; only right lanes (or left with `lefthand-left`). Two-way OSM
  streets become two unrelated roads whose inner borders coincide.
- Reference line = inner border of innermost lane (`getInnerLaneBorder`): SUMO lane polyline
  `move2side`'d by half a lane width. Polyline offsetting at kinks produces miter/self-intersection
  artefacts that are then baked into the reference line.
- 2-point shapes and pure-pedestrian edges: `writeGeomLines` (one `<line>` per segment -> heading jumps
  at every vertex).
- Otherwise `writeGeomSmooth(shape, speed, ...)` (lines 791-923):
  - `longThresh = speed` (**edge speed in m/s reused as a length in metres**), `curveCutout = speed/2`.
  - If angle change at a vertex > `straight-threshold` (default 1e-8 deg) and both adjacent segments
    are "long", two points at +-curveCutout are inserted and the vertex removed.
  - If max angle < threshold everything is written as lines.
  - Then segments >= longThresh -> `<line>`; shorter ones -> `NBNode::bezierControlPoints` with
    extrapolation `min(25, len/4)` -> quadratic/cubic Bezier unfactored into one `<paramPoly3
    pRange="normalized">`. Tangents at the ends are either the neighbouring segment or a *mean* of
    two segments (lines 874-898), so consecutive pieces are not guaranteed to share heading.
  - If control points cannot be computed, falls back to a `<line>` -> visible kink; a warning
    "Could not compute smooth shape" is printed.
  - Bezier length = polyline length of 12 samples (`init.bezier(12).length2D()`), i.e. the `length`
    attribute and therefore `s` are approximate; normalized pRange is not arc-length parametrized, so
    lane widths/road marks are sampled non-uniformly along curves.
  - No arcs, no spirals, no curvature continuity anywhere.

### 2.4 Lanes, widths, lane count changes
- Exactly one `<laneSection s="0">` per road; every lane `<width a=w b=0 c=0 d=0>` (constant).
- Lane count changes in OSM are separate ways -> separate SUMO edges -> separate roads joined by a
  *junction* (every node with connections becomes a `<junction>`, even a simple two-edge continuation).
  In the connecting road, lane width is linear `b=(wOut-wIn)/len`; a new lane is created by a second
  connection from the same incoming lane, i.e. two full-width overlapping connecting lanes. No tapers.
- `ramps.guess` adds acceleration/deceleration lanes by splitting the motorway edge and adding a full
  width lane over the ramp length -> abrupt lane appearance/disappearance.
- Two-way way with `lanes=1` and no width: each direction gets half of the default width
  (lines 753-764) -> 1.6 m (or 2.0 m in CARLA) lanes.
- Lane types: `getLaneType(permissions)` -> driving / sidewalk / biking / rail / tram / restricted /
  none. No `border`, `curb`, `shoulder`, `parking`, `median`, `entry/exit/onRamp/offRamp`.
- `level="true"` everywhere; lanes have no `<height>`; no `<link>` predecessor/successor on normal
  roads (only on connecting roads).
- Uneven stop lines are only warned about (`checkLaneGeometries`); `rectangular-lane-cut` is forced.

### 2.5 Junctions and connecting roads
- For each incoming edge, connections are grouped into runs of parallel lanes to the same outgoing
  edge; each group is one connecting road with `n` lanes (lines 180-227).
- Geometry: `bezierControlPoints(begShape, endShape, turnaround, 25, 25, ...)` on the *inner border of
  the leftmost lane* of the group, written as a single paramPoly3 (`writeGeomPP3`). The writer does
  **not** reuse SUMO's internal-lane shapes (`junctions.internal-link-detail`), so xodr and the SUMO
  net disagree. Fallback: straight line from end to start (heading discontinuity).
- Continuity: control points lie on the extended tangents -> G1 (heading) continuity at both ends, but
  curvature jumps at both ends (lines have k=0, Bezier ends do not) -> lateral jerk, steering spikes in
  CARLA autopilot. The other lanes of the group are parallel offsets of this curve, not of their own
  SUMO lane paths, so with differing in/out widths they drift.
- Roadmarks: `none` for turning lanes, `solid` on outer border, else `broken`; center `solid` only for
  geometry-like nodes/turnarounds.
- `<junction>` contains `<connection>`+`<laneLink>` only — **no `<controller>` child**. Junction ids
  are numeric; `name` carries the long cluster id. Junctions are written for nodes with no conflict
  (straight continuation) too.
- Pedestrian connecting lanes are only synthesized for the outermost lane (`addPedestrianConnection`);
  walking areas and crossings are not lanes in xodr.

### 2.6 Elevation, superelevation
- Normal roads: elevation records buffered while writing geometry: one record per `<line>` (linear,
  `b = dz/len`) and one cubic per paramPoly3 from the Bezier control-point z values. Result is C0
  only: grade jumps at every vertex -> "stepped" profile.
- Connecting roads: the flatness test is done on the 2-point fallback shape, not the curve.
- Elevation sources: OSM `ele` (rarely tagged, off in CARLA) or `--osm.layer-elevation` heuristic
  (layer * height spread to neighbouring nodes with a max-grade force model). No DEM, no bridge/tunnel
  awareness, no smoothing; `geometry.max-grade` (10 %) only warns/fixes per edge.
- `<lateralProfile/>` is always empty: no superelevation, no crossfall, no shape.

### 2.7 Road marks
- Center lane: `getDividerType` (OSM `divider` only if imported via all-attributes), default `solid`,
  width 0.13, colour `standard` (never `yellow`, so US/CA maps are wrong).
- Right lanes: outermost `solid`, lanes next to sidewalk/bike `solid`, else `broken`. SUMO lane-change
  prohibitions from `change:lanes` and `turn:lanes` are ignored; no `laneChange` attribute, no double
  lines from `overtaking=no`, no `<type>/<line>` explicit patterns, no stop lines, no arrows, no
  `lane_markings=no` handling.

### 2.8 Signals and controllers
- Traffic lights: for an edge whose to-node is TLS controlled, one signal per TLS link index
  (`--tls.group-signals` reduces count). `s = road length`, `t = -(offset to lane centre)`,
  `orientation="+"`, `dynamic="yes"`, **`zOffset="5"`**, `country="OpenDRIVE" type=1000001` (or 1000011
  arrow subtypes 10..50), `height=0.78 width=0.26`, `<validity>` per lane. Repeated links -> 
  `<signalReference>`. The signal therefore sits on the lane centre line, 5 m in the air, at the
  end of the road, inside the carriageway.
- Controllers: `<controller id=tlID>` with `<control signalId=...>`, written for every TLS node; no
  `type/sequence`, no link to the junction.
- Signs: only if the user separately runs polyconvert and passes POIs with `traffic_sign=*` plus
  `--opendrive-output.shape-match-dist`. Then `maxspeed/stop/give_way/hazard` or `CC:nnn` codes are
  parsed; signs are placed at the nearest offset on the edge, `t = side offset`, `zOffset 3.0 + 0.78k`,
  `orientation="-"` (always), `country` = OSM alpha-2 (1.4 requires alpha-3). `highway=stop/give_way`
  nodes and OSM `maxspeed` are never turned into signs.
- CARLA fork differences: one controller per incoming road (`controllerID++` per inEdge), junction
  `<controller id type=0>` records, `signalReference`s on connecting roads, and one *inertial* signal per
  approach placed by a heuristic (`center + heading*0.8*bbox_diagonal + lateral*(max+5)`), still
  1000001 with `zOffset 5` on the edge-based variant. U-turn connections dropped. Placement frequently
  lands poles in the junction or in opposite lanes.

### 2.9 Objects
- Crosswalks: SUMO crossings are turned into polygons (`crosswalk_shape` = crossing line +- width/2)
  and attached to `cw->edges[0]`, written as `<object type="crosswalk">` with `<outline>`/`cornerLocal`
  `fillType=concrete laneType=border`, `t=0` for RHT and `hdg=0` override. The crossing is located
  inside the junction but s is clamped to the incoming road -> crosswalks at road ends, partially
  misaligned.
- Other objects only from loaded polygons/POIs (`writeRoadObjectPoly/POI`), `type` = SUMO shape type
  string (not OpenDRIVE enums), z from shape layer. No poles, barriers, curbs, buildings by default.

### 2.10 Country / handedness
`--lefthand`: default LH output keeps lanes on the **right side with negative ids but reversed order**
(`s2x`: "lefthand (old)"), which non-SUMO consumers interpret as RHT geometry; `--opendrive-output.lefthand-left`
writes left lanes (positive ids). No `rule="LHT"` (1.5+ attribute). CARLA's OSM2ODR never sets
`--lefthand` at all. Speeds/sign codes default to European conventions.

---

## 3. Defects and root causes

| # | Defect (observed) | Root cause in code |
|---|---|---|
| D1 | Kinked / wobbly reference lines | Polyline -> lines for short or 2-pt edges; Bezier fallback to line; mean-angle tangents between short pieces; `longThresh = speed` unit hack; reference = offset polyline (miter artefacts) |
| D2 | Constant lane widths, no road widening | One laneSection, `b=c=d=0`; SUMO lanes have scalar width |
| D3 | Abrupt lane additions/drops (incl. ramps) | Lane count change = new edge + junction; overlapping full-width connecting lanes; `ramps.guess` adds full lanes |
| D4 | Connecting roads not curvature continuous; sometimes not even tangent | Single Bezier with 25 m extrapolation, curvature mismatch at ends; straight-line fallback; lanes offset from one reference |
| D5 | Stepped elevation, grade kinks | Per-segment linear elevation; node-only `ele`/layer heuristic; no DEM; no smoothing |
| D6 | No superelevation / crossfall | `<lateralProfile/>` hard-coded empty |
| D7 | Junction/controller ids > 32 chars (CARLA limit) | `createClusterId` (`max-join-ids` 4) + TLS id strings used verbatim as controller/signal ids |
| D8 | CARLA builds no traffic-light groups | Upstream writes no `<controller>` in `<junction>`; controllers per TLS not per junction |
| D9 | Traffic lights floating in lane at 5 m | `zOffset=5`, `s=length`, `t=-lane centre`, `orientation="+"`, generic 1000001 type |
| D10 | Wrong speeds (European defaults) | `osmNetconvert.typ.xml` rural DE speeds, country maps only for explicit `CC:zone` values; missing `maxspeed` -> 100 km/h on primary/secondary |
| D11 | Missing stop / yield / speed signs | `highway=stop/give_way`, `stop=all`, `direction` never read; signs only via optional polyconvert POIs; maxspeed never materialised as signs |
| D12 | Sidewalks flat, no curbs, gaps at junctions | Sidewalk = SUMO lane `level=true`, no height/border lane; walkingareas/crossings not exported as lanes |
| D13 | Over-merged complex intersections | `junctions.join` (10 m) clusters by distance; one huge junction with long overlapping connecting roads, one TLS for everything |
| D14 | Dual carriageways: separate roads + bad junctions | Each OSM oneway carriageway = its own road; median crossings become short edges + back-to-back junctions or a merged cluster; no notion of a divided road |
| D15 | Every road is unidirectional | One road per directed SUMO edge; opposite lanes not in same road -> overtaking/lane-change semantics and center marks lost, two coincident solid center lines |
| D16 | Spurious junctions on plain road continuations | Any node with connections writes a junction (tag change, way split) |
| D17 | Ramps poorly shaped | Ramps are separate ways meeting at a node; gore/taper not modelled; guessed lanes full width |
| D18 | Roundabouts: kinks and junction per entry | Ring = polyline edges; each entry a SUMO junction with Bezier connecting roads; no circular arc fit; no flare/splitter islands |
| D19 | `turn:lanes`, `change:lanes`, `width:lanes` (mismatch), `placement` (two-way), `maxspeed:lanes`, `destination` ignored or dropped | Options off in CARLA; writer ignores SUMO change permissions; silent fallback on count mismatch |
| D20 | Single-lane two-way roads 1.6-2.0 m lanes | Width halved when `lanes=1` and two-way (`insertEdge` 753-764) |
| D21 | Lefthand maps wrong | CARLA never passes `--lefthand`; default LH encoding non-standard; no `rule` |
| D22 | Unstable ids, poor traceability | Ids from iteration order; OSM way ids only in `userData` with `--output.original-names` |
| D23 | Bridges/tunnels flat or intersecting | `bridge`/`tunnel` ignored; `layer` heuristic off by default |
| D24 | Crosswalks mis-placed | Crossing polygon in junction, mapped to incoming road with clamped s, `t=0`, `hdg=0` |
| D25 | Service/parking roads missing for cars | Typemap `highway.service` disallows passenger |

---

## 4. What a native OpenDRIVE generator should do instead

General principle: build an **OpenDRIVE-native intermediate model** (roads with lane sections, width
polynomials, bidirectional lanes, junction areas) directly from OSM, rather than going through a
simulation graph of directed lanes.

1. **Reference geometry (D1, D18).** Treat the OSM way (after merging ways with continuous attributes)
   as noisy samples of a *bidirectional* road centreline. Fit a G2 sequence of `line` / `spiral` / `arc`
   (Euler-spiral transitions) by least-squares with a max lateral error (e.g. 0.3-0.5 m) and minimum
   radius by road class; fall back to `paramPoly3` with arc-length (`pRange="arcLength"`) only when
   needed. Compute `length` analytically. Roundabouts: fit a circle to the ring and write true arcs.
   Validate heading/curvature continuity at every geometry boundary (unit test: |dhdg| < 1e-6, |dk| bounded).
2. **Bidirectional roads (D15, D14).** One `<road>` per OSM street with left + right lanes and a center
   lane carrying the real divider mark. For dual carriageways (paired oneways with same name/ref within
   ~5-40 m and opposite directions), either emit two roads with a `median` / `border` lane and proper
   paired junctions, or merge into one road with a median lane when separation is small.
3. **Lane widths & sections (D2, D3, D17, D20).** Per-lane width from `width:lanes`, `width`
   (carriageway), `est_width`, class defaults; regional defaults (e.g. US 3.6 m, EU urban 3.0-3.25 m).
   Lane count changes become new `laneSection`s *within the same road* with cubic width tapers
   (`a,b,c,d` with zero slope at both ends, taper length from design speed, typically 30-90 m), side chosen
   from `turn:lanes`/`placement`. Ramps: model acceleration/deceleration lanes as tapered lanes on the
   mainline road and attach the ramp road at the gore with a junction or direct `successor` link.
   Never halve widths for `lanes=1` two-way; treat as single-lane two-way only if width < ~5 m.
4. **Junctions (D4, D13, D16).** Only create a `<junction>` where topology requires it (>=3 arms or
   conflicting movements). Derive a junction area from road edges (curb returns with radius by class),
   then build connecting roads as G2 curves: spiral-arc-spiral (clothoid) matched to entry/exit heading
   and k=0, or quintic/cubic curvature-continuous paramPoly3 solved for heading+curvature at both ends.
   Each connecting lane gets its own path (not offsets of one) or the group shares a reference with a
   `laneOffset` polynomial. Cluster nodes for joining by topology (dual-carriageway crossing pairs,
   short links < X m between signalised nodes), not pure distance; cap junction size and keep ids short.
5. **Elevation (D5, D6, D23).** Sample a DEM (SRTM/Copernicus/LiDAR) plus OSM `ele`, `layer`,
   `bridge`, `tunnel`, `incline`; fit a smooth C1 (cubic Hermite) `elevationProfile` per road with grade
   limits by class, and force junction roads to share a common plane/blend. Write `superelevation`
   from curve radius and design speed, and crossfall (e.g. 2 %) in `lateralProfile`.
6. **IDs (D7, D22).** Short deterministic numeric ids (e.g. hash/sequence of OSM way id + segment), all
   <= 32 chars; store OSM ids in `userData`. Never derive ids from cluster member lists.
7. **Signals/controllers (D8, D9, D11).** One `<controller>` per signal group (phase group: e.g. N/S
   vs E/W approaches), referenced from the `<junction>` via `<controller id type="0" sequence>`, which
   CARLA uses to create TrafficLightGroups. Place each signal at the stop line on the right curb
   (`t` = outer edge + 0.5-1 m), `zOffset` 0 with physical `height` (CARLA expects pole on ground),
   `orientation` facing the approach, `type="1000001"` country-appropriate; add `<validity>` per lane
   and `signalReference` on connecting roads. Use `traffic_signals:direction`/`direction` and
   approach-node signals (signal mapped 5-30 m before a junction) to assign to the right approach.
   Convert `highway=stop/give_way` (+ `direction`, `stop=all`) into country-coded signs (map each country's codes, e.g. US
   R1-1 / DE 206, onto the stop/yield type codes CARLA recognises) at the stop line, and emit speed-limit signs
   at the start of roads / after maxspeed changes. Emit stop lines (roadMark or object) for stop/signal.
8. **Speeds (D10).** Derive country from OSM `ISO3166` boundary or a user flag; apply country/urban
   defaults (`maxspeed:type`, `source:maxspeed`, implicit limits) and `maxspeed:lanes`; write per-lane
   `<speed max unit>` explicitly.
9. **Sidewalks/curbs (D12, D24).** Sidewalk lanes from `sidewalk*` (+`sidewalk:*:width`), separated by a
   `border`/`curb` lane with `<height inner/outer=0.15>` and solid road mark; continue sidewalks around
   junction corners as sidewalk lanes on connecting roads or `walkingArea`-like objects; crosswalks as
   `crosswalk` objects positioned in the junction road coordinate frame with correct heading and width,
   ideally also as `roadMark`/`object` zebra outline.
10. **Road marks (D15, D19, D21).** Center: `solid solid` / `broken` from `divider`, `overtaking`,
   lane counts; colour `yellow` for Americas; lane separators from `change:lanes` (`solid`/`broken`
   with `laneChange` attribute), `lane_markings=no` -> none; turn arrows from `turn:lanes` as objects.
11. **Turn lanes & connectivity (D19).** Always use `turn:lanes` to create lane-level connections; if
   absent, infer conservative defaults. Respect `restriction` relations.
12. **Handedness (D21).** Country-driven LHT: write `rule="LHT"` (OpenDRIVE 1.5+/1.6/1.8) with standard
   lane id semantics; mirror signal side, sidewalks and turn logic.
13. **Validation.** Load output with libOpenDRIVE/esmini `odrviewer` and CARLA `generate_opendrive_world`;
   check geometric gaps (< 1 mm), heading continuity, lane-link consistency, id lengths, signals inside
   junction bounds, and junction controller coverage.

---

## 5. Other OSM -> OpenDRIVE tools (brief)

| Tool | Approach | Better than SUMO | Worse |
|---|---|---|---|
| **CARLA OSM2ODR** (SUMO fork) | SUMO netconvert with fixed flags + modified writer | Junction `<controller>` records, int signal ids, per-approach signals, U-turns dropped | Same geometry/width/elevation pipeline; heuristic light placement; no lefthand; fixed EU typemap; old SUMO base |
| **TUM CommonRoad Scenario Designer (crdesigner)** | OSM -> graph (lane estimation, `turn:lanes`, intersection cropping) -> CommonRoad lanelets -> OpenDRIVE (and Lanelet2) | Lane-level graph with explicit boundaries; good handling of turn lanes and intersections; GUI for manual repair | Lanelet polylines exported as many short line segments / fitted polynomials (not G2); no signal controllers for CARLA; elevation mostly 2D |
| **osm2xodr** (JHMeusener, Python, GPL-3, last push 2022) | Direct OSM -> xodr, junction per node, simple curve connections, optional 16-bit heightmap | Simple, native xodr structure, DEM elevation | Drivable roads only, no signals/sidewalks, limited lane logic, unmaintained |
| **osm2opendrive (Apollo variant, PyPI)** | OSM -> Apollo-flavoured OpenDRIVE | Apollo HD map fields | Non-standard dialect, not usable in CARLA directly |
| **libOpenDRIVE** (reader) | C++ parser, lane/roadmark/mesh generation | Ideal validator and mesh/preview backend for our output | Not a generator |
| **RoadRunner** (commercial) | OSM import + interactive editing, xodr export | High-quality junctions, tapers, signals | Manual, licensed |

Takeaways for our generator: borrow crdesigner's lane-graph inference and intersection cropping,
CARLA fork's junction-controller convention, and use libOpenDRIVE/esmini for automatic validation, while
replacing SUMO's directed-lane model with bidirectional roads, lane sections with width polynomials,
G2 geometry fitting, DEM-based smooth elevation and country-aware semantics.
