# ASAM OpenDRIVE: implementation notes for an OSM to XODR generator

Researched 2026-10-11. Sources: the ASAM OpenDRIVE spec pages (the `latest/` link now redirects to **v1.9.0**), the Signal reference 1.0.0, and CARLA source (`ue4-dev`: LibCarla parser, `SignalType.cpp`, `TrafficLightManager.cpp`).
Base URL: `https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.9.0/specification/`

Legend: **[1.8]** means introduced in 1.8.0, **[1.9]** means introduced in 1.9.0, and **DEPR x** means deprecated since version x. Rule IDs have the form `asam.net:xodr:<ver>:<path>`, and only the `<path>` part is quoted below.
Items marked "(verify)" come from memory or partial reading, not from a spec page.

---

## 0. Versions and the target to emit

| Version | Notes relevant to us |
|---|---|
| 1.4 (2015, VIRES) | De facto baseline for CARLA and many tools. Has `country` as names or codes, `<lane><link>`, and `paramPoly3` (pRange added in 1.5?). |
| 1.5 | Header `<offset>`, virtual junctions (`mainRoad/sStart/sEnd/orientation`), `<outlines>`, `mwyEntry/mwyExit` lane types DEPR 1.5 (use `entry`/`exit`), object types `car,van,bus,…,wind` DEPR 1.5. |
| 1.6 (first ASAM) | Lane type `curb`, `cornerLocal` refinements. `revMinor=6`. |
| 1.7 | **direct junctions** (`linkedRoad`), `elementS/elementDir`, `perpToRoad`, most checker rule IDs. |
| 1.8.0 / 1.8.1 | `crossing` junctions, `crossPath`, junction `boundary` and `elevationGrid`, lane `direction/advisory/dynamicLane*/roadWorks`, `<access><restriction>`, signal `<semantics>`, boards (`staticBoard`, `vmsBoard`), `<defaultRegulations>`, `<license>`. Country is ISO 3166-1 **alpha-2 only**. 1.8.1 is a bug-fix release with the same schema (verify). |
| 1.9.0 (current "latest") | Lane **layers** (`<lanes layer="permanent|temporary">`), `invalidated`/`temporary` on object, signal and sign, `<curveLocal>` outlines (line/arc/paramPoly3), repeat `bT/cT/dT`, CRG `xOffset/yOffset`, semantics `<vehicle>/<person>/<animal>`. Backward compatible with 1.8.1. |

**Recommendation:** produce one internal model and emit it in two profiles.
- **Strict 1.8 profile:** `revMajor=1 revMinor=8`, alpha-2 `country`, `walking` lane type, `<access><restriction>`.
- **CARLA/legacy profile:** `revMinor=4`, deprecated-but-understood forms (`sidewalk`, `country="OpenDRIVE"`, `<access restriction=…>`), no direct junctions, numeric IDs.

Do not emit 1.9-only features (layers, `invalidated`) by default. The spec warns that lane layers break OpenSCENARIO ≤ 1.3.1.

---

## 1. Coordinate systems and header

### 1.1 Coordinate frames
- **Inertial (x, y, z):** right-handed, x→east, y→north, z→up in a projected CRS. Heading is measured counter-clockwise from +x, in radians.
- **Reference line (s, t, h):**
  - `s` is arc length along the reference line in m, measured **in the x/y plane** (elevation does not lengthen s).
  - `t` is lateral and **positive to the left**.
  - `h` is up. The frame is right-handed.
  - Superelevation rolls `t` about `s`. Lane widths are measured in the tilted plane, so the projected width is smaller.
  - There is no pitch in s/t/h.
- **Local (u, v, z):** used by objects and signals. `hdg`, `pitch` and `roll` are relative to the road direction or the x/y plane. `u` is forward and `v` is left.
- **Positive curvature means a left turn (counter-clockwise).** This applies to `arc@curvature` and `spiral@curvStart/curvEnd`.

### 1.2 Root and `<header>` (exactly one, first child of `<OpenDRIVE>`)

Header attributes:

| attr | type | use | unit |
|---|---|---|---|
| `revMajor` | int | **req** | must be `1` |
| `revMinor` | int | **req** | `8` for 1.8.x, `9` for 1.9, `4` for 1.4 |
| `name`, `version`, `vendor` | string | opt | |
| `date` | string | opt | ISO 8601 `YYYY-MM-DDThh:mm:ss` |
| `north`, `south`, `east`, `west` | double | opt | m (inertial bounding box) |

Header children:
- `<geoReference>` (0..1)
- `<offset>` (0..1)
- `<license name resource? spdxid? text?>` **[1.8]**
- `<defaultRegulations>` **[1.8]**, containing:
  - `<roadRegulations type=e_roadType>`, whose children may only be `<semantics>`-style `<speed>` and `<priority>` (rule `defaultRegulations.only_speed_priority`)
  - `<signalRegulations type subtype>`, for signs that change driving rules. It is not a signal catalogue.
  - This is a good place to put per-country defaults, such as an implicit urban speed limit. CARLA ignores it.

### 1.3 `<geoReference>` and `<offset>`
- `<geoReference>` holds a **PROJ string inside CDATA**, for example:
  `<geoReference><![CDATA[+proj=utm +zone=32 +ellps=WGS84 +datum=WGS84 +units=m +no_defs]]></geoReference>`
  - Only one projection is allowed. Without it, the coordinates are a local Cartesian system.
  - Use the official EPSG parameter sets and do not alter them.
  - The spec says nothing about WKT or a bare `EPSG:xxxx`, so emit a PROJ string.
- Shift the data with `<offset x y z hdg>` (all **required**; m, m, m, rad) instead of editing `+x_0/+y_0`:
  - `xW = xODR·cos(hdg) − yODR·sin(hdg) + x`
  - `yW = xODR·sin(hdg) + yODR·cos(hdg) + y`
  - `zW = zODR + z`
  - The offset is applied to local coordinates before PROJ inverse projection. Use `hdg="0"`, because the spec says to avoid rotation.
  - Keep coordinates near (0, 0) for float precision.
- **OSM pipeline:**
  - Pick UTM zone `floor((lon+180)/6)+1`, with `+south` for the southern hemisphere.
  - Alternatively, use `+proj=tmerc +lat_0=<lat> +lon_0=<lon> +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +units=m +no_defs` centred on the bounding box. This gives near-zero coordinates with no offset, and CARLA reads `+lat_0/+lon_0` as the map origin (verify).
  - For UTM, put the bounding-box centre into `<offset>` or subtract it.
  - esmini keeps all PROJ parameters except `+geoidgrids` and `+vunits`.

---

## 2. planView geometry (`<planView>` is mandatory, `<geometry>` 1..*)

`<geometry s x y hdg length>`:
- `s` ≥ 0 and `length` > 0, in m. `x`, `y` in m. `hdg` in rad, unconstrained in the spec; emit it normalized to (−π, π].
- It contains **exactly one** of: `<line/>`, `<arc curvature/>`, `<spiral curvStart curvEnd/>`, `<poly3 …>` (DEPR), or `<paramPoly3 …>`.

Rules:
- `road.geometry.elem_asc_order`: geometries are in ascending s.
- `road.geometry.s-value_sum` **[1.9]**: each `s` equals the sum of the previous lengths. Write the cumulative sums exactly.
- `road.geometry.refline_no_gaps`: no gaps.
- `road.geometry.refline_no_kinks`: no kinks ("should").
- The road `length` should equal the sum of the geometry lengths.
- `road.geometry.arc.no_zero_curvature`: an arc with curvature 0 is not allowed. Use `<line/>` instead.
- `road.geometry.spiral.curvature_change`: `curvStart ≠ curvEnd`.

Element definitions:
- **line:** `x = x0 + ds·cos(hdg)` and `y = y0 + ds·sin(hdg)`.
- **arc:** `curvature` k in 1/m, with k > 0 meaning left. With R = 1/k: `x = x0 + (sin(hdg+k·ds) − sin(hdg))/k` and `y = y0 − (cos(hdg+k·ds) − cos(hdg))/k`.
- **spiral (clothoid):** curvature is linear in ds, `k(ds) = curvStart + (curvEnd−curvStart)·ds/length`, and `θ(ds) = hdg + curvStart·ds + ½·(curvEnd−curvStart)/length·ds²`. Evaluate with Fresnel integrals or numerical integration. CARLA and esmini both support spirals with any start and end curvature.
- **poly3 (DEPR 1.8, attributes a, b, c, d):** `v(u) = a + b·u + c·u² + d·u³` in a local u/v frame. **Do not emit.**
- **paramPoly3:** `u(p) = aU + bU·p + cU·p² + dU·p³` and `v(p) = aV + bV·p + cV·p² + dV·p³`.
  - `pRange` is **required** and is `arcLength` (p ∈ [0, length]) or `normalized` (p ∈ [0, 1], dimensionless coefficients).
  - Rule: `aU = aV = bV = 0` and `bU > 0`, so that u/v is aligned with s/t at the start.
  - The integrated curve length should equal `@length`. p is *not* arc length except at the endpoints.
  - Always write `pRange` explicitly: CARLA's parser reads a missing value as an empty string.

**Recommended for OSM:**
- Fit polylines with line, arc and clothoid chains for G2 continuity. This is the most portable option and is exact in every tool.
- Alternatively, fit one `paramPoly3` per OSM segment with `pRange="normalized"` and heading continuity enforced. Hermite cubic from the endpoint tangents: `bU = L`, `cU = 3Δu − 2bU − dTu`, … in the local frame.
- Avoid geometries shorter than about 1e-3 m: tools divide by length. Merge or drop fragments under about 0.01–0.1 m.
- Print at least 1e-9 relative precision (e.g. `%.17g` or 12 significant digits after the offset). Heading mismatch at joints should be < 1e-6 rad.
- Avoid many tiny straight `<geometry>` elements. Annex D.2 says to merge redundant nodes.

---

## 3. Elevation, superelevation, shape, road type, speed

**Elevation:** `<elevationProfile>/<elevation s a b c d>`.
- `elev(ds) = a + b·ds + c·ds² + d·ds³` in m, with ds measured from `@s`.
- Units of a, b, c, d: m, –, 1/m, 1/m².
- Elements are in ascending s. Each is valid until the next one starts.
- Ensure C0 and preferably C1 continuity between elements and across road links (Annex D vertical gap and kink rules).

**Superelevation:** `<lateralProfile>/<superelevation s a b c d>`.
- Roll angle in rad. Units of a, b, c, d: rad, rad/m, rad/m², rad/m³.
- **Positive means the road falls to the right.**
- It applies to the whole cross-section. Lanes with `level="true"` are excluded.

**Shape:** `<lateralProfile>/<shape s t a b c d>`.
- `hShape(dt) = a + b·dt + c·dt² + d·dt³`, interpolated linearly in s between shape sets.
- Sorted by s, then t. The t coverage must span the full road width at each s.
- **Combining shape and superelevation is DEPR.** Neither may be combined with object `crossSectionSurface` **[1.8]**.

**Road type:** `<type s type country?>`.
- `type` (e_roadType): `unknown, rural, motorway, town, lowSpeed, pedestrian, bicycle, townExpressway, townCollector, townArterial, townPrivate, townLocal, townPlayStreet`.
- `country` must be **ISO 3166-1 alpha-2** (rule `road.type.only_alpha_2_country_codes`).
- Elements are in ascending s.
- Child `<speed max unit>`:
  - `max` is a number or the string `"no limit"` or `"undefined"`.
  - `unit` (e_unitSpeed) is `m/s`, `km/h` or `mph`. Always write it, even though the spec text says m/s is the default.
  - A new `<type>` is required when a speed changes.
  - Signals override `<type><speed>`.

**OSM highway → e_roadType mapping (suggested):**

| OSM `highway` | e_roadType |
|---|---|
| motorway, trunk + `motorroad=yes` | `motorway` |
| trunk, primary (urban) | `townArterial` / `townExpressway` |
| primary, secondary (rural) | `rural` |
| secondary, tertiary (urban) | `townCollector` |
| residential, unclassified (urban) | `townLocal` |
| living_street | `townPlayStreet` |
| service, private access | `townPrivate` |
| track | `lowSpeed` |
| pedestrian, footway | `pedestrian` |
| cycleway | `bicycle` |

Urban vs rural can be decided from `landuse` and `place` polygons.

---

## 4. Lanes

### 4.1 Structure

```
<lanes [layer="permanent"]>                      layer is [1.9]; default permanent
  <laneOffset s a b c d/>*                        offset(ds)=a+b·ds+c·ds²+d·ds³ (m), lateral shift of lane 0 vs reference line, +left
  <laneSection s [length] [singleSide=true|false]>+   length only for the temporary layer [1.9]
    <left>  <lane id=1..n …/>  </left>            ids positive, increasing outward; list in descending id
    <center><lane id="0"/></center>               exactly one; no width, material, speed or access; roadMark only
    <right> <lane id=-1..-n …/> </right>
```

Rules:
- At least one laneSection per road, in ascending s, with no zero-length sections (`road.lane_section.valid_length`).
- **A new laneSection is required whenever the number of lanes changes** (`lanesec_usage_lane_num`).
- Exactly one center lane at each s (`road.lane.center_lane_singular`).
- Each section needs at least one left or right lane (`road.lane.lane_section_drivable`).
- Avoid zero-width lanes over long distances (`lane_long_zero_width`).
- `<laneOffset>` and `<border>` are mutually exclusive (`road.lanes.lane_offset.no_offset_if_border_defined`).

**Use `laneOffset` for asymmetric roads.** Keep the reference line on the OSM way centreline, which is usually the road centre, and shift lane 0 with `laneOffset` for:
- one-ways with all lanes on one side,
- unequal `lanes:forward` and `lanes:backward`,
- turn pockets.

For a one-way road, a laneOffset of `+W/2` with only right lanes keeps the carriageway centred on the OSM line. CARLA reads laneOffset per road.

### 4.2 `<lane>` attributes (left and right)

| attribute | type | notes |
|---|---|---|
| `id` | int | **req**; +left, −right |
| `type` | e_laneType | **req** |
| `level` | bool | default false; true excludes the lane from superelevation and shape (sidewalks) |
| `advisory` **[1.8]** | `none\|inner\|outer\|both` | lane may be shared by a neighbour (e.g. an advisory bike lane) |
| `direction` **[1.8]** | `standard\|reversed\|both` | overrides the side-derived direction. Use `both` for single-lane two-way roads, center turn lanes, or shared bidirectional lanes. `reversed` must not be used to switch RHT/LHT. |
| `dynamicLaneDirection` **[1.8]** | bool | default false |
| `dynamicLaneType` **[1.8]** | bool | default false |
| `roadWorks` **[1.8]** | bool | lane under construction |

Center-lane `type` and `level` are DEPR 1.8, but CARLA expects `type="none"` on lane 0, which is harmless.

**Driving direction:**
- Road `@rule` is `RHT` (default) or `LHT`.
- In RHT, right lanes (negative ids) drive along +s.
- In LHT, left lanes (positive ids) drive along +s.
- Set `rule="LHT"` per road for GB, IE, JP, IN, AU, ZA, etc. Take it from the country, which can be derived from OSM `ISO3166-1` boundaries. CARLA reads `rule`.

### 4.3 e_laneType (1.9 list)

| Group | Lane types |
|---|---|
| Valid | `none, driving, stop, shoulder, biking, border, restricted, parking, median, curb` (1.6), `entry, exit, onRamp, offRamp, connectingRamp, tram, rail, walking` [1.8], `slipLane` [1.8], `shared` [1.8], `roadWorks` [1.8] |
| DEPR 1.8 | `sidewalk` (use `walking`); `bidirectional` (use `direction="both"`); `bus`, `taxi`, `HOV` (use `driving` + `<access>`); `special1/2/3` |
| DEPR 1.5 | `mwyEntry`, `mwyExit` |

Semantics:
- `border`: a hard edge at the same height as the carriageway.
- `curb`: a different height.
- `shoulder`: a soft edge.
- `stop`: a motorway hard shoulder.
- `restricted`: hatched, not for driving.
- `none`: the outermost "space".

**CARLA accepts only** (case-insensitive) `driving, stop, shoulder, biking, sidewalk, border, restricted, parking, bidirectional, median, special1-3, roadworks, tram, rail, entry, exit, offramp, onramp`.
- Everything else (`walking, curb, connectingRamp, slipLane, shared, bus, taxi, HOV, mwy*`) becomes `None`, which is not drivable and not walkable.
- In the CARLA profile, map `walking`→`sidewalk`, `curb`→`border`, `connectingRamp`/`slipLane`/`bus`/`taxi`→`driving`, `shared`→`driving` or `biking`.
- CARLA pedestrian navigation (RecastBuilder) needs `sidewalk` lanes (verify), so emit sidewalks as `sidewalk` with `level="true"` and `<height inner="0.15" outer="0.15">`.

### 4.4 Width vs border
- `<width sOffset a b c d>` (1..*): `width(ds) = a + b·ds + c·ds² + d·ds³`.
  - `sOffset` is relative to the laneSection start.
  - Width must be ≥ 0 (`road.lane.width.lane_width_validity`).
  - There must be an entry at `sOffset=0`, and the full section must be covered.
  - The center lane has no width.
- `<border sOffset a b c d>`: the absolute t of the outer edge.
  - Do not mix it with `<width>` in one lane group (if both are present, readers must use width).
  - Do not mix it with `<laneOffset>`.
  - **Emit width only.** CARLA and most tools handle width best.
- **Lane appearing or disappearing (turn pockets, merges):** start a new laneSection and give the new lane `width` from 0 to W over the taper length. A cubic `a=0, b=0, c=3W/L², d=−2W/L³` gives a smooth S-shaped taper. A lane with zero width at the start of the section gets **no predecessor** link, and one with zero width at the end gets **no successor**.

### 4.5 roadMark
`<roadMark sOffset type color [weight] [width] [height] [material] [laneChange]>` sits on each lane and describes the lane's **outer** border (`road.lane.road_mark.only_outer`).
- For left lanes this is the left edge, and for right lanes the right edge.
- The center lane's roadMark is the centreline.
- The marking is centred on the border, so half of it lies on the next lane.

Enumerations:
- `type`: `none, solid, broken, solid solid, solid broken, broken solid, broken broken, botts dots, grass, curb, custom, edge` (with spaces).
- `color`: `standard, white, yellow, blue, green, red, orange, violet, black` [1.8].
- `weight`: `standard, bold`.
- `laneChange`: `increase|decrease|both|none` (default both). This is the permitted crossing direction in terms of lane id: `increase` means from a lower to a higher id.

Children:
- `<type name width>` with `<line length space tOffset sOffset [rule] [width] [color]>+` repeating dashes. `rule` is `no passing|caution|none`.
- `<explicit><line length sOffset tOffset [rule] [width]>` for one-off lines.
- `<sway ds a b c d>*` for wavy lines.

Typical values:
- DE: broken 6 m dash / 12 m gap (urban 3/6), width 0.12 (0.15 on motorways), edge solid 0.25.
- US: 10 ft / 30 ft (3.05 / 9.14 m), yellow center lines.

Markings that are not lane borders (stop lines, arrows, zebra) go into **objects or signals**.

### 4.6 Lane link
`<lane><link><predecessor id [layer]/>*<successor id/>*</link>`
- Links follow the **reference-line direction**, not the driving direction.
- Across roads, the id refers to the first or last laneSection of the other road, depending on its contactPoint. With opposite reference lines, lane −1 links to lane +1.
- Omit `<link>` for lanes that end at a junction (junction `<laneLink>` handles those) or that have zero width at the connection.
- Multiple predecessors or successors are allowed only for abrupt splits and merges, where all linked lanes have non-zero width.
- **Within one road** between laneSections, link every continuing lane explicitly. CARLA's waypoint graph depends on it.

### 4.7 Other lane properties (all use `sOffset` relative to the section, ascending, never on the center lane)

- `<material sOffset friction [roughness] [surface]>`. **[1.8]** adds `roadMarkColor`.
- `<speed sOffset max [unit]>`: if `unit` is missing, m/s is assumed. Always write it.
- `<access sOffset rule="allow|deny"><restriction type=…/>*</access>` **[1.8]**:
  - `type` (e_accessRestrictionType): `simulator, autonomousTraffic, pedestrian, passengerCar, bus, delivery, emergency, taxi, throughTraffic, truck, bicycle, motorcycle, none, trucks, HOV`.
  - Semantics: with no `<access>`, the lane is unrestricted. `deny` blocks the listed users and allows everyone else. `allow` allows the listed users and blocks everyone else.
  - Do not mix allow and deny at one s (`road.lane.access.no_mix_of_deny_or_allow`).
  - Each new s must restate the full set. `deny` + `none` resets.
  - The old form `<access sOffset restriction="bus"/>` is DEPR 1.8 but is what CARLA parses.
  - OSM `bus:lanes`, `psv`, `access:lanes` and `hov:lanes` map here.
- `<height sOffset inner outer>` in m: raises the lane surface (curbs and sidewalks, e.g. 0.12–0.15). It is added on top of the cross-section and the junction elevationGrid. Temporary-layer lanes must not have height or material.
- `<rule sOffset value>`: free text, e.g. "no stopping at any time".
- `<visibility>` existed in 1.4, and CARLA still parses it (forward/back/left/right). It is not in current versions (verify).

---

## 5. Road linkage

`<road id junction length [name] [rule]>`:
- `id` is a string, unique, and should fit uint32 when numeric. **CARLA parses `as_uint`, so road ids must be numeric.**
- `junction` is `-1` for normal roads, or the id of the junction a connecting road belongs to.

`<link><predecessor|successor elementType="road|junction" elementId [contactPoint="start|end"] [elementS] [elementDir="+|-"]/></link>`:
- Linking to a road: `elementType`, `elementId` and **`contactPoint` are required**.
- Linking to a common or direct junction: only `elementType` and `elementId`; no contactPoint.
- Virtual junction: `elementS` and `elementDir` instead of `contactPoint`.
- **[1.9] `road.linkage.both_sides_consistency`:** both roads must declare the link, consistently. For example, if A.successor = B(start), then B.predecessor must be A(end), or B.successor if B is reversed.
- Link roads directly only when unambiguous (a 1:1 chain). As soon as more than two roads meet, use a junction (`junctions.common.not_only_two`: do *not* build a junction where only two roads meet).
- **Geometric continuity at links:**
  - Endpoints must coincide in x, y and z.
  - Heading must be equal, or differ by π when the reference lines are opposed.
  - The lane width of linked lanes should match.
  - The elevation slope should match.
  - Annex D wants no gaps or kinks in x, y, heading, z and roll for drivable lanes, without numeric tolerances. Aim for < 1 mm and < 1e-4 rad.
- `<neighbor>` (old 1.4 road-level `side/elementId/direction`) is obsolete and not in current spec chapters. Do not emit it.

---

## 6. Junctions

`<junction id [name] [type="default|direct|virtual|crossing"]>` (type defaults to `default`). Junctions have no predecessor or successor.

### 6.1 Common (default) junction
`<connection id incomingRoad connectingRoad contactPoint="start|end">` with `<laneLink from to [overlapZone]/>*`:
- `from` is a lane id on the incoming road, and `to` is a lane id on the connecting road.
- `contactPoint` refers to the **connecting road**: `start` means the connecting road runs along the linkage, and `end` means it runs opposite.
- Use one `<connection>` per (incoming road, connecting road) pair. Its laneLinks cover only the lanes that lead into the junction (`junctions.connection.one_link_to_incoming`).
- Omitting `<laneLink>` is DEPR, so always write it.
- Connecting roads cannot be incoming roads.

Connecting roads:
- `junction="<id>"`.
- Predecessor and successor are the incoming and outgoing roads, with explicit `contactPoint`.
- The reference line should follow the driving direction.
- Geometry must fit both ends smoothly (`junctions.connection.smooth_fit`).
- Usually one driving lane, with lane id −1 in RHT.
- A connecting road may carry several lanes, which allows lane change in the junction. Otherwise use one connecting road per lane-to-lane path; CARLA works best with **one lane per connecting road**.
- Its lanes must also carry `<link>` predecessor and successor ids to the incoming and outgoing lanes. CARLA and esmini use them.

Other junction children:
- `<priority high low>`: road ids of the prioritized and yielding connecting roads. Use it only when no signals exist (`junctions.priority.no_signals`). Both attributes are required **[1.8]**. This is good for OSM uncontrolled or priority junctions.
- `<controller id [type] [sequence]>*`: references a top-level `<controller>`. This is how traffic lights are synchronized per junction.
- **[1.8]** `<boundary>` contains `<segment type="lane" roadId boundaryLane sStart sEnd>` and `<segment type="joint" roadId contactPoint [jointLaneStart] [jointLaneEnd] [transitionLength]>`. Segments go counter-clockwise, must close the outline, and must touch every connected road end. Common junctions only.
- **[1.8]** `<planView>` with one `<geometry><line/></geometry>` is the junction reference line. It is required for `<elevationGrid gridSpacing sStart>` with `<elevation center="z z z…" left="…" right="…"/>*`, where grid lines are perpendicular to that reference line. Lane height is added on top. It is optional, and CARLA ignores it.
- **[1.8]** `<crossPath id crossingRoad roadAtStart roadAtEnd>` with `<startLaneLink s from to/>` and `<endLaneLink s from to/>`. It models pedestrian or bicycle paths across a junction and connects only `walking` or `biking` lanes. The crossing road has `junction=<id>`.

### 6.2 Direct junction (1.7+)
`<junction type="direct">` with `<connection id incomingRoad linkedRoad contactPoint>` and `<laneLink from to [overlapZone, default 100 m]>`:
- There are no connecting roads; `connectingRoad` is forbidden.
- Use it for merges and splits without crossing traffic: motorway on/off ramps, with one road on one side and several on the other.
- Place it where headings are identical. Flat only.
- **CARLA ignores `type` and `linkedRoad`** (it reads only `incomingRoad` and `connectingRoad`), so for CARLA emit ramps as common junctions with short connecting roads.

### 6.3 Virtual junction
`type="virtual"` with `mainRoad`, `sStart`, `sEnd` and `orientation` (`+`, `-` or `none`), all required:
- Used for driveways, parking entrances and farm accesses **without splitting the main road**.
- Connections are `type="default"`, with `incomingRoad=-1` if there is none.
- Connecting roads link with `elementS` and `elementDir`.
- No controllers or traffic lights are allowed.
- `<connection type="virtual">` is DEPR 1.8.
- CARLA does not support this; avoid it in the CARLA profile and split the road with a common junction instead.

### 6.4 Crossing junction [1.8]
`type="crossing"` contains only `<roadSection id roadId sStart sEnd>` (1..*):
- Used for railway level crossings and for pedestrian or cycle crossings of a road.
- Only one roadId may have high priority. `sStart` and `sEnd` must cover the overlap.
- Traffic cannot change roads. Flat only.

### 6.5 Junction groups
`<junctionGroup id [name] type="roundabout|complexJunction|highwayInterchange|unknown">` with `<junctionReference junction/>+`:
- It is a top-level element.
- Use it for OSM roundabouts (`junction=roundabout`), which become several junctions linked by ring roads.

---

## 7. Signals (`<road><signals>`)

### 7.1 `<signal>` attributes

| attr | type | use | notes |
|---|---|---|---|
| `s` | ≥0 m | req | |
| `t` | m | req | +left of the reference line. Physical placement: kerb side, about lane edge + 0.5–2 m |
| `id` | string | req | unique in the file |
| `name` | string | opt | |
| `dynamic` | `yes\|no` | req | `yes` for traffic lights |
| `orientation` | `+\|-\|none` | req | `+` = valid for traffic along +s (faces vehicles driving +s) |
| `zOffset` | m | req | above the reference line elevation, to the bottom of the sign or light (e.g. 2.0 for signs; light heads 2.5–5.5) |
| `country` | alpha-2 | opt | ISO 3166-1 alpha-2 in 1.8+. `"OpenDRIVE"` for the generic catalogue (traffic lights, stop line) |
| `countryRevision` | string | opt | year of the national regulation, e.g. `2017` |
| `type`, `subtype` | string | req | national sign code. Use `-1` or `none` when unknown. Unique only together with country and countryRevision |
| `value` + `unit` | double + e_unit | opt | unit is mandatory if value is given: speed `km/h\|m/s\|mph`, distance `m\|km\|ft\|mile`, mass `kg\|t`, slope `%` |
| `height`, `width` | ≥0 m | opt | |
| `length` [1.8] | ≥0 m | opt | |
| `text` | string | opt | |
| `hOffset` | rad | opt | relative to the orientation direction |
| `pitch`, `roll` | rad | opt | |
| `invalidated`, `temporary` [1.9] | bool | opt | |

Children:
- `<validity fromLane toLane [layer]>*`: with no validity, the signal applies to all lanes in the `orientation` direction. In RHT, `+` may only cover negative lane ids. `fromLane ≤ toLane`.
- `<dependency id [type]>*`: the signal is controlled by another signal (e.g. a supplementary sign, or a stop line dependent on a traffic light).
- `<reference elementType="signal|object" elementId [type]>*`: links to related elements, e.g. a traffic light to its stop line with `type="stopline"`.
- `<semantics>` [1.8] can contain:
  - `<speed type="maximum|minimum|recommended|zone|…End" value unit>`
  - `<priority type="stop|yield|trafficLight|priorityRoad|priorityRoadEnd|priorityToTheRightRule|4way|stopLine|waitingLine|keepClearLine|noParkingLine|noTurnOnRed|turnOnRedAllowed">`
  - `<lane type=…>`, `<prohibited>`, `<warning>`, `<routing>`, `<streetname>`, `<parking>`, `<tourist>`
  - `supplementary*`, plus `<vehicle type>`, `<person type>` and `<animal>` [1.9]

  This makes signs country-independent for readers.
- `<positionInertial x y z hdg pitch roll>`: still parsed by CARLA. `<positionRoad>` is DEPR 1.8 and **not read by CARLA**. The 1.8 method is to place the signal physically and model the logical relation with reference and dependency.

Related elements:
- `<signalReference s t id orientation>` with `<validity>*`: re-uses one physical signal on another road, e.g. a light over a junction valid for several incoming roads. CARLA reads it.
- **Boards [1.8]:** `<staticBoard>`, `<vmsBoard>`, `<vmsGroup>`, `<sign>`, `<displayArea>`.

### 7.2 Controllers (synchronized traffic lights)
- Top level: `<controller id [name] [sequence]><control signalId [type]/>+</controller>`.
- In the junction: `<controller id [type] [sequence]/>`.
- Each controller is a signal group, i.e. the lights that always show the same state (one phase approach).
- Controllers referenced by the same junction are synchronized against each other: when one is green, the others are red.
- The phase timing is **not** in OpenDRIVE; it comes from OpenSCENARIO `TrafficSignalController` or the simulator.

**OSM recipe:**
- For each `highway=traffic_signals` junction, create one `<signal dynamic="yes" country="OpenDRIVE" type="1000001" subtype="-1">` per incoming approach, on the **incoming road** near its end.
- Use `orientation` matching the approach direction and `<validity>` for the incoming lanes.
- Create one `<controller>` per approach, or one per pair of opposing approaches, with matching `<control signalId>`.
- List those controllers in the junction's `<controller id>`.

### 7.3 Generic "OpenDRIVE" catalogue (Signal reference 1.0.0, `country="OpenDRIVE"`)

| Code | Meaning |
|---|---|
| 1000001 | 3-light vehicle signal (red/yellow/green) |
| 1000002 | pedestrian signal (subtypes 10, 20, 30) |
| 1000007 | pedestrian and bicycle signal |
| 1000009 | 2-light signals |
| 1000010 / 1000011 | arrow signals: left 10, right 20, straight 30, straight-left 40, straight-right 50 |
| 1000013 | bicycle signal |
| 1000014 | tram signal |
| 1000020 / 1000008 / 1000012 | single red / yellow / green aspects, with arrow subtypes 10–100 |
| 1000021–1000023 | lane-control signals |
| 1000003 / 1000004 | pedestrian / bicycle crossing |
| **1100001** | **invisible stop line** |

### 7.4 Stop lines
- **Logical:** a signal with `country="OpenDRIVE" type="1100001"` (or the national stop-line code, e.g. DE StVO `294`) at the stop position, with `orientation`, `<validity>`, and a `<dependency>` on the traffic light or stop sign. The light or sign carries a `<reference elementType="signal" elementId=<stopline> type="stopline">`.
- **Visual:** an `<object type="roadMark" subtype="stopLine">` (or `crosswalk` for a zebra) with an outline.
- CARLA builds its own stop "trigger volumes" from traffic-light positions and does not need stop lines (verify).

### 7.5 National catalogues
- Type and subtype codes come from the national regulation:
  - DE StVO VzKat (e.g. 205 yield, 206 stop, 274 max speed with subtype `-50` or value, 306 priority road, 350 zebra crossing).
  - US MUTCD (`R1-1` stop, `R2-1` speed limit), with `country="US"`.
- ASAM publishes only the generic "OpenDRIVE" catalogue. Per-country catalogues are not normative in the spec and are application-defined.
- **Recommended strategy:** emit `<semantics>` (portable) and also a national code when known. For CARLA, use the StVO-like codes CARLA recognizes (below).

### 7.6 CARLA signal handling (`SignalType.cpp`, `TrafficLightManager.cpp`)

Signal types:
- `IsTrafficLight(type)` matches `1000001, 1000002, 1000007–1000020, F, W, A`. Country is not checked.
- Recognized sign types use German StVO numbers: `205` yield, `206` stop, `274` max speed, `101` danger, `306` priority way, `310` / `311` city begin and end, `330` highway, `380` / `381` recommended speed, `250`–`267` access forbidden, and so on.

Speed limits:
- `type="274"` is spawned with **subtype = speed value as a string**, and `SetSpeedLimit(value)` reads `value`.
- Prefabs exist for 30, 40, 50, 60, 70, 80, 90, 100, 110 and 120 km/h, so round to these.
- Emit `country="DE"` (or `"OpenDRIVE"`), `type="274" subtype="50" value="50" unit="km/h"`.

Traffic-light grouping:
- A light is grouped by: signal → its first `<controller>` (top-level `<control signalId>`) → that controller's first junction (junction `<controller id>`). This yields one `TrafficLightGroup` per junction.
- **A light without a controller is spawned only if its road is not a junction road**, and it becomes a standalone group with red time 10 s.
- **A controller not referenced by any junction logs an error and the light gets no controller.**
- So: put lights on incoming roads, and always create controllers and reference them from the junction.

Other parser behaviour:
- Signal ids are strings (`SignId`). Keep them numeric and short (≤ 32 chars is safe).
- Road, junction and connection ids are parsed as integers, so all ids must be numeric.
- Missing numeric attributes become 0 (pugixml), and a missing road `junction` becomes 0, not −1. **Always write every attribute**, including `junction="-1"`.
- CARLA does not read: road `link@contactPoint`/`elementType`, `junction@type`, `connection@contactPoint`/`linkedRoad`, `positionRoad`, or `controller/control@type`.

---

## 8. Objects (`<road><objects>`)

### 8.1 `<object>` attributes
- **Required:** `id`, `s`, `t`, `zOffset`.
- **Optional:** `type`, `subtype`, `name`, `validLength`, `orientation` (+, - or none), `hdg` / `pitch` / `roll` (rad, hdg relative to the road direction), `dynamic` (yes or no).
- Size: either a box (`length` > 0 along u, `width` along v, `height` ≥ 0) or a circle (`radius` > 0). Not both.
- `perpToRoad` [1.7] and `invalidated` / `temporary` [1.9].

`type` (e_objectType):
- Valid: `none, obstacle, pole, tree, vegetation, barrier, building, parkingSpace, trafficIsland, crosswalk, gantry, roadMark, roadSurface` [1.8].
- DEPR 1.5: `car, van, bus, trailer, bike, motorbike, tram, train, pedestrian, wind`.
- DEPR 1.8: `streetLamp`→`pole`, `railing`/`soundBarrier`→`barrier`, `patch`→`roadSurface`.

### 8.2 Outline
- `<outlines><outline [id] [fillType] [outer=true] [closed] [laneType]>` contains 2+ `<cornerRoad s t dz height [id]>` **or** 2+ `<cornerLocal u v z height [id]>` **or** `<curveLocal>` [1.9]. Never mix them.
- Area points should be listed **counter-clockwise**.
- `fillType`: `grass, concrete, cobble, asphalt, pavement, gravel, soil, paint` [1.8].
- Corner ids start at 0 and increase by 1. They are mandatory when markings reference them.
- Exactly one outer outline.

### 8.3 Repeat
`<repeat s length distance tStart tEnd zOffsetStart zOffsetEnd [widthStart/End heightStart/End lengthStart/End radiusStart/End] [detachFromReferenceLine] [bT cT dT]>`:
- `distance=0` makes a continuous extruded object, used for guardrails, walls and kerbs.
- `distance>0` places discrete copies, used for posts, lamps and trees.

### 8.4 Markings
`<markings><marking color lineLength spaceLength startOffset stopOffset [side] [weight] [width] [zOffset]><cornerReference id/>*`. The markings go inside the `<outline>` when an outline exists.
- **Zebra crosswalk:** `<object type="crosswalk">` with a closed outline (4 cornerRoad points spanning the carriageway) and a `<marking>` along the stripe direction, e.g. `lineLength=0.5 spaceLength=0.5 width=<crossing length>`. Alternatively, use `fillType="paint"`.
- **Stop line or arrows:** `type="roadMark"`.

### 8.5 Other object elements
- `<parkingSpace access="all|car|women|handicapped|bus|truck|electric|residents" [restrictions]>` inside an `<object type="parkingSpace">`. Use it for OSM `amenity=parking` / `parking_space`.
- `<objectReference s t id orientation [zOffset] [validLength]><validity>*`: one object shared by several roads.
- `<tunnel s length id type="standard|underpass" [name] [lighting 0..1] [daylight 0..1]>` with `<validity>*`.
- `<bridge s length id type="concrete|steel|brick|wood" [name]>` with `<validity>*`. Map OSM `bridge=yes` and `tunnel=yes`. Elevation must be real: OSM `layer` and `level` give only order, so estimate about 5–6 m clearance per layer.
- `<borders>` (object borders), `<skeleton>` [1.8], `<material>`, `<surface><CRG>` and `<crossSectionSurface>` [1.8] exist but are optional.
- Objects need `<validity>` only if lane-specific.

---

## 9. Railroads and surface (brief)
- **Rail:**
  - Tram tracks inside roads use lane type `tram` or `rail`.
  - Dedicated railways are roads with lanes of type `rail` plus `<railroad><switch name id position="dynamic|straight|turn"><mainTrack id s dir/><sideTrack id s dir/><partner?/></switch></railroad>`.
  - `<station id name type="small|medium|large">` contains `<platform>` and `<segment roadId sStart sEnd side>`.
  - Level crossings are `crossing` junctions [1.8], or 1.4-style signals/objects.
- **Surface:** `<surface><CRG file sStart sEnd orientation="same|opposite" mode="attached|attached0|genuine|global" [purpose="elevation|friction"] [sOffset tOffset zOffset zScale hOffset] [xOffset yOffset [1.9]]>` references OpenCRG files. It is not needed for OSM.

---

## 10. Common converter pitfalls (checklist)

1. **Numeric ids everywhere** (road, junction, connection, signal, controller, object) for CARLA and RoadRunner compatibility. Keep them unique per element class. Signal and object ids should be unique across the file.
2. **Write every attribute explicitly**, e.g. `junction="-1"`, `pRange`, `unit`, `contactPoint`. Readers using pugixml default missing values to 0 or an empty string.
3. **Bidirectional links:** if A→B then B→A, with correct contactPoints [1.9 rule]. Connecting roads in junctions must also link to their incoming and outgoing roads.
4. **Heading continuity:** equal heading at road joins (or +π for opposed reference lines). Junction connecting roads must start and end tangent to the incoming and outgoing lane centrelines. Use a clothoid or paramPoly3 fit rather than an arc with kinks.
5. **Lane width at junction entries:** the connecting road's lane width at its ends must equal the linked incoming and outgoing lane widths. Lateral position must match too: `t` of the lane centre plus laneOffset. Taper inside the connecting road.
6. **laneOffset vs reference line:** the connecting road's reference line must start exactly on the incoming road's lane 0 boundary position (t including laneOffset), or on the lane edge for single-lane connectors.
7. **Junction size:** pull incoming roads back from the OSM node by about the max of half the crossing road width plus the sidewalk, plus the turn radius (typically 5–15 m). Connecting roads then have room to curve. Very short connectors (< 1 m) break tools.
8. **Avoid tiny geometries and laneSections** (< 0.1 m). Snap or merge OSM nodes closer than about 0.5 m. No zero-length laneSections.
9. **Lane id contiguity:** ids are contiguous (1..n, −1..−m) in each section. Lists go in descending id order (left lanes n..1, center 0, right −1..−m).
10. **No overlapping roads outside junctions.** Divided highways (two OSM ways) should be two one-way roads, or merged into one road with a `median` lane.
11. **Elevation:** OSM has none. Use DEM (SRTM/Copernicus) samples smoothed into cubic `<elevation>` segments, with C1 joins and matching z at links. Add bridge and tunnel layer offsets. Default to flat (a=0) if no DEM.
12. **Signals placement:** put the signal on the incoming road at `s` close to the road end, with `orientation` matching the approach and `t` on the kerb side (RHT: negative t at the right edge for `+`). Give a `zOffset`. Add `<validity>`.
13. **Speed limits:** use `<type><speed>` per road segment (from OSM `maxspeed`, converting mph and `country:zone` implicit values) and optional lane `<speed>`. Speed signs only where OSM has `traffic_sign` nodes.
14. **LHT countries:** set `rule="LHT"` on each road. Lane semantics follow (left lanes drive +s). CARLA picks the LHT traffic-light blueprint per road.
15. **Precision:** write doubles with ≥ 10 significant digits. Shift coordinates by `<offset>` to keep |x|, |y| < 1e5 m.
16. **Header bounding box** `north/south/east/west` in inertial metres, plus a correct `<geoReference>` so CARLA's `transform_to_geolocation` and esmini lat/long work.
17. **Schema validation:** validate output against the ASAM XSD for the declared `revMinor` (`OpenDRIVE_Core.xsd`, `…Lane.xsd`, etc. from the ASAM deliverables). Run the `asam.net:xodr` checker rules (Annex F / the ASAM Quality Checker OpenDRIVE bundle) in CI.

---

## 11. Tool compatibility summary

| Feature | Spec 1.8 | CARLA (0.9.x) | esmini | RoadRunner |
|---|---|---|---|---|
| line / arc / spiral / paramPoly3 | yes | yes (poly3 too) | yes (all 1.4 types) | yes |
| laneOffset | yes | yes | yes | yes |
| `<border>` | yes (not with laneOffset) | parsed, avoid | partial (verify) | import yes |
| superelevation / shape | yes | not used for mesh (verify) | stored, "road banking" limited | yes |
| direct junction | 1.7+ | **no** (type and linkedRoad ignored) | yes (verify) | export from 1.6+ (verify) |
| virtual junction | 1.5+ | no | partial | – |
| lane types `walking`, `curb`, `slipLane`, `connectingRamp`, `shared` | yes | **mapped to None** | mostly yes | yes |
| `country` alpha-2 | required form | not checked | stored | uses alpha-2 / OpenDRIVE |
| controllers + junction `<controller>` | yes | **required for TL groups** | yes | exports |
| `<semantics>`, boards, layers | 1.8 / 1.9 | ignored | ignored (verify) | partial |

RoadRunner exports OpenDRIVE 1.4, 1.5, 1.6 and 1.7 (selectable). Its import is tolerant but needs consistent lane links (verify).
