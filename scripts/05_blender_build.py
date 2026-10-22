"""Blender stage (headless):  build/geometry.npz  ->  .blend + FBX per CARLA semantic label.

Run:  blender -b --factory-startup --python-exit-code 1 -P scripts/05_blender_build.py [-- --no-render]
"""
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import bmesh
import bpy
import numpy as np
from mathutils import Vector

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import config as C  # noqa: E402

ARGS = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
META = json.loads(C.META_FILE.read_text())
EXT = META["extent"]
MAP = C.MAP_NAME

# texture tile size in metres for world-scaled UV layers
UV_SCALE = {"Asphalt": (6.0, 6.0), "Concrete": (1.5, 1.5), "Curb": (1.5, 1.5),
            "White": (1.0, 4.0), "Yellow": (1.0, 4.0), "Grass": (2.0, 2.0), "Struct": (2.0, 2.0), "Pier": (2.0, 2.0),
            # bridges / tunnels (strip UVs are metres / 1.5 -> tile = 1.5 x value m)
            "BridgeConc": (3.0, 3.0), "PierConc": (3.0, 3.0), "TunnelTile": (0.8, 0.8), "TunnelCeil": (1.6, 1.6),
            "TunnelLight": (1.0, 1.0),
            "Metal": (1.0, 1.0), "SignalHead": (1.0, 1.0), "StopSign": (1.0, 1.0), "LampHead": (1.0, 1.0),
            "SignPost": (1.0, 1.0),
            **{r: (10.0, 10.0) for r in ("RoofWhite", "RoofLight", "RoofGray", "RoofDark")},
            # photographed facades (04 PHOTO_FACADES): floors in the texture x 3.6 m; Stucco procedural 4x4 bays
            "Glass": (28.8, 28.8), "GlassDark": (36.0, 36.0), "Office": (21.6, 21.6), "OfficeTan": (21.6, 21.6),
            "Brick": (21.6, 21.6), "Stucco": (12.0, 14.4)}
ORTHO_UV = {"Ortho", "Roof"}          # UV = position in the orthophoto
DIRECT_UV = {"Far", "Near", "SignAtlas", "ShopGlass", "ShopSign", "ShopTube", "Awning"}                   # UVs already final (far-terrain imagery)
GEOM_FILES = [C.GEOM_FILE, C.BUILD / "vegetation_geom.npz", C.BUILD / "far_geom.npz", C.BUILD / "furniture_geom.npz",
              C.BUILD / "storefront_geom.npz"]
# street furniture previews (03e): exported to their own FBX, but in CARLA the import script spawns
# CARLA's traffic-light / sign / streetlight blueprints at the same places instead
FURNITURE = {"Pole", "TrafficLight", "TrafficSign"}
PREVIEW_MATS = {"Metal", "SignalHead", "StopSign", "LampHead"}    # -> <Label>_Preview FBX (CARLA actors replace them)
SMOOTH = {"Road", "SideWalk", "RoadLine", "Terrain"}
WELD = {"Road__Asphalt", "SideWalk__Concrete", "Terrain__Ortho"}


# ------------------------------------------------------------------------------ materials
def tex_node(nt, name, non_color=False):
    n = nt.nodes.new("ShaderNodeTexImage")
    img = bpy.data.images.load(str(C.TEX / name), check_existing=True)
    if non_color:
        img.colorspace_settings.name = "Non-Color"
    n.image = img
    return n


def make_material(mat_name):
    m = bpy.data.materials.get(f"M_{MAP}_{mat_name}")
    if m:
        return m
    m = bpy.data.materials.new(f"M_{MAP}_{mat_name}")
    if bpy.app.version < (5, 0, 0):     # node trees are always on from Blender 5.0
        m.use_nodes = True
    nt = m.node_tree
    bsdf = next(n for n in nt.nodes if n.type == "BSDF_PRINCIPLED")
    link = nt.links.new
    pbr = {"Asphalt": "Asphalt", "Concrete": "Sidewalk", "Curb": "Curb", "Struct": "Curb", "Pier": "Curb",
           "BridgeConc": "BridgeConc", "PierConc": "BridgeConc", "TunnelTile": "TunnelTile", "TunnelCeil": "TunnelCeil"}
    if mat_name in pbr:
        k = pbr[mat_name]
        link(tex_node(nt, f"T_{k}_D.jpg").outputs["Color"], bsdf.inputs["Base Color"])
        link(tex_node(nt, f"T_{k}_R.jpg", True).outputs["Color"], bsdf.inputs["Roughness"])
        nm = nt.nodes.new("ShaderNodeNormalMap")
        link(tex_node(nt, f"T_{k}_N.png", True).outputs["Color"], nm.inputs["Color"])
        link(nm.outputs["Normal"], bsdf.inputs["Normal"])
    elif mat_name in ("ShopGlass", "ShopSign", "ShopTube"):
        # storefronts (03f): glazing / name signs / tube lights, lit at night (brighter than the floors above)
        if mat_name == "ShopGlass":
            d = tex_node(nt, "T_Shopfront_D.png"); e = tex_node(nt, "T_Shopfront_E.png")
            link(d.outputs["Color"], bsdf.inputs["Base Color"]); bsdf.inputs["Roughness"].default_value = 0.08
            bsdf.inputs["Metallic"].default_value = 0.2
            link(e.outputs["Color"], bsdf.inputs["Emission Color"])
        elif mat_name == "ShopSign":
            d = tex_node(nt, "T_ShopSigns_D.png")
            link(d.outputs["Color"], bsdf.inputs["Base Color"]); link(d.outputs["Color"], bsdf.inputs["Emission Color"])
            bsdf.inputs["Roughness"].default_value = 0.4
        else:
            bsdf.inputs["Base Color"].default_value = (0.9, 0.9, 0.88, 1)
            bsdf.inputs["Emission Color"].default_value = (1.0, 0.97, 0.9, 1)
        g = nt.nodes.new("ShaderNodeGroup"); g.node_tree = night_group()
        mul = nt.nodes.new("ShaderNodeMath"); mul.operation = "MULTIPLY"; mul.inputs[1].default_value = SHOP_NITS[mat_name]
        link(g.outputs["Night"], mul.inputs[0]); link(mul.outputs["Value"], bsdf.inputs["Emission Strength"])
    elif mat_name == "Awning":
        link(tex_node(nt, "T_Awnings_D.png").outputs["Color"], bsdf.inputs["Base Color"])
        bsdf.inputs["Roughness"].default_value = 0.85
    elif mat_name == "SignAtlas":
        link(tex_node(nt, "T_Signs_D.png").outputs["Color"], bsdf.inputs["Base Color"])
        bsdf.inputs["Roughness"].default_value = 0.45
    elif mat_name == "SignPost":
        bsdf.inputs["Base Color"].default_value = (0.55, 0.56, 0.58, 1)
        bsdf.inputs["Metallic"].default_value = 0.8
        bsdf.inputs["Roughness"].default_value = 0.5
    elif mat_name in ("Metal", "SignalHead", "StopSign"):
        col = {"Metal": (0.42, 0.43, 0.44, 1), "SignalHead": (0.05, 0.05, 0.04, 1), "StopSign": (0.62, 0.03, 0.03, 1)}[mat_name]
        bsdf.inputs["Base Color"].default_value = col
        bsdf.inputs["Metallic"].default_value = 0.8 if mat_name == "Metal" else 0.0
        bsdf.inputs["Roughness"].default_value = {"Metal": 0.45, "SignalHead": 0.6, "StopSign": 0.35}[mat_name]
    elif mat_name == "LampHead":
        bsdf.inputs["Base Color"].default_value = (0.9, 0.85, 0.7, 1)
        bsdf.inputs["Emission Color"].default_value = (1.0, 0.82, 0.55, 1)
        bsdf.inputs["Emission Strength"].default_value = 2.0
    elif mat_name == "TunnelLight":
        bsdf.inputs["Base Color"].default_value = (1.0, 0.92, 0.75, 1)
        bsdf.inputs["Emission Color"].default_value = (1.0, 0.88, 0.65, 1)
        bsdf.inputs["Emission Strength"].default_value = 25.0
    elif mat_name in ("White", "Yellow"):
        col = (0.80, 0.80, 0.78, 1) if mat_name == "White" else (0.85, 0.58, 0.05, 1)
        mix = nt.nodes.new("ShaderNodeMix"); mix.data_type = "RGBA"; mix.blend_type = "MULTIPLY"
        mix.inputs["Factor"].default_value = 1.0
        mix.inputs[6].default_value = col
        link(tex_node(nt, "T_Paint_Mask.jpg", True).outputs["Color"], mix.inputs[7])
        link(mix.outputs[2], bsdf.inputs["Base Color"])
        bsdf.inputs["Roughness"].default_value = 0.55
    elif mat_name in ("Ortho", "Roof"):
        link(tex_node(nt, META["ortho"]).outputs["Color"], bsdf.inputs["Base Color"])
        bsdf.inputs["Roughness"].default_value = 0.9
    elif mat_name in ("Far", "Near"):
        link(tex_node(nt, f"T_LA_{mat_name}_D.jpg").outputs["Color"], bsdf.inputs["Base Color"])
        bsdf.inputs["Roughness"].default_value = 1.0
    elif mat_name in ("RoofWhite", "RoofLight", "RoofGray", "RoofDark"):
        link(tex_node(nt, f"T_{mat_name}_D.jpg").outputs["Color"], bsdf.inputs["Base Color"])
        bsdf.inputs["Roughness"].default_value = 0.85
    elif mat_name == "Grass":
        link(tex_node(nt, "T_Grass_D.jpg").outputs["Color"], bsdf.inputs["Base Color"])
        bsdf.inputs["Roughness"].default_value = 0.95
    elif mat_name in ("PalmFrond", "Leaves"):
        t = tex_node(nt, f"T_{mat_name}_D.png")
        link(t.outputs["Color"], bsdf.inputs["Base Color"])
        link(t.outputs["Alpha"], bsdf.inputs["Alpha"])
        bsdf.inputs["Roughness"].default_value = 0.7
        if hasattr(m, "surface_render_method"):
            m.surface_render_method = "DITHERED"
        else:
            m.blend_method = "CLIP"
        m.use_backface_culling = False
    elif mat_name == "Bark":
        link(tex_node(nt, "T_Bark_D.jpg").outputs["Color"], bsdf.inputs["Base Color"])
        bsdf.inputs["Roughness"].default_value = 0.9
    else:  # facades
        link(tex_node(nt, f"T_Facade{mat_name}_D.jpg").outputs["Color"], bsdf.inputs["Base Color"])
        link(tex_node(nt, f"T_Facade{mat_name}_R.jpg", True).outputs["Color"], bsdf.inputs["Roughness"])
        # lit windows at night: emission map x room switching x the shared night factor (MM_Night)
        if (C.TEX / f"T_Facade{mat_name}_E.jpg").exists():
            add_window_emission(nt, bsdf, mat_name, tex_node(nt, f"T_Facade{mat_name}_E.jpg"))
        if mat_name.startswith("Glass"):
            bsdf.inputs["Metallic"].default_value = 0.3
    return m


WINDOW_NITS = {"Glass": 0.5, "GlassDark": 0.5, "Office": 0.5, "OfficeTan": 0.5, "Brick": 0.4, "Stucco": 0.4}
#                         emission strength of lit windows at full night (tuned in the viewport)


def night_group():
    """Shared node group holding the scene's night factor (0 = day, 1 = night) for all facades:
    change MM_Night's value once to light every window."""
    ng = bpy.data.node_groups.get("MM_Night")
    if ng is None:
        ng = bpy.data.node_groups.new("MM_Night", "ShaderNodeTree")
        ng.interface.new_socket("Night", in_out="OUTPUT", socket_type="NodeSocketFloat")
        out = ng.nodes.new("NodeGroupOutput")
        v = ng.nodes.new("ShaderNodeValue"); v.name = "NightValue"; v.outputs[0].default_value = 0.0
        ng.links.new(v.outputs[0], out.inputs["Night"])
    return ng


ROOM_W, FLOOR_H = 6.0, 3.6     # m: world-space light cells (one room wide, one floor high)
SHOP_NITS = {"ShopGlass": 1.5, "ShopSign": 0.9, "ShopTube": 9.0}    # ground floor: brighter than the windows
ROOM_ON = 0.62                 # share of rooms with the lights on


def add_window_emission(nt, bsdf, mat_name, etex):
    """Emission = window map x the building's light colour x per-room on/off and dimming (random per
    world-space cell, so the texture's repetition disappears) x WINDOW_NITS x night factor."""
    N, L = nt.nodes, nt.links.new
    geo = N.new("ShaderNodeNewGeometry")
    sc = N.new("ShaderNodeVectorMath"); sc.operation = "MULTIPLY"; sc.inputs[1].default_value = (1 / ROOM_W, 1 / ROOM_W, 1 / FLOOR_H)
    fl = N.new("ShaderNodeVectorMath"); fl.operation = "FLOOR"
    wn = N.new("ShaderNodeTexWhiteNoise"); wn.noise_dimensions = "3D"
    L(geo.outputs["Position"], sc.inputs[0]); L(sc.outputs["Vector"], fl.inputs[0]); L(fl.outputs["Vector"], wn.inputs["Vector"])
    on = N.new("ShaderNodeMath"); on.operation = "LESS_THAN"; on.inputs[1].default_value = ROOM_ON
    L(wn.outputs["Value"], on.inputs[0])
    sep = N.new("ShaderNodeSeparateColor"); L(wn.outputs["Color"], sep.inputs["Color"])
    dim = N.new("ShaderNodeMapRange"); dim.inputs["To Min"].default_value = 0.35; dim.inputs["To Max"].default_value = 1.15
    L(sep.outputs["Red"], dim.inputs["Value"])
    # one light colour per building: the building's LightTint vertex colour (03d), white if absent
    vc = N.new("ShaderNodeVertexColor"); vc.layer_name = "LightTint"
    tint = N.new("ShaderNodeMix"); tint.data_type = "RGBA"; tint.blend_type = "MULTIPLY"; tint.inputs["Factor"].default_value = 1.0
    L(etex.outputs["Color"], tint.inputs[6]); L(vc.outputs["Color"], tint.inputs[7])
    L(tint.outputs[2], bsdf.inputs["Emission Color"])
    g = N.new("ShaderNodeGroup"); g.node_tree = night_group()
    m1 = N.new("ShaderNodeMath"); m1.operation = "MULTIPLY"; L(g.outputs["Night"], m1.inputs[0]); L(on.outputs["Value"], m1.inputs[1])
    m2 = N.new("ShaderNodeMath"); m2.operation = "MULTIPLY"; L(m1.outputs["Value"], m2.inputs[0]); L(dim.outputs["Result"], m2.inputs[1])
    m3 = N.new("ShaderNodeMath"); m3.operation = "MULTIPLY"; m3.inputs[1].default_value = WINDOW_NITS.get(mat_name, 1.8)
    L(m2.outputs["Value"], m3.inputs[0]); L(m3.outputs["Value"], bsdf.inputs["Emission Strength"])


def set_night(on):
    night_group().nodes["NightValue"].outputs[0].default_value = 1.0 if on else 0.0


# ------------------------------------------------------------------------------ meshes
def build_uv(mat, V, UV):
    if mat in DIRECT_UV:
        return UV
    if mat in ORTHO_UV:
        x0, y0, x1, y1 = EXT
        uv = np.column_stack([(V[:, 0] - x0) / (x1 - x0), (V[:, 1] - y0) / (y1 - y0)])
        return uv
    su, sv = UV_SCALE[mat]
    uv = UV / np.array([su, sv])
    if mat in ("White", "Yellow"):
        return uv
    return uv - np.floor(uv.min(0))       # keep UVs small -> no 16-bit UV precision loss in UE


def make_object(name, V, F, UV, mat_name, label, face_rgb=None):
    me = bpy.data.meshes.new(name)
    me.vertices.add(len(V)); me.vertices.foreach_set("co", V.astype(np.float32).ravel())
    me.loops.add(F.size); me.loops.foreach_set("vertex_index", F.astype(np.int32).ravel())
    me.polygons.add(len(F)); me.polygons.foreach_set("loop_start", np.arange(0, F.size, 3, dtype=np.int32))
    uvl = me.uv_layers.new(name="UVMap")
    uvl.data.foreach_set("uv", build_uv(mat_name, V, UV)[F.ravel()].astype(np.float32).ravel())
    me.update(calc_edges=True)
    me.validate(clean_customdata=False)
    if face_rgb is None and label == "Building":   # storefronts / roofs: neutral (joins would fill black)
        face_rgb = np.full((len(F), 3), 255, np.uint8)
    if face_rgb is not None:                       # per-building light colour (03d) -> vertex colour
        ca = me.color_attributes.new("LightTint", "BYTE_COLOR", "CORNER")
        rgba = np.concatenate([np.asarray(face_rgb, np.float32) / 255.0, np.ones((len(face_rgb), 1), np.float32)], 1)
        ca.data.foreach_set("color", np.repeat(rgba, 3, axis=0).ravel())
    me.materials.append(make_material(mat_name))
    # smooth shading with sharp edges above 30 deg: curved roads / kerbs / terrain look smooth, kerb
    # tops, building corners and barrier edges stay crisp (normals go into the FBX)
    me.shade_smooth()
    if hasattr(me, "set_sharp_from_angle"):
        me.set_sharp_from_angle(angle=math.radians(30.0 if label in SMOOTH else 20.0))
    elif label not in SMOOTH:
        me.shade_flat()
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    return ob


def weld(ob, dist=0.002):
    bm = bmesh.new(); bm.from_mesh(ob.data)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=dist)
    bm.to_mesh(ob.data); bm.free()


def decimate_terrain(ob):
    """Planar dissolve (keeps silhouettes, removes flat interior triangles), then triangulate."""
    n0 = len(ob.data.polygons)
    # the decimate (planar) and triangulate modifiers' own operations, on the mesh directly (applying the
    # modifiers through operators cost ~0.5 s per terrain tile)
    bm = bmesh.new(); bm.from_mesh(ob.data)
    bmesh.ops.dissolve_limit(bm, angle_limit=math.radians(1.0), verts=bm.verts, edges=bm.edges, delimit={"NORMAL"})
    bmesh.ops.triangulate(bm, faces=bm.faces, quad_method="BEAUTY", ngon_method="BEAUTY")
    bm.to_mesh(ob.data); bm.free()
    return n0, len(ob.data.polygons)


def join(objs, name):
    if len(objs) == 1:
        objs[0].name = name; objs[0].data.name = name
        return objs[0]
    # the join operator on exactly these objects (a context override; deselecting the whole scene before each
    # of ~1100 joins made them slow as the scene grew)
    with bpy.context.temp_override(active_object=objs[0], object=objs[0], selected_objects=objs,
                                   selected_editable_objects=objs):
        bpy.ops.object.join()
    ob = objs[0]
    ob.name = name; ob.data.name = name
    return ob


def build_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.unit_settings.system = "METRIC"; sc.unit_settings.scale_length = 1.0
    per = defaultdict(list)                          # (label, tile) -> [objects]
    stats = defaultdict(lambda: [0, 0])
    items = []
    for path in GEOM_FILES:
        if path.exists():
            g = np.load(path)
            items += [(g, k) for k in sorted({k.rsplit("|", 1)[0] for k in g.files})]
    for g, key in items:
        layer, tile = key.split("|")
        label, mat = layer.split("__")
        V, F, UV = g[key + "|V"], g[key + "|F"], g[key + "|UV"]
        ob = make_object(f"{layer}_{tile}", V, F, UV, mat, label, g[key + "|C"] if key + "|C" in g.files else None)
        if layer in WELD:
            weld(ob)
        if layer == "Terrain__Ortho":
            a, b = decimate_terrain(ob)
            print(f"  terrain {tile}: {a} -> {b} faces")
        per[(label + "_Preview" if mat in PREVIEW_MATS else label, tile)].append(ob)
    cols = {}
    result = defaultdict(list)
    for (label, tile), objs in sorted(per.items()):
        if label not in cols:
            cols[label] = bpy.data.collections.new(label); sc.collection.children.link(cols[label])
        ob = join(objs, f"SM_{MAP}_{label}_{tile}")
        for c in ob.users_collection:
            c.objects.unlink(ob)
        cols[label].objects.link(ob)
        stats[label][0] += 1; stats[label][1] += len(ob.data.polygons)
        result[label].append(ob)
    for k, (n, f) in sorted(stats.items()):
        print(f"{k:10s} objects {n:3d}  tris {f:8d}")
    return result


# ------------------------------------------------------------------------------ vegetation
# canonical prototype sizes (instances are scaled to the lidar-measured height / crown radius)
PROTO = {"palm": dict(h=12.0, r=2.8), "broadleaf": dict(h=10.0, r=3.5), "shrub": dict(h=1.2, r=0.8)}


def _card(bm, uvl, c, u, v, w, h):
    """Quad centred at c spanned by unit vectors u (width) and v (height)."""
    pts = [c - u * w / 2 - v * h / 2, c + u * w / 2 - v * h / 2, c + u * w / 2 + v * h / 2, c - u * w / 2 + v * h / 2]
    f = bm.faces.new([bm.verts.new(p) for p in pts])
    for loop, uv in zip(f.loops, ((0, 0), (1, 0), (1, 1), (0, 1))):
        loop[uvl].uv = uv
    f.material_index = 1
    return f


def _trunk(bm, uvl, path, radii, seg=8):
    """Tapered tube along a polyline (material slot 0 = bark)."""
    rings = []
    for p, r in zip(path, radii):
        rings.append([bm.verts.new(p + Vector((math.cos(a) * r, math.sin(a) * r, 0)))
                      for a in np.linspace(0, 2 * math.pi, seg, endpoint=False)])
    for i in range(len(rings) - 1):
        for k in range(seg):
            f = bm.faces.new([rings[i][k], rings[i][(k + 1) % seg], rings[i + 1][(k + 1) % seg], rings[i + 1][k]])
            for loop, uv in zip(f.loops, ((k / seg, i / 4), ((k + 1) / seg, i / 4), ((k + 1) / seg, (i + 1) / 4), (k / seg, (i + 1) / 4))):
                loop[uvl].uv = uv
            f.material_index = 0


def make_prototype(kind, variant):
    rs = np.random.default_rng(1000 * len(kind) + variant)
    bm = bmesh.new(); uvl = bm.loops.layers.uv.new("UVMap")
    up = Vector((0, 0, 1))
    if kind == "palm":
        H = PROTO["palm"]["h"]; lean = Vector((rs.uniform(-0.6, 0.6), rs.uniform(-0.6, 0.6), 0))
        zs = np.linspace(0, H - 0.8, 9)
        path = [Vector((0, 0, z)) + lean * (z / H) ** 2 for z in zs]
        _trunk(bm, uvl, path, np.linspace(0.30, 0.19, len(zs)))
        top = path[-1]
        for i in range(22):                               # arching fronds
            az = 2 * math.pi * i / 22 + rs.uniform(-0.15, 0.15)
            d = Vector((math.cos(az), math.sin(az), 0)); side = Vector((-d.y, d.x, 0))
            L, rise, droop = rs.uniform(2.6, 3.3), rs.uniform(0.6, 1.4), rs.uniform(1.2, 2.2)
            prev = None
            for t0, t1 in zip(np.linspace(0, 1, 5)[:-1], np.linspace(0, 1, 5)[1:]):
                p0 = top + d * L * t0 + up * (rise * t0 - droop * t0 * t0)
                p1 = top + d * L * t1 + up * (rise * t1 - droop * t1 * t1)
                w0, w1 = 1.1 * math.sin(math.pi * (0.1 + 0.9 * t0)) + 0.1, 1.1 * math.sin(math.pi * (0.1 + 0.9 * t1)) + 0.1
                vs = [bm.verts.new(p0 - side * w0 / 2), bm.verts.new(p0 + side * w0 / 2),
                      bm.verts.new(p1 + side * w1 / 2), bm.verts.new(p1 - side * w1 / 2)]
                f = bm.faces.new(vs)
                for loop, uv in zip(f.loops, ((0, t0), (1, t0), (1, t1), (0, t1))):
                    loop[uvl].uv = uv
                f.material_index = 1
        mats = ("Bark", "PalmFrond")
    else:
        H, R = PROTO[kind]["h"], PROTO[kind]["r"]
        n_cards = 80 if kind == "broadleaf" else 22
        size = 2.4 if kind == "broadleaf" else 0.8
        cz, rz = (0.62 * H, 0.33 * H) if kind == "broadleaf" else (0.55 * H, 0.45 * H)
        if kind == "broadleaf":
            path = [Vector((0, 0, z)) for z in np.linspace(0, cz, 5)]
            _trunk(bm, uvl, path, np.linspace(0.22, 0.12, 5))
            for _ in range(4):                            # main branches into the crown
                a = rs.uniform(0, 2 * math.pi)
                b0 = Vector((0, 0, rs.uniform(0.3, 0.45) * H))
                b1 = b0 + Vector((math.cos(a) * R * 0.6, math.sin(a) * R * 0.6, 0.25 * H))
                _trunk(bm, uvl, [b0, (b0 + b1) / 2, b1], [0.09, 0.06, 0.03], seg=5)
        for _ in range(n_cards):                          # leaf-cluster cards inside the crown ellipsoid
            while True:
                q = Vector(rs.uniform(-1, 1, 3))
                if q.length <= 1:
                    break
            c = Vector((q.x * R, q.y * R, cz + q.z * rz))
            nrm = Vector(rs.normal(size=3)).normalized()
            u = nrm.cross(up if abs(nrm.z) < 0.9 else Vector((1, 0, 0))).normalized(); v = nrm.cross(u)
            _card(bm, uvl, c, u, v, size, size)
        mats = ("Bark", "Leaves")
    name = f"SM_{MAP}_Veg_{kind}_{variant}"
    me = bpy.data.meshes.new(name); bm.to_mesh(me); bm.free()
    for m in mats:
        me.materials.append(make_material(m))
    me.shade_flat()
    return bpy.data.objects.new(name, me)


def build_vegetation():
    vf = C.BUILD / "vegetation.json"
    if not vf.exists():
        return None, []
    veg = json.loads(vf.read_text())
    sc = bpy.context.scene
    protos = bpy.data.collections.new("VegPrototypes"); sc.collection.children.link(protos)
    proto_cols, proto_objs = {}, []
    for kind in PROTO:
        for v in range(3):
            ob = make_prototype(kind, v)
            col = bpy.data.collections.new(ob.name); protos.children.link(col); col.objects.link(ob)
            proto_cols[(kind, v)] = col; proto_objs.append(ob)
    inst = bpy.data.collections.new("Vegetation"); sc.collection.children.link(inst)
    n = 0
    for t in veg["trees"] + veg["shrubs"]:
        sp = t["species"]
        kind = "palm" if sp == "palm" else ("shrub" if sp == "shrub" else "broadleaf")
        P = PROTO[kind]
        sz = t["height"] / P["h"]
        sxy = float(np.clip(t["radius"] / P["r"], 0.5 * sz, 1.8 * sz)) if kind != "palm" else sz * float(np.clip(t["radius"] / P["r"], 0.7, 1.4))
        e = bpy.data.objects.new(f"veg_{n:05d}", None)
        e.instance_type = "COLLECTION"; e.instance_collection = proto_cols[(kind, t["variant"] % 3)]
        e.location = (t["x"], t["y"], t["z"] - 0.05)
        e.rotation_euler = (0, 0, math.radians(t["yaw"]))
        e.scale = (sxy, sxy, sz)
        inst.objects.link(e); n += 1
    print(f"vegetation: {n} instances of {len(proto_objs)} prototypes")
    return protos, proto_objs


def export_vegetation(proto_objs):
    if not proto_objs:
        return None
    bpy.ops.object.select_all(action="DESELECT")
    for o in proto_objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = proto_objs[0]
    path = C.EXPORT / f"{MAP}_Vegetation.fbx"
    bpy.ops.export_scene.fbx(filepath=str(path), use_selection=True, object_types={"MESH"},
                             apply_unit_scale=True, apply_scale_options="FBX_SCALE_NONE", axis_forward="-Z", axis_up="Y",
                             mesh_smooth_type="FACE", path_mode="COPY", embed_textures=False, add_leaf_bones=False)
    print("exported", path.name, f"{path.stat().st_size / 1e6:.1f} MB")
    return path.name


# ------------------------------------------------------------------------------ export
def export_fbx(objs_by_label):
    out = {}
    for label, objs in objs_by_label.items():
        bpy.ops.object.select_all(action="DESELECT")
        for o in objs:
            o.select_set(True)
        bpy.context.view_layer.objects.active = objs[0]
        path = C.EXPORT / f"{MAP}_{label}.fbx"
        bpy.ops.export_scene.fbx(
            filepath=str(path), use_selection=True, object_types={"MESH"},
            apply_unit_scale=True, apply_scale_options="FBX_SCALE_NONE", global_scale=1.0,
            axis_forward="-Z", axis_up="Y", bake_space_transform=False,
            mesh_smooth_type="EDGE", use_mesh_modifiers=True, use_tspace=False,
            path_mode="COPY", embed_textures=False, use_custom_props=False, add_leaf_bones=False)
        out[label] = path.name
        print("exported", path.name, f"{path.stat().st_size/1e6:.1f} MB")
    return out


# ------------------------------------------------------------------------------ preview
def setup_render():
    sc = bpy.context.scene
    sc.render.engine = "BLENDER_EEVEE" if "BLENDER_EEVEE" in {e.identifier for e in bpy.types.RenderSettings.bl_rna.properties["engine"].enum_items} else "BLENDER_EEVEE_NEXT"
    sc.render.resolution_x, sc.render.resolution_y = 1600, 900
    sc.view_settings.view_transform = "AgX"
    sc.view_settings.exposure = -1.3
    w = bpy.data.worlds.new("World"); sc.world = w
    bg = w.node_tree.nodes.get("Background")
    sky = w.node_tree.nodes.new("ShaderNodeTexSky")
    if hasattr(sky, "sun_disc"):
        sky.sun_disc = False          # the Sun lamp provides direct light; avoid a double sun
    w.node_tree.links.new(sky.outputs[0], bg.inputs[0]); bg.inputs[1].default_value = 0.35
    sun = bpy.data.objects.new("Sun", bpy.data.lights.new("Sun", "SUN"))
    sun.data.energy = 3.5; sun.data.angle = math.radians(1.0)
    sun.rotation_euler = (math.radians(40), 0, math.radians(150))
    sc.collection.objects.link(sun)


def render(name, loc, target, lens=24):
    sc = bpy.context.scene
    cam = bpy.data.objects.get("Cam") or bpy.data.objects.new("Cam", bpy.data.cameras.new("Cam"))
    if cam.name not in sc.collection.objects:
        sc.collection.objects.link(cam)
    cam.data.lens = lens; cam.data.clip_end = 120000      # far terrain reaches 35 km
    cam.location = Vector(loc)
    cam.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    sc.camera = cam
    sc.render.filepath = str(C.BUILD / f"render_{name}.png")
    bpy.ops.render.render(write_still=True)
    print("rendered", sc.render.filepath)


SHOP_LIGHT_RGB = {"restaurant": (1.0, 0.70, 0.43), "fast_food": (1.0, 0.93, 0.80), "cafe": (1.0, 0.75, 0.49),
                  "bar": (0.75, 0.31, 1.0), "bank": (0.89, 0.92, 1.0), "health": (0.90, 0.96, 1.0),
                  "hotel": (1.0, 0.77, 0.51), "theatre": (1.0, 0.88, 0.63), "fashion": (1.0, 0.93, 0.84),
                  "grocery": (0.91, 0.94, 1.0), "culture": (1.0, 0.93, 0.82), "office": (1.0, 0.84, 0.65),
                  "shop": (1.0, 0.89, 0.75), "generic": (1.0, 0.84, 0.66)}


def render_night(name, loc, target, lens=24, radius=450.0):
    """Night version of a view: sun off, dark sky, windows lit, streetlight lamps (03e) near the camera."""
    sc = bpy.context.scene
    sun = bpy.data.objects.get("Sun"); bg = sc.world.node_tree.nodes.get("Background")
    e0, b0, x0 = sun.data.energy, bg.inputs[1].default_value, sc.view_settings.exposure
    sun.data.energy = 0.0; bg.inputs[1].default_value = 0.004; sc.view_settings.exposure = 0.9
    set_night(True)
    lamps = []
    f = C.BUILD / "street_furniture.json"
    if f.exists():
        cx, cy = loc[0], loc[1]
        for p in json.loads(f.read_text())["street_lights"]:
            arm, a = (2.6 if p.get("kind") == "freeway" else 2.2), math.radians(p["heading"])
            hx, hy, hz = p.get("head", (p["x"] + arm * math.cos(a), p["y"] + arm * math.sin(a), p["z"] + p.get("height", 9.0) - 0.2))
            if math.hypot(hx - cx, hy - cy) > radius:
                continue
            L = bpy.data.lights.new("lamp", "SPOT"); L.energy = 1800.0 if p.get("kind") == "freeway" else 1200.0
            L.spot_size = math.radians(135); L.spot_blend = 0.5; L.color = (1.0, 0.82, 0.6)
            if hasattr(L, "use_shadow"):
                L.use_shadow = False
            ob = bpy.data.objects.new("lamp", L); ob.location = (hx, hy, hz)   # spots point down (-Z) by default
            sc.collection.objects.link(ob); lamps.append(ob)
    sf = C.BUILD / "storefronts.json"
    if sf.exists():
        S = json.loads(sf.read_text())
        for kind, power in (("shops", 260.0), ("generic", 110.0)):
            for p in S[kind]:
                if math.hypot(p["x"] - loc[0], p["y"] - loc[1]) > radius:
                    continue
                h = math.radians(p["heading"])
                L = bpy.data.lights.new("shoplight", "AREA"); L.shape = "RECTANGLE"
                L.size = max(p["width"] - 0.5, 1.0); L.size_y = 2.6
                L.energy = power * p["width"] / 8.0
                th = SHOP_LIGHT_RGB.get(p["theme"], (1.0, 0.85, 0.7)); L.color = th
                if hasattr(L, "use_shadow"):
                    L.use_shadow = False
                ob = bpy.data.objects.new("shoplight", L)
                ob.location = (p["x"] + 0.35 * math.cos(h), p["y"] + 0.35 * math.sin(h), p["z"] + 1.7)
                # area lights shine along local -Z: point it out of the shop and slightly down
                ob.rotation_euler = (Vector((math.cos(h), math.sin(h), -0.35))).to_track_quat("-Z", "Y").to_euler()
                sc.collection.objects.link(ob); lamps.append(ob)
    render(name, loc, target, lens)
    for ob in lamps:
        L = ob.data; bpy.data.objects.remove(ob); bpy.data.lights.remove(L)
    sun.data.energy, bg.inputs[1].default_value, sc.view_settings.exposure = e0, b0, x0
    set_night(False)
    print(f"night view {name}: {len(lamps)} street lamps")


def map_xy(lat, lon):
    """WGS84 -> map frame (local tangent approximation of the map's tmerc; ~1 m over the map)."""
    return ((lon - C.LON0) * math.cos(math.radians(C.LAT0)) * 111320.0, (lat - C.LAT0) * 110950.0)


def ground_z(x, y):
    from mathutils.bvhtree import BVHTree
    best = None
    for ob in bpy.data.collections["Road"].objects:
        bb = [ob.matrix_world @ Vector(c) for c in ob.bound_box]
        if min(v.x for v in bb) <= x <= max(v.x for v in bb) and min(v.y for v in bb) <= y <= max(v.y for v in bb):
            hit = ob.ray_cast(Vector((x, y, 1000)), Vector((0, 0, -1)))
            if hit[0]:
                best = hit[1].z if best is None else max(best, hit[1].z)
    return best if best is not None else 85.0


def setup_viewport_on_open():
    """Make the saved .blend open framed on the city (it sits at ~80-107 m and spans ~1.2 km)."""
    from mathutils import Euler
    for screen in bpy.data.screens:
        for area in screen.areas:
            if area.type != "VIEW_3D":
                continue
            for sp in area.spaces:
                if sp.type != "VIEW_3D":
                    continue
                sp.clip_start, sp.clip_end = 0.1, 6000.0
                sp.shading.type = "MATERIAL"
                r3d = sp.region_3d
                if r3d:
                    r3d.view_location = Vector((0, 0, 90.0))
                    r3d.view_distance = 1400.0
                    r3d.view_rotation = Euler((0.9, 0, 0.6)).to_quaternion()


if __name__ == "__main__":
    objs = build_scene()
    setup_viewport_on_open()
    fbx = export_fbx(objs)
    protos, proto_objs = build_vegetation()
    vfbx = export_vegetation(proto_objs)
    if vfbx:
        fbx["Vegetation"] = vfbx
    if protos is not None:                       # prototypes are only shown through their instances
        bpy.context.view_layer.layer_collection.children[protos.name].exclude = True
    bpy.ops.wm.save_as_mainfile(filepath=str(C.ROOT / "blender" / f"{MAP}.blend"), compress=True)
    META["fbx"] = fbx
    C.META_FILE.write_text(json.dumps(META, indent=1))
    if "--no-render" not in ARGS:
        setup_render()
        jx, jy = map_xy(34.04930, -118.25330)          # signalised intersection near Pershing Sq.
        z = ground_z(jx, jy)
        # driver's view from a real lane: the spawn point (UE frame: y and yaw mirrored) 25-70 m before
        # the intersection whose heading points at it
        best = None
        sp_file = C.BUILD / "spawn_points.json"
        for x_, y_, z_, yaw in (json.loads(sp_file.read_text())["points"] if sp_file.exists() else []):
            y_, hd = -y_, math.radians(-yaw)
            d = math.hypot(jx - x_, jy - y_)
            off = abs((math.atan2(jy - y_, jx - x_) - hd + math.pi) % (2 * math.pi) - math.pi)
            if 25 < d < 70 and off < math.radians(25) and (best is None or abs(d - 45) < abs(best[4] - 45)):
                best = (x_, y_, z_, hd, d)
        if best:
            cx, cy, cz, hd, _ = best
            render("street", (cx, cy, cz + 1.5), (cx + 80 * math.cos(hd), cy + 80 * math.sin(hd), cz + 3), lens=22)
        else:
            a = math.radians(54)
            cx, cy = jx - 45 * math.cos(a), jy - 45 * math.sin(a)
            render("street", (cx, cy, ground_z(cx, cy) + 1.5), (jx + 60 * math.cos(a), jy + 60 * math.sin(a), z + 4), lens=22)
        render("aerial", (jx - 380, jy - 420, z + 260), (jx - 60, jy - 60, z + 10), lens=28)
        # Pershing Square: palms and lawn, looking north-east
        px, py = map_xy(34.04810, -118.25260)            # Pershing Square
        pz = ground_z(px - 30, py - 25)
        render("park", (px - 30, py - 25, pz + 1.7), (px + 30, py + 25, pz + 6), lens=20)
        # horizon: over the city towards the San Gabriel Mountains (north-east, ~30 km)
        render("horizon", (jx - 300, jy - 300, z + 180), (20000, 25000, 900), lens=35)
        # Four Level Interchange (US-101 / I-110), oblique from the south-west
        ix, iy = map_xy(34.06170, -118.24680)
        iz = ground_z(ix, iy)
        render("interchange", (ix - 260, iy - 300, iz + 170), (ix, iy, iz + 5), lens=30)
        # Harbor Freeway in its trench with the 4th / 5th / 6th Street overpasses, looking north
        fx, fy = map_xy(34.04930, -118.26330)
        render("freeway", (fx - 60, fy - 160, 95.0), (fx + 60, fy + 160, 82.0), lens=26)
        # night: lit office windows + streetlights (driver view and over the financial district)
        if best:
            render_night("night_street", (cx, cy, cz + 1.5), (cx + 80 * math.cos(hd), cy + 80 * math.sin(hd), cz + 3), lens=22)
        render_night("night_aerial", (jx - 260, jy - 300, z + 140), (jx + 40, jy + 40, z + 40), lens=28, radius=600.0)
        bpy.ops.wm.save_as_mainfile(filepath=str(C.ROOT / "blender" / f"{MAP}.blend"), compress=True)




