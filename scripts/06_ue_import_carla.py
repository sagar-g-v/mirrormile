"""Import the generated map into CARLA (UE 5.5, ue5-dev).  Runs INSIDE the Unreal Editor.

Editor GUI:   Tools > Execute Python Script... > pick this file
Headless:     UnrealEditor-Cmd.exe <CarlaUnreal.uproject> -run=pythonscript
                  -script="C:/mywork/osm2carla/scripts/06_ue_import_carla.py" -unattended -nosplash

What it does
  1. imports textures, builds a small master material + one instance per Blender material
  2. imports each FBX into /Game/Carla/Static/<Label>/LA_DTLA so CARLA's Tagger assigns
     semantic labels from the folder (Road, RoadLine, SideWalk, Terrain, Building)
  3. complex-as-simple collision (no collision on paint), full-precision UVs
  4. duplicates BaseMap -> /Game/Carla/Maps/LA_DTLA/LA_DTLA, places all meshes at the origin
  5. copies LA_DTLA.xodr next to the map (Maps/LA_DTLA/OpenDrive) and to Maps/OpenDrive
  6. tunnel point lights; CARLA traffic lights / stop signs from the OpenDRIVE signals and CARLA
     streetlights at the 03e positions (CARLA_PROPS)
"""
import json
import os
import shutil

import unreal

PIPE = os.environ.get("OSM2CARLA_ROOT", "C:/mywork/osm2carla")
MAP = "LA_DTLA"
EXPORT = f"{PIPE}/export/{MAP}"
TEX = f"{PIPE}/textures"
META = json.load(open(f"{PIPE}/build/meta.json"))

MAP_DIR = f"/Game/Carla/Maps/{MAP}"
MAT_DIR = f"{MAP_DIR}/Materials"
TEX_DIR = f"{MAP_DIR}/Textures"
LABELS = ["Road", "RoadLine", "SideWalk", "Terrain", "Building", "Vegetation", "Bridge", "GuardRail", "Wall",
          "Pole", "TrafficSign"]             # Pole / TrafficSign: regulatory signs + posts (03e)
# street furniture: True = CARLA's own traffic-light / stop-sign actors (generated from the OpenDRIVE
# signals, so they are controllable and drive the Traffic Manager) + CARLA streetlight assets at the
# 03e positions; False = import the static preview meshes (Pole / TrafficLight / TrafficSign FBX)
CARLA_PROPS = True
if not CARLA_PROPS:                  # static stand-ins for CARLA's traffic lights / stop signs / streetlights
    LABELS += ["Pole_Preview", "TrafficLight_Preview", "TrafficSign_Preview"]
STREETLIGHT_NAMES = ("StreetLight", "Streetlight", "Street_Light", "StreetLamp", "Lamppost", "LampPost")
STREETLIGHT_YAW_OFFSET = 0.0     # degrees, if the CARLA asset's arm does not point along +X
# night lighting: every streetlight gets a downward LED spot at its lamp head unless the CARLA asset
# already carries a light component (CARLA's own street-light blueprints are switched by its LightManager)
LAMP = dict(lumens=12000.0, temperature=3000.0, outer_cone=68.0, inner_cone=35.0, radius_cm=3200.0,
            max_draw_cm=18000.0, freeway_lumens=20000.0)
MASKED = {"PalmFrond": "T_PalmFrond_D.png", "Leaves": "T_Leaves_D.png"}   # alpha-cut foliage cards

# lidar species -> CARLA vegetation assets (first existing one per variant is used); the
# procedural prototypes imported from LA_DTLA_Vegetation.fbx are the fallback
VEG = "/Game/Carla/Static/Vegetation"
CARLA_VEG = {
    "palm": [f"{VEG}/Trees/SM_PalmTree03_{v}" for v in "ABCD"],
    "broadleaf_l": [f"{VEG}/Trees/SM_AcerSaccharum01_{v}" for v in "ABC"],
    "broadleaf_m": [f"{VEG}/Trees/SM_AcerSaccharum01_S_{v}" for v in "DEF"],
    "broadleaf_s": [f"{VEG}/00_LegacyAssets/Trees/SM_JapaneseMaple01_{v}" for v in "ABCD"],
    "shrub": [f"{VEG}/Bushes/SM_Bush02_{v}" for v in "ABCDE"] + [f"{VEG}/Bushes/SM_Bush03_{v}" for v in "ABCDEFG"],
}
NO_COLLISION = {"RoadLine"}
USE_NANITE = {"Building": False, "Terrain": False}   # flip to True if your CARLA build uses Nanite

AT = unreal.AssetToolsHelpers.get_asset_tools()
EAL = unreal.EditorAssetLibrary
MEL = unreal.MaterialEditingLibrary

EMISSIVE = {"TunnelLight": 40.0, "LampHead": 3.0}       # material -> emissive strength (x tinted base colour)
TUNNEL_LIGHT = dict(intensity=2500.0, radius_cm=1400.0, max_draw_cm=20000.0)   # point light per fitting
# material name (Blender slot "M_LA_DTLA_<name>") -> (basecolor, roughness, normal, tint, rough_scale, metallic)
MATERIALS = {
    "Asphalt":   ("T_Asphalt_D.jpg", "T_Asphalt_R.jpg", "T_Asphalt_N.png", (1, 1, 1), 1.0, 0.0),
    "Concrete":  ("T_Sidewalk_D.jpg", "T_Sidewalk_R.jpg", "T_Sidewalk_N.png", (1, 1, 1), 1.0, 0.0),
    "Curb":      ("T_Curb_D.jpg", "T_Curb_R.jpg", "T_Curb_N.png", (1, 1, 1), 1.0, 0.0),
    "Struct":    ("T_Curb_D.jpg", "T_Curb_R.jpg", "T_Curb_N.png", (0.92, 0.9, 0.86), 1.0, 0.0),
    "Pier":      ("T_Curb_D.jpg", "T_Curb_R.jpg", "T_Curb_N.png", (0.85, 0.83, 0.8), 1.0, 0.0),
    # bridges / tunnels (04: Concrete036 weathered, Tiles010 glazed tile, Concrete028 grimy ceiling)
    "BridgeConc": ("T_BridgeConc_D.jpg", "T_BridgeConc_R.jpg", "T_BridgeConc_N.png", (1, 1, 1), 1.0, 0.0),
    "PierConc":   ("T_BridgeConc_D.jpg", "T_BridgeConc_R.jpg", "T_BridgeConc_N.png", (0.95, 0.94, 0.92), 1.0, 0.0),
    "TunnelTile": ("T_TunnelTile_D.jpg", "T_TunnelTile_R.jpg", "T_TunnelTile_N.png", (1, 1, 1), 1.0, 0.0),
    "TunnelCeil": ("T_TunnelCeil_D.jpg", "T_TunnelCeil_R.jpg", "T_TunnelCeil_N.png", (1, 1, 1), 1.0, 0.0),
    "TunnelLight": (None, None, None, (1.0, 0.9, 0.7), 0.4, 0.0),
    # street furniture (03e): regulatory signs + posts; previews only when CARLA_PROPS = False
    "SignAtlas":  ("T_Signs_D.png", None, None, (1, 1, 1), 0.45, 0.0),
    "SignPost":   (None, None, None, (0.55, 0.56, 0.58), 0.5, 0.8),
    "Metal":      (None, None, None, (0.42, 0.43, 0.44), 0.45, 0.8),
    "SignalHead": (None, None, None, (0.05, 0.05, 0.04), 0.6, 0.0),
    "StopSign":   (None, None, None, (0.62, 0.03, 0.03), 0.35, 0.0),
    "LampHead":   (None, None, None, (1.0, 0.85, 0.6), 0.4, 0.0),
    "White":     ("T_Paint_Mask.jpg", None, None, (0.80, 0.80, 0.78), 0.55, 0.0),
    "Yellow":    ("T_Paint_Mask.jpg", None, None, (0.85, 0.58, 0.05), 0.55, 0.0),
    "Ortho":     (META["ortho"], None, None, (1, 1, 1), 0.9, 0.0),
    "Roof":      (META["ortho"], None, None, (1, 1, 1), 0.85, 0.0),
    "Grass":     ("T_Grass_D.jpg", None, None, (1, 1, 1), 0.95, 0.0),
    **{r: (f"T_{r}_D.jpg", None, None, (1, 1, 1), 0.85, 0.0) for r in ("RoofWhite", "RoofLight", "RoofGray", "RoofDark")},
    "Near":      ("T_LA_Near_D.jpg", None, None, (1, 1, 1), 1.0, 0.0),
    "Far":       ("T_LA_Far_D.jpg", None, None, (1, 1, 1), 1.0, 0.0),
    "Bark":      ("T_Bark_D.jpg", None, None, (1, 1, 1), 0.9, 0.0),
    **{f: (f"T_Facade{f}_D.jpg", f"T_Facade{f}_R.jpg", None, (1, 1, 1), 1.0, 0.3 if f.startswith("Glass") else 0.0)
       for f in ("Glass", "GlassDark", "Office", "OfficeTan", "Brick", "Stucco")},
}


def log(*a):
    unreal.log("[osm2carla] " + " ".join(str(x) for x in a))


# ------------------------------------------------------------------------ textures
def import_file(path, dest, name=None, options=None):
    t = unreal.AssetImportTask()
    t.set_editor_property("filename", path)
    t.set_editor_property("destination_path", dest)
    if name:
        t.set_editor_property("destination_name", name)
    t.set_editor_property("automated", True)
    t.set_editor_property("replace_existing", True)
    t.set_editor_property("save", True)
    if options:
        t.set_editor_property("options", options)
    AT.import_asset_tasks([t])
    return [unreal.load_asset(p) for p in t.get_editor_property("imported_object_paths")]


def texture(fname, kind="color"):
    name = os.path.splitext(fname)[0]
    path = f"{TEX_DIR}/{name}"
    if not EAL.does_asset_exist(path):
        import_file(f"{TEX}/{fname}", TEX_DIR, name)
    tex = unreal.load_asset(path)
    if kind == "normal":
        tex.set_editor_property("compression_settings", unreal.TextureCompressionSettings.TC_NORMALMAP)
        tex.set_editor_property("srgb", False)
        tex.set_editor_property("flip_green_channel", True)   # Blender/OpenGL -> UE/DirectX
    elif kind == "linear":
        tex.set_editor_property("compression_settings", unreal.TextureCompressionSettings.TC_MASKS)
        tex.set_editor_property("srgb", False)
    EAL.save_loaded_asset(tex)
    return tex


def make_white_linear():
    """Tiny linear white texture used as the default roughness map."""
    return texture("T_WhiteLinear.png", "linear")   # written by 04_textures.py


# ------------------------------------------------------------------------ materials
def master_material(white_lin):
    path = f"{MAT_DIR}/M_{MAP}_Master"
    if EAL.does_asset_exist(path):
        return unreal.load_asset(path)
    m = AT.create_asset(f"M_{MAP}_Master", MAT_DIR, unreal.Material, unreal.MaterialFactoryNew())

    def param_tex(name, x, y, sampler, default):
        e = MEL.create_material_expression(m, unreal.MaterialExpressionTextureSampleParameter2D, x, y)
        e.set_editor_property("parameter_name", name)
        e.set_editor_property("sampler_type", sampler)
        e.set_editor_property("texture", default)
        return e

    base = param_tex("BaseColorTex", -800, -200, unreal.MaterialSamplerType.SAMPLERTYPE_COLOR,
                     unreal.load_asset("/Engine/EngineResources/WhiteSquareTexture"))
    rough = param_tex("RoughnessTex", -800, 100, unreal.MaterialSamplerType.SAMPLERTYPE_MASKS, white_lin)
    nrm = param_tex("NormalTex", -800, 400, unreal.MaterialSamplerType.SAMPLERTYPE_NORMAL,
                    unreal.load_asset("/Engine/EngineMaterials/DefaultNormal"))
    tint = MEL.create_material_expression(m, unreal.MaterialExpressionVectorParameter, -500, -350)
    tint.set_editor_property("parameter_name", "Tint")
    tint.set_editor_property("default_value", unreal.LinearColor(1, 1, 1, 1))
    rs = MEL.create_material_expression(m, unreal.MaterialExpressionScalarParameter, -500, 250)
    rs.set_editor_property("parameter_name", "RoughnessScale"); rs.set_editor_property("default_value", 1.0)
    met = MEL.create_material_expression(m, unreal.MaterialExpressionScalarParameter, -300, 0)
    met.set_editor_property("parameter_name", "Metallic"); met.set_editor_property("default_value", 0.0)
    mul_c = MEL.create_material_expression(m, unreal.MaterialExpressionMultiply, -300, -250)
    mul_r = MEL.create_material_expression(m, unreal.MaterialExpressionMultiply, -300, 150)
    MEL.connect_material_expressions(base, "RGB", mul_c, "A")
    MEL.connect_material_expressions(tint, "", mul_c, "B")
    MEL.connect_material_expressions(rough, "R", mul_r, "A")
    MEL.connect_material_expressions(rs, "", mul_r, "B")
    MEL.connect_material_property(mul_c, "", unreal.MaterialProperty.MP_BASE_COLOR)
    MEL.connect_material_property(mul_r, "", unreal.MaterialProperty.MP_ROUGHNESS)
    MEL.connect_material_property(met, "", unreal.MaterialProperty.MP_METALLIC)
    MEL.connect_material_property(nrm, "RGB", unreal.MaterialProperty.MP_NORMAL)
    em = MEL.create_material_expression(m, unreal.MaterialExpressionScalarParameter, -500, 500)
    em.set_editor_property("parameter_name", "EmissiveStrength"); em.set_editor_property("default_value", 0.0)
    mul_e = MEL.create_material_expression(m, unreal.MaterialExpressionMultiply, -300, 450)
    MEL.connect_material_expressions(mul_c, "", mul_e, "A")
    MEL.connect_material_expressions(em, "", mul_e, "B")
    MEL.connect_material_property(mul_e, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR)
    MEL.recompile_material(m)
    EAL.save_loaded_asset(m)
    return m


def masked_master():
    """Two-sided masked master for foliage cards (opacity from the texture alpha)."""
    path = f"{MAT_DIR}/M_{MAP}_MasterMasked"
    if EAL.does_asset_exist(path):
        return unreal.load_asset(path)
    m = AT.create_asset(f"M_{MAP}_MasterMasked", MAT_DIR, unreal.Material, unreal.MaterialFactoryNew())
    m.set_editor_property("blend_mode", unreal.BlendMode.BLEND_MASKED)
    m.set_editor_property("two_sided", True)
    t = MEL.create_material_expression(m, unreal.MaterialExpressionTextureSampleParameter2D, -600, 0)
    t.set_editor_property("parameter_name", "BaseColorTex")
    t.set_editor_property("texture", unreal.load_asset("/Engine/EngineResources/WhiteSquareTexture"))
    MEL.connect_material_property(t, "RGB", unreal.MaterialProperty.MP_BASE_COLOR)
    MEL.connect_material_property(t, "A", unreal.MaterialProperty.MP_OPACITY_MASK)
    r = MEL.create_material_expression(m, unreal.MaterialExpressionConstant, -300, 200)
    r.set_editor_property("r", 0.7)
    MEL.connect_material_property(r, "", unreal.MaterialProperty.MP_ROUGHNESS)
    MEL.recompile_material(m)
    EAL.save_loaded_asset(m)
    return m


def material_instances(master):
    out = {}
    mm = masked_master()
    for name, tex in MASKED.items():
        mi_name = f"MI_{MAP}_{name}"
        path = f"{MAT_DIR}/{mi_name}"
        mi = unreal.load_asset(path) if EAL.does_asset_exist(path) else AT.create_asset(
            mi_name, MAT_DIR, unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew())
        MEL.set_material_instance_parent(mi, mm)
        MEL.set_material_instance_texture_parameter_value(mi, "BaseColorTex", texture(tex))
        EAL.save_loaded_asset(mi)
        out[f"M_{MAP}_{name}"] = mi
    for name, (d, r, n, tint, rscale, metal) in MATERIALS.items():
        mi_name = f"MI_{MAP}_{name}"
        path = f"{MAT_DIR}/{mi_name}"
        mi = unreal.load_asset(path) if EAL.does_asset_exist(path) else AT.create_asset(
            mi_name, MAT_DIR, unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew())
        MEL.set_material_instance_parent(mi, master)
        if d:
            MEL.set_material_instance_texture_parameter_value(mi, "BaseColorTex", texture(d))
        if name in EMISSIVE:
            MEL.set_material_instance_scalar_parameter_value(mi, "EmissiveStrength", EMISSIVE[name])
        if r:
            MEL.set_material_instance_texture_parameter_value(mi, "RoughnessTex", texture(r, "linear"))
        if n:
            MEL.set_material_instance_texture_parameter_value(mi, "NormalTex", texture(n, "normal"))
        MEL.set_material_instance_vector_parameter_value(mi, "Tint", unreal.LinearColor(*tint, 1))
        MEL.set_material_instance_scalar_parameter_value(mi, "RoughnessScale", rscale)
        MEL.set_material_instance_scalar_parameter_value(mi, "Metallic", metal)
        EAL.save_loaded_asset(mi)
        out[f"M_{MAP}_{name}"] = mi
    return out


# ------------------------------------------------------------------------ meshes
def fbx_options(label):
    o = unreal.FbxImportUI()
    o.set_editor_property("import_mesh", True)
    o.set_editor_property("import_as_skeletal", False)
    o.set_editor_property("import_materials", False)
    o.set_editor_property("import_textures", False)
    o.set_editor_property("import_animations", False)
    o.set_editor_property("mesh_type_to_import", unreal.FBXImportType.FBXIT_STATIC_MESH)
    sm = o.get_editor_property("static_mesh_import_data")
    sm.set_editor_property("combine_meshes", False)            # one StaticMesh per 256 m tile
    sm.set_editor_property("auto_generate_collision", False)
    sm.set_editor_property("generate_lightmap_u_vs", False)   # CARLA uses dynamic lighting
    sm.set_editor_property("remove_degenerates", True)
    sm.set_editor_property("transform_vertex_to_absolute", True)
    sm.set_editor_property("normal_import_method", unreal.FBXNormalImportMethod.FBXNIM_IMPORT_NORMALS)
    sm.set_editor_property("convert_scene", True)
    sm.set_editor_property("convert_scene_unit", True)
    try:
        sm.set_editor_property("build_nanite", USE_NANITE.get(label, False))
    except Exception:
        pass
    return o


def is_surrounding(name):
    """Near band / horizon terrain tiles (SM_LA_DTLA_Terrain_Nxx / _Fxx): visual only."""
    return name.startswith(f"SM_{MAP}_Terrain_N") or name.startswith(f"SM_{MAP}_Terrain_F")


def no_collision(label, name):
    return label in NO_COLLISION or label == "Vegetation" or is_surrounding(name)


def import_meshes(mis):
    # make sure the legacy FBX pipeline (which honours FbxImportUI) is used
    unreal.SystemLibrary.execute_console_command(None, "Interchange.FeatureFlags.Import.FBX False")
    meshes = {}
    for label in LABELS:
        fbx = f"{EXPORT}/{MAP}_{label}.fbx"
        if not os.path.exists(fbx):
            continue
        dest = f"/Game/Carla/Static/{label.split('_')[0]}/{MAP}"   # folder name drives CARLA semantic tags
        objs = [o for o in import_file(fbx, dest, options=fbx_options(label)) if isinstance(o, unreal.StaticMesh)]
        for sm in objs:
            for i, slot in enumerate(sm.get_editor_property("static_materials")):
                mi = mis.get(str(slot.get_editor_property("material_slot_name")))
                if mi:
                    sm.set_material(i, mi)
            src = sm.get_editor_property("source_models")
            for sml in src:
                b = sml.get_editor_property("build_settings")
                b.set_editor_property("use_full_precision_u_vs", True)
                b.set_editor_property("generate_lightmap_u_vs", False)
                sml.set_editor_property("build_settings", b)
            sm.set_editor_property("source_models", src)
            body = sm.get_editor_property("body_setup")
            if body:
                body.set_editor_property(
                    "collision_trace_flag",
                    unreal.CollisionTraceFlag.CTF_USE_SIMPLE_AS_COMPLEX if no_collision(label, sm.get_name())
                    else unreal.CollisionTraceFlag.CTF_USE_COMPLEX_AS_SIMPLE)
            EAL.save_loaded_asset(sm)
        meshes[label] = objs
        log(label, len(objs), "meshes ->", dest)
    unreal.SystemLibrary.execute_console_command(None, "Interchange.FeatureFlags.Import.FBX True")
    return meshes


# ------------------------------------------------------------------------ vegetation
def place_vegetation(eas, prototypes):
    """One static-mesh actor per detected plant (build/vegetation.json, lidar + NDVI), using CARLA's
    vegetation assets scaled to the measured height / crown radius; prototypes as fallback."""
    vf = f"{PIPE}/build/vegetation.json"
    if not os.path.exists(vf):
        return
    for a in eas.get_all_level_actors():
        if a.get_actor_label().startswith(f"VEG_{MAP}_"):
            eas.destroy_actor(a)
    veg = json.load(open(vf))
    protos = {sm.get_name(): sm for sm in prototypes}
    cache = {}

    def candidates(species):
        if species not in cache:
            found = [unreal.load_asset(p) for p in CARLA_VEG.get(species, []) if EAL.does_asset_exist(p)]
            if not found:              # fallback: our procedural prototypes
                kind = "palm" if species == "palm" else ("shrub" if species == "shrub" else "broadleaf")
                found = [protos[n] for n in sorted(protos) if f"_Veg_{kind}_" in n]
            dims = []
            for sm in found:
                b = sm.get_bounding_box()
                dims.append(((b.max.z - b.min.z) / 100.0,
                             max(b.max.x - b.min.x, b.max.y - b.min.y) / 200.0))   # height, radius (m)
            cache[species] = list(zip(found, dims))
        return cache[species]

    n = 0
    for t in veg["trees"] + veg["shrubs"]:
        cands = candidates(t["species"])
        if not cands:
            continue
        sm, (mh, mr) = cands[t["variant"] % len(cands)]
        sz = t["height"] / max(mh, 0.1)
        sxy = min(max(t["radius"] / max(mr, 0.1), 0.6 * sz), 1.6 * sz)
        loc = unreal.Vector(t["x"] * 100, -t["y"] * 100, (t["z"] - 0.05) * 100)   # OpenDRIVE y -> UE -y
        a = eas.spawn_actor_from_object(sm, loc, unreal.Rotator(0, 0, -t["yaw"]))
        a.set_actor_scale3d(unreal.Vector(sxy, sxy, sz))
        a.set_actor_label(f"VEG_{MAP}_{n:05d}")
        a.set_folder_path(f"{MAP}/Vegetation/{t['species']}")
        comp = a.get_component_by_class(unreal.StaticMeshComponent)
        comp.set_mobility(unreal.ComponentMobility.STATIC)
        if t["species"] == "shrub":
            comp.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
        n += 1
    log("vegetation actors:", n, {k: len(v) for k, v in cache.items()})


# ------------------------------------------------------------------------ level
def build_level(meshes):
    level_path = f"{MAP_DIR}/{MAP}"
    if not EAL.does_asset_exist(level_path):
        EAL.duplicate_asset("/Game/Carla/Maps/BaseMap/BaseMap", level_path)
    les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    les.load_level(level_path)
    for a in eas.get_all_level_actors():                       # idempotent re-runs
        if a.get_actor_label().startswith(f"SM_{MAP}_"):
            eas.destroy_actor(a)
    for label, objs in meshes.items():
        if label == "Vegetation":
            continue                     # prototypes: placed per tree by place_vegetation()
        for sm in objs:
            a = eas.spawn_actor_from_object(sm, unreal.Vector(0, 0, 0))
            a.set_actor_label(sm.get_name())
            a.set_folder_path(f"{MAP}/{label}")
            comp = a.get_component_by_class(unreal.StaticMeshComponent)
            comp.set_mobility(unreal.ComponentMobility.STATIC)
            if no_collision(label, sm.get_name()):
                comp.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
            if label in NO_COLLISION or is_surrounding(sm.get_name()):
                comp.set_editor_property("cast_shadow", False)
    place_vegetation(eas, meshes.get("Vegetation", []))
    place_tunnel_lights(eas)
    if CARLA_PROPS:
        place_street_furniture(eas)
    # vehicle spawn points (map.get_spawn_points()) from CARLA's own waypoints, 0.5 m above the lane
    sp_file = f"{PIPE}/build/spawn_points.json"
    sp_cls = unreal.load_class(None, "/Script/Carla.VehicleSpawnPoint")
    for a in eas.get_all_level_actors():
        if a.get_actor_label().startswith(f"SP_{MAP}_"):
            eas.destroy_actor(a)
    if sp_cls and os.path.exists(sp_file):
        pts = json.load(open(sp_file))["points"]
        for i, (x, y, z, yaw) in enumerate(pts):
            a = eas.spawn_actor_from_class(sp_cls, unreal.Vector(x * 100, y * 100, (z + 0.5) * 100),
                                           unreal.Rotator(0, 0, yaw))   # (roll, pitch, yaw)
            a.set_actor_label(f"SP_{MAP}_{i:03d}")
            a.set_folder_path(f"{MAP}/SpawnPoints")
        log("spawn points:", len(pts))
    les.save_current_level()
    # sanity check of the axis convention: south-west tile must end up at UE x<0, y>0
    for sm in meshes.get("Terrain", [])[:1]:
        b = sm.get_bounding_box()
        log("first terrain tile bounds (cm):", b.min, b.max, "expect x<0 and y>0 for the SW tile")
    return level_path


def place_tunnel_lights(eas):
    """One point light under every tunnel ceiling fitting (03: build/tunnel_lights.json, map frame).
    Movable without shadows (UE5/Lumen needs no light baking); culled beyond max_draw_cm."""
    for a in eas.get_all_level_actors():
        if a.get_actor_label().startswith(f"TL_{MAP}_"):
            eas.destroy_actor(a)
    f = f"{PIPE}/build/tunnel_lights.json"
    if not os.path.exists(f):
        return
    pts = json.load(open(f))
    for i, (x, y, z, _h) in enumerate(pts):
        a = eas.spawn_actor_from_class(unreal.PointLight, unreal.Vector(x * 100, -y * 100, z * 100))
        a.set_actor_label(f"TL_{MAP}_{i:04d}")
        a.set_folder_path(f"{MAP}/TunnelLights")
        c = a.get_component_by_class(unreal.PointLightComponent)
        c.set_mobility(unreal.ComponentMobility.MOVABLE)
        c.set_editor_property("intensity", TUNNEL_LIGHT["intensity"])
        c.set_editor_property("attenuation_radius", TUNNEL_LIGHT["radius_cm"])
        c.set_editor_property("light_color", unreal.Color(255, 224, 170, 255))
        c.set_editor_property("cast_shadows", False)
        c.set_editor_property("max_draw_distance", TUNNEL_LIGHT["max_draw_cm"])
    log("tunnel lights:", len(pts))


def generate_carla_signals(eas):
    """Traffic lights + stop signs from the OpenDRIVE <signal>s with CARLA's own generator (its
    TrafficLightManager / sign manager actor in the level), so the actors carry the signal ids the
    simulator and Traffic Manager expect. Function names differ between CARLA versions."""
    names = ("GenerateSignalsAndTrafficLights", "GenerateTrafficLightsAndSigns", "GenerateTrafficLights",
             "GenerateSignals", "SpawnSignals")
    for a in eas.get_all_level_actors():
        cls = a.get_class().get_name()
        if "TrafficLightManager" in cls or "TrafficSignManager" in cls or "SignalManager" in cls:
            for n in names:
                try:
                    a.call_method(n)
                    log("signals generated with", cls, n)
                    return True
                except Exception:
                    continue
    log("WARNING: could not call CARLA's signal generator from Python. In the level, select the "
        "TrafficLightManager actor and use its generate-signals button (Details panel) once; "
        "traffic-light groups come from the OpenDRIVE controllers.")
    return False


def find_asset(keywords, root="/Game/Carla"):
    reg = unreal.AssetRegistryHelpers.get_asset_registry()
    hits = [a for a in reg.get_assets_by_path(root, recursive=True)
            if any(k.lower() in str(a.asset_name).lower() for k in keywords)]
    bps = [a for a in hits if "Blueprint" in str(a.asset_class_path.asset_name)]
    sms = [a for a in hits if "StaticMesh" in str(a.asset_class_path.asset_name)]
    pick = (bps or sms or [None])[0]
    return (pick.get_asset() if pick else None), bool(bps)


def place_street_furniture(eas):
    generate_carla_signals(eas)
    for a in eas.get_all_level_actors():
        if a.get_actor_label().startswith((f"SL_{MAP}_", f"SLL_{MAP}_")):
            eas.destroy_actor(a)
    f = f"{PIPE}/build/street_furniture.json"
    if not os.path.exists(f):
        return
    lights = json.load(open(f))["street_lights"]
    asset, is_bp = find_asset(STREETLIGHT_NAMES)
    if asset is None:
        for i, p in enumerate(lights):
            add_lamp(eas, p, i)
        log("WARNING: no CARLA streetlight asset found: added", len(lights), "lamps only; "
            "set CARLA_PROPS = False to also import the preview poles")
        return
    n_lamps = 0
    for i, p in enumerate(lights):
        loc = unreal.Vector(p["x"] * 100, -p["y"] * 100, p["z"] * 100)
        rot = unreal.Rotator(0, 0, -p["heading"] + STREETLIGHT_YAW_OFFSET)
        a = (eas.spawn_actor_from_object(asset.generated_class(), loc, rot) if is_bp
             else eas.spawn_actor_from_object(asset, loc, rot))
        if a is None:
            continue
        a.set_actor_label(f"SL_{MAP}_{i:04d}")
        a.set_folder_path(f"{MAP}/StreetLights")
        if not a.get_components_by_class(unreal.LightComponent):
            add_lamp(eas, p, i)
            n_lamps += 1
    log("streetlights:", len(lights), "x", asset.get_name(), "| own lamps added:", n_lamps)


def add_lamp(eas, p, i):
    """Downward shadowless LED spot at the lamp head (UE5 / Lumen: no light baking needed)."""
    hx, hy, hz = p.get("head", (p["x"], p["y"], p["z"] + p.get("height", 9.0)))
    sp = eas.spawn_actor_from_class(unreal.SpotLight, unreal.Vector(hx * 100, -hy * 100, hz * 100),
                                    unreal.Rotator(0, -90, 0))          # pitch -90: straight down
    sp.set_actor_label(f"SLL_{MAP}_{i:04d}")
    sp.set_folder_path(f"{MAP}/StreetLights/Lamps")
    c = sp.get_component_by_class(unreal.SpotLightComponent)
    c.set_mobility(unreal.ComponentMobility.MOVABLE)
    c.set_editor_property("intensity_units", unreal.LightUnits.LUMENS)
    c.set_editor_property("intensity", LAMP["freeway_lumens"] if p.get("kind") == "freeway" else LAMP["lumens"])
    c.set_editor_property("use_temperature", True)
    c.set_editor_property("temperature", LAMP["temperature"])
    c.set_editor_property("outer_cone_angle", LAMP["outer_cone"])
    c.set_editor_property("inner_cone_angle", LAMP["inner_cone"])
    c.set_editor_property("attenuation_radius", LAMP["radius_cm"])
    c.set_editor_property("cast_shadows", False)
    c.set_editor_property("max_draw_distance", LAMP["max_draw_cm"])


def copy_xodr():
    content = unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_content_dir())
    src = f"{EXPORT}/{MAP}.xodr"
    for d in (f"{content}Carla/Maps/{MAP}/OpenDrive", f"{content}Carla/Maps/OpenDrive"):
        os.makedirs(d, exist_ok=True)
        shutil.copy2(src, f"{d}/{MAP}.xodr")
        log("xodr ->", d)


def main():
    master = master_material(make_white_linear())
    mis = material_instances(master)
    meshes = import_meshes(mis)
    copy_xodr()                      # before the level: CARLA's signal generator reads it
    lvl = build_level(meshes)
    log("DONE. Open", lvl, "and press Play, or client.load_world('" + MAP + "')")


main()

