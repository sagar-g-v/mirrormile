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
import math
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
REIMPORT = False         # True: re-import every FBX even if the label folder already holds the meshes
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

EMISSIVE = {"TunnelLight": 40.0, "LampHead": 3.0}
WINDOW_NITS = 8.0      # lit-window emissive strength at night (facades with T_Facade*_E.jpg)
NIGHT_SIGN = 1.0       # flip to -1.0 if windows light up by day instead of at night (sun-vector convention)
ROOM_CM, FLOOR_CM, ROOM_ON = 600.0, 360.0, 0.62   # light cells (one room x one floor) and share of rooms lit
# storefront night light: material -> (emissive texture, strength); RoomOn 1 = always lit at night
SHOP_LIGHT = {"ShopGlass": ("T_Shopfront_E.png", 12.0), "ShopSign": ("T_ShopSigns_D.png", 6.0),
              "ShopTube": ("/Engine/EngineResources/WhiteSquareTexture", 40.0)}
SHOP_LAMP = dict(lumens=4500.0, generic_lumens=1800.0, height_cm=260.0, radius_cm=1400.0, max_draw_cm=12000.0)       # material -> emissive strength (x tinted base colour)
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
    # storefronts (03f): night light through EmissiveTex x WindowNits (SHOP_LIGHT), never room-switched
    "ShopGlass":  ("T_Shopfront_D.png", None, None, (1, 1, 1), 0.08, 0.2),
    "ShopSign":   ("T_ShopSigns_D.png", None, None, (1, 1, 1), 0.4, 0.0),
    "ShopTube":   (None, None, None, (0.9, 0.9, 0.88), 0.4, 0.0),
    "Awning":     ("T_Awnings_D.png", None, None, (1, 1, 1), 0.85, 0.0),
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


_WARNED = set()


def warn_once(what, ex):
    if what not in _WARNED:
        _WARNED.add(what)
        unreal.log_warning(f"[osm2carla] {what} skipped: {ex}")


SMES = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)


def guarded(name, fn, *args):
    """Run one import stage; on failure log the traceback and carry on with the next stage."""
    import traceback
    try:
        return fn(*args)
    except Exception:  # noqa: BLE001
        unreal.log_error(f"[osm2carla] stage '{name}' failed: " + traceback.format_exc())
        return None


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
    # lit windows at night: EmissiveTex x WindowNits x night factor; the night factor comes from the
    # sky-atmosphere sun direction, so CARLA's day/night weather switches the windows by itself
    win_tex = param_tex("EmissiveTex", -800, 700, unreal.MaterialSamplerType.SAMPLERTYPE_COLOR,
                        unreal.load_asset("/Engine/EngineResources/Black"))
    nits = MEL.create_material_expression(m, unreal.MaterialExpressionScalarParameter, -800, 950)
    nits.set_editor_property("parameter_name", "WindowNits"); nits.set_editor_property("default_value", 0.0)
    sign = MEL.create_material_expression(m, unreal.MaterialExpressionScalarParameter, -1100, 1100)
    sign.set_editor_property("parameter_name", "NightSign"); sign.set_editor_property("default_value", NIGHT_SIGN)
    sun = MEL.create_material_expression(m, unreal.MaterialExpressionSkyAtmosphereLightDirection, -1300, 1000)
    zc = MEL.create_material_expression(m, unreal.MaterialExpressionComponentMask, -1100, 1000)
    zc.set_editor_property("r", False); zc.set_editor_property("g", False); zc.set_editor_property("b", True)
    MEL.connect_material_expressions(sun, "", zc, "")
    zup = MEL.create_material_expression(m, unreal.MaterialExpressionMultiply, -950, 1050)
    MEL.connect_material_expressions(zc, "", zup, "A"); MEL.connect_material_expressions(sign, "", zup, "B")
    dusk = MEL.create_material_expression(m, unreal.MaterialExpressionSubtract, -800, 1050)
    dusk.set_editor_property("const_a", 0.05)                    # fully lit once the sun is ~3 deg below
    MEL.connect_material_expressions(zup, "", dusk, "B")
    gain = MEL.create_material_expression(m, unreal.MaterialExpressionMultiply, -650, 1050)
    gain.set_editor_property("const_b", 10.0)
    MEL.connect_material_expressions(dusk, "", gain, "A")
    night = MEL.create_material_expression(m, unreal.MaterialExpressionSaturate, -500, 1050)
    MEL.connect_material_expressions(gain, "", night, "")
    # rooms: random per world-space cell (ROOM_CM wide, one floor high) -> on/off and dimming;
    # breaks the texture repetition into a natural patchwork of lit and dark rooms
    wpos = MEL.create_material_expression(m, unreal.MaterialExpressionWorldPosition, -1500, 1300)
    cell = MEL.create_material_expression(m, unreal.MaterialExpressionMultiply, -1300, 1300)
    csz = MEL.create_material_expression(m, unreal.MaterialExpressionConstant3Vector, -1500, 1400)
    csz.set_editor_property("constant", unreal.LinearColor(1 / ROOM_CM, 1 / ROOM_CM, 1 / FLOOR_CM, 0))
    MEL.connect_material_expressions(wpos, "", cell, "A"); MEL.connect_material_expressions(csz, "", cell, "B")
    rnd = MEL.create_material_expression(m, unreal.MaterialExpressionVectorNoise, -1100, 1300)
    rnd.set_editor_property("noise_function", unreal.VectorNoiseFunction.VNF_CELLNOISE_ALU)
    MEL.connect_material_expressions(cell, "", rnd, "Position")
    rr = MEL.create_material_expression(m, unreal.MaterialExpressionComponentMask, -950, 1250)
    rr.set_editor_property("r", True); rr.set_editor_property("g", False); rr.set_editor_property("b", False)
    rg = MEL.create_material_expression(m, unreal.MaterialExpressionComponentMask, -950, 1350)
    rg.set_editor_property("r", False); rg.set_editor_property("g", True); rg.set_editor_property("b", False)
    MEL.connect_material_expressions(rnd, "", rr, ""); MEL.connect_material_expressions(rnd, "", rg, "")
    room_on = MEL.create_material_expression(m, unreal.MaterialExpressionScalarParameter, -950, 1150)
    room_on.set_editor_property("parameter_name", "RoomOn"); room_on.set_editor_property("default_value", ROOM_ON)
    on = MEL.create_material_expression(m, unreal.MaterialExpressionStep, -800, 1250)
    MEL.connect_material_expressions(rr, "", on, "Y")                # 1 where RoomOn >= random
    MEL.connect_material_expressions(room_on, "", on, "X")
    dim = MEL.create_material_expression(m, unreal.MaterialExpressionLinearInterpolate, -800, 1350)
    dim.set_editor_property("const_a", 0.35); dim.set_editor_property("const_b", 1.15)
    MEL.connect_material_expressions(rg, "", dim, "Alpha")
    # one light colour per building: the building's LightTint vertex colour (03d via the FBX)
    tint = MEL.create_material_expression(m, unreal.MaterialExpressionVertexColor, -800, 1450)
    w0 = MEL.create_material_expression(m, unreal.MaterialExpressionMultiply, -650, 750)
    MEL.connect_material_expressions(win_tex, "RGB", w0, "A"); MEL.connect_material_expressions(tint, "", w0, "B")
    room = MEL.create_material_expression(m, unreal.MaterialExpressionMultiply, -650, 1300)
    MEL.connect_material_expressions(on, "", room, "A"); MEL.connect_material_expressions(dim, "", room, "B")
    w1 = MEL.create_material_expression(m, unreal.MaterialExpressionMultiply, -500, 750)
    MEL.connect_material_expressions(w0, "", w1, "A"); MEL.connect_material_expressions(nits, "", w1, "B")
    w1r = MEL.create_material_expression(m, unreal.MaterialExpressionMultiply, -420, 850)
    MEL.connect_material_expressions(w1, "", w1r, "A"); MEL.connect_material_expressions(room, "", w1r, "B")
    w2 = MEL.create_material_expression(m, unreal.MaterialExpressionMultiply, -350, 800)
    MEL.connect_material_expressions(w1r, "", w2, "A"); MEL.connect_material_expressions(night, "", w2, "B")
    em_sum = MEL.create_material_expression(m, unreal.MaterialExpressionAdd, -200, 600)
    MEL.connect_material_expressions(mul_e, "", em_sum, "A"); MEL.connect_material_expressions(w2, "", em_sum, "B")
    MEL.connect_material_property(em_sum, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR)
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
        if name in SHOP_LIGHT:
            src, nits_v = SHOP_LIGHT[name]
            tex = unreal.load_asset(src) if src.startswith("/Engine") else texture(src)
            MEL.set_material_instance_texture_parameter_value(mi, "EmissiveTex", tex)
            MEL.set_material_instance_scalar_parameter_value(mi, "WindowNits", nits_v)
            MEL.set_material_instance_scalar_parameter_value(mi, "RoomOn", 1.01)
        if d and d.startswith("T_Facade") and os.path.exists(f"{TEX}/{d.replace('_D.jpg', '_E.jpg')}"):
            MEL.set_material_instance_texture_parameter_value(mi, "EmissiveTex", texture(d.replace("_D.jpg", "_E.jpg")))
            MEL.set_material_instance_scalar_parameter_value(mi, "WindowNits", WINDOW_NITS)
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
    # keep the FBX vertex colours: buildings carry their single night light colour in them
    sm.set_editor_property("vertex_color_import_option", unreal.VertexColorImportOption.REPLACE)
    try:
        sm.set_editor_property("build_nanite", USE_NANITE.get(label, False))
    except Exception:
        pass
    return o


def is_surrounding(name):
    """Near band / horizon terrain tiles (SM_LA_DTLA_Terrain_Nxx / _Fxx): visual only."""
    return f"SM_{MAP}_Terrain_N" in name or f"SM_{MAP}_Terrain_F" in name


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
        if not REIMPORT and EAL.does_directory_exist(dest) and EAL.list_assets(dest, recursive=False):
            objs = [unreal.load_asset(p) for p in EAL.list_assets(dest, recursive=False)
                    if f"SM_{MAP}_{label}_" in p.split("/")[-1].split(".")[0]]
            objs = [o for o in objs if isinstance(o, unreal.StaticMesh)]
            if objs:
                meshes[label] = objs
                log(label, len(objs), "meshes already imported (REIMPORT = False)")
                continue
        objs = [o for o in import_file(fbx, dest, options=fbx_options(label)) if isinstance(o, unreal.StaticMesh)]
        for sm in objs:
            for i, slot in enumerate(sm.get_editor_property("static_materials")):
                mi = mis.get(str(slot.get_editor_property("material_slot_name")))
                if mi:
                    sm.set_material(i, mi)
            # UE5: mesh build settings go through the StaticMeshEditorSubsystem (source models are not
            # exposed as a property); full-precision UVs keep the world-scale UVs exact
            try:
                bs = SMES.get_lod_build_settings(sm, 0)
                bs.set_editor_property("use_full_precision_u_vs", True)
                bs.set_editor_property("generate_lightmap_u_vs", False)
                SMES.set_lod_build_settings(sm, 0, bs)
            except Exception as ex:  # noqa: BLE001
                warn_once("build settings", ex)
            try:
                body = sm.get_editor_property("body_setup")
                if body:
                    body.set_editor_property(
                        "collision_trace_flag",
                        unreal.CollisionTraceFlag.CTF_USE_SIMPLE_AS_COMPLEX if no_collision(label, sm.get_name())
                        else unreal.CollisionTraceFlag.CTF_USE_COMPLEX_AS_SIMPLE)
            except Exception as ex:  # noqa: BLE001
                warn_once("collision setup", ex)
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
    """Create (or open) the map /Game/Carla/Maps/LA_DTLA/LA_DTLA as its own level, from CARLA's BaseMap
    template (sky, weather, CARLA game mode). new_level_from_template creates, saves and opens it in
    one step; duplicating BaseMap and loading the unsaved copy crashes the editor (world leak)."""
    level_path = f"{MAP_DIR}/{MAP}"
    les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    if EAL.does_asset_exist(level_path):
        les.load_level(level_path)
        log("opened existing map", level_path)
    else:
        if not les.new_level_from_template(level_path, "/Game/Carla/Maps/BaseMap/BaseMap"):
            raise RuntimeError(f"could not create {level_path} from BaseMap")
        log("created new map", level_path, "from BaseMap")
    for a in eas.get_all_level_actors():                       # idempotent re-runs
        if f"SM_{MAP}_" in a.get_actor_label():                  # names carry the FBX prefix
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
    guarded("place_vegetation", place_vegetation, eas, meshes.get("Vegetation", []))
    guarded("place_tunnel_lights", place_tunnel_lights, eas)
    guarded("place_shop_lights", place_shop_lights, eas)
    if CARLA_PROPS:
        guarded("place_street_furniture", place_street_furniture, eas)
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
    guarded("setup_sky", setup_sky, eas)
    guarded("register_weather", register_weather)
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


def place_shop_lights(eas):
    """A rect light in front of every lit storefront (03f) facing the street: shops light the
    sidewalk and the road edge at night. Shadowless, culled beyond max_draw_cm."""
    for a in eas.get_all_level_actors():
        if a.get_actor_label().startswith(f"SH_{MAP}_"):
            eas.destroy_actor(a)
    f = f"{PIPE}/build/storefronts.json"
    if not os.path.exists(f):
        return
    S = json.load(open(f))
    n = 0
    for kind in ("shops", "generic"):
        for p in S[kind]:
            h = math.radians(p["heading"])
            loc = unreal.Vector((p["x"] + 0.3 * math.cos(h)) * 100, -(p["y"] + 0.3 * math.sin(h)) * 100, p["z"] * 100 + SHOP_LAMP["height_cm"])
            a = eas.spawn_actor_from_class(unreal.RectLight, loc, unreal.Rotator(0, -20, -p["heading"]))   # out of the shop, 20 deg down
            a.set_actor_label(f"SH_{MAP}_{n:04d}"); a.set_folder_path(f"{MAP}/ShopLights")
            c = a.get_component_by_class(unreal.RectLightComponent)
            c.set_mobility(unreal.ComponentMobility.MOVABLE)
            c.set_editor_property("intensity_units", unreal.LightUnits.LUMENS)
            c.set_editor_property("intensity", SHOP_LAMP["lumens"] if kind == "shops" else SHOP_LAMP["generic_lumens"])
            c.set_editor_property("source_width", max(p["width"] - 0.5, 1.0) * 100)
            c.set_editor_property("source_height", 220.0)
            c.set_editor_property("attenuation_radius", SHOP_LAMP["radius_cm"])
            c.set_editor_property("cast_shadows", False)
            c.set_editor_property("max_draw_distance", SHOP_LAMP["max_draw_cm"])
            n += 1
    log("shop lights:", n)


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
    log("traffic lights / signs: generated by CARLA from the OpenDRIVE signals when the simulation "
        "starts (game mode -> TrafficLightManager::InitializeTrafficLights); nothing to place here")
    return False


def find_asset(keywords, root="/Game/Carla"):
    reg = unreal.AssetRegistryHelpers.get_asset_registry()
    hits = [a for a in reg.get_assets_by_path(root, recursive=True)
            if any(k.lower() in str(a.asset_name).lower() for k in keywords)]
    bps = [a for a in hits if "Blueprint" in str(a.asset_class_path.asset_name)]
    sms = [a for a in hits if "StaticMesh" in str(a.asset_class_path.asset_name)]
    pick = (bps or sms or [None])[0]
    return (pick.get_asset() if pick else None), bool(bps)


STREETLIGHT_ID0 = 100000
CARLA_LIGHT_CLASS = unreal.load_class(None, "/Script/Carla.CarlaLight")


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
        # CARLA's light subsystem ignores lights whose Id is already registered (no day/night switching)
        for k, comp in enumerate(a.get_components_by_class(CARLA_LIGHT_CLASS) if CARLA_LIGHT_CLASS else []):
            try:
                comp.set_editor_property("id", STREETLIGHT_ID0 + i * 4 + k)
            except Exception as ex:  # noqa: BLE001
                warn_once("street light id", ex)
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


# ------------------------------------------------------------------------ sky / lighting
SKY_BP = "/Game/Carla/Blueprints/LevelDesign/BP_Carla_Sky"
SCENE_SETTINGS_BP = "/Game/Carla/Blueprints/BP_GeneralSceneSettings"


def _bp_class(path):
    a = unreal.load_asset(path)
    return a.generated_class() if a else None


def setup_sky(eas):
    """CARLA's town lighting rig (as in Town10HD): BP_Carla_Sky (time-of-day sun / sky light / sky
    atmosphere driven by CARLA's weather) + BP_GeneralSceneSettings, and atmosphere / volumetric
    clouds / height fog where the sky blueprint does not bring its own. BaseMap's plain sun and sky
    light are removed once the CARLA sky provides them (no double sun)."""
    actors = eas.get_all_level_actors()
    names = [a.get_class().get_name() for a in actors]
    placed = []
    for path, label in ((SKY_BP, "CarlaSky"), (SCENE_SETTINGS_BP, "SceneSettings")):
        cls = _bp_class(path)
        if cls is None:
            log("sky: missing", path)
            continue
        if any(a.get_class() == cls for a in actors):
            continue
        a = eas.spawn_actor_from_class(cls, unreal.Vector(0, 0, 0))
        a.set_actor_label(f"{label}_{MAP}"); a.set_folder_path(f"{MAP}/Lighting")
        placed.append(label)
    actors = eas.get_all_level_actors()

    def has_component(kind):
        return any(a.get_components_by_class(kind) for a in actors)
    sky = next((a for a in actors if "Carla_Sky" in a.get_class().get_name()), None)
    sky_comps = sorted({c.get_class().get_name() for c in sky.get_components_by_class(unreal.ActorComponent)}) if sky else []
    log("sky: CARLA sky components:", ", ".join(sky_comps) or "none")
    # the CARLA sky drives sun + sky light: drop BaseMap's own so there is only one sun
    if sky and sky.get_components_by_class(unreal.DirectionalLightComponent):
        for a in actors:
            if isinstance(a, unreal.DirectionalLight) or isinstance(a, unreal.SkyLight):
                if a is not sky and a.get_folder_path() != f"{MAP}/Lighting":
                    log("sky: removed BaseMap", a.get_class().get_name(), a.get_actor_label())
                    eas.destroy_actor(a)
        actors = eas.get_all_level_actors()
    for kind, actor_cls, label in ((unreal.SkyAtmosphereComponent, unreal.SkyAtmosphere, "SkyAtmosphere"),
                                   (unreal.VolumetricCloudComponent, unreal.VolumetricCloud, "VolumetricCloud"),
                                   (unreal.ExponentialHeightFogComponent, unreal.ExponentialHeightFog, "HeightFog")):
        if not has_component(kind):
            a = eas.spawn_actor_from_class(actor_cls, unreal.Vector(0, 0, 0))
            a.set_actor_label(f"{label}_{MAP}"); a.set_folder_path(f"{MAP}/Lighting")
            if label == "HeightFog":                  # LA haze towards the San Gabriel Mountains
                c = a.get_component_by_class(unreal.ExponentialHeightFogComponent)
                c.set_editor_property("fog_density", 0.008)
                c.set_editor_property("fog_height_falloff", 0.12)
                c.set_editor_property("start_distance", 1500.0)
            placed.append(label)
    if not has_component(unreal.SkyLightComponent) and not (sky and sky.get_components_by_class(unreal.SkyLightComponent)):
        a = eas.spawn_actor_from_class(unreal.SkyLight, unreal.Vector(0, 0, 0))
        a.set_actor_label(f"SkyLight_{MAP}"); a.set_folder_path(f"{MAP}/Lighting")
        c = a.get_component_by_class(unreal.SkyLightComponent)
        c.set_editor_property("real_time_capture", True)
        c.set_mobility(unreal.ComponentMobility.MOVABLE)
        placed.append("SkyLight")
    log("sky: placed", ", ".join(placed) or "nothing (already set up)")


WEATHER_BP = "/Game/Carla/Blueprints/Weather/BP_CarlaWeather"
WEATHER_TEMPLATE_MAP = "Town10HD_Opt"


def register_weather():
    """CARLA's weather blueprint keeps per-map default weathers (DefaultWeathers: map name -> weather).
    Without an entry it logs '<map> default weather not found' and the sky never gets its sun / sky
    light values -> black scene. Register this map with Town10HD's defaults."""
    bp = unreal.load_asset(WEATHER_BP)
    if bp is None:
        log("weather: missing", WEATHER_BP); return
    cdo = unreal.get_default_object(bp.generated_class())
    dw = cdo.get_editor_property("DefaultWeathers")
    keys = [str(k) for k in dw.keys()]
    if MAP in keys:
        log("weather: default weather for", MAP, "already registered"); return
    if WEATHER_TEMPLATE_MAP not in keys:
        log("weather: template", WEATHER_TEMPLATE_MAP, "not in DefaultWeathers:", ", ".join(keys)); return
    dw[MAP] = dw[WEATHER_TEMPLATE_MAP]
    cdo.set_editor_property("DefaultWeathers", dw)
    EAL.save_loaded_asset(bp, only_if_is_dirty=False)
    log("weather: registered", MAP, "default weather (copy of", WEATHER_TEMPLATE_MAP + ")")


def sky_only():
    """Apply only the sky / lighting stage to the existing map (OSM2CARLA_STAGE=sky)."""
    les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    les.load_level(f"{MAP_DIR}/{MAP}")
    guarded("register_weather", register_weather)
    guarded("setup_sky", setup_sky, eas)
    les.save_current_level()
    log("DONE (sky stage)")


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


if os.environ.get("OSM2CARLA_STAGE") == "sky":
    sky_only()
else:
    main()
