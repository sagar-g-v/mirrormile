# OSM + USGS lidar DEM + NAIP -> OpenDRIVE (OdrForge) + Blender + FBX for CARLA (UE5)
# Same as run_all.ps1 -SkipDownload, with the road network from OdrForge (02_odrforge_xodr.py) instead of SUMO
# netconvert and its refinements (02, 02b, 02d, 02e, 02f). Reuses the data/ downloads.
# Usage:  .\run_odrforge.ps1   (ODRFORGE_EXE: osm2xodr.exe, default the odrforge Release build)
$ErrorActionPreference = "Continue"
Set-Location $PSScriptRoot
$env:MIRRORMILE_XODR = "odrforge"
$py = ".\venv\Scripts\python.exe"
$blender = "C:\Program Files\Blender Foundation\Blender 5.2\blender.exe"

function Step($name, [scriptblock]$cmd) {
    Write-Host "`n=== $name ===" -ForegroundColor Cyan
    $t = Get-Date
    & $cmd
    if ($LASTEXITCODE) { throw "$name failed (exit $LASTEXITCODE)" }
    Write-Host ("    {0:n0} s" -f ((Get-Date) - $t).TotalSeconds)
}

Step "02 OdrForge -> OpenDRIVE (carla profile)" { & $py scripts\02_odrforge_xodr.py }
Step "02g road levels + highway classes from the OdrForge file" { & $py scripts\02g_odrforge_levels.py }
Step "02c building massing from lidar (roof tiers)" { & $py scripts\02c_building_tiers.py }
Step "04 textures" { & $py scripts\04_textures.py }
Step "03 meshes" { & $py scripts\03_build_geometry.py }
Step "03d roof materials" { & $py scripts\03d_roofs.py }
Step "03e street furniture (traffic lights, stop signs, streetlights)" { & $py scripts\03e_street_furniture.py }
Step "03f storefronts (real shop data, signs, night lighting)" { & $py scripts\03f_storefronts.py }
Step "03b vegetation (lidar + NDVI)" { & $py scripts\03b_vegetation.py }
Step "04b ground cover mask (lawn / dry ground / pavers / parking)" { & $py scripts\04b_ground_mask.py }
Step "03c surrounding terrain / mountains" { & $py scripts\03c_far_terrain.py }
Step "CARLA validation (LibCarla)" { & .\venv38\Scripts\python.exe scripts\validate_carla_map.py }
Step "mesh vs OpenDRIVE + spawn points" { & $py scripts\check_mesh_vs_xodr.py }
Step "QA audit" { & $py scripts\qa_check.py }
Step "05 Blender build + FBX export" { & $blender -b --factory-startup --python-exit-code 1 -P scripts\05_blender_build.py }
Write-Host "`nDone. Outputs in export\LA_DTLA (xodr + FBX) and blender\LA_DTLA.blend"
