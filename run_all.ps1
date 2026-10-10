# OSM + USGS lidar DEM + NAIP  ->  OpenDRIVE + Blender + FBX for CARLA (UE5)
# Usage:  .\run_all.ps1            (add -SkipDownload to reuse data/)
param([switch]$SkipDownload)
$ErrorActionPreference = "Continue"   # native tools write warnings to stderr; exit codes are checked
Set-Location $PSScriptRoot
$py = ".\venv\Scripts\python.exe"
$blender = "C:\Program Files\Blender Foundation\Blender 5.2\blender.exe"

function Step($name, [scriptblock]$cmd) {
    Write-Host "`n=== $name ===" -ForegroundColor Cyan
    & $cmd
    if ($LASTEXITCODE) { throw "$name failed (exit $LASTEXITCODE)" }
}

if (-not $SkipDownload) {
    Step "01 download OSM / DEM / imagery / near-infrared" { & $py scripts\01_fetch_data.py }
    Step "01b download lidar point cloud (LA County 2023)" { & $py scripts\01b_fetch_lidar.py }
}
Step "02 netconvert -> OpenDRIVE (+ junction joins)" { & $py scripts\02_build_xodr.py }
Step "02b refine OpenDRIVE (junctions, lidar elevation)" { & $py scripts\02b_refine_xodr.py }
Step "02d stop signs from OSM into the OpenDRIVE" { & $py scripts\02d_stop_signs.py }
Step "02e speed limits (CA defaults) + speed-limit signs" { & $py scripts\02e_speed_signs.py }
Step "02f traffic-light groups (junction controllers)" { & $py scripts\02f_signal_groups.py }
Step "02c building massing from lidar (roof tiers)" { & $py scripts\02c_building_tiers.py }
Step "04 textures" { & $py scripts\04_textures.py }
Step "03 meshes" { & $py scripts\03_build_geometry.py }
Step "03d roof materials" { & $py scripts\03d_roofs.py }
Step "03e street furniture (traffic lights, stop signs, streetlights)" { & $py scripts\03e_street_furniture.py }
Step "03f storefronts (real shop data, signs, night lighting)" { & $py scripts\03f_storefronts.py }
Step "03b vegetation (lidar + NDVI)" { & $py scripts\03b_vegetation.py }
Step "03c surrounding terrain / mountains" { & $py scripts\03c_far_terrain.py }
Step "CARLA validation (LibCarla)" { & .\venv38\Scripts\python.exe scripts\validate_carla_map.py }
Step "mesh vs OpenDRIVE + spawn points" { & $py scripts\check_mesh_vs_xodr.py }
Step "QA audit" { & $py scripts\qa_check.py }
Step "05 Blender build + FBX export" { & $blender -b --factory-startup --python-exit-code 1 -P scripts\05_blender_build.py }
Write-Host "`nDone. Outputs in export\LA_DTLA (xodr + FBX) and blender\LA_DTLA.blend"
Write-Host "QA report: build\qa_report.json   Import into CARLA: scripts\06_ue_import_carla.py (inside the Unreal Editor)"

