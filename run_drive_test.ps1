# CARLA drive test on the imported map: starts the CARLA server (-game, off-screen, no dialogs), drives Traffic
# Manager vehicles for each seed (scripts\carla_drive_test.py), stops the server at the end (also on failure).
# Results: build\drive_tests\<Tag>_<Map>_<seed>.json (+ .txt log).
# Usage:  & .\run_drive_test.ps1 -Map LA_DTLA_ODR -Seeds @(7, 11, 23, 37, 51) [-Routes 3000] [-Tag name]
#   -Routes N: every vehicle a checked random path of N m (scripts\tm_path_check.py); 0 = free autopilot
#   CARLA_UE_BIN / CARLA_PROJECT: the engine's Binaries\Win64 folder / CarlaUnreal.uproject (defaults below)
# Pass several seeds with & (powershell -File keeps only the first value of an [int[]]).
param([string]$Map = "LA_DTLA_ODR", [int[]]$Seeds = @(7, 11, 23, 37, 51), [double]$Routes = 0,
      [int]$Vehicles = 60, [int]$Ticks = 3000, [string]$Tag = "drive")
$ErrorActionPreference = "Continue"
Set-Location $PSScriptRoot
$py38 = ".\venv38\Scripts\python.exe"
$ue = if ($env:CARLA_UE_BIN) { $env:CARLA_UE_BIN } else { "C:\mywork\git_repos\UnrealEngine5_carla\Engine\Binaries\Win64" }
$proj = if ($env:CARLA_PROJECT) { $env:CARLA_PROJECT } else { "C:\mywork\git_repos\CarlaUE5\Unreal\CarlaUnreal\CarlaUnreal.uproject" }
$out = "build\drive_tests"
New-Item -ItemType Directory -Force $out | Out-Null
try {
    Start-Process -FilePath "$ue\UnrealEditor.exe" -ArgumentList @("`"$proj`"", "/Game/Carla/Maps/$Map/$Map", "-game",
        "-RenderOffScreen", "-unattended", "-carla-rpc-port=2000", "-log", "-abslog=$PSScriptRoot\$out\${Tag}_server.log") | Out-Null
    & $py38 "scripts\carla_wait_server.py" --limit 1800
    if ($LASTEXITCODE) { throw "CARLA server not ready" }
    foreach ($s in $Seeds) {
        $f = "$out\${Tag}_${Map}_$s"
        & $py38 "scripts\carla_drive_test.py" --map $Map --vehicles $Vehicles --ticks $Ticks --seed $s --routes $Routes `
            --out "$f.json" *> "$f.txt"
        "$Map seed $s rc=$LASTEXITCODE  ($f.json)"
    }
}
finally {
    Get-Process UnrealEditor -ErrorAction SilentlyContinue | ForEach-Object {
        $c = (Get-CimInstance Win32_Process -Filter "ProcessId=$($_.Id)").CommandLine
        if ($c -match "-game" -and $c -match "carla-rpc-port") { Stop-Process -Id $_.Id -Force; "server stopped" } }
}
