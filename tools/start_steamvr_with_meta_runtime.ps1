param(
    [string]$MetaRoot = "D:\Meta Horizon",
    [switch]$RestartSteamVR
)

$ErrorActionPreference = "Stop"

$runtime = Join-Path $MetaRoot "Support\oculus-runtime"
$libovr = Join-Path $runtime "LibOVRRT64_1.dll"
$vrstartup = "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\vrstartup.exe"

if (-not (Test-Path $libovr)) {
    throw "Cannot find $libovr"
}
if (-not (Test-Path $vrstartup)) {
    throw "Cannot find $vrstartup"
}

$env:PATH = "$runtime;$env:PATH"

if ($RestartSteamVR) {
    Get-Process vrserver, vrmonitor, vrwebhelper, vrcompositor -ErrorAction SilentlyContinue |
        Stop-Process -Force
    Start-Sleep -Seconds 3
}

Start-Process -FilePath $vrstartup -WindowStyle Hidden
Write-Host "Started SteamVR with Meta runtime DLL path: $runtime"
