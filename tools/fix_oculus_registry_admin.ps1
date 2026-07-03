param(
    [string]$MetaRoot = "D:\Meta Horizon"
)

$ErrorActionPreference = "Stop"

$principal = New-Object Security.Principal.WindowsPrincipal(
    [Security.Principal.WindowsIdentity]::GetCurrent()
)
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw "Run this script from PowerShell as Administrator."
}

$metaRootWithSlash = $MetaRoot.TrimEnd("\") + "\"
$key = "HKLM:\SOFTWARE\Oculus VR, LLC\Oculus"

New-Item -Path $key -Force | Out-Null
New-ItemProperty -Path $key -Name "Base" -PropertyType String -Value $metaRootWithSlash -Force | Out-Null
New-ItemProperty -Path $key -Name "Active" -PropertyType DWord -Value 1 -Force | Out-Null
New-ItemProperty -Path $key -Name "DriverVersion" -PropertyType String -Value "1.77.0.000001" -Force | Out-Null

Write-Host "Wrote 64-bit Oculus registry key:"
Get-ItemProperty -Path $key | Select-Object Base, Active, DriverVersion
