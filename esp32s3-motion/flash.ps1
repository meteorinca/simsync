param(
    [string]$Port = "",
    [switch]$Monitor,
    [switch]$BuildOnly
)

$ErrorActionPreference = "Stop"
$FirmwareDir = Join-Path $PSScriptRoot "firmware"

Write-Host "Loading ESP-IDF v5.5.4 environment..." -ForegroundColor Cyan
. "C:\Espressif\tools\Microsoft.v5.5.4.PowerShell_profile.ps1" | Out-Null

Push-Location $FirmwareDir
try {
    if (-not (Test-Path "sdkconfig")) {
        idf.py set-target esp32s3
    }
    idf.py build

    if ($BuildOnly) {
        Write-Host "Build complete." -ForegroundColor Green
        return
    }

    $argsList = @()
    if ($Port -ne "") {
        $argsList += @("-p", $Port)
    }
    $argsList += "flash"
    if ($Monitor) {
        $argsList += "monitor"
    }

    Write-Host "Flashing ESP32-S3 ($($argsList -join ' '))..." -ForegroundColor Cyan
    idf.py @argsList
} finally {
    Pop-Location
}
