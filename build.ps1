<#
.SYNOPSIS
    Builds SimSync for Windows and/or Linux.

.DESCRIPTION
    Compiles the SimSync Go motion engine and embedded web UI.
    By default, it builds both the Windows executable (simsync.exe)
    and the Linux executable (simsync).

.PARAMETER Target
    Specify 'all' (default), 'windows', or 'linux'.

.EXAMPLE
    .\build.ps1
    Builds both simsync.exe (Windows) and simsync (Linux).

.EXAMPLE
    .\build.ps1 -Target windows
    Builds only simsync.exe.
#>
[CmdletBinding()]
param (
    [ValidateSet('all', 'windows', 'linux')]
    [string]$Target = 'all',
    [string]$OutputDirectory = '.'
)

$ErrorActionPreference = 'Stop'

# Ensure we operate in the directory containing this script
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $ScriptDir
$BuildOutput = [System.IO.Path]::GetFullPath((Join-Path $ScriptDir $OutputDirectory))
New-Item -ItemType Directory -Path $BuildOutput -Force | Out-Null

# Check for Go
if (-not (Get-Command go -ErrorAction SilentlyContinue)) {
    Write-Error "Go was not found in PATH. Please install/activate Go and try again."
    exit 1
}

function Build-Binary {
    param (
        [string]$OS,
        [string]$Arch,
        [string]$OutputFile,
        [string]$DisplayName
    )

    Write-Host "Building $DisplayName ($OS/$Arch) -> $OutputFile..." -ForegroundColor Cyan

    $tempFile = Join-Path $ScriptDir "simsync-build.$OS.$Arch.$PID.tmp"
    if (Test-Path $tempFile) {
        Remove-Item $tempFile -Force -ErrorAction SilentlyContinue
    }

    $env:GOOS = $OS
    $env:GOARCH = $Arch

    try {
        & go build -trimpath -buildvcs=false -o $tempFile ./cmd/simsync
        if ($LASTEXITCODE -ne 0) {
            throw "go build failed with exit code $LASTEXITCODE"
        }
        $destination = Join-Path $BuildOutput $OutputFile
        Move-Item -LiteralPath $tempFile -Destination $destination -Force
        $fileItem = Get-Item -LiteralPath $destination
        $sizeMB = [math]::Round($fileItem.Length / 1MB, 2)
        Write-Host "Successfully built $destination ($sizeMB MB)" -ForegroundColor Green
    }
    catch {
        if (Test-Path $tempFile) {
            Remove-Item $tempFile -Force -ErrorAction SilentlyContinue
        }
        Write-Error "Failed to build $DisplayName`: $_"
        exit 1
    }
}

Write-Host "=== SimSync Multi-Platform Build ===" -ForegroundColor Magenta

if ($Target -in @('all', 'windows')) {
    Build-Binary -OS "windows" -Arch "amd64" -OutputFile "simsync.exe" -DisplayName "Windows Executable"
}

if ($Target -in @('all', 'linux')) {
    Build-Binary -OS "linux" -Arch "amd64" -OutputFile "simsync" -DisplayName "Linux Executable"
}

Write-Host "`nBuild complete!" -ForegroundColor Green
Write-Host "Usage:"
if ($Target -in @('all', 'windows')) {
    Write-Host "  Windows: .\simsync.exe --no-udp"
}
if ($Target -in @('all', 'linux')) {
    Write-Host "  Linux:   ./simsync --no-udp"
}
