$ErrorActionPreference = "Stop"
$HostDir = Join-Path $PSScriptRoot "host"
$OutExe  = Join-Path $PSScriptRoot "simsync-s3.exe"

Push-Location $HostDir
try {
    Write-Host "Running Go unit tests..." -ForegroundColor Cyan
    go test -v ./...
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

    Write-Host "Running go vet..." -ForegroundColor Cyan
    go vet ./...
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

    Write-Host "Building $OutExe..." -ForegroundColor Cyan
    go build -trimpath -ldflags="-s -w" -o $OutExe .\cmd\simsync-s3
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

    Write-Host "Built $OutExe successfully." -ForegroundColor Green
} finally {
    Pop-Location
}
