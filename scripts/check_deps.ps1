param(
    [ValidateSet('Debug', 'Release')]
    [string]$Config = 'Release'
)

$ErrorActionPreference = 'Stop'
. "$PSScriptRoot\deps_env.ps1"

$projectRoot = Split-Path -Parent $PSScriptRoot
$exe = Join-Path $projectRoot "build\$Config\lvf_deps_check.exe"

if (-not (Test-Path -LiteralPath $exe)) {
    throw "lvf_deps_check.exe not found at $exe. Build the project first."
}

$runtimeDirs = @(Enable-LvfDependencyPath -ProjectRoot $projectRoot)
Write-Host "Runtime PATH:" -ForegroundColor Cyan
$runtimeDirs | ForEach-Object { Write-Host "  $_" }
Write-Host ""

& $exe
exit $LASTEXITCODE
