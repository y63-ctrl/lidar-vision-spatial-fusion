param(
    [string]$DataRoot = 'data/sample',
    [string]$Frame = '000000',
    [string]$Out = 'results',
    [string]$Calib = '',
    [float]$Resolution = 0.2,
    [float]$Voxel = 0.2,
    [int]$Inflation = 0,
    [int]$Scale = 2,
    [switch]$NoRaycast,
    [switch]$NoGrid
)

$ErrorActionPreference = 'Stop'
. "$PSScriptRoot\deps_env.ps1"

$projectRoot = Split-Path -Parent $PSScriptRoot
$exe = Join-Path $projectRoot 'build\Release\lvf_map_frame.exe'
if (-not (Test-Path -LiteralPath $exe)) {
    throw "lvf_map_frame.exe not found at $exe. Build the project first."
}

$null = Enable-LvfDependencyPath -ProjectRoot $projectRoot

function Resolve-ProjectPath([string]$Path) {
    if ([System.IO.Path]::IsPathRooted($Path)) {
        return $Path
    }
    return Join-Path $projectRoot $Path
}

$toolArgs = @(
    '--data-root', (Resolve-ProjectPath $DataRoot),
    '--frame', $Frame,
    '--out', (Resolve-ProjectPath $Out),
    '--resolution', "$Resolution",
    '--voxel', "$Voxel",
    '--inflation', "$Inflation",
    '--scale', "$Scale"
)
if ($Calib) {
    $toolArgs += @('--calib', (Resolve-ProjectPath $Calib))
}
if ($NoRaycast) {
    $toolArgs += '--no-raycast'
}
if ($NoGrid) {
    $toolArgs += '--no-grid'
}

& $exe @toolArgs
exit $LASTEXITCODE
