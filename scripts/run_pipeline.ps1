param(
    [string]$DataRoot = 'data/sample',
    [string]$Frame = '000000',
    [string]$Out = 'results',
    [string]$Calib = '',
    [switch]$All,
    [ValidateSet('auto', 'pinhole', 'depth')]
    [string]$Mode = 'auto',
    [int]$Threads = 0,
    [float]$Resolution = 0.2,
    [float]$Voxel = 0.2,
    [int]$Inflation = 0,
    [int]$PreviewScale = 2,
    [switch]$NoRaycast,
    [switch]$NoViz,
    [switch]$NoPcd,
    [switch]$NoGrid,
    [switch]$FailOnDegraded
)

$ErrorActionPreference = 'Stop'
. "$PSScriptRoot\deps_env.ps1"

$projectRoot = Split-Path -Parent $PSScriptRoot
$exe = Join-Path $projectRoot 'build\Release\lidar_fusion.exe'
if (-not (Test-Path -LiteralPath $exe)) {
    throw "lidar_fusion.exe not found at $exe. Build the project first."
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
    '--out', (Resolve-ProjectPath $Out),
    '--mode', $Mode,
    '--threads', "$Threads",
    '--resolution', "$Resolution",
    '--voxel', "$Voxel",
    '--inflation', "$Inflation",
    '--preview-scale', "$PreviewScale"
)
if ($All) {
    $toolArgs += '--all'
} else {
    $toolArgs += @('--frame', $Frame)
}
if ($Calib) {
    $toolArgs += @('--calib', (Resolve-ProjectPath $Calib))
}
if ($NoRaycast) { $toolArgs += '--no-raycast' }
if ($NoViz) { $toolArgs += '--no-viz' }
if ($NoPcd) { $toolArgs += '--no-pcd' }
if ($NoGrid) { $toolArgs += '--no-grid' }
if ($FailOnDegraded) { $toolArgs += '--fail-on-degraded' }

& $exe @toolArgs
exit $LASTEXITCODE
