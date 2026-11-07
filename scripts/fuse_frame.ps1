param(
    [string]$DataRoot = 'data/sample',
    [string]$Frame = '000000',
    [string]$Out = 'results',
    [ValidateSet('auto', 'pinhole', 'depth')]
    [string]$Mode = 'auto',
    [int]$Threads = 0,
    [int]$PointSize = 2,
    [string]$Calib = '',
    [switch]$NoText
)

$ErrorActionPreference = 'Stop'
. "$PSScriptRoot\deps_env.ps1"

$projectRoot = Split-Path -Parent $PSScriptRoot
$exe = Join-Path $projectRoot 'build\Release\lvf_fuse_frame.exe'

if (-not (Test-Path -LiteralPath $exe)) {
    throw "lvf_fuse_frame.exe not found at $exe. Build the project first."
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
    '--mode', $Mode,
    '--threads', "$Threads",
    '--point-size', "$PointSize"
)
if ($Calib) {
    $toolArgs += @('--calib', (Resolve-ProjectPath $Calib))
}
if ($NoText) {
    $toolArgs += '--no-text'
}

& $exe @toolArgs
exit $LASTEXITCODE
