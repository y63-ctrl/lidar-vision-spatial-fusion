param(
    [string]$DataRoot = 'data/sample',
    [string]$Frame = '000000',
    [string]$Calib = ''
)

$ErrorActionPreference = 'Stop'
. "$PSScriptRoot\deps_env.ps1"

$projectRoot = Split-Path -Parent $PSScriptRoot
$exe = Join-Path $projectRoot 'build\Release\lvf_inspect_frame.exe'

if (-not (Test-Path -LiteralPath $exe)) {
    throw "lvf_inspect_frame.exe not found at $exe. Build the project first."
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
    '--frame', $Frame
)
if ($Calib) {
    $toolArgs += @('--calib', (Resolve-ProjectPath $Calib))
}

& $exe @toolArgs
exit $LASTEXITCODE
