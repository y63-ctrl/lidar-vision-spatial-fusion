param(
    [string]$DataRoot = 'data/sample',
    [string]$Frame = '000000'
)

$ErrorActionPreference = 'Stop'
. "$PSScriptRoot\deps_env.ps1"

$projectRoot = Split-Path -Parent $PSScriptRoot
$exe = Join-Path $projectRoot 'build\Release\lvf_degradation_bench.exe'
if (-not (Test-Path -LiteralPath $exe)) {
    throw "lvf_degradation_bench.exe not found at $exe. Build the project first."
}

$null = Enable-LvfDependencyPath -ProjectRoot $projectRoot
$dataPath = if ([System.IO.Path]::IsPathRooted($DataRoot)) { $DataRoot } else { Join-Path $projectRoot $DataRoot }

& $exe --data-root $dataPath --frame $Frame
exit $LASTEXITCODE
