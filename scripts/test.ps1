param(
    [ValidateSet('Debug', 'Release')]
    [string]$Config = 'Release'
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $projectRoot 'build'

$ctest = Get-Command ctest -ErrorAction SilentlyContinue
if ($ctest) {
    $ctestPath = $ctest.Source
} else {
    $vsCtest = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\ctest.exe'
    if (Test-Path -LiteralPath $vsCtest) {
        $ctestPath = $vsCtest
    } else {
        throw 'ctest not found. Install CMake or add it to PATH.'
    }
}

& $ctestPath --test-dir $buildDir -C $Config --output-on-failure
exit $LASTEXITCODE
