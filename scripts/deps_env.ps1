function Get-LvfCacheValue {
    param(
        [Parameter(Mandatory = $true)][string]$CachePath,
        [Parameter(Mandatory = $true)][string]$Name
    )

    $line = Select-String -LiteralPath $CachePath -Pattern "^$([regex]::Escape($Name)):[^=]*=(.*)$" |
        Select-Object -First 1
    if (-not $line) { return $null }
    return $line.Matches[0].Groups[1].Value.Trim()
}

function Enable-LvfDependencyPath {
    param(
        [Parameter(Mandatory = $true)][string]$ProjectRoot
    )

    $cachePath = Join-Path $ProjectRoot 'build\CMakeCache.txt'
    if (-not (Test-Path -LiteralPath $cachePath)) {
        throw "CMakeCache.txt not found. Configure the project first: cmake -S . -B build ..."
    }

    $pclDir = Get-LvfCacheValue -CachePath $cachePath -Name 'PCL_DIR'
    $cvDir = Get-LvfCacheValue -CachePath $cachePath -Name 'OpenCV_DIR'
    $vtkDir = Get-LvfCacheValue -CachePath $cachePath -Name 'VTK_DIR'

    $runtimeDirs = New-Object System.Collections.Generic.List[string]
    if ($pclDir) {
        $pclRoot = Split-Path -Parent $pclDir
        $runtimeDirs.Add((Join-Path $pclRoot 'bin'))
        $runtimeDirs.Add((Join-Path $pclRoot '3rdParty\FLANN\bin'))
        $runtimeDirs.Add((Join-Path $pclRoot '3rdParty\Qhull\bin'))
    }
    if ($vtkDir) {
        $vtkRoot = (Resolve-Path (Join-Path $vtkDir '..\..\..')).Path
        $runtimeDirs.Add((Join-Path $vtkRoot 'bin'))
    }
    if ($cvDir) {
        $runtimeDirs.Add((Join-Path $cvDir 'x64\vc16\bin'))
    }

    $existing = @($runtimeDirs | Where-Object { Test-Path -LiteralPath $_ })
    if ($existing.Count -gt 0) {
        $env:PATH = (($existing -join ';') + ';' + $env:PATH)
    }
    return $existing
}
