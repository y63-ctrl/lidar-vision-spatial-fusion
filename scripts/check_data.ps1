param(
    [string]$DataRoot = (Join-Path $PSScriptRoot '..\data\sample'),
    [string]$Frame = '000000'
)

$ErrorActionPreference = 'Stop'
$failures = New-Object System.Collections.Generic.List[string]

function Fail([string]$Message) {
    $failures.Add($Message)
    Write-Host "[FAIL] $Message" -ForegroundColor Red
}

function Ok([string]$Message) {
    Write-Host "[OK]   $Message" -ForegroundColor Green
}

try {
    $root = (Resolve-Path -LiteralPath $DataRoot).Path
} catch {
    Write-Host "[FAIL] data root not found: $DataRoot" -ForegroundColor Red
    exit 1
}

$imagePath = Join-Path $root "image_2\$Frame.png"
$cloudPath = Join-Path $root "velodyne\$Frame.bin"
$calibPath = Join-Path $root "calib\$Frame.txt"

foreach ($item in @(
    @{ Name = 'image'; Path = $imagePath },
    @{ Name = 'velodyne'; Path = $cloudPath },
    @{ Name = 'calib'; Path = $calibPath }
)) {
    if (Test-Path -LiteralPath $item.Path -PathType Leaf) {
        $length = (Get-Item -LiteralPath $item.Path).Length
        if ($length -le 0) {
            Fail "$($item.Name) file is empty: $($item.Path)"
        } else {
            Ok "$($item.Name): $($item.Path) ($length bytes)"
        }
    } else {
        Fail "$($item.Name) file missing: $($item.Path)"
    }
}

if (Test-Path -LiteralPath $cloudPath -PathType Leaf) {
    $bytes = (Get-Item -LiteralPath $cloudPath).Length
    if (($bytes % 16) -ne 0) {
        Fail "velodyne .bin size must be a multiple of 16 bytes (float32 x,y,z,intensity); got $bytes"
    } else {
        Ok "velodyne layout: $($bytes / 16) points (4 float32 per point)"
    }
}

if (Test-Path -LiteralPath $calibPath -PathType Leaf) {
    $calib = Get-Content -LiteralPath $calibPath -Raw
    foreach ($key in @('P2', 'R0_rect', 'Tr_velo_to_cam')) {
        if ($calib -match "(?m)^\s*$([regex]::Escape($key))\s*:") {
            Ok "calib key present: $key"
        } else {
            Fail "calib key missing: $key"
        }
    }
}

if (Test-Path -LiteralPath $imagePath -PathType Leaf) {
    try {
        Add-Type -AssemblyName System.Drawing
        $image = [System.Drawing.Image]::FromFile($imagePath)
        try {
            if ($image.Width -le 0 -or $image.Height -le 0) {
                Fail "image has invalid dimensions: $($image.Width)x$($image.Height)"
            } else {
                Ok "image dimensions: $($image.Width)x$($image.Height)"
            }
        } finally {
            $image.Dispose()
        }
    } catch {
        Fail "image could not be decoded: $($_.Exception.Message)"
    }
}

Write-Host ''
if ($failures.Count -gt 0) {
    Write-Host "Data check failed with $($failures.Count) problem(s)." -ForegroundColor Red
    exit 1
}

Write-Host "Data check passed: $root (frame $Frame)" -ForegroundColor Green
exit 0
