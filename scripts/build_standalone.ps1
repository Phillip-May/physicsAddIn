param(
    [ValidateSet('Release', 'Debug')]
    [string]$Configuration = 'Release',
    [string]$BuildDir,
    [switch]$Deploy
)

. (Join-Path $PSScriptRoot 'common.ps1')

$config = Get-DependencyConfig
Test-StandaloneDependencies $config

if (-not $BuildDir) {
    $BuildDir = Join-Path (Split-Path -Parent $RepoRoot) ('physicsAddIn-build-standalone-{0}' -f $Configuration.ToLowerInvariant())
}

$outDir = Join-Path $BuildDir ('bin\{0}' -f $Configuration.ToLowerInvariant())
$exe = Join-Path $outDir 'QtCadViewer.exe'
$buildStart = Get-Date

New-Item -ItemType Directory -Force -Path $outDir | Out-Null

# Legacy output locations from earlier build conventions; a stale exe there shadows the real one.
foreach ($stalePath in @(
    (Join-Path $BuildDir 'QtCadViewer.exe'),
    (Join-Path $BuildDir 'release\QtCadViewer.exe'),
    (Join-Path $BuildDir 'debug\QtCadViewer.exe'),
    $exe
)) {
    Remove-Item -LiteralPath $stalePath -Force -ErrorAction SilentlyContinue
}

Invoke-QMakeBuild -Config $config -ProFile (Join-Path $RepoRoot 'QtCadViewer\QtCadViewer.pro') `
    -Configuration $Configuration -BuildDir $BuildDir -QMakeAssignments @("DESTDIR=$outDir")

$exeInfo = Assert-FreshOutput -Path $exe -Since $buildStart -What 'Standalone'

if ($Deploy) {
    Copy-RuntimeDlls -ExePath $exe -Config $config
}

Write-Host "Standalone build complete: $($exeInfo.FullName)"
