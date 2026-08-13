param(
    [ValidateSet('Release', 'Debug')]
    [string]$Configuration = 'Release',
    [switch]$Deploy
)

. (Join-Path $PSScriptRoot 'common.ps1')

$config = Get-DependencyConfig
Test-StandaloneDependencies $config

$configLower = $Configuration.ToLowerInvariant()
# Deliberately no -BuildDir parameter, and no DESTDIR passed to qmake: the .pro pins the
# executable to one repo-owned location so no shadow tree can publish a plausible but stale
# RobotSimulator.exe.
$BuildDir = Join-Path (Split-Path -Parent $RepoRoot) 'physicsAddIn-build-robot-simulator'
$outDir = Join-Path $RepoRoot ('dist\RobotSimulator\{0}' -f $configLower)
$exe = Join-Path $outDir 'RobotSimulator.exe'
$buildStart = Get-Date

New-Item -ItemType Directory -Force -Path $outDir | Out-Null
Remove-Item -LiteralPath $exe -Force -ErrorAction SilentlyContinue

Invoke-QMakeBuild -Config $config -ProFile (Join-Path $RepoRoot 'RobotSimulator\RobotSimulator.pro') `
    -Configuration $Configuration -BuildDir $BuildDir

$exeInfo = Assert-FreshOutput -Path $exe -Since $buildStart -What 'Robot simulator'

# RobotSimulator owns a desktop PhysX drag-chain scene. Keep its dynamic runtime libraries
# beside the executable even for a non-deploy developer build; otherwise the build succeeds
# but the command line handed to a tester fails before main().
$physxBin = Join-Path $config.PhysXRoot ('bin\{0}\{1}' -f $config.PhysXPlatform, $configLower)
foreach ($dllName in @('PhysX_64.dll', 'PhysXFoundation_64.dll', 'PhysXCommon_64.dll', 'PhysXCooking_64.dll', 'PVDRuntime_64.dll')) {
    $sourceDll = Join-Path $physxBin $dllName
    Assert-File $sourceDll "PhysX runtime DLL missing: $dllName"
    Copy-Item -LiteralPath $sourceDll -Destination (Join-Path $outDir $dllName) -Force
}

if ($Deploy) {
    # RobotSimulator links no Qt (CONFIG -= qt), so windeployqt rejects it; the PhysX runtime
    # DLLs copied above are the whole deploy story. The MSVC /MD runtime comes from the normal
    # Visual C++ redistributable.
    Write-Host 'RobotSimulator deploy: no Qt runtime; PhysX runtime DLLs copied.'
}

# The built-in model library, which the exe reads from `packages/` beside itself. The source
# folder contains only catalogue assets; examples and test fixtures live elsewhere and therefore
# cannot accidentally appear in the Library tab.
$unexpectedExamples = @(Get-ChildItem -LiteralPath (Join-Path $RepoRoot 'examples') -File |
    Where-Object { $_.Name -ne 'README.md' })
if ($unexpectedExamples.Count -gt 0) {
    $names = ($unexpectedExamples.Name | Sort-Object) -join ', '
    throw "Loose files do not belong in examples/: $names. Use examples/stations, library/, tests/, or templates/."
}
$catalogue = Join-Path $outDir 'packages'
if (Test-Path -LiteralPath $catalogue) {
    Remove-Item -LiteralPath $catalogue -Recurse -Force
}
New-Item -ItemType Directory -Force -Path $catalogue | Out-Null
$packages = Get-BuiltinPackages
foreach ($package in $packages) {
    Copy-Item -LiteralPath $package.FullName -Destination (Join-Path $catalogue $package.Name) -Force
}
Write-Host "Built-in model library: $($packages.Count) package(s) in $catalogue"

& (Join-Path $PSScriptRoot 'check_conveyor_traces.ps1') -Exe $exeInfo.FullName

Write-Host "Robot simulator build complete: $($exeInfo.FullName)"
