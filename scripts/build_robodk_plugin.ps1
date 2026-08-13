param(
    [ValidateSet('Release', 'Debug')]
    [string]$Configuration = 'Release',
    [string]$BuildDir,
    [string]$DestDir,
    [string]$RoboDkRoot,
    # Build without installing. For a machine with no RoboDK, or to leave an installed plugin alone
    # while checking that a change still compiles.
    [switch]$NoDeploy
)

. (Join-Path $PSScriptRoot 'common.ps1')

$config = Get-DependencyConfig
Test-StandaloneDependencies $config

if (-not $BuildDir) {
    $BuildDir = Join-Path (Split-Path -Parent $RepoRoot) ('physicsAddIn-build-robodk-plugin-{0}' -f $Configuration.ToLowerInvariant())
}
if (-not $DestDir) {
    $DestDir = Join-Path $BuildDir 'plugins'
}

$plugin = Join-Path $DestDir 'PluginPhysics.dll'
$buildStart = Get-Date

New-Item -ItemType Directory -Force -Path $DestDir | Out-Null
Remove-Item -LiteralPath $plugin -Force -ErrorAction SilentlyContinue

# Generate the plugin resource catalogue with aliases matching the WASM package paths.
$qrc = Join-Path $RepoRoot 'physicsAddIn\builtin_packages.qrc'
$packages = Get-BuiltinPackages
$entries = $packages | ForEach-Object {
    '    <file alias="packages/{0}">../library/packages/{0}</file>' -f $_.Name
}
$qrcLines = @('<!DOCTYPE RCC><RCC version="1.0">', '  <qresource prefix="/">') +
            $entries +
            @('  </qresource>', '</RCC>')
$qrcText = $qrcLines -join "`n"
# Written only when it differs, so an unchanged library does not force rcc to re-run every build.
$existing = if (Test-Path -LiteralPath $qrc) { Get-Content -LiteralPath $qrc -Raw } else { '' }
if ($existing.TrimEnd() -ne $qrcText.TrimEnd()) {
    Set-Content -LiteralPath $qrc -Value $qrcText -Encoding utf8
    Write-Host "Built-in library manifest written: $($packages.Count) package(s)"
} else {
    Write-Host "Built-in library manifest unchanged: $($packages.Count) package(s)"
}

Invoke-QMakeBuild -Config $config -ProFile (Join-Path $RepoRoot 'physicsAddIn\PluginPhysics.pro') `
    -Configuration $Configuration -BuildDir $BuildDir -QMakeAssignments @("PLUGIN_DESTDIR=$DestDir")

$pluginInfo = Assert-FreshOutput -Path $plugin -Since $buildStart -What 'RoboDK plugin'
Write-Host "RoboDK plugin build complete: $($pluginInfo.FullName)"

if ($NoDeploy) {
    Write-Host 'Deployment skipped (-NoDeploy).'
    return
}

# An explicitly named root must exist; the default one merely might.
$rootWasNamed = -not [string]::IsNullOrWhiteSpace($RoboDkRoot)
if (-not $rootWasNamed) { $RoboDkRoot = $config.RoboDkRoot }

# Debug RoboDK loads from bind\plugins, which is the layout PluginPhysics.pro documents.
$binDirectory = if ($Configuration -eq 'Debug') { 'bind' } else { 'bin' }
$roboDkBin = Join-Path $RoboDkRoot $binDirectory
$installDir = Join-Path $roboDkBin 'plugins'

if (-not (Test-Path $roboDkBin -PathType Container)) {
    $message = "RoboDK was not found at $roboDkBin."
    if ($rootWasNamed) {
        throw "$message Pass a -RoboDkRoot that contains $binDirectory\, or use -NoDeploy."
    }
    Write-Warning "$message The plugin was NOT installed."
    Write-Warning "Set ROBODK_ROOT in build\local-env.pri, pass -RoboDkRoot, or use -NoDeploy to silence this."
    return
}

# A running RoboDK holds the DLL open, so the copy fails with a sharing violation that says nothing
# about the cause. Named before it happens.
$running = @(Get-Process -Name 'RoboDK' -ErrorAction SilentlyContinue)
if ($running.Count -gt 0) {
    throw "RoboDK is running (PID $($running[0].Id)) and holds PluginPhysics.dll open. Close it and re-run, or use -NoDeploy."
}

New-Item -ItemType Directory -Force -Path $installDir | Out-Null
$installed = Join-Path $installDir 'PluginPhysics.dll'
$deployStart = Get-Date
try {
    Copy-Item -LiteralPath $pluginInfo.FullName -Destination $installed -Force
} catch {
    throw "Could not install the plugin to $installed. $($_.Exception.Message)"
}

# The PhysX runtime goes beside RoboDK.exe, not beside the plugin. Windows resolves a plugin's
# imports from the host executable's directory, so a DLL dropped next to PluginPhysics.dll is never
# found and the plugin fails to load with nothing said about why. RoboDK ships no PhysX of its own,
# so nothing here is being overwritten.
$physxBin = Join-Path $config.PhysXRoot ('bin\{0}\{1}' -f $config.PhysXPlatform, $Configuration.ToLowerInvariant())
$physxRuntime = @('PhysX_64.dll', 'PhysXCommon_64.dll', 'PhysXFoundation_64.dll', 'PhysXCooking_64.dll')
foreach ($name in $physxRuntime) {
    $source = Join-Path $physxBin $name
    Assert-File $source "The PhysX runtime the plugin links against is missing."
    $target = Join-Path $roboDkBin $name
    $existing = if (Test-Path $target) { (Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash } else { '' }
    if ($existing -ne (Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash) {
        try {
            Copy-Item -LiteralPath $source -Destination $target -Force
        } catch {
            throw "Could not install $name to $roboDkBin. $($_.Exception.Message)"
        }
        Write-Host "PhysX runtime installed: $target"
    }
}

$installedInfo = Assert-FreshOutput -Path $installed -Since $deployStart -What 'RoboDK plugin install'
Write-Host "RoboDK plugin installed: $($installedInfo.FullName)"
Write-Host "Restart RoboDK to load it, or start it with: $(Join-Path $roboDkBin 'RoboDK.exe') -PLUGINSLOAD"
