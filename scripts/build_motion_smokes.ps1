<#
.SYNOPSIS
    Builds every tools/ smoke and equivalence check from source and runs them.

.DESCRIPTION
    Builds fresh binaries so checks cannot run against stale executables. Requires only MSVC.

.EXAMPLE
    .\scripts\build_motion_smokes.ps1
    .\scripts\build_motion_smokes.ps1 -NoRun
#>
param(
    # Compile only. For checking that the sources still build against a changed header without
    # waiting for the runs.
    [switch]$NoRun
)

. (Join-Path $PSScriptRoot 'common.ps1')

$toolsDir = Join-Path $RepoRoot 'tools'
$qtRoot = (Get-DependencyConfig).QtRoot
# The equivalence checkers compile and link real Qt because their whole job is to prove the
# Qt-free replacements reproduce Qt's behaviour; the smokes need only MSVC.
$smokes = @(
    @{ Name = 'motion_core_weave_smoke' }
    @{ Name = 'motion_core_endpoint_config_smoke' }
    @{ Name = 'motion_core_window_runner_smoke' }
    @{ Name = 'conveyor_core_smoke' }
    @{ Name = 'conveyor_scenery_smoke' }
    @{ Name = 'view_ray_smoke' }
    @{ Name = 'mounting_snap_smoke' }
    @{ Name = 'library_catalogue_smoke'
       Extra = @('Common\LibraryCatalogue.cpp', 'Common\CadNodePackage.cpp',
                 'Common\RobotRuntime.cpp', 'Common\MountingSnap.cpp',
                 'third_party\miniz\miniz.c')
       Flags = '/DMINIZ_NO_ZLIB_COMPATIBLE_NAMES /I "third_party\miniz"'
       RunArgs = @('library\packages') }
    @{ Name = 'placed_mechanism_schema_smoke'
       Extra = @('Common\PlacedMechanismSchema.cpp', 'Common\LibraryCatalogue.cpp',
                 'Common\CadNodePackage.cpp', 'Common\RobotRuntime.cpp',
                 'third_party\miniz\miniz.c')
       Flags = '/DMINIZ_NO_ZLIB_COMPATIBLE_NAMES /I "third_party\miniz"'
       RunArgs = @('library\packages') }
    @{ Name = 'orientation_format_check' }
    @{ Name = 'wide_to_utf8_check' }
    @{ Name = 'json_compat_equivalence'; Qt = @('Core') }
    @{ Name = 'qt_number_format_equivalence'; Qt = @('Core') }
    @{ Name = 'string_util_equivalence'; Qt = @('Core') }
    @{ Name = 'scene_math_equivalence'; Qt = @('Core', 'Gui') }
)
$needsQt = @($smokes | Where-Object { $_.ContainsKey('Qt') })
if ($needsQt.Count -gt 0) {
    Assert-File (Join-Path $qtRoot 'lib\Qt5Core.lib') 'Qt 5.15.2 MSVC2019 is required by the Qt equivalence checkers.'
}

# orientation_format_check reaches third_party for CadNode.h's JSON type and
# scene_math_equivalence includes RobotSimulator/SceneMath.h. One base include list for all.
$includes = '/I "Common" /I "RobotSimulator" /I "third_party" /I "physicsAddIn"'

foreach ($smoke in $smokes) {
    Assert-File (Join-Path $toolsDir "$($smoke.Name).cpp") "Smoke source is missing."
}

# Objects and the compiler's intermediate files land beside the exes; .gitignore covers
# tools/*.obj and tools/*.pdb alongside the exes it already ignored. /Fe, /Fo and /Fd are set
# explicitly so two smokes cannot collide on the default vc140.pdb.
$commands = @()
foreach ($smoke in $smokes) {
    $extraIncludes = ''
    $linkArgs = ''
    if ($smoke.ContainsKey('Qt')) {
        $extraIncludes = ' /I "{0}\include"' -f $qtRoot
        foreach ($module in $smoke.Qt) { $extraIncludes += ' /I "{0}\include\Qt{1}"' -f $qtRoot, $module }
        $linkArgs = ' /link /LIBPATH:"{0}\lib"' -f $qtRoot
        foreach ($module in $smoke.Qt) { $linkArgs += ' Qt5{0}.lib' -f $module }
    }
    if ($smoke.ContainsKey('Flags')) { $extraIncludes += ' ' + $smoke.Flags }
    $extraSources = ''
    $objectPath = 'tools\{0}.obj' -f $smoke.Name
    if ($smoke.ContainsKey('Extra')) {
        foreach ($source in $smoke.Extra) { $extraSources += ' "{0}"' -f $source }
        $objectDir = Join-Path $toolsDir ('{0}.objs' -f $smoke.Name)
        New-Item -ItemType Directory -Force -Path $objectDir | Out-Null
        # Forward slash: a trailing backslash inside the quotes would escape the closing quote.
        $objectPath = 'tools/{0}.objs/' -f $smoke.Name
    }
    $commands += ('cl /nologo /std:c++17 /EHsc /O2 /W3 {1}{2} "tools\{0}.cpp"{5} /Fe"tools\{0}.exe" /Fo"{4}" /Fd"tools\{0}.pdb"{3}' -f $smoke.Name, $includes, $extraIncludes, $linkArgs, $objectPath, $extraSources)
    $commands += 'if errorlevel 1 exit /b %errorlevel%'
}

Write-Host "Building $($smokes.Count) smoke(s) from source..."
Invoke-VsCmd -Commands $commands | Out-Null
Write-Host "Build complete."

if ($NoRun) { return }

# The Qt-linked checkers load Qt5Core.dll/Qt5Gui.dll at startup.
if ($needsQt.Count -gt 0) { $env:PATH = (Join-Path $qtRoot 'bin') + ';' + $env:PATH }

$failed = @()
foreach ($entry in $smokes) {
    $smoke = $entry.Name
    $exe = Join-Path $toolsDir "$smoke.exe"
    Write-Host ""
    Write-Host "=== $smoke ==="
    if ($entry.ContainsKey('RunArgs')) {
        & $exe @($entry.RunArgs | ForEach-Object { Join-Path $RepoRoot $_ })
    } else {
        & $exe
    }
    if ($LASTEXITCODE -ne 0) {
        $failed += $smoke
        Write-Host "$smoke FAILED (exit $LASTEXITCODE)"
    }
}

Write-Host ""
if ($failed.Count -gt 0) {
    throw ("Smoke failures: {0}" -f ($failed -join ', '))
}
Write-Host "PASS: all $($smokes.Count) smokes built from source and passed."
