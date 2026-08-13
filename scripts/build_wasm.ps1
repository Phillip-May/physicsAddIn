<#
.SYNOPSIS
    Builds the WebAssembly RobotSimulator with Emscripten.

.DESCRIPTION
    Calls em++ directly rather than going through qmake and Qt's wasm-emscripten mkspec.

    That mkspec was the only reason this build ever needed Qt, and what it contributes is now
    actively wrong for this target: it injects -s USE_PTHREADS=1 -s PTHREAD_POOL_SIZE=4, which
    needs COOP/COEP headers on every page serving the build or the module will not start at all;
    -s TOTAL_MEMORY=1GB alongside our own ALLOW_MEMORY_GROWTH, which is a contradiction; and
    -s EXTRA_EXPORTED_RUNTIME_METHODS, renamed out of Emscripten years ago. It also pins
    Emscripten 1.39.8, which is what Qt 5.15 was built against.

    The source list here mirrors RobotSimulator.pro plus build/imgui.pri and build/thirdparty.pri.
    Keep them in step: this file is the wasm build and the .pro is the desktop one.

.EXAMPLE
    .\scripts\build_wasm.ps1
    .\scripts\build_wasm.ps1 -BuildDir C:\path\to\wasm-build
#>
[CmdletBinding()]
param(
    [string]$BuildDir,
    [string]$EmsdkRoot,
    [string]$PhysXRoot,
    [string]$PhysXBuildDir,
    # Compile only what has changed. Off by default: the object files carry the flag set they were
    # built with, and only their own .cpp mtime is compared - neither a flag change nor a header
    # edit triggers a rebuild, so both need a clean build to mean anything.
    [switch]$Incremental
)

. (Join-Path $PSScriptRoot 'common.ps1')

$config = Get-DependencyConfig
if (-not $EmsdkRoot) { $EmsdkRoot = $config.EmsdkRoot }
if (-not $PhysXRoot) { $PhysXRoot = $config.PhysXRoot }
if (-not $BuildDir) {
    $BuildDir = Join-Path (Split-Path -Parent $RepoRoot) 'physicsAddIn-build-robot-simulator-wasm'
}
if (-not $PhysXBuildDir) {
    $PhysXBuildDir = Join-Path (Split-Path -Parent $RepoRoot) 'physicsAddIn-build-physx-wasm'
}

$toolchain = Initialize-Emscripten -EmsdkRoot $EmsdkRoot
$emscriptenDir = $toolchain.EmscriptenDir
$emcc = Get-EmscriptenTool -EmscriptenDir $emscriptenDir -Name 'emcc'
$empp = Get-EmscriptenTool -EmscriptenDir $emscriptenDir -Name 'em++'

Write-Host "Emscripten: $emscriptenDir"

# Build the same PhysX 5.9 CPU solver used on desktop for Emscripten before compiling the app.
# Ninja makes this a cheap no-op when the SDK and flags have not changed.
& (Join-Path $PSScriptRoot 'build_physx_wasm.ps1') -BuildDir $PhysXBuildDir -PhysXRoot $PhysXRoot -EmsdkRoot $EmsdkRoot
$physxLibraries = Get-PhysXWasmLibraries -BuildDir $PhysXBuildDir

# ---------------------------------------------------------------------------------------------
# What gets compiled. Mirrors RobotSimulator.pro's SOURCES, build/imgui.pri and
# build/thirdparty.pri. GLFW comes from Emscripten's own port, so external/glfw is desktop-only.
$imguiRoot = Join-Path $RepoRoot 'external\imgui'
$implotRoot = Join-Path $RepoRoot 'external\implot'
foreach ($needed in @("$imguiRoot\imgui.cpp", "$imguiRoot\backends\imgui_impl_glfw.cpp", "$implotRoot\implot.cpp")) {
    if (-not (Test-Path $needed)) { throw "Missing dependency: $needed. See build/imgui.pri for the clone commands." }
}

$cppSourcesRel = @(
    'RobotSimulator\main.cpp'
    'RobotSimulator\AccessoryBuilders.cpp'
    'RobotSimulator\AppState.cpp'
    'RobotSimulator\ConveyorRuntime.cpp'
    'RobotSimulator\SceneConveyorHost.cpp'
    'RobotSimulator\FirmwareProgram.cpp'
    'RobotSimulator\MasteringIo.cpp'
    'RobotSimulator\UiFormHelpers.cpp'
    'RobotSimulator\CliHardwareCommands.cpp'
    'RobotSimulator\CliInspectCommands.cpp'
    'RobotSimulator\CliProgramCommands.cpp'
    'RobotSimulator\CliStationCommands.cpp'
    'RobotSimulator\ProgramTextIo.cpp'
    'RobotSimulator\StationParameterLinks.cpp'
    'RobotSimulator\StationSceneLoad.cpp'
    'RobotSimulator\LibraryPlacement.cpp'
    'RobotSimulator\ViewerBridge.cpp'
    'RobotSimulator\TrajectoryAndLiveRun.cpp'
    'RobotSimulator\ProgramEditing.cpp'
    'RobotSimulator\ProgramWidgets.cpp'
    'RobotSimulator\HardwareIoPanel.cpp'
    'RobotSimulator\ProgramPanel.cpp'
    'RobotSimulator\SimulationInfoPanel.cpp'
    'RobotSimulator\TimelinePanel.cpp'
    'RobotSimulator\RobotPanel.cpp'
    'RobotSimulator\PlacementWindows.cpp'
    'RobotSimulator\StationPanels.cpp'
    'RobotSimulator\ContextBar.cpp'
    'RobotSimulator\AppFrame.cpp'
    'RobotSimulator\ImGuiApp.cpp'
    'RobotSimulator\GlLoader.cpp'
    'RobotSimulator\HardwareIo.cpp'
    'RobotSimulator\ConveyorPhysics.cpp'
    'RobotSimulator\DragChainPhysics.cpp'
    'RobotSimulator\LiveRunDriver.cpp'
    'RobotSimulator\MeshRobotViewer.cpp'
    'RobotSimulator\RobotLibraryPanel.cpp'
    'RobotSimulator\RobotProgramModel.cpp'
    'RobotSimulator\RobotProgramSimulator.cpp'
    'RobotSimulator\WebFiles.cpp'
    'Common\AccessoryGeometry.cpp'
    'Common\AccessoryPropertySchema.cpp'
    'Common\CadNodeDraw.cpp'
    'Common\CadNodePackage.cpp'
    'Common\ConveyorCore.cpp'
    'Common\ConveyorGeometry.cpp'
    'Common\LibraryCatalogue.cpp'
    'Common\MountingSnap.cpp'
    'Common\PlacedMechanismSchema.cpp'
    'Common\PlacementSession.cpp'
    'Common\RobotRuntime.cpp'
    'Common\SerialPort.cpp'
    'Common\StationPackage.cpp'
    'Common\ViewRay.cpp'
    'external\imgui\imgui.cpp'
    'external\imgui\imgui_draw.cpp'
    'external\imgui\imgui_tables.cpp'
    'external\imgui\imgui_widgets.cpp'
    'external\imgui\backends\imgui_impl_glfw.cpp'
    'external\imgui\backends\imgui_impl_opengl3.cpp'
    'external\implot\implot.cpp'
    'external\implot\implot_items.cpp'
)

# Keep first-party translation units aligned with RobotSimulator.pro.
$proFile = Join-Path $RepoRoot 'RobotSimulator\RobotSimulator.pro'
$proText = Get-Content $proFile -Raw
if ($proText -notmatch '(?ms)^SOURCES\s*\+=\s*(.+?)(?:\r?\n\s*\r?\n|\z)') {
    throw "Could not parse SOURCES from $proFile"
}
$proSources = [regex]::Matches($matches[1], '[\w./\\-]+\.cpp') | ForEach-Object {
    $rel = $_.Value -replace '/', '\' -replace '^\.\.\\', ''
    if ($rel -notmatch '\\') { $rel = 'RobotSimulator\' + $rel }
    $rel
}
$firstParty = @($cppSourcesRel | Where-Object { $_ -notmatch '^(external|third_party)\\' })
$missingHere = @($proSources | Where-Object { $firstParty -notcontains $_ })
$extraHere = @($firstParty | Where-Object { $proSources -notcontains $_ })
if ($missingHere.Count -gt 0 -or $extraHere.Count -gt 0) {
    throw "build_wasm.ps1 source list drifted from RobotSimulator.pro SOURCES. Missing here: $($missingHere -join ', '). Extra here: $($extraHere -join ', ')."
}

$cppSources = $cppSourcesRel | ForEach-Object { Join-Path $RepoRoot $_ }

# miniz is C, and clang++ would compile a .c as C++. Built with the C driver instead.
$cSources = @('third_party\miniz\miniz.c') | ForEach-Object { Join-Path $RepoRoot $_ }

$includes = @(
    'RobotSimulator'
    'Common'
    'external\imgui'
    'external\imgui\backends'
    'external\implot'
    'third_party'
    'third_party\miniz'
) | ForEach-Object { '-I' + (Join-Path $RepoRoot $_) }
# Public PhysX headers are self-contained. Internal source directories remain private to its CMake
# build, preventing the simulator from depending on implementation headers.
$includes += '-I' + (Join-Path $PhysXRoot 'include')

# ImGui and the application must agree on the 32-bit ImDrawIdx configuration.
$defines = @(
    '-DROBOTSIM_NO_SERIAL'
    # Angle brackets survive clang response-file parsing.
    '-DIMGUI_USER_CONFIG=<imgui_user_config.h>'
    '-DIMGUI_IMPL_OPENGL_ES3'
    '-DMINIZ_NO_ZLIB_COMPATIBLE_NAMES'
    '-DROBOTSIM_WITH_PHYSX'
    '-DPX_PHYSX_STATIC_LIB'
)

# USE_GLFW is a link-time setting - Emscripten ships the GLFW headers unconditionally, and passing
# it here only earns an ignored-setting warning on every translation unit.
$compileFlags = @('-O2', '-Wall', '-msimd128', '-msse2')

# WEBGL2/FULL_ES3 supports the #version 300 es shader. ASYNCIFY lets the single-threaded live-run
# driver yield between slices. Pthreads are omitted so the output works without COOP/COEP headers.
$linkFlags = @(
    '-O2'
    '-sUSE_GLFW=3'
    '-sUSE_WEBGL2=1'
    '-sFULL_ES3=1'
    '-sASYNCIFY=1'
    '-sASYNCIFY_STACK_SIZE=131072'
    '-sALLOW_MEMORY_GROWTH=1'
    '-msimd128'
    '-msse2'
    '--js-library', (Join-Path $RepoRoot 'RobotSimulator\web\web_serial.js')
)

# Preload the authoritative package catalogue at the path used by builtin:<id> references.
$preload = @(Get-BuiltinPackages | ForEach-Object {
    "$($_.FullName)@/packages/$($_.Name)"
})
$preload += @(
    "$RepoRoot\examples\stations\fairino_gudel_machine_tending.station.json@/stations/fairino_gudel_machine_tending.station.json"
    "$imguiRoot\misc\fonts\DroidSans.ttf@/fonts/DroidSans.ttf"
)
foreach ($entry in $preload) { $linkFlags += @('--preload-file', $entry) }

# ---------------------------------------------------------------------------------------------
if (-not $Incremental) { Remove-Item -Recurse -Force $BuildDir -ErrorAction SilentlyContinue }
$objDir = Join-Path $BuildDir 'obj'
New-Item -ItemType Directory -Force -Path $objDir | Out-Null

# Response files, because these command lines carry quoted defines and Windows paths and passing
# them through two layers of shell quoting is how a define silently becomes empty.
function Invoke-Em {
    param([string]$Tool, [string[]]$Arguments, [string]$What)
    $rsp = Join-Path $BuildDir 'args.rsp'
    # Forward slashes, because a clang response file treats a backslash as an escape - so
    # C:\Users\... arrives as C:Users... and the compiler reports a missing input file. Every
    # argument here is a flag or a path, and clang takes forward slashes on Windows.
    $escaped = @($Arguments | ForEach-Object { $_ -replace '\\', '/' })
    [System.IO.File]::WriteAllLines($rsp, $escaped, (New-Object System.Text.UTF8Encoding($false)))
    Invoke-NativeCommand -Tool $Tool -Arguments @("@$rsp") -What $What
}

$objects = @()
$compiled = 0
foreach ($source in ($cppSources + $cSources)) {
    # Object names mirror the source's repo-relative path, so two same-named sources in different
    # directories cannot silently overwrite each other's object.
    $relative = if ($source.StartsWith($RepoRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
        $source.Substring($RepoRoot.Length).TrimStart('\')
    } else {
        Split-Path -Leaf $source
    }
    $objectName = ($relative -replace '[\\/]', '_') -replace '\.(cpp|c)$', ''
    $object = Join-Path $objDir ($objectName + '.o')
    $objects += $object
    if ($Incremental -and (Test-Path $object) -and
        (Get-Item $object).LastWriteTime -gt (Get-Item $source).LastWriteTime) {
        continue
    }
    $tool = if ($source.EndsWith('.c')) { $emcc } else { $empp }
    $std = if ($source.EndsWith('.c')) { @() } else { @('-std=c++17') }
    Write-Host ("compiling {0}" -f (Split-Path -Leaf $source))
    # @() around the first operand deliberately. PowerShell unwraps a single-element array out of an
    # if-expression, so $std above is a bare string for a .cpp - and `string + array` is string
    # concatenation, which silently collapsed every argument onto one line and produced things like
    # -Wall-DROBOTSIM_NO_SERIAL.
    $arguments = @($std) + $compileFlags + $defines + $includes + @('-c', $source, '-o', $object)
    Invoke-Em -Tool $tool -Arguments $arguments -What ("Compile " + (Split-Path -Leaf $source))
    $compiled++
}
Write-Host "compiled $compiled of $($objects.Count) translation unit(s)"

$outJs = Join-Path $BuildDir 'RobotSimulator.js'
Write-Host "linking..."
$physxLinkGroup = @('-Wl,--start-group') + $physxLibraries + @('-Wl,--end-group')
Invoke-Em -Tool $empp -Arguments (@($objects) + $physxLinkGroup + $linkFlags + @('-o', $outJs)) -What 'Link'

# The GLFW build supplies its own HTML entry point.
Copy-Item (Join-Path $RepoRoot 'RobotSimulator\web\index.html') $BuildDir -Force

Write-Host ""
Write-Host "WebAssembly build complete: $BuildDir" -ForegroundColor Green
foreach ($name in @('RobotSimulator.wasm', 'RobotSimulator.js', 'RobotSimulator.data', 'index.html')) {
    $file = Join-Path $BuildDir $name
    if (Test-Path $file) {
        $bytes = (Get-Item $file).Length
        Write-Host ("  {0,-22} {1,12:N0} bytes  ({2,7:N2} MiB)" -f $name, $bytes, ($bytes / 1MB))
    }
}
