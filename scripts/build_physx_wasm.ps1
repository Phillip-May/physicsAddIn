<#
.SYNOPSIS
    Builds the CPU-only PhysX static libraries used by RobotSimulator WebAssembly.

.DESCRIPTION
    PhysX 5.9 has no Emscripten preset, so this rides the linux-family CMake path (whose
    platform naming calls the target UNKNOWN) as a static, CPU-only build, after applying the
    small compatibility patch maintained in scripts/patches. Requires a full PhysX git clone:
    the trimmed deps cache carries only headers and prebuilt Windows binaries.
#>
[CmdletBinding()]
param(
    [string]$BuildDir,
    [string]$PhysXRoot,
    [string]$EmsdkRoot,
    [switch]$Clean
)

. (Join-Path $PSScriptRoot 'common.ps1')

$config = Get-DependencyConfig
if (-not $PhysXRoot) { $PhysXRoot = $config.PhysXRoot }
if (-not $EmsdkRoot) { $EmsdkRoot = $config.EmsdkRoot }
if (-not $BuildDir) {
    $BuildDir = Join-Path (Split-Path -Parent $RepoRoot) 'physicsAddIn-build-physx-wasm'
}
# Combine against $PWD, not [IO.Path]::GetFullPath alone: the .NET process cwd is not
# PowerShell's $PWD, so a relative -BuildDir would land somewhere unexpected.
$PhysXRoot = [System.IO.Path]::GetFullPath([System.IO.Path]::Combine($PWD.Path, $PhysXRoot))
$BuildDir = [System.IO.Path]::GetFullPath([System.IO.Path]::Combine($PWD.Path, $BuildDir))
$physxRepoRoot = Split-Path -Parent $PhysXRoot
$patchPath = Join-Path $RepoRoot 'scripts\patches\physx-5.9-wasm.patch'
$outputLibRoot = Join-Path $BuildDir 'lib'
$binDir = Join-Path $BuildDir 'bin'

Assert-File (Join-Path $PhysXRoot 'include\PxPhysicsAPI.h') 'PhysX source tree is unavailable.'
Assert-File $patchPath 'PhysX WASM compatibility patch is unavailable.'
if (-not (Test-Path (Join-Path $physxRepoRoot '.git')) -or
    -not (Test-Path (Join-Path $PhysXRoot 'source\physx\src'))) {
    throw "PhysX at $physxRepoRoot is not a full source clone. The WASM build compiles PhysX itself and needs the complete PhysX 5.9 git checkout, not the trimmed deps cache restored by bootstrap_deps.ps1."
}

$toolchain = Initialize-Emscripten -EmsdkRoot $EmsdkRoot
$emscriptenDir = $toolchain.EmscriptenDir
$emcmake = Get-EmscriptenTool -EmscriptenDir $emscriptenDir -Name 'emcmake'
$empp = Get-EmscriptenTool -EmscriptenDir $emscriptenDir -Name 'em++'

$nativeCmake = 'C:\Program Files\CMake\bin\cmake.exe'
$cmakeCommand = Get-Command cmake.exe -ErrorAction SilentlyContinue
$cmake = if (Test-Path $nativeCmake) { $nativeCmake } elseif ($cmakeCommand) { $cmakeCommand.Source } else { $nativeCmake }
$ninjaCommand = Get-Command ninja.exe -ErrorAction SilentlyContinue
$ninja = if ($ninjaCommand) {
    $ninjaCommand.Source
} else {
    'C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe'
}
Assert-File $cmake 'CMake is unavailable.'
Assert-File $ninja 'Ninja is unavailable.'
# emcmake launches a command named `cmake`; keep MSYS cmake from winning on machines where its
# bin directory precedes the native CMake install, since it rewrites C:/... as an MSYS-relative
# path and configures the wrong source directory.
$env:PATH = (Split-Path -Parent $cmake) + ';' + (Split-Path -Parent $ninja) + ';' + $env:PATH

# The patch is deliberately idempotent: apply it when pristine, accept it when already applied,
# and reject any third state so a different PhysX revision cannot be silently modified. A failed
# forward check is the expected result when the patch is already present.
$forwardPatchStatus = Invoke-NativeCommand -Tool 'git' -Arguments @('-C', $physxRepoRoot, 'apply', '--check', $patchPath) -What 'Patch probe' -IgnoreExitCode
if ($forwardPatchStatus -eq 0) {
    Invoke-NativeCommand -Tool 'git' -Arguments @('-C', $physxRepoRoot, 'apply', $patchPath) -What 'Applying the PhysX WASM compatibility patch'
    Write-Host 'Applied PhysX WASM compatibility patch.'
} else {
    $reversePatchStatus = Invoke-NativeCommand -Tool 'git' -Arguments @('-C', $physxRepoRoot, 'apply', '--reverse', '--check', $patchPath) -What 'Patch probe' -IgnoreExitCode
    if ($reversePatchStatus -ne 0) {
        throw 'PhysX sources match neither the pristine nor patched 5.9 source expected by this project.'
    }
    Write-Host 'PhysX WASM compatibility patch is already applied.'
}

if ($Clean -and (Test-Path $BuildDir)) {
    $repoPrefix = $RepoRoot.TrimEnd('\') + '\'
    if ($BuildDir.TrimEnd('\') -ieq $RepoRoot.TrimEnd('\') -or
        $BuildDir.StartsWith($repoPrefix, [System.StringComparison]::OrdinalIgnoreCase) -or
        -not (Test-Path (Join-Path $BuildDir 'CMakeCache.txt'))) {
        throw "Refusing to clean ${BuildDir}: expected a PhysX WASM build directory (has CMakeCache.txt) outside the repo."
    }
    Remove-Item -LiteralPath $BuildDir -Recurse -Force
}
New-Item -ItemType Directory -Force -Path $BuildDir, $outputLibRoot, $binDir | Out-Null

$commonFlags = '-Wno-missing-include-dirs -Wno-unsupported-floating-point-opt -msimd128 -msse2'
$configureArgs = @(
    'cmake', '-S', ($PhysXRoot -replace '\\', '/'), '-B', ($BuildDir -replace '\\', '/'), '-G', 'Ninja',
    '-DCMAKE_BUILD_TYPE=release',
    "-DCMAKE_MAKE_PROGRAM=$($ninja -replace '\\', '/')",
    "-DCMAKE_CXX_FLAGS=$commonFlags",
    # PhysX re-SETs PHYSX_CXX_FLAGS every configure, so only the _RELEASE override actually
    # defeats its -Werror.
    '-DPHYSX_CXX_FLAGS_RELEASE=-O3 -Wno-error',
    '-DPX_GENERATE_GPU_PROJECTS=OFF',
    '-DPX_GENERATE_STATIC_LIBRARIES=ON',
    '-DPX_BUILDSNIPPETS=OFF',
    '-DPX_BUILDPVDRUNTIME=OFF',
    "-DPX_OUTPUT_LIB_DIR=$($outputLibRoot -replace '\\', '/')",
    "-DPX_OUTPUT_BIN_DIR=$($binDir -replace '\\', '/')"
)
Invoke-NativeCommand -Tool $emcmake -Arguments $configureArgs -What 'PhysX CMake configure'

$jobs = [Math]::Max(1, [Environment]::ProcessorCount)
Invoke-NativeCommand -Tool $cmake -Arguments @('--build', $BuildDir, '--target',
    'PhysXFoundation', 'PhysXPvdSDK', 'PhysX', 'PhysXCommon', 'PhysXCooking', 'PhysXExtensions',
    '--', '-j', $jobs) -What 'PhysX WASM build'

$archives = Get-PhysXWasmLibraries -BuildDir $BuildDir
$libDir = Split-Path -Parent $archives[0]

# Exercise the produced WebAssembly, not merely the archive table: create a zero-worker CPU scene,
# drop a dynamic box on a static floor, simulate three seconds, and assert its resting height.
if (-not $toolchain.NodeExe -or -not (Test-Path $toolchain.NodeExe -PathType Leaf)) {
    throw "Emsdk Node runtime is unavailable under $EmsdkRoot\node."
}
$smokeSource = Join-Path $RepoRoot 'scripts\tests\physx_wasm_smoke.cpp'
$smokeJs = Join-Path $BuildDir 'physx_wasm_smoke.js'
$smokeWasm = Join-Path $BuildDir 'physx_wasm_smoke.wasm'
Assert-File $smokeSource 'PhysX WASM smoke test source is unavailable.'
$newestInput = @($smokeSource) + $archives |
    ForEach-Object { (Get-Item $_).LastWriteTimeUtc } |
    Sort-Object -Descending |
    Select-Object -First 1
if (-not (Test-Path $smokeWasm) -or (Get-Item $smokeWasm).LastWriteTimeUtc -lt $newestInput) {
    $smokeArguments = @(
        '-std=c++17', '-O2', '-msimd128', '-msse2',
        '-DROBOTSIM_WITH_PHYSX', '-DPX_PHYSX_STATIC_LIB',
        "-I$(Join-Path $PhysXRoot 'include')", $smokeSource,
        '-Wl,--start-group'
    ) + $archives + @('-Wl,--end-group', '-sALLOW_MEMORY_GROWTH=1', '-o', $smokeJs)
    # em++ takes forward slashes on Windows; backslashed paths have already broken this once.
    $smokeArguments = @($smokeArguments | ForEach-Object { $_ -replace '\\', '/' })
    Invoke-NativeCommand -Tool $empp -Arguments $smokeArguments -What 'PhysX WASM smoke test link'
}
Invoke-NativeCommand -Tool $toolchain.NodeExe -Arguments @($smokeJs) -What 'PhysX WASM collision smoke test'

Write-Host "PhysX WASM CPU libraries complete: $libDir" -ForegroundColor Green
