# Building physicsAddIn

Run the commands below from the repository root in Windows PowerShell. If local execution policy
blocks a script, invoke it explicitly:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\build_robot_simulator.ps1 -Configuration Release
```

Four targets, two dependency configurations:

- standalone viewer: `QtCadViewer/QtCadViewer.pro` — `build/deps.pri` (Qt, OpenCascade, PhysX, CoACD)
- RoboDK plugin: `physicsAddIn/PluginPhysics.pro` — Qt Widgets, `build/physx-runtime.pri`,
  `build/thirdparty.pri`; no OpenCascade, so every import resolves from RoboDK's own directory
- robot simulator: `RobotSimulator/RobotSimulator.pro` — no Qt or OCCT; uses `build/imgui.pri`,
  `build/thirdparty.pri`, `build/physx-runtime.pri`
- robot simulator, WebAssembly: `scripts/build_wasm.ps1` — see [WebAssembly build](#webassembly-build)

## Known-good toolchain

- Visual Studio: VS 2019 16.11 / MSVC v142, x64
- Qt: 5.15.2 `msvc2019_64`
- OpenCascade: `V7_6_0`
- PhysX: `110.1-omni-and-physx-5.9.0`, CPU, `/MD`
- CoACD: `b678aa0802996fa03e1ec0e68bd05acf8cd20cf9`, `/MD`
- CMake: Kitware CMake, current release tested

VS 2022 can usually consume v142-compatible binaries, but the scripts prefer VS 2019 so rebuilds match the dependency artifacts.

## Dependency configuration

Shared dependency paths live in:

```text
build/deps.pri
```

Machine-specific overrides belong in:

```text
build/local-env.pri
```

That file is ignored by git. Start from:

```powershell
Copy-Item build/local-env.pri.example build/local-env.pri
```

## RobotSimulator UI dependencies

RobotSimulator's UI dependencies live under `external/` and are not committed.
`bootstrap_deps.ps1` does not fetch them; on a fresh clone, before building the simulator:

```powershell
git clone --depth 1 --branch docking https://github.com/ocornut/imgui.git external/imgui
git clone --depth 1 https://github.com/epezent/implot.git external/implot
```

The docking branch of ImGui is required — docking is not in master. GLFW is the prebuilt
Windows binary package: unzip GLFW 3.4 into `external/glfw` (desktop only; the web build gets
GLFW from Emscripten). `build/imgui.pri` fails with a specific message for whichever of the
three is missing.

## Fast path: cached dependencies

The expected cache file is:

```text
deps-cache/physicsAddIn-deps-msvc2019-x64-physx-5.9.0.zip
```

Restore/install dependencies:

```powershell
.\scripts\bootstrap_deps.ps1
```

Build the standalone app:

```powershell
.\scripts\build_standalone.ps1 -Configuration Release -Deploy
```

The default Release executable is written to
`..\physicsAddIn-build-standalone-release\bin\release\QtCadViewer.exe`. `-Deploy` copies the
runtime DLLs beside it.

Build the robot simulator (see [RobotSimulator UI dependencies](#robotsimulator-ui-dependencies)
first on a fresh clone):

```powershell
.\scripts\build_robot_simulator.ps1 -Configuration Release
```

The native simulator has one canonical output path:

```text
dist\RobotSimulator\release\RobotSimulator.exe
```

The build script does not accept an alternate build directory, and `RobotSimulator.pro` enforces
the same destination even when qmake is invoked directly. Shadow directories contain intermediates
only; do not launch executables from them.

Build the RoboDK plugin. It is installed into RoboDK as part of the build, so the plugin that
loads is always the one just compiled:

```powershell
.\scripts\build_robodk_plugin.ps1 -Configuration Release
```

The build writes to its own directory and copies from there into `<ROBODK_ROOT>\bin\plugins`
(`bind\plugins` for `-Configuration Debug`), so a failed build leaves the installed plugin alone.
The default Release build output is
`..\physicsAddIn-build-robodk-plugin-release\plugins\PluginPhysics.dll`.
The four PhysX runtime DLLs are installed alongside `RoboDK.exe` in `<ROBODK_ROOT>\bin` — not
beside the plugin, because Windows resolves a plugin's imports from the host executable's
directory. A plugin whose imports cannot be resolved fails to load with nothing said about why.
`ROBODK_ROOT` defaults to `C:\RoboDK` and can be set in `build\local-env.pri` or passed as
`-RoboDkRoot`. A machine with no RoboDK still builds, with a warning that nothing was installed;
naming `-RoboDkRoot` explicitly turns that into an error instead. RoboDK holds the DLL open while
it runs, so close it first or pass `-NoDeploy`:

```powershell
.\scripts\build_robodk_plugin.ps1 -Configuration Release -NoDeploy
```

Restart RoboDK to pick up a new build, or start it with `RoboDK.exe -PLUGINSLOAD`.

Validate or bake robot packages:

```powershell
RobotSimulator.exe --validate-package library\packages\ar4_6dof_robot.zip
RobotSimulator.exe --bake-hulls input.zip output.zip
```

## Create a dependency cache

On a machine where OCCT, PhysX, and CoACD are already built and validated:

```powershell
.\scripts\package_deps_cache.ps1 -Force
```

The generated zip can be stored in `deps-cache/`, a release artifact, or an internal file share.

## Source fallback

If no cache is available:

```powershell
.\scripts\bootstrap_deps.ps1 -BuildFromSource
```

This installs/verifies Qt and CMake, then builds missing OCCT, PhysX, and CoACD dependencies from pinned versions. The cache path is still preferred because OCCT/PhysX/CoACD source builds are the expensive part.

## WebAssembly build

The web target does not go through qmake: Qt 5.15's `wasm-emscripten` mkspec pins
Emscripten 1.39 and injects pthread/memory flags that are wrong for this build, so
`RobotSimulator.pro` deliberately fails with `error()` under `wasm`. The supported path is:

```powershell
.\scripts\build_wasm.ps1
.\scripts\package_wasm.ps1 -BuildDir ..\physicsAddIn-build-robot-simulator-wasm
```

Prerequisites: emsdk installed at `%USERPROFILE%\emsdk` (or pass `-EmsdkRoot` / set `EMSDK_ROOT`
in `build/local-env.pri`), and a **full PhysX 5.9 git clone** whose `physx` directory is
`PHYSX_ROOT`. The parent of `PHYSX_ROOT` must contain the clone's `.git` directory. The trimmed
deps cache restored by `bootstrap_deps.ps1` carries only headers and prebuilt Windows binaries,
which cannot be compiled for Emscripten. The default application output is
`..\physicsAddIn-build-robot-simulator-wasm`. `build_wasm.ps1` first runs
`build_physx_wasm.ps1`, which
builds CPU-only static PhysX libraries for Emscripten — a no-op when already built — then
compiles the application and preloads every package in `library/packages` plus the default
station. `scripts/patches/physx-5.9-wasm.patch` is applied to the PhysX checkout **in place**
(idempotently, via `git apply`); re-running `bootstrap_deps.ps1` copies the cache over that
checkout and silently reverts the patch, so re-run the wasm build afterwards.
`package_wasm.ps1` collects the output into `dist\RobotSimulator-wasm.zip`, flat so the unzipped
folder serves directly.

## Verifying a change

Two local regression workflows cover the portable core and deterministic simulator output:

```powershell
.\scripts\build_motion_smokes.ps1
```

compiles and runs all 15 checks in `tools/` from source against the current headers. They cover the
motion core, conveyors, scenery, placement, package/catalogue behavior, formatting, encoding, and
Qt-equivalence boundaries. The equivalence checks require the configured Qt installation; the
other checks require only MSVC. Always build these from source: a stale committed binary once let a
smoke pass for months without compiling.

```powershell
.\scripts\regression_capture.ps1 -Exe dist\RobotSimulator\release\RobotSimulator.exe -OutDir baseline
# ...make changes, rebuild...
.\scripts\regression_capture.ps1 -Exe dist\RobotSimulator\release\RobotSimulator.exe -OutDir after
.\scripts\regression_capture.ps1 -Compare baseline, after
```

captures deterministic simulation, dump, IK, validation, generation, and trajectory outputs and
compares them byte-for-byte.

## Optional features

CPU PhysX is the default. GPU PhysX is opt-in:

```qmake
CONFIG += physx_gpu
```

GPU mode requires CUDA/nvcc and PhysX GPU libraries. Without that, use the default CPU path.

Optional OCCT groups are also opt-in:

```qmake
CONFIG += occt_draw occt_inspector occt_vtk occt_opengles
```

The default link set intentionally avoids OCCT VTK/OpenGLES/DRAW test libraries so a normal standalone rebuild does not require those optional packages.
