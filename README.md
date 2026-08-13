# physicsAddIn

Physics simulation and robot-cell tooling built on a shared `Common/` core: PhysX rigid-body
simulation, OPW6 robot kinematics, the motion planner, and the station package format.

Four build targets:

- **RoboDK plugin** (`physicsAddIn/PluginPhysics.pro`) — adds PhysX roles, conveyors, and a
  built-in package library with placement tools to RoboDK. The build script compiles and installs
  `PluginPhysics.dll`; enable it under Tools → Add-ins.
- **QtCadViewer** (`QtCadViewer/QtCadViewer.pro`) — standalone Qt viewer for the shared CAD
  scene tree, with material editing, convex decomposition, and rigid-/soft-body simulation tools.
- **RobotSimulator** (`RobotSimulator/RobotSimulator.pro`) — GLFW/Dear ImGui application, no Qt.
  Opens self-contained robot packages and multi-robot stations: kinematic jogging, program
  editing and simulation, PhysX conveyors and drag chains, live runs against Teensy hardware
  over serial, and a set of deterministic CLI subcommands (`--validate-package`,
  `--simulate-program`, `--live-run`, ...).
- **RobotSimulator (WebAssembly)** — the same application compiled with Emscripten by
  `scripts/build_wasm.ps1`.

Build instructions and dependency setup: [BUILDING.md](BUILDING.md).
Design docs — architecture, the conveyor rules, library placement, and how anything is
verified: [docs/](docs/README.md).
Contribution and code-style guidance: [CONTRIBUTING.md](CONTRIBUTING.md).
PhysX, OpenCascade, CoACD, ImGui, ImPlot, and GLFW are external dependencies restored or built
by the scripts under `scripts/`; the only committed third-party code is described in
[third_party/README.md](third_party/README.md).

Content directories, each with its own README:

- `library/` — the built-in asset catalogue ([library/README.md](library/README.md))
- `examples/` — station examples ([examples/README.md](examples/README.md))
- `templates/` — authoring templates ([templates/README.md](templates/README.md))
- `tests/robot_simulator/` — regression fixtures ([tests/robot_simulator/README.md](tests/robot_simulator/README.md))
- `firmware/` — AR4 Teensy 4.1 firmware ([firmware/README.md](firmware/README.md))

The plugin is built on the [RoboDK Plug-In Interface](https://github.com/RoboDK/Plug-In-Interface)
([documentation](https://robodk.com/doc/en/PlugIns/index.html)).
