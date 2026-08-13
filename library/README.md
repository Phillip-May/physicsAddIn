# Built-in asset library

This directory is the source for the built-in assets used by RobotSimulator, the RoboDK plugin,
and the WebAssembly build.

- `packages/` contains the distributable ZIP packages. RobotSimulator copies them beside its
  executable, the WebAssembly build preloads them, and the plugin build embeds them in
  `physicsAddIn/builtin_packages.qrc`.
- `sources/<asset-id>/` contains each package's generated manifest and mesh sources.

At runtime the plugin extracts its embedded packages to the application cache under
`PhysicsAddIn/packages`. To use a different catalogue, set the RoboDK station parameter
`PhysicsLibraryRoot` or send the plugin command `libraryroot <path>`, then use **Rescan** in the
Library dock.

The package filename stem is its stable built-in ID. For example,
`packages/roller_conveyor.zip` is referenced by stations as `builtin:roller_conveyor`.
Do not put stations, test fixtures, or scratch exports in `packages/`; the build intentionally
catalogues every ZIP in that directory.
