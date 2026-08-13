# RoboDK library package importer

`convert_robodk_library_package.py` is the shared conversion path for articulated robots,
one-axis gantries, and named static objects from RoboDK library files or stations. A JSON spec supplies source provenance,
kinematics/mechanism metadata and output locations; model-specific geometry does not live in the
converter.

## Usage

From the repository root:

```powershell
py -3 .\scripts\convert_robodk_library_package.py `
  .\scripts\robodk_import_specs\fairino_fr20.json

py -3 .\scripts\convert_robodk_library_package.py `
  .\scripts\robodk_import_specs\gudel_tmf1_6m.json
```

The default extraction mode opens a disposable hidden RoboDK process for each link. This avoids
touching an interactive RoboDK station and works around the API-call cap in unlicensed RoboDK
installations. Use `--reuse-session` with a licensed installation for faster extraction. Once the
VRML and pose sidecars are cached, `--reuse-vrml` rebuilds the simulator package without opening
RoboDK.

For a static object already open in a licensed RoboDK station, use `kind: "accessory"`, provide
the exact `source.itemName`, and pass `--existing-port PORT`. The converter exports local geometry,
disconnects when finished, and never closes the operator-owned RoboDK process.

The source may be an HTTPS `url` or a repository-relative/local `path`. Always include `sha256` so
an upstream model change fails explicitly instead of silently changing checked-in assets.

## Conversion contract

- RoboDK remains the proprietary-file reader; each link is exported through its public API.
- Robot link-local geometry is assembled into the RoboDK home joint frames before packaging;
  gantry geometry remains link-local so its moving frame can place the carriage at runtime.
- Robot imports fail if any extracted home frame disagrees with the supplied modified-DH chain,
  checking the full rotation and translation matrix rather than only link count or mesh presence.
- Link geometry and its RoboDK base-frame pose are baked into simulator Y-up coordinates.
- Equal-material VRML shapes are merged, written as `meshbin`, and receive baked convex hulls.
- Source-only covers or placeholders can be omitted declaratively with
  `excludedGeometryGroups`, keyed by link index; the converter and collision bake both exclude
  those groups.
- When a source-only placeholder shares a material group with valid geometry,
  `excludedGeometryComponents` can omit individual welded connected components. It is keyed first
  by link index and then by material-group index; component indices use deterministic bounds order.
- Robot kinematics, limits and motion defaults come from the spec, not mesh inference.
- Gantry specs can declare a procedural drag chain, including one reusable link mesh, link count,
  bend radius, joint limit, anchors and optional endpoint brackets.
- A drag chain's `physicsEnabled` (a `dragChain` sub-key) defaults to `true`. A large
  continuously supported carrier can set it to `false` to use the deterministic U-contour while
  shorter free carriers continue using PhysX.
- ZIP entry order, timestamps and permissions are normalized for reproducible output.

## Adding another asset

Copy the closest spec and change its `source`, `output`, and `robot`, `gantry`, or `accessory` block. For robots,
provide the complete modified-DH vector, home pose, joint limits and link count. For gantries,
provide the travel axis/limits and, when applicable, a `dragChain` block. Keep assumptions in the
spec's `notes` field so generated assets retain their provenance and limitations.

Validate generated outputs with the canonical executable. Robot packages can be checked directly;
mechanisms are validated through a station because package validation currently accepts robot and
static-accessory roots only:

```powershell
.\dist\RobotSimulator\release\RobotSimulator.exe `
  --validate-package .\library\packages\fairino_fr20_robot.zip

.\dist\RobotSimulator\release\RobotSimulator.exe `
  --validate-station .\examples\stations\fairino_gudel_machine_tending.station.json
```
