# Architecture

Two applications run the same simulation code:

- **RobotSimulator** — GLFW/Dear ImGui, `CONFIG -= qt`. Links no Qt at all.
- **PluginPhysics** — a RoboDK plugin, Qt Widgets, hosted inside RoboDK's process.

`Common/` is what they share. `QtCadViewer` is a thin standalone shell over the same CAD tree.

## The sharing rule

Widgets cannot be shared. One is immediate-mode ImGui redrawn every frame, the other is retained Qt
objects, and an abstraction over both would be larger than the two editors it replaced.

Everything the widgets are *about* can be shared: the field set, the ranges, which fields apply to
which generator, the enumerations, the validation, the geometry, and the rules.

Each editor is a rendering of a shared schema and holds no knowledge of what a field means. A field
added to `Common/` appears in both UIs without either being edited.

The same line applies to every subsystem here:

| Shared | Per-host |
| --- | --- |
| `ConveyorCore` transport rules | the host that owns the scene graph |
| `ConveyorGeometry` appearance | how it is drawn |
| `MountingSnap` snap solver | what a placement delivers into |
| `PlacementSession` armed state | where the ghost is drawn |
| `AccessoryPropertySchema` fields | the panel that renders them |
| `LibraryCatalogue` contents | the list widget and its thumbnails |
| `ViewRay` projection | the camera it reads |

What a placement delivers into is a `StationDocument` entry plus a pose controller in RobotSimulator,
and a generic RoboDK item carrying JSON in the plugin. That difference is the whole of the split.

## Build constraints

**The plugin must never link OpenCascade.** Windows resolves a plugin's imports from RoboDK's own
`bin` directory, not from beside the plugin. Linking OCCT made the plugin loadable only while RoboDK
happened to ship a matching version of nine `TK*` DLLs. RoboDK 6.0 ships 7.6.0 and it worked; a RoboDK
that moved to 7.7 would have broken it silently.

So `PluginPhysics.pro` includes neither `Common/Common.pri` nor `build/deps.pri`. It gets Qt (RoboDK's
own), PhysX, and `CadNode.h`, which is header-only without `CADNODE_ENABLE_OCCT`. Any `Common/*.cpp`
the plugin compiles must build without that define.

Dropping OCCT costs the CAD viewer, which was the only part that needed OCCT, XCAF and
`SimulationManager`. QtCadViewer is that same viewer standalone and is unaffected.

Add a new `Common/*.cpp` only to the targets that consume it. `RobotSimulator.pro` and
`scripts/build_wasm.ps1` intentionally mirror the same first-party source list, and the WebAssembly
build has a drift guard that parses the `.pro` and fails on a mismatch. `PluginPhysics.pro` lists
only the Qt/OCCT-free subset used by the plugin. QtCadViewer consumes the viewer-oriented set through
`Common/Common.pri`.

`CadNode.h` is shared with a Qt-free target, so it must not pull in QtGui. `CadVec3` exists for that
reason and is double precision, matching the JSON it is read from. Conversion to `QColor` lives in
`CadNodeQtAdapter.h`.

## QtCadViewer boundary

`QtCadViewer/main.cpp` owns only application startup and the top-level window. Its reusable viewer
work is split by responsibility in `Common/`:

- `CadXcafIo` reads and writes OpenCascade/XCAF documents.
- `CadNodeOps` changes an already-loaded `CadNode` tree.
- `CadDecomposition` owns the V-HACD and CoACD implementations.
- `CadViewerWiring` connects Qt tree models, OpenGL views, and context menus.

`CustomModelTreeModel` is the single Qt adapter for a `CadNode` tree. Viewer panes belong to a
`CadViewerWorkspace` owned by the application; they are not kept in process-global lists.

## RoboDK API notes

These cost real debugging time and are not obvious from the headers.

**`IItem::setParam` is not `IRoboDK::setParam`.** The first is per-item storage saved with the
station. The second is the live IO publish. Same name, two mechanisms.

**`IItem::setDO` / `waitDI` / `setAO` emit instructions into a program.** They are not live setters.
`setDO` is the name that looks correct for publishing a simulated value and is the wrong one;
`IRoboDK::setParam` is what releases a native *Wait for I/O*.

**`EventChanged` arrives synchronously from inside the API calls a step makes.** A step can be
re-entered and a prune can arrive mid-iteration. Both are deferred in `PhysicsWorld`. Run control
starts and stops runs from its own timer, not from inside `step()`, because `stop()` deletes items and
that comes back as an event.

**Never enumerate by name.** `ItemList(kind, True)` returns names, and stations repeat them — four
objects in the palletizing showcase are called `Box_24X16`, and two placed pedestals are both
`Pedestal H12in`. `Item(name)` answers with the same item every time. Use `ItemList(kind, False)` or
`Childs()`, which return items. Where an item's name *is* a conveyor's name, a duplicate is two
conveyors claiming the same IO variables, so a colliding name is a reported refusal.

**RoboDK does not uniquify a generic item's name when the name is set.** Adding two conveyors left
both called `Conveyor`. Names are chosen against what the station already holds.

**There is no selection-changed event.** `TypeEvent` is Render / Moved / Changed / ChangedStation /
About2Save / About2ChangeStation / About2CloseStation / TrajectoryStep. `Selection()` and
`setSelection()` are readable and writable, but nothing tells you when it changed.

**RoboDK's library drag-and-drop is not available to a plugin.** There is no drop hook and no way into
RoboDK's own library panel. The plugin ships its own dock.

**In `-HIDDEN` there is no `MainGL`**, so there is no viewport and no camera. Anything the mouse path
computes is unreachable to a headless check, which is why frame conversions are gated through verbs
that skip the projection rather than through a scripted cursor.

**A panel must not rebuild at event rate.** Sync rows in place, keyed on the item, and never
dereference a pointer read back out of a widget.

## Coordinate conventions

RobotSimulator's world is y-up, RoboDK's is z-up, and packages for both are authored y-up.

The plugin does not convert. `PhysicsWorld::floorNode()` is a placement grid declared in the accessory
convention and turned into station axes by `conveyorAccessoryFrame()`. Because `mountingsnap` mates by
aligning a source interface's frame to its target's, a package authored y-up lands upright with no host
arithmetic. A floor declared directly in station axes would lay every package on its side.

A pose *stated* rather than snapped means whatever the caller said it means.

`ViewPose` is camera-from-world and "View angle" is vertical. Both were undocumented and are settled by
`tools/view_ray_smoke.cpp`.

An item's pose is not its geometry. `Pallet 40inx48in` is parented into a frame that turns it ninety
degrees, so anything aimed at its `PoseAbs` misses by 600 mm.
