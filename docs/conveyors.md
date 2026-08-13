# Conveyors

`Common/ConveyorCore` owns the transport rules. RobotSimulator and the RoboDK plugin both run on it.

| Piece | Where |
| --- | --- |
| The rules, scene-agnostic | `Common/ConveyorCore.{h,cpp}` |
| CadNode host | `RobotSimulator/SceneConveyorHost.{h,cpp}` |
| RoboDK host | `physicsAddIn/RoboDkConveyorHost.{h,cpp}` |
| Path model | `conveyorPathPoseAt`, `Common/AccessoryGeometry.cpp` |
| Field ranges | `Common/ConveyorGeometry.cpp` |
| Description | `TransformNodeData::accessory*`, `Common/CadNode.h` |

`appendPhysicsCollisionMeshes` and the tool actuator / gripper model are RobotSimulator's and are not
part of the shared core.

## The transport contract

Eighteen rules. A change to `ConveyorCore` is correct only if all of them still hold and the golden
traces are byte-identical.

1. Spawner cadence: `secondsUntilSpawn -= dt`; when due, spawn and **add** the interval (never assign),
   floored at 0.05 s, so fractional tick time survives and cadence does not drift with simulation speed.
2. Active-spawn cap: products counted by `originSpawner`, not by parent; zero means unlimited.
3. A grasped product is skipped entirely.
4. A product whose conveyor no longer resolves is destroyed.
5. Logical advance: `progress += speedMmS * dt / pathLengthMm`, signed by `forward`.
6. Physical advance: progress is *derived* from the body pose by closest-point, not integrated.
7. Deleter, logical: role is deleter and `0.5 <= progress <= 1.0`.
8. Deleter, physical: the body intersects the deleter volume.
9. Leaving the path, logical: `progress > 1` or `< 0`; `leavingForward = progress > 1`.
10. Leaving the path, physical: `physicalBodyReachedEnd` with an allowance of `speedMmS * dt`.
11. On leaving: transfer to `nextConveyor`, resetting progress to 0 or 1 by the target's direction.
12. **A disconnected endpoint is a stop, not a chute.** Progress is clamped and the product stays.
13. A product that is not physical is placed at its path pose; a physical one is not.
14. After the physics step, body poses are written back as conveyor-local transforms.
15. Scene-dirty is raised once per tick, and only if the graph actually changed.

Rules 16 to 18 are logical accumulation, added because two products at a dead end otherwise occupy
the same point. In PhysX mode rigid bodies queue for free, so this only bites logical mode, which is
the mode a RoboDK cell runs in.

16. The leader rests `endInsetMm` short of the end, which is where the end stop is. At the end only.
17. Nobody passes the product ahead: everything behind the leader sits `productPitchMm` apart.
18. A full conveyor refuses the spawn. Distinct from the rule 2 cap, and a line needs both — the cap
    limits how many exist, this limits whether there is room at the entry for one more.

`endInsetMm` and `productPitchMm` are RobotSimulator's initial-workpiece layout,
`1 - (endInset + spacing*k)/length`, maintained rather than laid out once.

`pathLengthMm` can be zero, which would give `inf`/`nan` progress. A non-positive length advances
nothing, which is the same as the stop it already amounts to. `nan` spreads through the leaving test
into a transfer and onto whatever the conveyor feeds, so this is covered by
`conveyor_core_smoke` case 15.

## A conveyor is an accessory instance

Each declared conveyor is a RoboDK frame item whose accessory parameters live on the item as JSON in
`IItem::setParam("PhysicsConveyor", ...)`. The item is the lane's start frame, renamed to the
conveyor's name.

The lane comes from that item's pose and `accessoryLengthMm`. It is not two frames.

Two frames were a workaround for not having an accessory node. Deriving the lane instead deleted the
plugin's parallel straight-line path model (`endpoints`, `projectOnto`, `placeProduct`, `reachedEnd`),
absorbed the `scenery*` fields into `accessoryWidthMm` / `accessoryHeightMm`, and gave the plugin
curved and sloped conveyors that two frames could never describe.

The cost is that where the conveyor *stands* is now derived, and so is the delivered pose, which is the
vendor's own pick target. `run_conveyor_smoke.py` checks the delivered pose to `0.000 mm`
permanently.

The alternative — a dedicated empty frame per conveyor with the lane frames as children — is tidier on
paper. It adds an item whose only purpose is to be clicked, needs retrofitting into every existing
station, and has to slot into the `Conveyor Belt Base` parent chain that makes a moved conveyor carry
its lane. Take it only if a cell ever needs a conveyor whose lane frames are shared.

**`spawner=` stays a string in `PhysicsAddIn`.** It has no frames, and neither of its two references is
the spawner: the prototype is a hidden object and the parent is just where clones go. It is a rule
about where products come from, not a machine in the cell, so there is nothing to select, draw or
delete. The distinction is crisp: an item declaration is a conveyor in the cell, a string declaration
is a rule about products with no machine.

## Stations carry no Python

`ItemList(ITEM_TYPE_PROGRAM_PYTHON)` comes back empty on the generated showcase. The cell is data,
native instructions, and a plugin that watches. Two API facts make that possible:

**Run control reads `IItem::Busy()`.** `PluginPhysics::pollStationRunControl` runs on a 100 ms timer
whether or not a simulation is going, and begins a fresh run on the rising edge of "any program is
busy". Only for a cell whose configuration set `autostart`, so an operator's hand-marked station is
never restarted underneath them. `Reset` is a program like any other, so this also ends the run it is
about to empty.

The *edge* is the signal and the state is not: `Main` calls `Init_24X16`, which calls five more
programs, and the busy set never falls empty in between. A restart part way through `Main` would take
the stack with it.

**The photo-eye is a station variable, not a verb.** `IRoboDK::setParam` releases RoboDK's own *Wait
for I/O* instruction, so the station gained one native instruction and no scripting.
`PhysicsWorld::Sensor` is declared as `sensor=<variable>:<conveyor>:<from>:<to>` and written at the end
of each step, only when the value changes. A station parameter set sixty times a second is sixty
station edits a second for everything else watching. Grasped products are excluded, so the eye goes
dark the instant the tool takes the box.

Nothing in `ConveyorCore` changed for this. A sensor is not a transport rule.

Rule 12 is what makes the pick possible. The pick is a fixed pose with an `Attach` straight after it,
so the box has to be stationary when the robot arrives, or the photo-eye only starts a chase. Declaring
the lane with an empty `next` puts the disconnected-endpoint stop exactly at the pick pose.

## Indexed lines

The palletizing showcase's original conveyor was step-indexed and synchronous with the robot program.
`Conv_Move_24X16` indexed the belt by exactly -700 mm once per call, and `Pick_Box_24X16` called it
after its own `Attach`, so one pick advanced the line by one box. Nothing waited for a box because the
program itself decided when one arrived.

The core is free-running at a speed, so swapping in free-running transport is not a substitution. Three
ways out were considered:

1. An indexed advance in the core. Faithful to what that machine is, but it is another rule, and
   RobotSimulator would need it too, where nothing wants it. It encodes one demo's workaround as a rule
   of the engine.
2. Leave the index to the station. Needs no new code, but does not fix a robot that stops being able to
   pick after the belt has indexed away from a fixed pick pose.
3. Keep free-running transport and add a photo-eye. Costs the core nothing.

**Chosen: 3.** It is the only one that makes the RoboDK cell a conveyor cell rather than a puppet. Take
1 only if the engine should model indexed lines as a first-class thing, in which case do it in both
hosts.

## Spawn caps

A product only leaves the cap when a tool lets it go. A conveyor with no deleter and no end therefore
holds a cap slot for ever if nobody picks from it, and a capped spawner whose products cannot leave the
flow is a deadlock.

`conveyorcore::Runtime::spawnerAtCap` exposes rule 2's own test, so a cell cannot work out "at cap" for
itself and disagree with the rule that actually withholds the product. `summary()` appends `N
spawner(s) at cap and not feeding`, and each `conveyors` record carries `cap <live>/<max>`, marked `AT
CAP, not feeding` when it is refusing.

This is a statement of fact, not an error. Under the photo-eye design a spawner sitting at cap while a
box waits at the stop-gate is the normal resting state of the line.

The showcase's cap is 1. Two products both clamp to progress 1.0 at a stop-gate, coincident because
nothing queues, and `Attach` takes the closest object, so it would take one and abandon the other,
holding the cap for ever. Logical accumulation (rules 16-18) later gave the line a real queue.

## Known duplication, left alone

`RobotSimulator/ConveyorRuntime.cpp`'s `spawnConveyorWorkpiece` carries its own copy of rule 2's cap
expression for the manual spawn the CLI and panel offer. It counts `g_scene.conveyorWorkpieces`, which
is the same list the host hands the core through `setProducts`, so it cannot disagree with
`spawnerAtCap`, only repeat it. Pointing it at the runtime would mean threading a `ConveyorId` through
the manual path inside the goldened host for no behaviour change. Worth doing next time that file is
open for another reason.

## Traps

- **Rule 12.** A disconnected endpoint stops the product. It is one `else` branch and the obvious thing
  to "simplify" into a deletion. The golden traces and `conveyor_core_smoke` both cover it; the fixture
  station couples it to the spawn cap so a wrong branch diverges by hundreds of lines.
- **Cadence drift.** `secondsUntilSpawn += interval`, never `=`, and outside the cap test.
- **Progress is derived, not integrated, in PhysX mode.** Integrating as well double-counts.
- **The host queries take the whole `Product`, not an id.** Several need its progress and direction at
  the instant they are asked, and a host reading its own mirror answers about last tick.
- **Transport changes nothing PhysX knows about.** `PhysicsWorld::step` must report that the station
  changed, or RoboDK is never asked to render, and item poses are written but not committed. That looks
  exactly like a stalled run.
- **Two convex hulls let go interpenetrating throw each other across the cell.** A box dropped at a
  fixed height onto a stack that has grown past it explodes because it started inside, not because
  collision is wrong. Aim a drop from the resting geometry.
- **`accessoryLengthMm` is clamped 600..6000 mm** where a measured frame distance was not. A lane
  shorter than 600 mm silently becomes 600.
- **The committed goldens print progress at 9 decimals.** If a different MSVC or optimizer perturbs the
  last ulp, drop to 6 decimals and re-golden. Do not loosen what the harness checks.
