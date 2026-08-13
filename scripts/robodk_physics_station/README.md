# Palletizing showcase, with physics

`scripts/make_physics_palletizing_station.py` turns the RoboDK palletizing showcase into a variant
where the stack is simulated instead of assumed. It writes a new `.rdk` and never modifies the
source station.

```powershell
py -3 .\scripts\make_physics_palletizing_station.py `
  "C:\Users\you\Downloads\PhysisPluginPalletizing showcase.rdk"
# -> "PhysisPluginPalletizing showcase physics.rdk" beside it
```

The plugin must be installed for the result to do anything; see
[BUILDING.md](../../BUILDING.md) and `scripts/build_robodk_plugin.ps1`.

**The variant has no scripting.** Not "adds none" — `ItemList(ITEM_TYPE_PROGRAM_PYTHON)` comes back
empty, all eight of the vendor's scripts included, and the generator asserts that on the station it
writes. The whole cell is data, native RoboDK instructions and a plugin that watches: it notices when a
run begins and when a box has arrived, the two things that looked like they would need scripts and do
not.

## What the source station already did

Worth stating, because it is most of the cell and none of it was rewritten:

| Piece | How the showcase does it |
| --- | --- |
| Spawning | `Pick_Box_24X16` calls a `New_Box_24X16` script that clones `BoxRef_24X16` |
| Transport | `Conv_Move_24X16` indexes the `Conveyor Belt` mechanism by exactly −700 mm per pick |
| Carrying | RoboDK's native *Attach to RobotiQ EPick Four* instruction |
| Releasing | `Drop_Box`, a single native *Detach* instruction |

That is a complete logical simulation in RobotSimulator's sense: while a workpiece is transported or
carried, the station owns its pose, it costs the solver nothing, and it cannot fight the mechanism
holding it. The showcase simply had nothing after the detach — a dropped box appeared at its
programmed target and stayed there.

The transport is worth reading twice, because it is not a conveyor. It is a **step index, synchronous
with the robot program**: `Conv_Move_24X16` is called from inside `Pick_Box_24X16`, *after* its own
Attach, so one pick advances the line by exactly one box. Nothing waits for a box, because the program
itself decides when one arrives. Swapping in free-running transport is therefore not a substitution, and
the section below is about what has to be added for it to be one.

## What the variant changes

**One station parameter.** `PhysicsAddIn` describes the cell:

```
autostart=1;gravity=-9.81;role=Pallet 40inx48in:static;role=Floor:static;
role=RobotiQ EPick Four:kinematic
```

RoboDK saves station parameters inside the `.rdk` and shows them under the station's *shared
parameters*, so the cell describes itself and the plugin reads it when the station opens. It is a
flat line rather than JSON precisely because it is edited in that table.

**The conveyor is not in it.** It is an *item*: a custom generic node called `Box feed` carrying its own
accessory parameters in RoboDK's per-item data — see "The conveyor, and why it is not the belt" below. The
distinction is the rule rather than an accident: **an item declaration is a conveyor in the cell; a
string declaration is a rule about products with no machine.** That is why `spawner=` stays in the
string and `conveyor=` left it.

There is **no second frame** for the far end of the lane. A conveyor is where it stands plus how far it
runs, which is how RobotSimulator has always described one. Where it stands is *derived from the
station's own frames* rather than typed in.

**Every Python program deleted.** All eight of the vendor's scripts, because each is either something
the plugin now does or something there is nothing left to do:

| Script | Why it can go |
| --- | --- |
| `New_Box_24X16`, `New_Box_12X10` | the declared conveyor produces the boxes |
| `Conv_Move_24X16`, `Conv_Move_12X10` | the declared conveyor moves them |
| `Conv_Init_Position` | with the `Conv_Move_*` scripts gone nothing moves the belt, and the generator homes it at build time |
| `Clear_Conveyor`, `Clear_Pallet`, `Clear_Tool` | the only workpieces in the cell are the plugin's, and `stop` removes every one of them and restores every pose the run changed |

`Init_24X16` goes with them, having nothing left to call, and `Main`'s call to it goes too.
`Pick_Box_12X10` is deleted outright rather than converted — `Main` never called it, so converting it
would ship a second line nothing exercises — along with its own `AppConveyor 12X10` and
`Pick_Conveyor 12X10` targets. Program first, then targets: a target deleted while a move instruction
still names it is how a station becomes unopenable. `BoxRef_12X10` and its `New_Box 12X10` frame stay,
hidden prototype geometry that costs nothing and lets a 12X10 conveyor be declared later as data alone.

The generator asserts, on the station it just wrote, that no Python program items remain and that no
instruction in any surviving program names a deleted item. That makes zero Python a **build-time
acceptance criterion** rather than a claim in a README.

**One native instruction.** `Pick_Box_24X16` gains a *Wait for I/O* on `Box feed ready`, before every
move, so the robot waits where it is until a box has arrived. Nothing declares that variable: it exists
because the conveyor called `Box feed` does.

`Drop_Box` is **not** touched. It keeps RoboDK's own Detach instruction.

**And four more deletions, of workpieces.** The source station was saved part way through a cycle: one
box on the gripper, one on the pallet, two on the conveyor 550 mm and 1250 mm along it. Its own
`Clear_Tool`, `Clear_Pallet` and `Clear_Conveyor` deleted exactly those, and `Init_24X16` called all
three — so the station only looked like that until it was played. Harmless while a dropped box merely
appeared at its target; now that the plugin adopts what a tool is holding, the one on the gripper would
fall onto the pallet before the first real box arrived. The generator applies those same three
predicates at build time, so the saved variant starts from a clean cell — and that is also what let the
three scripts be deleted rather than replaced.

Do not look for those boxes by name. Four objects here are called `Box_24X16`, `ItemList(kind, True)`
returns names, and `Item('Box_24X16')` answers with the same one every time — which is how one leftover
on the gripper was first miscounted as four.

## How the plugin knows, without being told

The plugin watches each workpiece's parent chain. A native *Attach* re-parents the box under the
tool; a native *Detach* moves it back out. Those transitions are the handoff:

| Transition | What the plugin does |
| --- | --- |
| appears on the conveyor | logical — no body, the station owns its pose |
| moves under a tool | still logical; any body it had is destroyed |
| leaves a tool | re-parented to the station root at that exact pose, and given a dynamic convex body |

An object the plugin **did not create** is picked up by the same watching: anything found directly
beneath a kinematic tool is adopted as carried and held. Without that the handoff was closed in a loop
— it is driven off the participant list, and an object the plugin never spawned was never noticed, so
never became a participant, so was never noticed. A box the station's own script made would be carried
to the pallet and land there with no body and no row in the panel, and the sentence above this table
would only have been true of the plugin's own clones.

A box is only handed to the solver once a tool has taken it **and let it go**, which is why one
still riding the conveyor never falls off it. Watching the change rather than the state is what
makes that distinction possible.

Proximity was the alternative and was rejected: a grab radius has to be tuned per cell, and when it
is wrong it takes the wrong box or drops one early — failures that look like physics bugs and are
not.

## When a run begins

`Reset` used to delete the objects a run is holding — the boxes on the conveyor, the stack on the
pallet, whatever the tool carries — with three `Clear_` scripts.

**Playing any program begins a fresh run**, for a cell whose configuration says `autostart=1`. `Reset`
is a program like any other, so that is what ends the run it is about to empty: `stop` takes the spawned
boxes with it and restores every pose the run changed, which is between them everything the three
`Clear_` scripts did — which is why they could go. The new run begins with no products and every cadence
at zero, and a run an operator had stopped comes back rather than being played without physics.

RoboDK has no event for a program starting, so this is a poll — `Busy()` over every program, forty times
a second — and a play is its **rising edge**. That distinction is the whole mechanism: `Main` calls
`Pick_Box_24X16` and `Drop_Box` thirty times between them, and a nested call must not read as a play or
the run would restart part way through and take the stack with it. Measured on this station, `Main`
reports busy from its first instruction to its last and the busy set never falls empty in between.
`run_reset_smoke.py` is what holds that: it samples the simulated time all the way through `Main` and
fails if it ever goes backwards.

**`Reset` is now a single native `Pause`, and the pause is load-bearing.** With its scripts gone it has
nothing left to do, and a program with nothing in it finishes inside a poll period: the play would be
missed, and a Reset that is missed looks exactly like a Reset that ran and did nothing. So it holds for
500 ms — twenty poll periods — and the poll itself came down from 100 ms to 25 ms, because "far shorter
than the shortest program" stops being true once a station's programs are pure native instructions.

A station variable set by a native *Set I/O* was the alternative and is worse: a bit that is already 1
cannot signal that a program was played *again*, so a replay would run against a stale stack. The
`Busy()` edge handles a replay without being told. Two one-line Python programs calling `stop` and
`start` was the other alternative, and it would have cost the claim at the top of this file.

## The conveyor, and why it is not the belt

The `Conveyor Belt` mechanism is **deleted**. It had been decorative since the transport moved to the
plugin, and decorative is the charitable word: it *looked like* the indexed belt the cell no longer has,
so the station showed a machine it did not own. Its child `Conveyor Box` goes with it. `Conveyor Belt
Base` **stays** — `Pick_Box_24X16` begins `Set Ref.: Conveyor Belt Base` and every pick target is
defined against that frame.

The boxes travel along a lane the conveyor itself describes — `itemPose * conveyorPathPoseAt(t)`, which
runs half a length either side of where the conveyor stands, at deck height above it — and the plugin
moves them at 700 mm/s.

**The lane is the accessory's own path, and there is no other path model.** Where progress *t* is, where
the nearest point to a world point is, which way the belt runs there, where it ends: every one of those
is `Common/AccessoryGeometry` answering about the same parameters RobotSimulator's conveyor answers
about. The plugin used to carry a second, simpler model — a straight line between two frames, with its
own projection, its own interpolation and its own end test — and all of it is gone. Which is also why
the plugin now draws and transports along **curved and sloped** conveyors: `accessoryTurnAngleDeg`,
`accessoryCurveRadiusMm` and the four corner heights were always in the shared path, and two frames
could only ever describe a straight, level lane.

The conveyor's frame is a RoboDK frame — X down the belt, Y across it, Z up — and its origin is at the
near end of its own feet rather than on the deck, because that is where a generated accessory's origin
is. The accessory's own frame is Y-up; the change of basis between the two is one constant matrix in
`ConveyorScenery`, and it is the only place the two cells' axis conventions meet.

**The conveyor is a custom item called `Box feed`, and it is one item.** One name, not three: the node is
called that, the conveyor is called that, and its three IO variables are prefixed with it, so there is
nothing that can disagree with anything else. Renaming it in RoboDK's tree renames the conveyor. Its
description — the same `TransformNodeData` accessory recipe a RobotSimulator station edits, as the same
JSON — lives on the item, in `IItem::setParam("PhysicsConveyor", …)`, which RoboDK saves inside the `.rdk`.
That one node, `ITEM_TYPE_GENERIC` from `IRoboDK::Command("AddItem", name)`, carries the name, the
description, the custom icon and the placement. It is what you double-click, right-click and delete, and
what products are parented under.

**Generic rather than an object**, because an object is a *RoboDK* object: it answers RoboDK's own object
commands and opens RoboDK's own dialog on a double click. A conveyor is not an imported mesh and should
not pretend to be one. A generic item is a node whose meaning is this plugin's, and it is the only kind
that takes a custom icon — `setParam("IconSet", path)` works on generic items alone.

**Both of a generic item's limits were measured, not assumed**, and between them they are why the plugin
draws, picks and places the conveyor itself rather than handing any of it to RoboDK:

- `AddShape` **refuses** a generic item — "Invalid item provided". A generic node can hold no geometry.
- a generic node's `PoseAbs()` is **always identity**, however its pose is set. It has no place in space.

So **where the conveyor stands is twelve numbers under `pose` in its own JSON**, stated relative to the
node's RoboDK parent, the way a `CadNode`'s transform is relative to its parent — and a conveyor is moved
by moving the frame it hangs under. A generic node does *not* break the pose chain, and that is measured
on this station: `Conveyor Belt Base` stands a quarter turn round and 430, 550, 370 mm from the origin,
the node under it answers `PoseAbs()` as identity, and each product under *that* answers exactly
`parent.PoseAbs() * its own local pose` — 370 mm and 740 mm away from what it would answer if the chain
were broken.

Nothing reports a parent being dragged, so the plugin compares each conveyor's placement against the pose
its triangles were baked at, on the panel timer, and re-bakes the ones that have drifted.

That answers these with RoboDK's own machinery instead of new machinery:

| Ask | How |
| --- | --- |
| select | click the node in RoboDK's tree — or click the conveyor in the 3D view, which `ConveyorPicker` casts a ray for, because RoboDK hit-tests nothing the plugin paints |
| see what is selected | the plugin's own silhouette, over the same triangles, for the same reason |
| delete | delete the node; the products go with it, because they are its children |
| copy / paste / merge one in from another `.rdk` | RoboDK's own, because the description travels with the item |

> **No pose question about a conveyor is asked of an item.** `placement()` composes the stored `pose`
> with the node's parent's `PoseAbs()`, and asking the node itself answers the station origin for every
> conveyor in the cell.
>
> Standing a conveyor up therefore copies the old frame's *local* pose, which is what a placement is
> measured against and needs no absolute arithmetic. Getting this wrong put the showcase's conveyor 1.2 m
> away and a quarter turn round, with the delivered-pose gate still reading `0.000 mm` because the plugin
> and the check were reading the same wrong pose. There is now a constant in that gate.

`IItem::setParam(name, QByteArray)` is **per-item storage** (`iitem.h:756`). `IRoboDK::setParam` is
the live IO publish and a different function with the same name — the same trap as `IItem::setDO`,
which emits an instruction into a program rather than setting anything.

> **Renaming a conveyor renames its IO variables, and that silently breaks a native *Wait for I/O*,**
> which holds `Box feed ready` as a literal string in the instruction. That was already true; making
> the item's name the conveyor's name only puts the rename one click away. The plugin zeroes the old
> name's three variables when it notices the rename, so a Wait against them stops rather than passing
> on a stale 1 — but the instruction still names a variable nothing writes, and it has to be edited.

**And what the plugin paints is the real conveyor.** RobotSimulator's roller conveyor — its frame, feet,
rollers and cover, the role enclosure over the spawner's half of the deck, and the cyan product-and-∞
marking with the workpiece's own silhouette in it — from `Common/ConveyorGeometry`, which is the *same
translation unit* RobotSimulator renders from rather than a copy of it. The two cells look like the same
machine because they are.

The geometry is **drawn, not added**: `IRoboDK::DrawGeometry` paints it during an `EventRender`, which is
what lets a conveyor be one node instead of a node and an object standing in for it. It is generated, not
loaded — no mesh package, no zip, no cook at save time — because `ConveyorGeometry` reads no mesh at all
and produces every vertex from the parameters. The triangles are baked in **station coordinates**, at the
conveyor's placement, so a conveyor that has moved is re-baked rather than re-placed.

What RoboDK will not do for painted geometry, the plugin does over those same triangles: `ConveyorPicker`
casts a ray from the click and draws the selection outline as a silhouette. Both are `Common/ViewRay`,
which is host-free arithmetic held by `view_ray_smoke`. Open the station without the plugin and the
conveyor is a node in the tree with no picture — its description is all it ever was.

**A workpiece's origin is not where its body is**, and that shows up twice. `BoxRef_24X16`'s origin is
a *corner* — its mesh is `[0..398, 0..609, 0..269]` from it — and products are placed by their origin.
So a spawn is put down one workpiece along instead of at progress 0, or the whole box stands behind the
conveyor; and a product is placed with its **body** centred across the lane, so its origin sits off the
lane by the measured offset. Both are measured from the prototype's mesh, which the plugin already
exports for the ∞ marking, so nothing is declared and it is right for any cell.

That correction used to be applied to the *conveyor* — the deck was shifted sideways off the lane to get
under the load. It inverted, and that is the point: the lane is the deck's centreline now, rigidly, so
the deck cannot be shifted off it. A conveyor's deck is where it is, and the box sits on it. Same
measurement, applied to the product instead, and the drawing stops needing a correction at all — which
is also why `conveyors` no longer reports `ends <x> mm off the lane`. The deck and the lane are the same
expression through the same pose; there is nothing left for that figure to measure.

Both the offset and the workpiece's **attitude** come from the product's *originating spawner's*
prototype, not from the conveyor it is on now. Two reasons, and each one was a bug first:

- a product that transfers onto a conveyor with **no prototype** would otherwise lose the offset, and
  ride 300 mm off that lane while riding the first one correctly;
- a clone's own rotation is **not** a statement about the workpiece. `Copy()`/`Paste()` bakes geometry
  into the parent a copy is pasted under, so the rotation RoboDK reports is about that parent — and the
  conveyor's frame runs X down the belt while this box's long side runs down the belt on its Y. Left
  inherited, the showcase still stacked fifteen boxes and `Box feed ready` came on **once** instead of
  sixteen times: an early box sat at the end stop all run while the native Attach took others. A
  silently wrong pick, which is exactly what counting the ready signal's rising edges is for.

Its deck width and height are measured, not chosen — the workpiece's cross-lane footprint plus a rail
either side, and the drop from the lane to the top of the declared floor, so it stands on the floor
rather than through it.

Where the conveyor stands is **derived from three static frames, not typed in and not run for**:

```
start = BoxRef_24X16.PoseAbs()
end   = start + (Conveyor Belt Base.pos − New_Box 24X16.pos)
```

The lane runs from where the station creates boxes to the conveyor's own base frame, and the workpiece
keeps the lateral offset its prototype has from the creation frame. That comes out at
`[10, 1950, 370] → [10, 550, 370]`, 1400 mm, which is two −700 mm indexes. The conveyor's own pose then
works backwards from the delivery end, because **the pick pose must not move** — `Pick_Conveyor 24X16`
is the vendor's own target and the pick is a fixed pose with an `Attach` straight after it:

```
origin = end + across·(offset·across) − along·(length/2) − up·deckHeight
```

which for this cell is `[208.8, 1250, −496]`: half a length back, a deck height down, and the
workpiece-centre offset across. `run_conveyor_smoke.py` recomputes that expression against the pose the
plugin actually delivers a box to, and requires `0.000 mm` — permanently, because nothing it depends on
has been deleted.

It used to be *measured*, by running `Init_24X16` and taking the box that ended up furthest from the
spawn point. Both give the same answer to `0.0000 mm` — checked once, against the source station, while
all eight vendor scripts were still alive to run. That check is not repeatable now and does not need to
be: the point of replacing it is that the generator no longer plays a program at all, and so can build a
station that has no scripts left to play. The generator asserts what must hold instead of the numbers —
every frame resolves, the lane has length, and it runs *towards* the conveyor base rather than away.

Not by distance to `Pick_Conveyor 24X16`, either. That target is a TCP pose through a rotated reference
frame and is not where the box is. Both frames take their **rotation** from the prototype, because
`placeProduct` interpolates position along the lane and takes orientation from the conveyor's own frame
alone: a frame rotated any other way delivers boxes that arrive in the right place facing the wrong way,
which reads as a gripper bug.

`next` is deliberately empty, and that is the load-bearing part. **A disconnected endpoint is a stop**,
so a box advances to the end of the lane and waits there — and the end of the lane is the pick pose. The
pick is a fixed pose with an `Attach` immediately after it, so the box has to be *stationary* when the
robot arrives; the stop-gate that makes it so was already one of the fifteen rules, which is why none of
this cost the core anything.

**There is no cap.** There used to be one, of exactly one box, and it was a hack: two products both
clamped to progress 1.0 at a stop-gate — coincident, because nothing queued behind them — and RoboDK's
Attach takes the *closest* object, so it would take one and abandon the other at the gate for ever. A cap
of one is the smallest number that hides that. The conveyor **accumulates** now (rules 16–18): the leader
stops at the end and each box behind it comes to rest one pitch back, until the spawner has no room and
its cadence fires into a refusal.

**What "no room" means for a spawner is its own half of the deck** — the half the dark enclosure is
drawn over. A box only has to *overlap* it. That is what limits this lane to **two** boxes rather than
three: with the third slot the enclosure would have two boxes standing on it, and a spawner creating
underneath a box already there is not a queue. The rule is `RobotSimulator`'s own long-standing
`conveyorSpawnAreaClear`, moved into the shared rules so both cells obey it — and it refuses exactly
what that host was already refusing, which the goldens confirmed by changing nothing but serial numbers.

**The pitch is measured, not chosen.** It is the prototype's footprint along the lane —
`610.2 mm` for `BoxRef_24X16`, taken from its exported mesh the same way the smokes measure the pallet.
This matters more than it looks: the transport rules default to `90 mm`, which suits RobotSimulator's
small accessory workpieces and is a seventh of this box, and a pitch smaller than the workpiece is a
queue of boxes standing inside one another. The station picks with a fixed pose and a native Attach, so
overlapping boxes make it take whichever box is nearest the tool rather than the one at the gate — which
is a *silently wrong pick*, not a crash. Measured against the station's own history, 610 mm is the right
order: `Conv_Move_24X16` indexed the belt by 700 mm, the box plus a hand's width of gap.

**The end inset is a stated zero**, and stating it is the point. A roller conveyor's path runs on past
the end stop standing on it, so its leader rests 70 mm short of the path's end; here `Physics_Feed_End`
*is* the delivered pose, so the stop is the end of the lane. Taking the default would put every box
70 mm short of where the robot reaches. It and the pitch are `initialWorkpieceEndInsetMm` and
`initialWorkpieceSpacingMm` on the conveyor's item — the accessory block's own initial-workpiece layout,
which is the same pair of numbers as the queue an accumulating conveyor maintains, so a station cannot
describe the two differently.

## Adding and removing one

**Add Conveyor**, in the *Physics Simulation* menu and on its toolbar, creates **one custom node** with a
default parameter block, selects it and opens its properties. That is the whole of a conveyor: no package,
no mesh and no zip, because `Common/ConveyorGeometry` generates every vertex from those parameters and
reads no mesh at all.
The defaults are `CadNode.h`'s own accessory defaults brought up to the schema's floors by the geometry's
own clamp — nothing invents a number, and the properties panel is where the smallest valid conveyor
becomes the one a cell wants.

**Drag-and-drop works, in the only sense available.** Because the description travels with the item,
RoboDK's own copy, paste and merge-a-`.rdk` bring a conveyor with them, and the plugin adopts it. A
"library" is then a folder of `.rdk` files an operator drags into RoboDK, which is RoboDK's idiom rather
than one imposed on it. **Intercepting RoboDK's own library panel is not possible** — `IAppRoboDK`
offers `PluginLoad`, `PluginLoadToolbar`, `PluginItemClick`, `PluginItemClickMulti`, `PluginCommand` and
`PluginEvent` (`iapprobodk.h:244-297`), and there is no drop hook.

**Right-clicking a conveyor** offers *Conveyor properties…* and *Delete conveyor*. Deleting is
refused mid-run for the same reason marking a role is: the scene is cooked when a run starts.

**Deleting drives its three IO variables to zero before it forgets the conveyor**, and only then deletes
the item — whose products go with it, because they are its children. That order is the whole of it: a
station waiting on `<name> ready` would otherwise wait on a value nothing will ever write again. Deleting
it with RoboDK's own Delete key takes the same path, because the plugin notices on `EventChanged`
that a conveyor's item has gone.

> **`clearconveyors` is momentary now**, and that is a consequence rather than a defect. A conveyor is an
> item carrying its own description, so the plugin re-reads the station's conveyors on the next edit and
> a forgotten one comes straight back. To be rid of one, delete it.

Adoption is *requested* by `EventChanged` and done on the panel's 100 ms timer, never inside the event:
RoboDK delivers `EventChanged` synchronously from inside the API calls a step makes, and adopting a
conveyor is a string of further RoboDK calls with a mesh export among them.

## Seeing and editing one

**Double-click a conveyor in the tree** and its own properties window opens: its name, and every field its
parameters have. *Conveyor properties…* in its right-click menu does the same thing, and so does adding
one. A click in the 3D view selects it — RoboDK reports a double click only for items it hit-tests
itself, and it hit-tests nothing the plugin paints.

**One window per conveyor, not one that follows the selection.** A panel is about the item it was opened
on, so two conveyors can be compared side by side and neither changes under the operator when they click
somewhere else. Windows are keyed on the *item*, because a conveyor can be renamed from the very window
that maps to it. A closed one is kept and hidden, so reopening it is instant; one whose conveyor has been
deleted is destroyed, because its key is then a pointer RoboDK no longer owns.

The open windows are refreshed on the panel's own 100 ms timer. There is no selection-changed or
item-edited event in `IAppRoboDK` — the whole `TypeEvent` list is Render, Moved, Changed, ChangedStation,
About2Save, About2ChangeStation, About2CloseStation and TrajectoryStep — so a poll is what there is.

> **Whether a double click also stops RoboDK opening its own object dialog is not documented.**
> `ClickDouble` reaches the plugin through `PluginItemClick` (`iapprobodk.h:208`), whose contract in the
> header is "a new context menu is created for an item" and whose `bool` return is undocumented. The plugin
> returns `true` and claims the gesture; if RoboDK opens its dialog anyway, both windows appear and the
> context-menu route is the one to use.

**The panel is a rendering of `Common/AccessoryPropertySchema` and knows nothing about conveyors.** That
table describes each editable field once: its key, its label, its unit, its range, whether it is a
number, a choice, a toggle or a reference to something else in the cell, which generators and roles have
it, whether it may be touched while a run is going, and what it is coupled to. A field added there
appears in this panel *and* in RobotSimulator's without either being edited — which is the only honest
thing to share between an ImGui editor and a Qt one, because the widgets themselves cannot be.

**The ranges live there too.** They used to be buried inside `rebuildRollerConveyor`'s own clamping,
where neither UI could show them and a spin box's bounds were a second copy of numbers nobody could see.
The builder still clamps — that is a backstop and should be — but it asks the schema for each bound by
key, so a spin box and the geometry cannot disagree about what 6000 mm means.

An edit commits on Enter or focus-out, never per keystroke: adopting one re-declares the conveyor, which
re-exports its prototype's mesh to re-measure the workpiece. Everything is disabled unless the run is
stopped, because the scene is cooked when a run starts — and *that* is a property of the field, so the
schema carries it rather than each UI deciding.

**Two rows are the panel's own and not the schema's**, for the same reason in both cases. The **name** is
the RoboDK item's. The **placement** — X, Y, Z in mm and Rx, Ry, Rz in degrees, RoboDK's own
`XYZRPW_2_Mat` convention, the six numbers its own pose dialog shows — is the plugin's, because in a
CadNode station the *node* carries where an accessory stands and the accessory block is only the recipe;
a pose in the shared schema would be a second answer for RobotSimulator.

Those six are also **the only way to move a conveyor by hand other than dragging the frame it hangs
under**: RoboDK's own move tools reach an item's pose, and a conveyor's item is a generic node whose pose
RoboDK ignores. They are relative to that frame, so dragging it moves the conveyor and leaves all six
unchanged — which the rows say in a tooltip, because otherwise they read as wrong rather than as
relative. They stay editable mid-run, since the same move is available by dragging and a conveyor's
products are placed every step anyway.

A placement commits only when one of the six actually changed, and that is not an optimisation: the
boxes round to three decimals, so six numbers read back out of them do not rebuild the pose they came
from. Committing an unchanged focus-out would move the conveyor a little and mark the station dirty every
time an operator clicked through the panel.

No smoke can read a Qt window, so `moveconveyor` is the gate for these rows — the same call the panel
makes, in the same six numbers. `run_conveyor_smoke` moves the added conveyor to a stated pose, composes
that pose independently with `robomath`'s KUKA convention (the same `transl * rotz * roty * rotx` that
`XYZRPW_2_Mat` documents, so a check that asked the plugin would agree with any order it used), and
confirms the stored placement matches and the drawn lane went with it.

## Conveyor state as simulated IO

Free-running transport needs the program to wait for a box, and the station must not gain a script to do
the waiting. It does not need one: **`RDK.setParam` releases RoboDK's own *Wait for I/O* instruction**,
and `IRoboDK::setParam` is in the plugin interface. So the cell waits with a native instruction and the
plugin is what satisfies it.

`IItem::setDO` / `waitDI` / `setAO` are the names that *look* right and are not: they **emit instructions
into a program**. `setParam` is the live publish, and it is the only one.

**Every declared conveyor publishes its own state, and there is nothing to declare.** A conveyor named
`Box feed` writes three station variables:

```
Box feed ready   1 while a workpiece is resting at the end stop, ungrasped
Box feed full    1 while there is no room at the entry
Box feed count   how many workpieces the conveyor is carrying
```

- **`ready` is the rules' own rule 16**, so it is correct for any conveyor without anyone choosing a
  window: the rest position comes from the conveyor's own length and end inset. A conveyor whose far end
  *feeds another* is never ready — it has no end stop, and whatever is down there is passing through.
- **`full` is literally rule 18's own condition**, the one that withholds the spawn, so the report cannot
  drift from the behaviour. The same discipline `spawnerAtCap` follows.
- All three **exclude grasped products**, so `ready` goes dark the instant a tool takes the box and the
  next cycle waits again rather than walking straight through. It comes back when the queue behind
  advances into the slot, which on an accumulating line is fast — that is a real handshake, not a stale
  value, and `run_conveyor_smoke.py` checks both halves of it.
- Written **only when a value changes.** A station parameter set sixty times a second is sixty station
  edits a second for everything else watching the station.
- **0 whenever the run is not going,** including on `stop` and on `clearconveyors` — which zeroes a
  conveyor's variables *before* forgetting the conveyor that owned them. A station left waiting on a
  variable that stopped being written would wait for ever.

This replaces an invented `sensor=` entry that took a progress window, `0.99:1`. That was a
progress-window probe wearing a machine part's clothes: a debug query someone tuned, correct only for a
conveyor whose length happened to make that window one product long — and item 1 moved the products, at
which point it was correct for nothing. There is no `sensor=` key, no `sensor` verb and no `PhysicsWorld::Sensor`.
There is also no `NO SUCH CONVEYOR` diagnosis any more, which is the point: **a signal that cannot name a
conveyor that does not exist cannot be waited on for ever by mistake.**

`conveyors` reports one `io` record per conveyor with the three variables and their published values,
because a Wait against a variable stuck at 0 and a line that is merely slow look identical from outside.

## The spawner

Not modelled on RobotSimulator's conveyor spawner any more — it *is* RobotSimulator's, in
`Common/ConveyorCore`. Cadence, cap, transport, transfer and deletion are one implementation obeying
one set of rules in both cells:

- the interval is **added**, not assigned, so the cadence does not drift;
- the cap counts by **originating spawner**, not by what a product is parented to now;
- the prototype is **named**, never inferred from whatever the products land on;
- zero means unlimited.

The cap lets a line keep feeding as a pallet fills. It used to count only products the plugin
considered "carried"; now the core counts every product it owns, and the plugin stops owning one at the
moment a tool lets it go onto the stack. RobotSimulator retires a product with a deleter; here being put
down is what retires it. Same rule, different retirement.

**This showcase declares no cap at all** — accumulation makes the lane's own length the limit, which is
what a conveyor's capacity actually is. The cap stays a rule because it means something different: a
budget across the whole cell that follows a product wherever it goes, which RobotSimulator stations set
and the smoke below exercises.

**A cap that is refusing says so**, because a cap doing exactly what it was asked to and a line that
has broken look identical otherwise — and a capped spawner whose products can never leave the flow is a
deadlock. `status` appends `N spawner(s) at cap and not feeding`, and each `conveyors` record carries
`cap <live>/<max>`, marked `AT CAP, not feeding` while it is. With the product records under it, which
show a progress of `1.000000` that never changes, that is the whole diagnosis in one reply. It is a
statement of fact and not a warning: a spawner at cap while a box waits to be picked is a working line.

The test behind it asks the rules rather than repeating them — `Runtime::spawnerAtCap` is the same
expression `step()` uses to withhold a product, so the report cannot drift from the behaviour.

## Conveyors

RoboDK has no conveyor and no path — its showcase moves boxes by parenting them under a one-axis
mechanism — so a conveyor is declared rather than discovered. **A conveyor is a generic item carrying a
`PhysicsConveyor` parameter,** and that parameter is `TransformNodeData`'s own accessory recipe as JSON:
the same fields, the same keys and the same reader a RobotSimulator station uses for the same roller
conveyor. There is no second format. Two keys are the plugin's, because a RoboDK station has no other
way to say them:

| Key | Meaning |
| --- | --- |
| `next` | what this conveyor's far end feeds, by name. Empty is a **disconnected endpoint**, which stops the product where the path ends rather than dropping it — the same rule the CadNode cell keeps, and the one most likely to be "simplified" into a deletion |

That is the only one. A CadNode station finds what a conveyor feeds *geometrically* — an endpoint within
15 mm — and the plugin could now do the same, since it has the whole path. It deliberately does not: a
RoboDK conveyor's frame is placed by hand, and a lane that silently stopped feeding because someone
nudged a frame 20 mm is worse than a declaration that says what feeds what.

`spawnObjectId` names the prototype, which is how the accessory block names one anyway — by an id its
host resolves, and here a host resolves an object by name.

**Declaring one from a script** goes through the same parameter. The `conveyor` verb takes twelve
positional fields, bar-separated:

```
<conveyor item>|<length mm>|<speed mm/s>|<role>|<next>|<prototype>|<interval s>|<max active>|<end inset mm>|<pitch mm>|<deck width mm>|<deck height mm>
```

The first names the **conveyor**, because an item and a conveyor are one thing — there is no separate
name to disagree with it. The second is how far the lane runs rather than where it ends; the item's pose
says where it stands, and between them that is a whole path. `accessoryLengthMm` is **clamped to
600..6000 mm** where a measured frame distance was not, so a 300 mm lane silently becomes 600 — the
clamp is applied to the block the plugin *keeps*, so the lane a product rides and the deck that is drawn
cannot disagree about it.

Everything after the third is optional. `role` is `normal`, `spawner`,
`deleter` or `pick_feeder`. Fields nine and ten describe the queue the conveyor accumulates into and
default to the `70` / `90` that RobotSimulator's accessories carry; **left blank they take that default,
and a stated `0` stays a zero**, which is the difference between "I have no opinion" and "this lane's
stop is its end". The last two are appearance only — how wide the drawn deck is and how far it stands
above its own feet — and nothing in the rules reads either. It is a command surface rather than a
storage format: what it does is write that JSON onto the item.

A `spawner=` entry in `PhysicsAddIn` is the same rules with **no item**, and that is not a degenerate
case: it says the station carries these products and the plugin must not place them. It has no frames
and neither of its two references is the spawner — the prototype is a hidden object and the parent is
just where clones go — so there is nothing to select, draw or delete, and nothing for an item to be.
That is what the variant used to declare, while `Conv_Move_24X16` was still the transport. It still has
a cadence and a cap, because those are rules.

**A station written before conveyors were items still opens.** A `conveyor=` entry is read once, moved
onto the frame it named, and that frame is renamed to the conveyor's name; the entry is then dropped
from `PhysicsAddIn`, because leaving it would be a second description that the next open would write
back over whatever had been edited since. An entry whose frame already carries parameters is left alone
for the same reason.

**And a station written before a conveyor was a custom node still opens.** Any item that is not generic and
carries a `PhysicsConveyor` parameter is *stood up* when it is declared: a generic node takes its name and
its place in the tree, a child object takes its pose and its children — statically, so nothing moves — and
the old item is deleted. The description travels across as *bytes* rather than being regenerated, so a
conveyor that is stood up is byte-identical to the one that was not, and merely opening a station does not
mark it dirty. Declaring a conveyor onto a frame the cell made itself takes the same path, which is why
the `conveyor` command still names a frame and still works.

A conveyor whose parameters still carry an `endFrame` key is **refused, loudly**, and asks to be
regenerated. That key belonged to a build in which the conveyor's item stood at the *start* of its lane
rather than at its own placement — half a length back and a deck height below — so declaring one anyway
would put the whole conveyor and every box it delivers somewhere else. A shifted stack and a gripper
closing beside the box is not a failure worth being quiet about.

`run_conveyor_smoke.py` drives both halves against a disposable copy of the showcase: a capped
spawner into a dead end, where the products that stop there must **queue one pitch apart instead of
piling into the same point**, must keep the cap closed, and **both reports must say the cap is
refusing**; and the same feed into a deleter, where retiring them must keep the line producing. It
declares no inset or pitch, so the `70` / `90` defaults are what it measures.

## The command surface

Station parameters cover a cell that describes itself. For anything scripted, every verb below is
available through `RDK.PluginCommand('Physics Simulation', verb, value)` and answers `OK`,
`OK <detail>`, or the reason it could not.

| Verb | Value | Effect |
| --- | --- | --- |
| `start` / `pause` / `stop` | — | run control; `stop` restores every pose the run changed |
| `status` | — | one-line summary of the scene; the step count and simulated seconds are **per run** |
| `items` | — | the panel's per-item status column as one line, records separated by ` \|\| ` |
| `role` | `<item>\|static\|dynamic\|kinematic\|none` | describe one item |
| `release` / `attach` | `<item>` | force the handoff either way |
| `spawn` | `<prototype>[\|<parent>]` | one clone; replies `OK <new name>` |
| `spawner` | `<prototype>\|<parent>\|<interval s>\|<max active>` | a source with no path — the station carries its products |
| `conveyor` | twelve fields, see above | writes a conveyor's parameters onto its own item |
| `addconveyor` | — | one generic node with a default parameter block; replies `OK <name>` |
| `moveconveyor` | `<name>\|<x>\|<y>\|<z>\|<rx>\|<ry>\|<rz>` | where it stands, mm and degrees, relative to the frame its item hangs under — the same six numbers its properties panel edits |
| `deleteconveyor` | `<name>` | zeroes its three variables, forgets it, then deletes its node, which takes its products |
| `conveyors` | — | the declared conveyors, their caps, the IO each publishes, and every product, one line |
| `clearspawned` / `clearconveyors` | — | remove spawned products / conveyor configuration |
| `saveconfig` / `loadconfig` | — | write the current setup to `PhysicsAddIn`, or re-read it |

`saveconfig` is how a cell configured by hand in the panel becomes a station that configures itself.

## Checking that it runs

`run_physics_smoke.py` drives the whole cycle against this station in a hidden, disposable RoboDK,
and is what proved the plugin executes physics at all:

```powershell
py -3 .\scripts\robodk_physics_station\run_physics_smoke.py `
  "C:\Users\you\Downloads\PhysisPluginPalletizing showcase physics.rdk"
```

It spawns a box, parents it under the tool and out again — RoboDK's native Attach and Detach, which
is what the plugin actually watches — and then drops one box on the pallet and one with nothing
underneath it. Both halves are needed. A box that settles everywhere is resting on something the
cell does not have; one that settles nowhere never had collision at all; and one that settles at the
floor's height fell straight through the pallet. Only the three together say the pallet's cooked
mesh is what caught it.

`run_reset_smoke.py` drives the run control described above against the same station, and has to let
`Main` run to the end to make its last claim.

`run_showcase_smoke.py` is the one that holds the whole thing. It presses play and requires a stack:

```
Main finished after 34 s; 17 boxes produced, 'Box feed ready' came on 16 time(s)
  2 box(es) queued on the lane, none closer than 610 mm to another
  layers, bottom up: 5 box(es) at z=-392.3, 5 box(es) at z=-122.2, 5 box(es) at z=141.6
  15 boxes on the pallet in 3 layers
```

Counting `Box feed ready`'s **rising edges** is what says the robot waited rather than happened to find a
box there. A cell whose belt speed merely suits the robot's cycle time passes every other check in this
directory and is a race; this is the only one that can tell. Since the line accumulates, the signal's
dark window is one *pitch* of travel rather than a whole lane — under a second — so the sampling interval
is part of the check rather than an implementation detail. Placed boxes are told from in-flight ones by
whether the plugin gave them a body, which is exactly what the handoff means — the line keeps feeding
after `Main` ends, so the boxes still travelling are not a failure. What they must be is a **queue**:
the check reads the pitch the station declares and fails if two of them are closer than that, which is
the regression test for the coincident products the cap of one was hiding.

The drop is aimed from the item's exported geometry rather than its pose, because in this station
they are 600 mm apart: `Pallet 40inx48in` is parented into a frame that turns it ninety degrees, so
its `PoseAbs` is not on the pallet at all. Measuring it from RoboDK rather than asking the plugin is
deliberate — a plugin that placed its collision wrongly would aim the drop at the same wrong place.
