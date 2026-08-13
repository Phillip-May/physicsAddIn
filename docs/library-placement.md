# Library placement

Dragging a package out of the library and snapping it onto another object's mounting holes is one
implementation shared by both cells.

| Piece | Where |
| --- | --- |
| Interfaces a package offers, surfaces a scene offers, the snap search | `Common/MountingSnap` |
| Armed-placement state machine | `Common/PlacementSession` |
| Catalogue contents | `Common/LibraryCatalogue` |
| Tree to draw groups | `Common/CadNodeDraw` |
| Project / unproject | `Common/ViewRay`, behind a `View` interface |
| The list widget, thumbnails, the drag gesture | per host |
| Delivery | per host |

The only things written twice are a list widget, a thumbnail renderer, a mouse gesture and a commit.
Everything numeric is one implementation.

The projection is an interface rather than `viewray` used directly. `MeshRobotViewer` implements it
with its own matrices and the plugin implements it with `viewray::Camera`, which is what made moving
the solver provably behaviour-neutral for RobotSimulator. It is an abstract base rather than a
`std::function` because the inner loop projects every target point of every surface.

## What a placement delivers into

RobotSimulator writes a `StationDocument` entry and a pose controller.

The plugin writes **one generic custom item** carrying `PhysicsLibraryItem` JSON: the package
reference, its preset, and the same parent-relative `pose` key a conveyor carries. It is drawn by the
plugin, hit-tested by the plugin's own ray, and outlined by its own silhouette.

A generic node contributes identity to the pose chain without breaking it, so a placed item stores a
parent-relative pose and its children ride along.

An object would be drawn, picked and dragged by RoboDK for free, but an object also answers RoboDK's
own object dialog and commands, and a library item is this plugin's. One mechanism for conveyors and
library items beats two.

**The conveyor is the first library item.** It needs no package, its generated tree already carries
both a foot pattern (`placementSource`) and an end grid on each connection transform (`mateOpposite`,
`interfaceId` `"start"`/`"end"`, a `deckHeight` binding), and it is already drawn and picked. So a
conveyor is both a snap source and a snap target for free, and conveyor-end-to-conveyor-end snapping
was the first thing gated, before any package was loaded.

## Frames

Three frames matter and confusing them is the recurring bug here.

`solve` answers a pose mapping the package's **own** frame to world. A placed item stores a
**parent-relative** pose. A conveyor's generated tree is one **accessory frame** further out.

So a snapped commit is `poseLocal = parentWorld⁻¹ · solvedPose` for a package, and
`parentWorld⁻¹ · solvedPose · conveyorAccessoryFrame()⁻¹` for a conveyor. Getting this wrong reads fine
and lands a conveyor a quarter turn out with every gate still self-consistent. It is the same shape as
the bug that put the showcase's conveyor 1.2 m away.

`PlacedItem` carries both `sceneryAt` and `treeToWorld` for this reason. Drift detection wants the
placement; anything that puts the *tree* in station coordinates — a mounting interface, a snap target —
wants the other, and the difference is a quarter turn.

A rail is a further case. `BuildMechanism` gives a mechanism a base frame of its own and parents it
there, so for a rail the item's parent is its own placement, and composing the stored placement onto it
again would place the rail by its own offset twice.

## Picking a mate without a cursor

Proximity in millimetres is not enough. Four of a conveyor's end-to-end mates satisfy every hole, and
the nearest one to an object standing at the origin is the *reversed* conveyor lying back along the one
it was joined to.

`mateWithoutACursor` states the rule: a declared connection beats a general pattern, the wheel stays
where it was, and of what is left the mate that leaves the two objects' placement patterns farthest
apart wins. Two solids cannot stand in the same place, which is what a cursor supplies implicitly by
being pointed at the space beside a machine. Naming both interface ids overrides all of it.

`mountingsnap::mate` is `solve`'s inner loop with the pixel acquisition test lifted out. Both go
through the same `matingAlignment` / `anchoredAt` / `countMatchedHoles`, and the two land on the same
pose to the generated geometry's own precision. They do not agree bit for bit because which anchor pair
wins is tiebroken on screen distance by one, and there is no screen for the other.

The retained mate holds **which end is mated**, not merely which object. Asked cold at the same cursor
and half turn, the solver snaps to the same target through the conveyor's other end with every hole
matched. Rotating about the free-drag centre swings the near end a lane length upstream and brings the
far end to the joint, so the cheaper answer is the wrong one.

## Placed robots articulate

A robot package is drawn geometry driven by `Common/RobotRuntime`, the same kinematics RobotSimulator
poses arms with. It is not a RoboDK robot: RoboDK programs cannot target it and RoboDK's jog panel will
not drive it.

| Route | What you get | What you give up |
| --- | --- | --- |
| Drawn only | geometry as scenery | a frozen arm that looks like it should move |
| **Drawn, driven by `RobotRuntime`** | joints that move, from the same code RobotSimulator uses | not a RoboDK robot |
| `BuildMechanism` | a genuine RoboDK robot: programs, targets, jogging | RoboDK wants a DH formulation plus limits, senses and home joints; packages carry a *geometric* axis formulation, so the conversion reads fine and is subtly wrong |

Route 2 is a stepping stone route 3 would throw away. That is accepted: a cell that needs its robot to
run a RoboDK program still imports it with File-Open. This is for the arm that is scenery with a pose —
reach studies, layout, showing a cell before its programs exist.

Joints live in the item's own JSON beside the `builtin:<id>` reference, so a station reopens with the
arm where it was left. Default is the package's home, not zeros, because a zeroed arm is a shape no
vendor authored. `bakePlacedItem` poses before it flattens.

**Hold the controller, rebind only on reload.** `bindToRobot` reads each link's current `loc` as its
home bind pose, so rebinding a posed arm takes the posed transforms for the home ones and compounds the
next move on top of them.

**`setJoints` mutates the tree the snap targets were collected from.** A robot posed after arming must
have its interfaces re-collected, or a mounting hole is offered where the arm no longer is.

Every row offered for pointer placement must resolve to a tree with a `placementSource`. The shipped
robot and rail packages declare those sources and can be armed from the dock. A package without one
is refused with a message directing the caller to place it at a stated pose instead.

Parametric conveyor variants are also armed from the dock. `armLibraryEntry` expands their parameter
tree through `buildConveyorScenery`, then the normal conveyor commit path creates the accessory.

## Out of scope

- **RoboDK's own objects as snap targets.** They carry no mounting-hole metadata, and inventing holes
  from a mesh is a different project. Targets are the plugin's own placed items, which includes every
  conveyor, plus the floor. The dock says so, because a pedestal that will not snap to an imported
  table otherwise reads as a bug.
- **The station document.** RobotSimulator records a placement in `StationDocument`, RoboDK records it
  on the item, and RoboDK saves items. Nothing is shared and nothing needs to be.

## Traps

- **The ghost is transformed per frame; a committed item is baked once.** A placed item's triangles are
  baked in station coordinates at its placement and re-baked only when it moves. Baking the ghost per
  mouse move would re-flatten a package tree at pointer rate.
- **The preview must not touch the station.** Nothing is created and no parameter is written until
  commit. A ghost that created an item would be undoable only by deleting it, and a cancelled drag
  would leave litter.
- **Two event filters on `MainGL`, and only one is permissive.** `PlacedItemPicker` deliberately does
  not consume a press, so RoboDK's own picking and orbiting keep working. The placer must consume, and
  suspends the picker while armed, or the click that commits also orbits the view and selects whatever
  is behind the ghost.
- **The snap threshold is in pixels**, so it depends on a camera the plugin cannot fully read. Wrong by
  a factor and it still snaps near the middle of the view. The measured reach for a directional end is
  23 px against a declared 24, held by `mounting_snap_smoke`, so a plugin that snaps at a wildly
  different distance has the camera wrong, not the solver.
- **Composing an identity onto a frame does not return the frame bit for bit**, because a negative zero
  comes out positive. A child that states no placement of its own keeps its parent's frame rather than
  composing.
- **A snapped conveyor end should set `next`.** `parameterBindings` already carries `deckHeight` on
  those interfaces. Mating `end` to `start` declares what feeds what, which is the one conveyor key no
  geometry can derive.
