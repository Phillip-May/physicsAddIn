# Verification

## Gates

Core gates for a change that touches the simulation:

```powershell
.\scripts\build_robot_simulator.ps1 -Configuration Release   # also runs the golden conveyor traces
.\scripts\build_motion_smokes.ps1                            # the tools/ smokes
.\scripts\build_robodk_plugin.ps1                            # close RoboDK, or pass -NoDeploy
```

Run the target-specific build as well when its boundary changes:

```powershell
.\scripts\build_standalone.ps1 -Configuration Release -Deploy  # QtCadViewer / Qt / OCCT
.\scripts\build_wasm.ps1                                       # portable Common / web target
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\firmware\build_ar4_teensy41_rtos_status.ps1
```

Against a physics variant of the palletizing showcase:

```
py -3 scripts\robodk_physics_station\run_physics_smoke.py   <station>
py -3 scripts\robodk_physics_station\run_conveyor_smoke.py  <station>
py -3 scripts\robodk_physics_station\run_reset_smoke.py     <station>
py -3 scripts\robodk_physics_station\run_showcase_smoke.py  <station>
py -3 scripts\robodk_physics_station\run_library_smoke.py   <station>
```

`run_showcase_smoke.py` holds the whole claim. It plays `Main` and requires a stack, and takes minutes
rather than seconds. The other four each hold one mechanism.

## What each smoke holds

| Smoke | Claim |
| --- | --- |
| `conveyor_core_smoke` | the transport rules |
| `conveyor_scenery_smoke` | drawn conveyor geometry, so a moved bound shows up here and not on screen |
| `mounting_snap_smoke` | the snap solver: reach, hole matching, quarter-steps, mate-without-cursor |
| `view_ray_smoke` | project and unproject, both directions, to `0.000000 mm` |
| `library_catalogue_smoke` | what is in the catalogue and which rows arm |
| `motion_core_*_smoke` | planner windows, weave schedules, endpoint config |
| `*_equivalence` | a shared implementation against the one it replaced |
| `run_physics_smoke` | carry, release, and a box coming to rest on another |
| `run_conveyor_smoke` | transport, transfer, caps, stops, deletes, delivered pose to `0.000 mm` |
| `run_reset_smoke` | a run is replaced rather than inherited, and simulated time never goes backwards |
| `run_showcase_smoke` | `Main` finishes and 15 boxes are stacked in 3 layers |
| `run_library_smoke` | package and generated rows arm; placed mechanisms draw, move, copy, reopen, snap, and delete cleanly |

## Observation surface

`PluginCommand` verbs `status`, `items` and `conveyors` are how you see what the plugin thinks.
`conveyors` is the RoboDK equivalent of `--dump-conveyor-trace` and reports each declared conveyor,
whether its path resolves right now, and every product's conveyor, progress and direction.

**A stalled run and a working run look identical without those verbs.** That cost hours once. Use them
first.

`summary()`'s step count and simulated seconds are per run, zeroed by `start()`. They were monotonic
across the session, which made a restart indistinguishable from a run that had carried on.

## Golden traces

`tests/robot_simulator/golden/conveyor/` holds three logical traces, byte-compared.
`scripts/check_conveyor_traces.ps1` drops a diverging capture beside its golden as `*.actual` so it can
be diffed by hand. That file is evidence for one run and is gitignored.

Nothing that is not a rule change may move a golden. A golden that moves means a range or a default
moved with it.

## Driving RoboDK headlessly

```python
robolink.Robolink(args=['-NEWINSTANCE', '-HIDDEN', '-EXIT_LAST_COM'],
                  robodk_path=..., quit_on_close=False)
```

then `AddFile`. **Do not pass `-SKIPINI`.** With it, `AddFile` on the palletizing showcase never
returns; without it the same station opens in 0.3 s.

Every smoke runs a hidden, disposable RoboDK and never saves.

In `-HIDDEN` there is no `MainGL`, so there is no viewport and no camera. Anything on the mouse path is
unreachable, which is why frame conversions are gated through verbs that skip the projection.

## Known flakiness

Both are pre-existing and neither has been chased. Re-run before believing a failure.

- `run_reset_smoke`, roughly 1 in 4: "simulated time went backwards during Main", a nested call read as
  a play, in the 25 ms `Busy()` poll.
- `run_showcase_smoke`: "Invalid item provided" while listing spawned boxes.

## What no gate covers

Nothing here reads pixels. Someone has to look at whether the ghost reads as a ghost at 55% alpha,
whether the blue target and yellow source hole guides are legible against RoboDK's grid in both themes,
whether the snap engages where the pointer expects at 1.5% of the shorter side, and whether the dock's
greyed rows read as "not for this" rather than as broken.

That blind spot is what let `RenderUpdateOnly` hide every drawn change in the plugin until someone
dragged a library row and saw nothing.
