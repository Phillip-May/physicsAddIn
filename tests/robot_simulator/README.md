# RobotSimulator tests

`fixtures/` contains programs, calibration data, and other deterministic CLI regression inputs.
Synthetic asset profiles should be generated into a temporary test output directory rather than
placed in the built-in library. In particular, the AR4 `stress-high` limit profile remains available
through `scripts/convert_ar4_robot_package.py --limit-profile stress-high`, but is not a shipped
robot or a Library variant.

## Conveyor characterization traces

`golden/conveyor/` holds the committed output of `--dump-conveyor-trace`, checked by
`scripts/check_conveyor_traces.ps1`, which `scripts/build_robot_simulator.ps1` runs at the end of
every build. These traces are the integration coverage for transport over complete station files.
`scripts/build_motion_smokes.ps1` complements them with direct `conveyor_core_smoke` and
`conveyor_scenery_smoke` checks built from the current sources.

The trace prints, per tick, every product that appeared, was destroyed, or changed conveyor, plus
the full state of every product on a sampled tick. Product identity is `ConveyorWorkpiece::id`, so a
capture does not depend on heap addresses or on the order the scene's vector happens to be packed
in. `dirty=` is the number of times the tick raised the scene-graph hook.

Logical mode is goldened byte for byte; the per-conveyor `simulationMode` in the station file is
overridden so a station that asks for PhysX still produces a comparable capture. PhysX mode is not
goldened - it is not reproducible enough between machines - so those cases assert only the
invariants the command checks itself: no product outside 0..1 after a tick, no product on a
conveyor that is not in the station, no spawner over its cap.

### The fixture

`fixtures/conveyor_rules.station.json` is not a showcase. It exists because the two shipped
conveyor stations do not reach every rule:

| Rule | Reached by |
| --- | --- |
| A disconnected downstream interface stops the product, it is not a chute | `dead-end-run`, whose end touches nothing |
| `maxActiveSpawns` counts by originating spawner, not by current parent | `dead-end-feed`, capped at 3, whose three products stop at the dead end and must keep the cap closed |
| Transfer, deletion and cadence as the control | the `sink-feed` / `sink` line |

The two are coupled on purpose. A port that deletes at a disconnected end also frees the cap, so
the fixture then spawns for the rest of the run and the capture diverges by hundreds of lines
rather than by a rounding difference.

`examples/stations/conveyor_showcase.station.json` never reaches the disconnected-end rule - every
one of its five lines ends in a deleter. `examples/stations/fairino_gudel_machine_tending.station.json`
does reach it at the pick feeder's downstream end, and it is the only shipped cover for
`maxActiveSpawns`.

### Changing a golden

`scripts\check_conveyor_traces.ps1 -Update` rewrites them. That is only ever right when the rules
were meant to change. A refactor that moves this code is supposed to leave every byte alone.
