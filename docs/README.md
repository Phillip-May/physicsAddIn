# Design docs

- [architecture.md](architecture.md) — the two applications, what `Common/` shares and what it cannot,
  build constraints, and the RoboDK API behaviour that is not obvious from the headers.
- [conveyors.md](conveyors.md) — the transport rules, the accessory-instance model, and how a
  station runs without Python.
- [library-placement.md](library-placement.md) — the snap solver, frames, placed items, and articulated
  robot packages.
- [testing.md](testing.md) — gates, what each smoke holds, golden traces, and driving RoboDK headlessly.

Build instructions are in [BUILDING.md](../BUILDING.md).
