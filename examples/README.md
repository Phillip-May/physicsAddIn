# Examples

`stations/` contains loadable RobotSimulator station examples. Example stations reference shipped
assets with `builtin:<asset-id>` and therefore do not carry or depend on sibling package ZIPs.
Per-instance data (calibration, motion settings) lives in `stations/instances/<id>/config.json`,
resolved relative to the station file — see `two_ar4_cell.station.json`.

Generated/library asset sources belong in `../library/sources`, and regression inputs belong in
`../tests/robot_simulator/fixtures`.
