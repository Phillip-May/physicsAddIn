# AR4 Teensy 4.1 RTOS Status Firmware

This sketch is the small RTOS-backed hardware IO test for the AR4 controller.
It reports limit switch state, encoder state, limit polling rate, and basic
queue counters over USB serial as newline-delimited JSON.

Status: compiles clean; it has never been flashed to hardware. Everything below
describes behavior verified only by compilation and the desktop-side motion core
tests, not on a bench.

`TeensyRtosTsandmannSmoke/` is a minimal sketch proving the `freertos-teensy`
library produces USB serial output from RTOS tasks; it is not part of the AR4
firmware.

It uses the `freertos-teensy` Arduino library with `arduino_freertos.h`. Do not
switch it back to `FreeRTOS_TEENSY4.h`; that library compiled on this machine but
did not produce USB serial output from RTOS tasks.

## Prerequisites

The scripts expect Arduino CLI at
`%LOCALAPPDATA%\Programs\ArduinoCLI\arduino-cli.exe`; install it there or update the path at the top
of the build and upload scripts. Configure the PJRC Teensy package index, then install the pinned
core and library:

```powershell
$arduinoCli = "$env:LOCALAPPDATA\Programs\ArduinoCLI\arduino-cli.exe"
& $arduinoCli config add board_manager.additional_urls https://www.pjrc.com/teensy/package_teensy_index.json
& $arduinoCli core update-index
& $arduinoCli core install teensy:avr@1.62.0
& $arduinoCli lib install freertos-teensy@11.2.0-3
& $arduinoCli core list
& $arduinoCli lib list
```

Compilation does not require connected hardware. Upload and serial verification require a Teensy
4.1 connected by USB.

## Build

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\firmware\build_ar4_teensy41_rtos_status.ps1
```

Upload to the connected Teensy 4.1:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\firmware\upload_ar4_teensy41_rtos_status.ps1
```

The upload script auto-detects the Teensy upload port and serial COM port with
`teensy_ports.exe`, checks that the COM port can be opened before upload, and
uses an internal timeout so failed Teensy tooling does not leave stale
`arduino-cli`, `teensy_*`, or hardware-IO `RobotSimulator.exe` processes holding
the serial port.

The firmware build and upload scripts pass the repo `Common` directory to the
Teensy compiler so the sketch uses the shared `RobotMotionCore.h` planner rather
than carrying a separate copy of the MoveJ admission logic.

If the simulator GUI is open and connected to the hardware tab, close it first
or let the script close it:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\firmware\upload_ar4_teensy41_rtos_status.ps1 -CloseRobotSimulator
```

Verify through the simulator CLI:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\firmware\test_ar4_teensy41_rtos_status.ps1
```

The test script also auto-detects the COM port and kills any stale hardware-IO
status/monitor command before it starts.

Expected verification output starts with `robot_status` and includes
`limit_poll_period_ms=5` with a measured rate near `200 Hz`.

Encoder pins follow the AR4 7.0 firmware:

| Joint | A | B | Multiplier |
| --- | ---: | ---: | ---: |
| J1 | 14 | 15 | 5.0 |
| J2 | 17 | 16 | 5.0 |
| J3 | 19 | 18 | 5.0 |
| J4 | 20 | 21 | 5.0 |
| J5 | 23 | 22 | 2.5 |
| J6 | 24 | 25 | 5.0 |

The current diagnostic firmware does not use the Arduino `Encoder` library for
encoder feedback. It explicitly configures all 12 encoder input pins as
`INPUT_PULLUP` and reports their raw sampled state during each jog. The local
Encoder library used by the original AR4 sketch also configures encoder pins as
`INPUT_PULLUP`, so this keeps the same pin mode while removing the decoder
library from the test path.

Each jog also traces Teensy 4.1 Arduino digital pins `0..54`
(`NUM_DIGITAL_PINS`) and reports every pin that changed during the jog window.

Jog support is intentionally narrow and always starts disarmed after boot:

```json
{"cmd":"jog_arm","enabled":true}
{"cmd":"jog_joint","joint":1,"steps":50,"dir":1}
{"cmd":"jog_joint","joint":1,"steps":50,"dir":-1}
{"cmd":"jog_joint","joint":6,"steps":50,"dir":1}
{"cmd":"jog_joint","joint":6,"steps":50,"dir":-1}
{"cmd":"jog_disarm"}
```

The firmware rejects jogs unless armed and rejects commands above 500 steps.
J1-J6 use step pins 0/2/4/6/8/10 and direction pins 1/3/5/7/9/11. The pulse
polarity matches the normal AR4 motor move path: step idle HIGH, and each pulse
drives LOW for 30 us before returning HIGH.

MoveJ commands are also checked on the robot before any step pulse is emitted:

```json
{"cmd":"plan_movej","target_deg":[0,0,0,0,0,0],"speed_deg_s":10}
{"cmd":"movej","id":1,"target_deg":[0,0,0,0,0,0],"speed_deg_s":10}
```

`plan_movej` performs the same robot-side admission checks as `movej` without
moving. `movej` rejects non-finite targets, unmastered joints, bad calibration,
software joint limit violations, rounded step targets outside limits, active
limit switches in the commanded limit direction, and plans above the configured
tick budget. During execution it also stops if a limit switch becomes active in
the commanded limit direction. `movej` is queued by the firmware; the optional
`id` field is echoed in `movej_queued`, `movej_done`, `movej_rejected`, and
`movej_stopped`.

MoveL requires the host to load the package-derived robot motion model first.
The simulator sends this automatically after package load / serial connect:

```json
{"cmd":"load_robot_model","dhm":[...24...],"q_home":[...6...],"q_min":[...6...],"q_max":[...6...],"dhm_signs":[...6...],"tool_bind":[...12...]}
```

Then the host can dry-run the endpoint reachability check or execute a streamed
linear Cartesian move:

```json
{"cmd":"plan_movel","target_tcp":[...12...],"target_deg":[0,0,0,0,0,0],"speed_mm_s":25}
{"cmd":"movel","id":2,"target_tcp":[...12...],"target_deg":[0,0,0,0,0,0],"speed_mm_s":25}
```

`target_tcp` is the Cartesian line endpoint and `target_deg` is the intended
joint configuration for that same endpoint. `plan_movel` rejects TCP-only input
and also rejects a pose whose nearest IK solution does not match `target_deg`
within 0.5 degrees per joint. `movel` starts from that endpoint gate, then plans
each Cartesian sample in real time through the shared motion core. Each sample is
executed as a joint segment whose speed is derived from `speed_mm_s` and the
sample distance, still bounded by the firmware step clock and joint speed cap.
Samples that round to zero motor steps still consume their nominal sample time,
so step quantization does not collapse the Cartesian timeline.
It aborts on no IK solution, singularity proximity, segment limit failure, stop,
estop, or limit-switch activation. The firmware does not perform collision
checks; the simulator's collision preflight remains an additional host-side check.
`movel` is queued like `movej`; the optional `id` field is echoed in
`movel_queued`, `movel_done`, `movel_rejected`, and `movel_aborted`.

Motion queue behavior:

```json
{"cmd":"movej","id":10,"target_deg":[0,0,0,0,0,0],"speed_deg_s":10}
{"msg":"movej_queued","id":10,"depth":1}
{"msg":"movej_done","id":10,"code":"ok","ticks":1234}
```

The firmware stores queued move commands, not full sampled trajectories. The
motion queue holds 24 commands (`RobotMotionCore::kMotionLookaheadQueuedCommands`);
the serial command queue holds 8. If a queue is full, the command is rejected
with `queue_full`. `stop` is immediate-priority: it aborts the active move,
clears the command queue, clears the motion queue, and replies with `stop_ack`.

Multi-command motion programs stream through the same motion queue: messages
carrying program data are appended to the lookahead ring and executed by the
shared `RobotMotionCore::MotionWindowRunner` — the same planner windowing the
desktop simulator runs — with `program_final` marking the last message.

`set_limit_poll` adjusts the limit-switch polling period at runtime and
acknowledges with `set_limit_poll_ack`.
