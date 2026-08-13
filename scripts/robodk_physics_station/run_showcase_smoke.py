from __future__ import annotations

import argparse
import re
import sys
import time

from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_physics_smoke import (SmokeFailure, close_robodk, command, connect, conveyor_item,
                               conveyor_parameters, conveyor_placement, item_rows, position_of,
                               require_ok, world_aabb, DEFAULT_EXE, FEED_CONVEYOR, PALLET_NAME,
                               SPAWN_PROTOTYPE)

MAIN_PROGRAM = 'Main'
READY_PARAM = f'{FEED_CONVEYOR} ready'
MAIN_LIMIT_S = 360.0
# After the last Drop_Box: the boxes let go during the final cycles are still settling.
SETTLE_S = 8.0
BOXES_EXPECTED = 15
POLL_S = 0.05
BOX_SCAN_S = 1.0
LANE_OVERHANG_MM = 5.0
LAYER_TOLERANCE_MM = 60.0
SPAWNED = re.compile(r'^' + re.escape(SPAWN_PROTOTYPE) + r' #(\d+)$')


def declared_pitch(rdk) -> float:
    pitch = conveyor_parameters(rdk).get('initialWorkpieceSpacingMm')
    if not pitch:
        raise SmokeFailure(f'{FEED_CONVEYOR!r} declares no queue pitch. Without one this check '
                           'cannot tell a queue from a pile.')
    return float(pitch)


def spawned_boxes(rdk) -> dict:
    """{serial: item} for every box the plugin has produced and not retired."""
    from robodk import robolink

    found = {}
    for item in rdk.ItemList(robolink.ITEM_TYPE_OBJECT, False):
        match = SPAWNED.match(item.Name())
        if match:
            found[int(match.group(1))] = item
    return found


def layers_of(heights: list) -> list:
    """`heights` grouped into layers, as [(height, count)] from the bottom up."""
    grouped = []
    for height in sorted(heights):
        if grouped and height - grouped[-1][-1] <= LAYER_TOLERANCE_MM:
            grouped[-1].append(height)
        else:
            grouped.append([height])
    return [(sum(group) / len(group), len(group)) for group in grouped]


def play_main(rdk) -> None:
    from robodk import robolink

    program = rdk.Item(MAIN_PROGRAM, robolink.ITEM_TYPE_PROGRAM)
    if not program.Valid():
        raise SmokeFailure(f'This station has no {MAIN_PROGRAM!r} program')
    program.RunProgram()

    started = time.monotonic()
    highest_serial = 0
    waits = 0
    eye_was = None
    next_scan = 0.0
    while program.Busy() and time.monotonic() - started < MAIN_LIMIT_S:
        eye = str(rdk.getParam(READY_PARAM)).strip()
        if eye_was is not None and eye != eye_was and eye.startswith('1'):
            waits += 1
        eye_was = eye
        now = time.monotonic()
        if now >= next_scan:
            highest_serial = max([highest_serial] + list(spawned_boxes(rdk)))
            next_scan = now + BOX_SCAN_S
        time.sleep(POLL_S)

    running = program.Busy()
    elapsed = time.monotonic() - started
    print(f'{MAIN_PROGRAM} {"hit the time limit" if running else "finished"} after {elapsed:.0f} s; '
          f'{highest_serial} boxes produced, {READY_PARAM!r} came on {waits} time(s)')
    print(f'  status:    {command(rdk, "status")}')
    print(f'  conveyors: {command(rdk, "conveyors")}')
    if running:
        program.Stop()
        raise SmokeFailure(
            f'{MAIN_PROGRAM} was still running after {MAIN_LIMIT_S:.0f} s. Either the line stopped '
            'delivering and the wait never cleared, or a pick missed and the cell is stuck.')
    if waits < BOXES_EXPECTED - 1:
        raise SmokeFailure(
            f'{READY_PARAM!r} only came on {waits} times for {BOXES_EXPECTED} picks. The robot is '
            'not waiting for boxes, so this cell is a race that happens to be winning.')


def check_the_stack(rdk) -> None:
    pallet = rdk.Item(PALLET_NAME)
    pallet_low, pallet_high = world_aabb(rdk, pallet)
    print(f'pallet occupies x {pallet_low[0]:.0f}..{pallet_high[0]:.0f}, '
          f'y {pallet_low[1]:.0f}..{pallet_high[1]:.0f}, top z {pallet_high[2]:.1f}')

    rows = item_rows(rdk)
    placed = {}
    in_flow = {}
    for serial, box in sorted(spawned_boxes(rdk).items()):
        row = rows.get(f'{SPAWN_PROTOTYPE} #{serial}')
        if not row:
            raise SmokeFailure(f'#{serial} is in the station and not a participant at all')
        (placed if row[3] == 'body' else in_flow)[serial] = (position_of(box), row)

    for serial, ((x, y, z), row) in sorted(in_flow.items()):
        print(f'  #{serial} is still in the flow at {x:.0f}, {y:.0f}, {z:.1f} | {row}')
    # Measure queue spacing and lane containment from each box mesh.
    from robodk import robolink

    node = conveyor_item(rdk)
    if node.Type() != robolink.ITEM_TYPE_GENERIC:
        raise SmokeFailure(f'{node.Name()!r} is type {node.Type()}, not a generic item. A conveyor read '
                           'from an older station must be stood up, or its old frame is still in the tree.')
    pose = conveyor_placement(rdk)
    stands = list(pose.Pos())
    print(f'  stood up as a custom node standing at '
          f'{stands[0]:.0f}, {stands[1]:.0f}, {stands[2]:.0f} mm')
    half = 0.5 * float(conveyor_parameters(rdk)['lengthMm'])
    lane_low, lane_high = sorted((stands[1] - pose[1, 0] * half, stands[1] + pose[1, 0] * half))
    for serial in sorted(in_flow):
        low, high = world_aabb(rdk, spawned_boxes(rdk)[serial])
        if low[1] < lane_low - LANE_OVERHANG_MM or high[1] > lane_high + LANE_OVERHANG_MM:
            raise SmokeFailure(
                f'#{serial} spans y {low[1]:.0f}..{high[1]:.0f}, off a lane that runs '
                f'{lane_low:.0f}..{lane_high:.0f}. A box that is not on the conveyor it is riding '
                'was placed by an origin that is not where its body is.')
    print(f'  every box in the flow is on the lane, y {lane_low:.0f}..{lane_high:.0f}')

    pitch = declared_pitch(rdk)
    along = sorted(position[1] for position, _ in in_flow.values())
    for behind, ahead in zip(along, along[1:]):
        if ahead - behind < pitch - 5.0:
            raise SmokeFailure(
                f'two boxes in the flow are {ahead - behind:.1f} mm apart along the lane, not the '
                f'{pitch:.0f} mm pitch the station declares: {sorted(in_flow)}. Boxes that overlap '
                'make the Attach take whichever is nearest rather than the one at the gate.')
    if in_flow:
        print(f'  {len(in_flow)} box(es) queued on the lane, none closer than {pitch:.0f} mm '
              'to another')

    off_pallet = {}
    for serial, ((x, y, z), row) in placed.items():
        if not (pallet_low[0] - 200.0 <= x <= pallet_high[0] + 200.0 and
                pallet_low[1] - 200.0 <= y <= pallet_high[1] + 200.0 and
                z > pallet_high[2] - 50.0):
            off_pallet[serial] = (x, y, z)
            print(f'  #{serial} was let go but is NOT on the pallet: {x:.0f}, {y:.0f}, {z:.1f}')
    if off_pallet:
        raise SmokeFailure(
            f'{len(off_pallet)} released boxes are not on the pallet: {sorted(off_pallet)}. They were '
            'dropped somewhere the stack is not, or they fell off it.')
    if len(placed) != BOXES_EXPECTED:
        raise SmokeFailure(
            f'{len(placed)} boxes came to rest on the pallet, not {BOXES_EXPECTED}. Main places '
            f'{BOXES_EXPECTED}, so a pick found nothing to take.')

    layers = layers_of([position[2] for position, _ in placed.values()])
    print('  layers, bottom up: ' + ', '.join(f'{count} box(es) at z={height:.1f}'
                                              for height, count in layers))
    if len(layers) < 3:
        raise SmokeFailure(
            f'the fifteen boxes rest at {len(layers)} height(s), not the three layers Main builds. '
            'They are not stacking on each other.')
    print(f'  {len(placed)} boxes on the pallet in {len(layers)} layers')


def run(rdk) -> None:
    if not command(rdk, 'status'):
        raise SmokeFailure('The physics plugin did not answer.')
    require_ok(rdk, 'loadconfig')
    if f'{READY_PARAM}=' not in command(rdk, 'conveyors'):
        raise SmokeFailure(
            f'no conveyor publishes {READY_PARAM!r}: {command(rdk, "conveyors")!r}. Without it the '
            'pick cannot wait, so this station is not the one this check is for.')
    play_main(rdk)
    print(f'settling for {SETTLE_S:.0f} s')
    time.sleep(SETTLE_S)
    check_the_stack(rdk)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('station', type=Path, help='the physics variant .rdk')
    parser.add_argument('--robodk', type=Path, default=DEFAULT_EXE)
    parser.add_argument('--port', type=int, default=20611)
    args = parser.parse_args()

    station: Path = args.station.resolve()
    if not station.is_file():
        print(f'No such station: {station}', file=sys.stderr)
        return 1

    rdk = connect(args.robodk, station, args.port)
    try:
        run(rdk)
    except SmokeFailure as failure:
        print(f'FAIL: {failure}', file=sys.stderr)
        return 1
    finally:
        try:
            command(rdk, 'stop')
        except Exception:
            pass
        close_robodk(rdk)
    print('PASS: the showcase fed itself, waited for each box, and built a stack.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
