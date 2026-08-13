from __future__ import annotations

import argparse
import math
import re
import sys
import time

from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_physics_smoke import (SmokeFailure, close_robodk, command, connect, conveyor_item,
                               conveyor_parameters, conveyor_placement, describe_rows, item_rows,
                               position_of, require_ok, world_aabb, DEFAULT_EXE, FEED_CONVEYOR,
                               PALLET_NAME, TOOL_NAME)

PROTOTYPE = 'BoxRef_24X16'
REPO_ROOT = Path(__file__).resolve().parents[2]
CONVEYOR_PACKAGE = 'roller_conveyor.zip'
# Somewhere clear of the cell, so the declared conveyors cannot touch the showcase's own geometry.
LANE_Y_MM = -4000.0
SEGMENT_MM = 2000.0
SPEED_MM_S = 500.0
# The conveyor rules' own queue defaults, left undeclared below so that is what is under test.
END_INSET_MM = 70.0
PITCH_MM = 90.0
LANE_CENTRE_TOLERANCE_MM = 5.0
# kConveyorRollerCoverSurfaceOffsetMm: how far a roller cover raises the surface a workpiece rests on.
ROLLER_COVER_SURFACE_OFFSET_MM = 8.0
DELIVERY_LIMIT_S = 10.0
PICK_ORIGIN_MM = [10.0, 550.0, 370.0]
SPAWNED = re.compile(r'^' + re.escape(PROTOTYPE) + r' #(\d+)$')


def frame_at(rdk, name: str, x: float, y: float, z: float):
    from robodk import robolink, robomath

    existing = rdk.Item(name)
    if existing.Valid() and existing.Type() == robolink.ITEM_TYPE_GENERIC:
        require_ok(rdk, 'deleteconveyor', name)
        existing = rdk.Item(name)
    frame = existing if existing.Valid() else rdk.AddFrame(name)
    frame.setPose(robomath.transl(x, y, z))
    return frame


def spawned_items(rdk) -> dict:
    """{serial: item} for every clone the plugin has produced and not yet retired."""
    from robodk import robolink

    found = {}
    for name in rdk.ItemList(robolink.ITEM_TYPE_OBJECT, True):
        match = SPAWNED.match(name)
        if match:
            found[int(match.group(1))] = rdk.Item(name, robolink.ITEM_TYPE_OBJECT)
    return found


def declare(rdk, conveyor, length, speed, role, nxt='', prototype='', interval='', cap=''):
    fields = [conveyor, str(length), str(speed), role, nxt, prototype, str(interval), str(cap)]
    require_ok(rdk, 'conveyor', '|'.join(fields))


def workpiece_centre_offset(rdk, name: str = PROTOTYPE) -> list:
    prototype = rdk.Item(name)
    low, high = world_aabb(rdk, prototype)
    centre = [(low[axis] + high[axis]) * 0.5 for axis in range(3)]
    return [c - p for c, p in zip(centre, position_of(prototype))]


def derived_lane_end(rdk, conveyor: str = FEED_CONVEYOR) -> list:
    parameters = conveyor_parameters(rdk, conveyor)
    pose = conveyor_placement(rdk, conveyor)
    origin = list(pose.Pos())
    along = [pose[row, 0] for row in range(3)]
    across = [pose[row, 1] for row in range(3)]
    up = [pose[row, 2] for row in range(3)]

    length = float(parameters['lengthMm'])
    deck = 0.5 * (float(parameters['endLeftHeightMm']) + float(parameters['endRightHeightMm']))
    if parameters.get('rollerCoverEnabled'):
        deck += ROLLER_COVER_SURFACE_OFFSET_MM
    sideways = sum(o * a for o, a in zip(workpiece_centre_offset(rdk), across))
    return [origin[axis] + along[axis] * length * 0.5 + up[axis] * deck - across[axis] * sideways
            for axis in range(3)]


def delivered_pose_agrees(rdk) -> None:
    """The conveyor delivers where its own pose and length say it does, to 0.000 mm."""
    print('--- the delivered pose, against the conveyor pose and length it comes from')
    require_ok(rdk, 'stop')
    require_ok(rdk, 'clearspawned')
    expected = derived_lane_end(rdk)
    require_ok(rdk, 'start')

    deadline = time.monotonic() + DELIVERY_LIMIT_S
    while time.monotonic() < deadline:
        if str(rdk.getParam(f'{FEED_CONVEYOR} ready')).strip().startswith('1'):
            break
        time.sleep(0.1)
    else:
        raise SmokeFailure(
            f"'{FEED_CONVEYOR} ready' never came on within {DELIVERY_LIMIT_S:.0f} s, so nothing was "
            f'ever delivered. conveyors: {command(rdk, "conveyors")!r}')

    # The leader: furthest along the lane, which is what `ready` is about.
    pose = conveyor_placement(rdk)
    along = [pose[row, 0] for row in range(3)]
    items = spawned_items(rdk)
    if not items:
        raise SmokeFailure('the conveyor reports a workpiece at its end stop and the station has none')
    leader = max((position_of(item) for item in items.values()),
                 key=lambda at: sum(a * b for a, b in zip(at, along)))
    error = sum((a - b) ** 2 for a, b in zip(leader, expected)) ** 0.5
    print(f'  derived  {[round(v, 3) for v in expected]}')
    print(f'  delivered {[round(v, 3) for v in leader]}, {error:.3f} mm apart')
    if error > 0.0005:
        raise SmokeFailure(
            f'the conveyor delivered to {[round(v, 3) for v in leader]} but its own pose and length '
            f'say {[round(v, 3) for v in expected]}, {error:.3f} mm out. The pick is a fixed pose with '
            'an Attach after it, so this is a gripper closing beside the box.')
    print('  the lane is the item pose and the length, and nothing else')

    off = sum((a - b) ** 2 for a, b in zip(leader, PICK_ORIGIN_MM)) ** 0.5
    if off > 0.5:
        raise SmokeFailure(
            f'the conveyor delivers to {[round(v, 3) for v in leader]}, {off:.3f} mm from '
            f'{PICK_ORIGIN_MM}, which is where `Pick_Conveyor 24X16` reaches. The conveyor itself has '
            'moved - the derivation above agrees with it because both read the same wrong pose.')
    print(f'  and it is where the vendor\'s pick target reaches, {off:.3f} mm from {PICK_ORIGIN_MM}')


def ready_is(rdk, conveyor: str, expected: int, because: str) -> None:
    """`<conveyor> ready` must be `expected`. Nothing declares it; the conveyor publishes it."""
    value = str(rdk.getParam(f'{conveyor} ready')).strip()
    if not value.startswith(str(expected)):
        raise SmokeFailure(
            f"'{conveyor} ready' is {value!r}, expected {expected}, because {because}. "
            f'conveyors: {command(rdk, "conveyors")!r}')


def declared_conveyors(rdk) -> list:
    names = []
    for record in command(rdk, 'conveyors').split(' || '):
        fields = record.split(' | ')
        if fields and fields[0].startswith('c ') and fields[0][2:].strip():
            names.append(fields[0][2:].strip())
    return names


def reset_run(rdk):
    require_ok(rdk, 'stop')
    require_ok(rdk, 'clearspawned')
    for name in declared_conveyors(rdk):
        require_ok(rdk, 'deleteconveyor', name)
    require_ok(rdk, 'clearconveyors')
    left = declared_conveyors(rdk)
    if left:
        raise SmokeFailure(f'conveyors still declared after deleting every one: {left}')


def dead_end_scenario(rdk) -> None:
    print('--- capped spawner into a dead end')
    reset_run(rdk)
    frame_at(rdk, 'smoke-feed', SEGMENT_MM * 0.5, LANE_Y_MM, 0.0)
    frame_at(rdk, 'smoke-run', SEGMENT_MM * 1.5, LANE_Y_MM, 0.0)
    declare(rdk, 'smoke-feed', SEGMENT_MM, SPEED_MM_S, 'spawner',
            nxt='smoke-run', prototype=PROTOTYPE, interval=1.0, cap=2)
    # No `next`, so its far end touches nothing.
    declare(rdk, 'smoke-run', SEGMENT_MM, SPEED_MM_S, 'normal')
    require_ok(rdk, 'start')

    # Four seconds a segment, so twelve is comfortably past both boxes reaching the dead end.
    time.sleep(13.0)
    items = spawned_items(rdk)
    print(f'live products: {sorted(items)}')
    if len(items) != 2:
        raise SmokeFailure(
            f'a cap of two produced {len(items)} live products ({sorted(items)}). Products that '
            'stop at a disconnected end must keep holding the cap closed.')

    end_x = SEGMENT_MM * 2.0
    across_y = LANE_Y_MM - workpiece_centre_offset(rdk)[1]
    resting = []
    for serial, item in sorted(items.items()):
        x, y, _ = position_of(item)
        print(f'  #{serial} at x={x:.1f}, y={y:.1f}')
        if abs(y - across_y) > LANE_CENTRE_TOLERANCE_MM:
            raise SmokeFailure(
                f'#{serial} rides at y={y:.1f}, where a body centred on a lane at y={LANE_Y_MM:.1f} '
                f'puts its origin at y={across_y:.1f}')
        resting.append(x)
    resting.sort(reverse=True)
    if abs(resting[0] - (end_x - END_INSET_MM)) > 5.0:
        raise SmokeFailure(
            f'the leader came to rest at x={resting[0]:.1f}, not {END_INSET_MM:.0f} mm short of the '
            f'dead end at x={end_x:.1f}. It either never transferred or did not stop where the '
            'queue says it should.')
    if abs((resting[0] - resting[1]) - PITCH_MM) > 5.0:
        raise SmokeFailure(
            f'the two boxes are {resting[0] - resting[1]:.1f} mm apart, not the {PITCH_MM:.0f} mm '
            'pitch. Products at a stop-gate that do not queue end up coincident, which is the '
            'defect a spawn cap of one was hiding.')
    print(f'  both transferred, the leader stopped {END_INSET_MM:.0f} mm short of the dead end, the '
          f'second queued {PITCH_MM:.0f} mm behind it, and the cap held')

    ready_is(rdk, 'smoke-run', 1, 'a workpiece is resting at the dead end')
    ready_is(rdk, 'smoke-feed', 0, 'its far end feeds smoke-run rather than stopping')
    if str(rdk.getParam('smoke-run full')).startswith('1'):
        raise SmokeFailure('a 2000 mm run holding two boxes is not full')
    if not str(rdk.getParam('smoke-run count')).startswith('2'):
        raise SmokeFailure(f"'smoke-run count' says {rdk.getParam('smoke-run count')!r}, not 2")

    from robodk import robolink

    tool = rdk.Item(TOOL_NAME, robolink.ITEM_TYPE_TOOL)
    if not tool.Valid():
        raise SmokeFailure(f'This station has no {TOOL_NAME!r} to grasp with')
    ordered = [item for _, item in sorted(items.items(),
                                          key=lambda entry: -position_of(entry[1])[0])]
    ordered[0].setParentStatic(tool)
    time.sleep(1.0)
    ready_is(rdk, 'smoke-run', 1, 'the next box in the queue took the slot the leader left')
    if not str(rdk.getParam('smoke-run count')).startswith('1'):
        raise SmokeFailure("'smoke-run count' should have dropped to 1: the grasped box is the "
                           f"tool's, not the conveyor's. It says {rdk.getParam('smoke-run count')!r}")

    ordered[1].setParentStatic(tool)
    time.sleep(1.0)
    ready_is(rdk, 'smoke-run', 0, 'the tool now has every box that was on the line')
    print("  ready is the conveyor's own end stop: it drops when a tool takes the box, and comes "
          'back only because the queue behind it advances')

    summary = command(rdk, 'status')
    conveyors = command(rdk, 'conveyors')
    print(f'  status:    {summary}')
    print(f'  conveyors: {conveyors}')
    if 'spawner(s) at cap' not in summary:
        raise SmokeFailure(
            f'the line has stopped producing and the summary does not say why: {summary!r}')
    if 'AT CAP' not in conveyors:
        raise SmokeFailure(f'no conveyor record reports its cap as refusing: {conveyors!r}')
    print('  and the cap is reported rather than left to look like a fault')


def deleter_scenario(rdk) -> None:
    print('--- the same feed into a deleter')
    reset_run(rdk)
    frame_at(rdk, 'smoke-feed', SEGMENT_MM * 0.5, LANE_Y_MM, 0.0)
    frame_at(rdk, 'smoke-run', SEGMENT_MM * 1.5, LANE_Y_MM, 0.0)
    declare(rdk, 'smoke-feed', SEGMENT_MM, SPEED_MM_S, 'spawner',
            nxt='smoke-run', prototype=PROTOTYPE, interval=1.0, cap=2)
    declare(rdk, 'smoke-run', SEGMENT_MM, SPEED_MM_S, 'deleter')
    require_ok(rdk, 'start')

    time.sleep(16.0)
    items = spawned_items(rdk)
    highest = max(items) if items else 0
    print(f'live products: {sorted(items)}, highest serial produced: {highest}')
    if len(items) > 2:
        raise SmokeFailure(f'the cap of two was exceeded: {sorted(items)}')
    if highest < 5:
        raise SmokeFailure(
            f'only {highest} products were ever produced. A deleter has to retire them so the cap '
            'stops binding; this looks like nothing is being deleted.')
    print('  the deleter kept retiring products and the line kept feeding')


MOVED_TO = [1500.0, -2500.0, 250.0, 0.0, 0.0, 30.0]


def moved_conveyor_stands_where_it_was_put(rdk, name: str) -> None:
    from robodk import robomath

    print('--- moved to a stated pose, which is what the panel\'s placement rows do')
    require_ok(rdk, 'moveconveyor', '|'.join([name] + [str(value) for value in MOVED_TO]))
    x, y, z, rx, ry, rz = MOVED_TO
    wanted = robomath.KUKA_2_Pose([x, y, z, rz, ry, rx])
    stored = conveyor_placement(rdk, name)
    worst = max(abs(stored[row, col] - wanted[row, col]) for row in range(3) for col in range(4))
    if worst > 1.0e-6:
        raise SmokeFailure(f'{name!r} was moved to {MOVED_TO} and stands at {stored}, {worst:.6f} out. '
                           'A placement that does not survive being written and read back is a conveyor '
                           'that quietly stands somewhere else.')
    print(f'  its stored pose is the one it was given, to {worst:.6f}')

    record = next((line for line in command(rdk, 'conveyors').split(' || ')
                   if line.startswith(f'c {name} |')), '')
    path = re.search(r'path ([-+\d.,eE]+) -> ([-+\d.,eE]+)', record)
    if not path:
        raise SmokeFailure(f'{name!r} reports no path after being moved: {record!r}')
    ends = [[float(value) for value in path.group(half).split(',')] for half in (1, 2)]
    centre = [(ends[0][axis] + ends[1][axis]) * 0.5 for axis in range(3)]
    if abs(centre[0] - x) > 0.001 or abs(centre[1] - y) > 0.001:
        raise SmokeFailure(f'{name!r} stands at {x}, {y} and its lane runs about {centre[0]:.3f}, '
                           f'{centre[1]:.3f}. What is drawn and what products ride have come apart.')
    print(f'  and its lane runs about that pose, {centre[0]:.1f}, {centre[1]:.1f}, {centre[2]:.1f}')


def add_and_delete_scenario(rdk) -> None:
    from robodk import robolink

    print('--- one added and one deleted')
    reset_run(rdk)
    name = require_ok(rdk, 'addconveyor').split(' ', 1)[1].strip()
    print(f'added {name!r}')
    if f'c {name} |' not in command(rdk, 'conveyors'):
        raise SmokeFailure(f'{name!r} was added and is not declared: {command(rdk, "conveyors")!r}')
    # Conveyor JSON persists as per-item RoboDK data.
    added = rdk.Item(name, robolink.ITEM_TYPE_GENERIC)
    if not added.Valid():
        raise SmokeFailure(f'{name!r} was added without a generic item to be. A conveyor is a *custom* '
                           'tree node, not an object: an object answers RoboDK\'s own object commands and '
                           'opens its own dialog on a double click, which is not what a conveyor is.')
    parameters = conveyor_parameters(rdk, name)
    if parameters.get('generator') != 'roller_conveyor':
        raise SmokeFailure(f'{name!r} carries {parameters!r}, which is not a conveyor')
    print(f'  its own item carries {len(parameters)} parameters, so a save keeps it')
    record = next((line for line in command(rdk, 'conveyors').split(' || ')
                   if line.startswith(f'c {name} |')), '')
    drawn = re.search(r'drawn (\d+) tri', record)
    if not drawn or int(drawn.group(1)) <= 0:
        raise SmokeFailure(f'{name!r} draws nothing: {record!r}. A conveyor is one custom node and what '
                           'stands in the cell is what the plugin paints, so nothing drawn is nothing '
                           'to see or click.')
    print(f'  and {drawn.group(1)} triangles for the plugin to draw and hit-test it with')
    moved_conveyor_stands_where_it_was_put(rdk, name)

    rdk.setParam(f'{name} ready', '1')
    require_ok(rdk, 'deleteconveyor', name)
    if f'c {name} |' in command(rdk, 'conveyors'):
        raise SmokeFailure(f'{name!r} was deleted and is still declared: '
                           f'{command(rdk, "conveyors")!r}')
    if added.Valid(check_deleted=True):
        raise SmokeFailure(f'{name!r} was deleted and its item is still in the station')
    ready = str(rdk.getParam(f'{name} ready')).strip()
    if not ready.startswith('0'):
        raise SmokeFailure(f"'{name} ready' reads {ready!r} after the conveyor was deleted. Nothing "
                           'will write it again, so a native Wait on it would wait for ever.')
    print('  deleted: no conveyor, no item, and its ready signal driven to 0 before it was forgotten')


def fed_over(rdk, seconds: float) -> int:
    before = set(spawned_items(rdk))
    time.sleep(seconds)
    return len(set(spawned_items(rdk)) - before)


def choosing_a_workpiece_sizes_the_queue(rdk) -> None:
    print('--- choosing a workpiece sizes the queue pitch to it')
    reset_run(rdk)
    frame_at(rdk, 'smoke-fit', SEGMENT_MM * 0.5, LANE_Y_MM, 0.0)
    declare(rdk, 'smoke-fit', SEGMENT_MM, SPEED_MM_S, 'spawner', interval=1.0, cap=3)
    before = float(conveyor_parameters(rdk, 'smoke-fit')['initialWorkpieceSpacingMm'])

    require_ok(rdk, 'conveyorworkpiece', f'smoke-fit|{PROTOTYPE}')
    sized = float(conveyor_parameters(rdk, 'smoke-fit')['initialWorkpieceSpacingMm'])
    low, high = world_aabb(rdk, rdk.Item(PROTOTYPE))
    measured = high[0] - low[0]
    if abs(sized - measured) > 1.0:
        raise SmokeFailure(
            f'choosing {PROTOTYPE!r} set the queue pitch to {sized:.3f} mm and the box measures '
            f'{measured:.3f} mm along the lane. A pitch that is not the workpiece\'s own footprint is a '
            'queue of solids standing inside one another.')
    if sized <= before + 1.0:
        raise SmokeFailure(f'the pitch was {before} mm and choosing a workpiece left it at {sized} mm, so '
                           'nothing was sized at all')
    print(f'  pitch went from {before:.1f} mm to {sized:.3f} mm, and the box measures {measured:.3f} mm')

    require_ok(rdk, 'start')
    time.sleep(14.0)
    riding = spawned_items(rdk)
    if len(riding) < 3:
        raise SmokeFailure(f'only {len(riding)} product(s) to measure an accumulated queue with')
    ordered = sorted(position_of(item)[0] for item in riding.values())
    gaps = [second - first for first, second in zip(ordered, ordered[1:])]
    tightest = min(gaps)
    if tightest < measured - 1.0:
        raise SmokeFailure(
            f'the nearest two products stand {tightest:.3f} mm apart and the box is {measured:.3f} mm '
            f'long, so they overlap by {measured - tightest:.3f} mm. All gaps: {gaps}')
    if tightest > measured * 1.5:
        raise SmokeFailure(f'the tightest gap is {tightest:.3f} mm for a {measured:.3f} mm box, so this '
                           f'queue never accumulated and proves nothing about the pitch. Gaps: {gaps}')
    print(f'  and an accumulated queue stands {tightest:.3f} mm apart for a {measured:.3f} mm box, '
          'so none is inside another')
    require_ok(rdk, 'stop')
    require_ok(rdk, 'clearspawned')

    require_ok(rdk, 'conveyorworkpiece', f'smoke-fit|{PROTOTYPE}')
    unchanged = float(conveyor_parameters(rdk, 'smoke-fit')['initialWorkpieceSpacingMm'])
    if abs(unchanged - sized) > 1.0:
        raise SmokeFailure(f'choosing the *same* workpiece again moved the pitch from {sized} to '
                           f'{unchanged}, so it resizes on every edit rather than on a change')
    print('  and applying an edit that does not change the workpiece leaves the pitch where it is')

    pallet_low, pallet_high = world_aabb(rdk, rdk.Item(PALLET_NAME))
    require_ok(rdk, 'conveyorworkpiece', f'smoke-fit|{PALLET_NAME}')
    resized = float(conveyor_parameters(rdk, 'smoke-fit')['initialWorkpieceSpacingMm'])
    if abs(resized - (pallet_high[0] - pallet_low[0])) > 1.0:
        raise SmokeFailure(
            f'switched to {PALLET_NAME!r}, which measures {pallet_high[0] - pallet_low[0]:.3f} mm along '
            f'the lane, and the pitch is {resized:.3f} mm')
    print(f'  switched to a different workpiece and the pitch followed, to {resized:.3f} mm')

    # A fresh conveyor starts with the default pitch before workpiece sizing is applied.
    reset_run(rdk)
    frame_at(rdk, 'smoke-floor', SEGMENT_MM * 0.5, LANE_Y_MM, 0.0)
    declare(rdk, 'smoke-floor', SEGMENT_MM, SPEED_MM_S, 'spawner',
            prototype=PROTOTYPE, interval=1.0, cap=3)
    floored = float(conveyor_parameters(rdk, 'smoke-floor')['initialWorkpieceSpacingMm'])
    if abs(floored - measured) > 1.0:
        raise SmokeFailure(
            f'a conveyor declared with the 90 mm default and a {measured:.3f} mm workpiece kept a pitch of '
            f'{floored:.3f} mm. A pitch shorter than the workpiece is two solids in one place, so it is a '
            'floor and not a preference.')
    print(f'  a pitch of 90 mm under a {measured:.3f} mm box was floored to {floored:.3f} mm')

    wide = measured + 300.0
    require_ok(rdk, 'conveyor', '|'.join(
        ['smoke-floor', str(SEGMENT_MM), str(SPEED_MM_S), 'spawner', '', PROTOTYPE, '1.0', '3', '70',
         str(wide)]))
    kept_wide = float(conveyor_parameters(rdk, 'smoke-floor')['initialWorkpieceSpacingMm'])
    if abs(kept_wide - wide) > 1.0:
        raise SmokeFailure(f'a deliberate {wide:.3f} mm pitch was changed to {kept_wide:.3f} mm, so the '
                           'floor is overriding a choice rather than refusing an impossibility')
    print(f'  and a deliberate {wide:.1f} mm pitch is left alone')

    settled = conveyor_parameters(rdk, 'smoke-floor')

    declare(rdk, 'smoke-floor', SEGMENT_MM, SPEED_MM_S * 0.5, 'spawner',
            prototype=PROTOTYPE, interval=2.0, cap=4)
    after = conveyor_parameters(rdk, 'smoke-floor')
    for key in ('initialWorkpieceEndInsetMm', 'initialWorkpieceSpacingMm', 'widthMm', 'heightMm'):
        if abs(float(after[key]) - float(settled[key])) > 1.0:
            raise SmokeFailure(
                f're-declaring the conveyor with a different speed and rate changed {key} from '
                f'{settled[key]} to {after[key]}. A declaration that says nothing about a number has to '
                'leave it alone, the way it already does for the placement.')
    print('  and a re-declaration that states neither leaves the queue and the deck as they were')

    turning_the_source_box_is_noticed(rdk)


def turning_the_source_box_is_noticed(rdk) -> None:
    from robodk import robomath

    print('--- the source box turned after it was measured, which nothing reports')
    prototype = rdk.Item(PROTOTYPE)
    was = prototype.Pose()
    try:
        reset_run(rdk)
        frame_at(rdk, 'smoke-turn', SEGMENT_MM * 0.5, LANE_Y_MM, 0.0)
        declare(rdk, 'smoke-turn', SEGMENT_MM, SPEED_MM_S, 'spawner',
                prototype=PROTOTYPE, interval=1.0, cap=3)
        before = world_aabb(rdk, prototype)
        endOn = before[1][0] - before[0][0]
        sized = float(conveyor_parameters(rdk, 'smoke-turn')['initialWorkpieceSpacingMm'])
        if abs(sized - endOn) > 1.0:
            raise SmokeFailure(f'setting up: the pitch is {sized:.3f} mm for a {endOn:.3f} mm box')

        prototype.setPose(was * robomath.rotz(math.pi / 2.0))
        turned = world_aabb(rdk, prototype)
        broadside = turned[1][0] - turned[0][0]
        if broadside <= endOn + 1.0:
            raise SmokeFailure(f'turning the box took it from {endOn:.3f} to {broadside:.3f} mm along the '
                               'lane, so it needs no more room and this proves nothing - it must not be '
                               'square in plan')

        pitch = sized
        for _ in range(60):
            rdk.Command('Wait', '50')
            pitch = float(conveyor_parameters(rdk, 'smoke-turn')['initialWorkpieceSpacingMm'])
            if pitch >= broadside - 1.0:
                break
        if abs(pitch - broadside) > 1.0:
            raise SmokeFailure(
                f'the box was turned from {endOn:.3f} to {broadside:.3f} mm along the lane and the queue '
                f'pitch is {pitch:.3f} mm. `placeProduct` reads the prototype\'s attitude live on every '
                'tick, so the products turn at once and a stale pitch is an overlap on screen.')
        print(f'  turned from {endOn:.3f} to {broadside:.3f} mm along the lane, and the pitch followed '
              f'to {pitch:.3f} mm')
    finally:
        prototype.setPose(was)
        for _ in range(40):
            rdk.Command('Wait', '50')
            if not turning_pending(rdk, prototype, was):
                break


def turning_pending(rdk, prototype, wanted) -> bool:
    """Whether the prototype is not yet back where it was, to a rounding error."""
    now = prototype.Pose()
    return any(abs(now[row, col] - wanted[row, col]) > 1e-9 for row in range(3) for col in range(4))


def a_conveyor_can_become_a_spawner_after_it_exists(rdk) -> None:
    print('--- a conveyor that becomes a spawner after it was declared, and stops being one')
    reset_run(rdk)
    frame_at(rdk, 'smoke-late', SEGMENT_MM * 0.5, LANE_Y_MM, 0.0)
    # Declared with no role and no workpiece, exactly as `Add Conveyor` leaves one.
    declare(rdk, 'smoke-late', SEGMENT_MM, SPEED_MM_S, 'normal')
    # Then edited into a spawner, which is the re-declaration path.
    declare(rdk, 'smoke-late', SEGMENT_MM, SPEED_MM_S, 'spawner',
            prototype=PROTOTYPE, interval=1.0, cap=4)
    record = next((line for line in command(rdk, 'conveyors').split(' || ')
                   if line.startswith('c smoke-late |')), '')
    if '| spawner |' not in record:
        raise SmokeFailure(f'it was re-declared as a spawner and does not report as one: {record!r}')

    require_ok(rdk, 'start')
    fed = fed_over(rdk, 6.0)
    if fed < 2:
        raise SmokeFailure(
            f'a conveyor edited into a spawner produced {fed} product(s) in 6 s at a 1 s cadence. It '
            'reports as a spawner and feeds nothing, which is a role registered on the path that creates '
            f'a conveyor and not on the path that edits one.\n    {record}')
    print(f'  edited into a spawner after the fact, it fed {fed} products in 6 s')

    declare(rdk, 'smoke-late', SEGMENT_MM, SPEED_MM_S, 'normal')
    still = fed_over(rdk, 4.0)
    if still:
        raise SmokeFailure(f'it was made a normal conveyor and produced {still} more product(s), so a '
                           'role that has been taken away is still in force')
    print('  and unmade, it stopped feeding while its products kept riding')

    require_ok(rdk, 'stop')
    require_ok(rdk, 'clearspawned')
    reset_run(rdk)
    require_ok(rdk, 'role', f'{PROTOTYPE}|static')
    try:
        frame_at(rdk, 'smoke-heir', SEGMENT_MM * 0.5, LANE_Y_MM, 0.0)
        declare(rdk, 'smoke-heir', SEGMENT_MM, SPEED_MM_S, 'normal')
        require_ok(rdk, 'start')
        inherited = fed_over(rdk, 4.0)
        if inherited:
            raise SmokeFailure(
                f'a plain conveyor took a deleted spawner\'s slot and produced {inherited} product(s). A '
                'ConveyorId is an index into the segment list, so an entry left behind revives as '
                'whatever conveyor is added next.')
        print('  a plain conveyor in the deleted spawner\'s slot fed nothing')
    finally:
        require_ok(rdk, 'stop')
        require_ok(rdk, 'role', f'{PROTOTYPE}|none')
        require_ok(rdk, 'clearspawned')


def products_by_conveyor(rdk) -> dict:
    """{conveyor name: [product id]}, from the plugin's own `conveyors` report."""
    found = {}
    for record in command(rdk, 'conveyors').split(' || '):
        match = re.match(r'p (\d+) \| on (.+?) \|', record.strip())
        if match:
            found.setdefault(match.group(2), []).append(int(match.group(1)))
    return found


def products_with_progress(rdk) -> dict:
    """{product id: (conveyor name, progress)}, for asking whether one advanced rather than stuck."""
    found = {}
    for record in command(rdk, 'conveyors').split(' || '):
        match = re.match(r'p (\d+) \| on (.+?) \| progress ([-+\d.eE]+)', record.strip())
        if match:
            found[int(match.group(1))] = (match.group(2), float(match.group(3)))
    return found


def a_chain_placed_end_to_end_feeds_without_being_told(rdk) -> None:
    print('--- two lanes end to end, with nothing saying what feeds what')
    reset_run(rdk)
    frame_at(rdk, 'smoke-up', SEGMENT_MM * 0.5, LANE_Y_MM, 0.0)
    frame_at(rdk, 'smoke-down', SEGMENT_MM * 1.5, LANE_Y_MM, 0.0)
    declare(rdk, 'smoke-up', SEGMENT_MM, SPEED_MM_S, 'spawner',
            prototype=PROTOTYPE, interval=1.5, cap=2)
    declare(rdk, 'smoke-down', SEGMENT_MM, SPEED_MM_S, 'normal')
    for name in ('smoke-up', 'smoke-down'):
        stated = str(conveyor_parameters(rdk, name).get('next', '')).strip()
        if stated:
            raise SmokeFailure(f"{name!r} states that it feeds {stated!r}, so this would pass on the "
                               'stated link and prove nothing about finding one')
    print('  neither lane names a successor')

    require_ok(rdk, 'start')
    # Four seconds a lane, so twelve is comfortably past the first product crossing the join.
    crossed = {}
    for _ in range(48):
        time.sleep(0.25)
        crossed = products_by_conveyor(rdk)
        if crossed.get('smoke-down'):
            break
    if not crossed.get('smoke-down'):
        raise SmokeFailure(
            f'nothing reached the second lane in 12 s: {crossed}. Two lane ends that meet are one line, '
            'and a plug-in that only reads a typed-in name cannot see that - which is why a chain built by '
            'snapping in the 3D view stood still.')
    print(f'  a product crossed the join with nothing declaring it: {crossed}')
    require_ok(rdk, 'stop')
    require_ok(rdk, 'clearspawned')
    a_product_rides_round_a_corner(rdk)


def a_conveyor_offers_its_end_to_be_connected_to(rdk) -> None:
    print('--- a conveyor added from the menu offers its end to be connected to')
    reset_run(rdk)
    require_ok(rdk, 'libraryroot', str(REPO_ROOT / 'library' / 'packages'))
    added = require_ok(rdk, 'addconveyor').split(' ', 1)[1].strip()
    for path, name in (('the menu', added),):
        if str(conveyor_parameters(rdk, name).get('endStopEnabled', '')).lower() in ('true', '1'):
            raise SmokeFailure(f'{name!r}, added from {path}, has its end stop on - which closes the very '
                               'interface anything downstream has to mate to')
    print(f'  {added!r} was created with its end open')

    require_ok(rdk, 'conveyor', '|'.join(
        [added, '1800', '250', 'normal', '', '', '1', '0', '70', '90', '700', '890']))
    if str(conveyor_parameters(rdk, added).get('endStopEnabled', '')).lower() in ('true', '1'):
        raise SmokeFailure(f'declaring {added!r} through the `conveyor` verb turned its end stop on')
    reply = command(rdk, 'librarysnap', f'{CONVEYOR_PACKAGE}|straight-890|{added}|start|end')
    if not reply.startswith('OK'):
        raise SmokeFailure(f'nothing could be mated onto {added!r}: {reply}')
    print(f'  and something mated onto it: {reply}')

    reset_run(rdk)
    frame_at(rdk, 'smoke-rest', SEGMENT_MM * 0.5, LANE_Y_MM, 0.0)
    declare(rdk, 'smoke-rest', SEGMENT_MM, SPEED_MM_S, 'spawner',
            prototype=PROTOTYPE, interval=1.0, cap=1)
    require_ok(rdk, 'start')
    resting = 0.0
    for _ in range(40):
        time.sleep(0.5)
        found = [progress for name, progress in products_with_progress(rdk).values()
                 if name == 'smoke-rest']
        if found and max(found) > 0.9:
            resting = max(found)
            break
    inset = float(conveyor_parameters(rdk, 'smoke-rest')['initialWorkpieceEndInsetMm'])
    expected = 1.0 - inset / SEGMENT_MM
    if abs(resting - expected) > 1e-3:
        raise SmokeFailure(f'with no end stop the leader rests at {resting:.6f} and rule 16 says '
                           f'{expected:.6f}, so the barrier was doing the stopping after all')
    print(f'  and with no barrier its leader still rests at {resting:.6f}, which is rule 16')
    require_ok(rdk, 'stop')
    require_ok(rdk, 'clearspawned')


def a_product_rides_round_a_corner(rdk) -> None:
    print('--- a corner snapped onto a straight lane, and a product carried round it')
    require_ok(rdk, 'libraryroot', str(REPO_ROOT / 'library' / 'packages'))
    for variant in ('corner-left-90', 'corner-right-90'):
        reset_run(rdk)
        feed = require_ok(rdk, 'librarysnap', f'{CONVEYOR_PACKAGE}|object-spawner||'
                          ).removeprefix('OK').split('|')[0].strip()
        # Somewhere awkward and turned, so a frame error cannot cancel against the station's own axes.
        require_ok(rdk, 'moveconveyor', f'{feed}|1500|-2500|400|0|0|25')
        require_ok(rdk, 'conveyorworkpiece', f'{feed}|{PROTOTYPE}')
        corner = require_ok(rdk, 'librarysnap', f'{CONVEYOR_PACKAGE}|{variant}|{feed}|start|end'
                            ).removeprefix('OK').split('|')[0].strip()
        if corner == feed:
            raise SmokeFailure(f'{variant} mated onto itself')

        require_ok(rdk, 'start')
        onto = {}
        for _ in range(60):
            time.sleep(0.5)
            onto = {pid: place for pid, place in products_with_progress(rdk).items()
                    if place[0] == corner}
            if onto:
                break
        if not onto:
            raise SmokeFailure(
                f'nothing handed over onto the {variant} in 30 s: {products_with_progress(rdk)}. Its lane '
                f'end meets {feed!r} to 0.000 mm - `mounting_snap_smoke` and the library smoke hold the '
                'mate itself - so a product resting at the inset means the hand-over search is reading '
                'those ends in the wrong frame.')
        # Handed over is not carried: the curve has to advance it, or it has joined and stuck at the seam.
        before = min(progress for _, progress in onto.values())
        time.sleep(4.0)
        after = [progress for name, progress in products_with_progress(rdk).values()
                 if name == corner]
        if not after or max(after) <= before + 0.01:
            raise SmokeFailure(f'a product joined the {variant} at progress {before:.6f} and is at '
                               f'{after} four seconds later, so it stuck at the seam')
        print(f'  {variant}: handed over at {before:.6f} and rode on to {max(after):.6f}')
        require_ok(rdk, 'stop')
        require_ok(rdk, 'clearspawned')


def a_spawner_alone_is_a_whole_cell(rdk) -> None:
    print('--- a spawner and nothing marked at all, which is still a whole cell')
    reset_run(rdk)
    marked = sorted(item_rows(rdk))
    for name in marked:
        require_ok(rdk, 'role', f'{name}|none')
    left = item_rows(rdk)
    if left:
        raise SmokeFailure(f'roles were cleared and the plugin still holds participants:\n'
                           f'{describe_rows(left)}')
    print(f'  unmarked {len(marked)} participant(s): nothing in this cell has a physics role')

    frame_at(rdk, 'smoke-alone', SEGMENT_MM * 0.5, LANE_Y_MM, 0.0)
    declare(rdk, 'smoke-alone', SEGMENT_MM, SPEED_MM_S, 'spawner',
            prototype=PROTOTYPE, interval=1.0, cap=3)
    reply = command(rdk, 'start')
    if not reply.startswith('OK'):
        raise SmokeFailure(
            f'a spawner conveyor with nothing marked would not start: {reply}\n'
            '    A conveyor carries its products by pose and a carried product has no body, so an empty '
            'participant list is not an empty cell.')

    products = {}
    for _ in range(60):
        time.sleep(0.5)
        products = spawned_items(rdk)
        if len(products) >= 2:
            break
    if len(products) < 2:
        raise SmokeFailure(
            f'it started and fed {len(products)} product(s) in 30 s at a 1 s cadence: '
            f'{sorted(products)}. Starting without spawning is the guard relaxed in the wrong place.')
    print(f'  it started with an empty participant list and fed {len(products)} products: '
          f'{sorted(products)}')

    require_ok(rdk, 'stop')
    require_ok(rdk, 'clearspawned')
    reset_run(rdk)
    refusal = command(rdk, 'start')
    if refusal.startswith('OK'):
        raise SmokeFailure('a station with nothing marked and no conveyor at all still started')
    if 'spawn' not in refusal.lower() or 'physics role' not in refusal.lower():
        raise SmokeFailure(f'the refusal names only one of the two ways out of it: {refusal}')
    print(f'  and an empty cell is still refused: {refusal}')
    # Put the showcase's own cell back, because everything after this expects it.
    require_ok(rdk, 'loadconfig')


def run(rdk) -> None:
    if not command(rdk, 'status'):
        raise SmokeFailure('The physics plugin did not answer.')
    require_ok(rdk, 'loadconfig')
    if not item_rows(rdk):
        raise SmokeFailure('The station parameter declared no participants to start a run from.')
    # First, and against the showcase's own conveyor, because the scenarios below clear it.
    delivered_pose_agrees(rdk)
    dead_end_scenario(rdk)
    deleter_scenario(rdk)
    add_and_delete_scenario(rdk)
    choosing_a_workpiece_sizes_the_queue(rdk)
    a_conveyor_can_become_a_spawner_after_it_exists(rdk)
    a_chain_placed_end_to_end_feeds_without_being_told(rdk)
    a_conveyor_offers_its_end_to_be_connected_to(rdk)
    # Last, because it unmarks the showcase's whole cell to get at the guard and puts it back afterwards.
    a_spawner_alone_is_a_whole_cell(rdk)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('station', type=Path)
    parser.add_argument('--robodk', type=Path, default=DEFAULT_EXE)
    parser.add_argument('--port', type=int, default=20600)
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
    print('PASS: the RoboDK host transports, transfers, caps, stops at a dead end, deletes products, '
          'adds and removes a conveyor, and feeds a cell whose only mechanism is the conveyor itself.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
