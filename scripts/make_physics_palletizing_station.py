from __future__ import annotations

import argparse
import shutil
import struct
import sys
import tempfile

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_EXE = Path(r'C:\RoboDK\bin\RoboDK.exe')
DEFAULT_PORT = 20598

TOOL_NAME = 'RobotiQ EPick Four'
PALLET_NAME = 'Pallet 40inx48in'
FLOOR_NAME = 'Floor'
PICK_PROGRAM = 'Pick_Box_24X16'
SPAWN_CALL = 'New_Box_24X16'
SPAWN_PROTOTYPE = 'BoxRef_24X16'
SPAWN_FRAME = 'New_Box 24X16'
SPAWN_PARENT = 'Conveyor Box'
INIT_PROGRAM = 'Init_24X16'
INDEX_CALL = 'Conv_Move_24X16'
BELT_MECHANISM = 'Conveyor Belt'
CONVEYOR_BASE = 'Conveyor Belt Base'

# Conveyor item name and IO-variable prefix.
FEED_CONVEYOR = 'Box feed'
FEED_SPEED_MM_S = 700.0
FEED_INTERVAL_S = 1.0
# Zero leaves capacity governed by the conveyor's physical length.
FEED_MAX_ACTIVE = 0
# The taught pick pose is at the lane endpoint, so this station has no end inset.
FEED_END_INSET_MM = 0.0
# Extra width for the side frames around the measured workpiece.
DECK_RAIL_MM = 150.0
DEFAULT_DECK_HEIGHT_MM = 890.0

# Published by the named conveyor while an ungrasped workpiece rests at its end stop.
READY_PARAM = f'{FEED_CONVEYOR} ready'

MAIN_PROGRAM = 'Main'
# Superseded by plugin-managed spawning, transport, initialization, and cleanup.
VENDOR_SCRIPTS = ('New_Box_24X16', 'New_Box_12X10', 'Conv_Move_24X16', 'Conv_Move_12X10',
                  'Conv_Init_Position', 'Clear_Conveyor', 'Clear_Pallet', 'Clear_Tool')
# Main does not call this unserved 12x10 program. Delete its program before its referenced targets.
PICK_12X10_PROGRAM = 'Pick_Box_12X10'
PICK_12X10_TARGETS = ('AppConveyor 12X10', 'Pick_Conveyor 12X10')

RESET_PROGRAM = 'Reset'
# Keep Reset busy long enough for the 25 ms run-control poll to observe its rising edge.
RESET_PAUSE_MS = 500

CONFIG_PARAM = 'PhysicsAddIn'


def connect(exe: Path, station: Path, port: int):
    from robodk import robolink

    rdk = robolink.Robolink(
        port=port,
        args=['-NEWINSTANCE', '-HIDDEN', '-EXIT_LAST_COM'],
        robodk_path=str(exe),
        quit_on_close=False)
    rdk.TIMEOUT = 30
    opened = rdk.AddFile(str(station))
    if not opened.Valid():
        close_robodk(rdk)
        raise RuntimeError(f'RoboDK could not load {station}')
    return rdk


def close_robodk(rdk) -> None:
    """Shut the disposable instance down, releasing its Item handles immediately."""
    process = getattr(rdk, 'NEW_INSTANCE', None)
    try:
        rdk.CloseRoboDK()
    except Exception:
        pass
    if process is None:
        return
    try:
        process.wait(timeout=8.0)
    except Exception:
        try:
            process.kill()
        except Exception:
            pass


def clear_leftover_workpieces(rdk) -> list:
    from robodk import robolink

    gone = []
    for frame_name, kind in ((SPAWN_PARENT, robolink.ITEM_TYPE_FRAME),
                             ('Pallet', robolink.ITEM_TYPE_FRAME),
                             (TOOL_NAME, robolink.ITEM_TYPE_TOOL)):
        parent = rdk.Item(frame_name, kind)
        if not parent.Valid():
            continue
        for child in parent.Childs():
            if child.Type() != robolink.ITEM_TYPE_OBJECT:
                continue
            if child.Name().lower().startswith('pallet'):
                continue
            gone.append(f'{child.Name()!r} from {frame_name!r}')
            child.Delete()
    return gone


def separation(here, there) -> float:
    return sum((a - b) ** 2 for a, b in zip(here, there)) ** 0.5


def derive_lane(rdk):
    """Derive lane endpoints from the prototype, creation, and conveyor-base frames."""
    from robodk import robolink

    prototype = rdk.Item(SPAWN_PROTOTYPE, robolink.ITEM_TYPE_OBJECT)
    base = rdk.Item(CONVEYOR_BASE, robolink.ITEM_TYPE_FRAME)
    creation = rdk.Item(SPAWN_FRAME, robolink.ITEM_TYPE_FRAME)
    for name, item in ((SPAWN_PROTOTYPE, prototype), (CONVEYOR_BASE, base), (SPAWN_FRAME, creation)):
        if not item.Valid():
            raise RuntimeError(f'This station has no {name!r}, so the lane cannot be derived')

    start = prototype.PoseAbs()
    travel = [b - c for b, c in zip(base.PoseAbs().Pos(), creation.PoseAbs().Pos())]
    end = robolink.robomath.Mat(start)
    end.setPos([s + t for s, t in zip(start.Pos(), travel)])

    length = sum(t * t for t in travel) ** 0.5
    if length < 1.0:
        raise RuntimeError(f'{CONVEYOR_BASE!r} and {SPAWN_FRAME!r} are {length:.3f} mm apart, so the '
                           'lane has no length and nothing would ever arrive')
    if separation(end.Pos(), base.PoseAbs().Pos()) >= separation(start.Pos(), base.PoseAbs().Pos()):
        raise RuntimeError('the derived lane runs away from the conveyor base rather than towards '
                           'it, so these frames do not describe this conveyor')
    print(f'lane derived from static frames: {[round(v, 2) for v in start.Pos()]} -> '
          f'{[round(v, 2) for v in end.Pos()]}, {length:.1f} mm')
    return start, end


def mesh_bounds(rdk, item):
    export = Path(tempfile.gettempdir()) / 'physics_station_measure.stl'
    export.unlink(missing_ok=True)
    rdk.Save(str(export), item)
    if not export.is_file():
        raise RuntimeError(f'RoboDK exported no geometry for {item.Name()!r}')
    data = export.read_bytes()
    export.unlink(missing_ok=True)
    triangles = struct.unpack('<I', data[80:84])[0] if len(data) >= 84 else 0
    if triangles == 0 or len(data) != 84 + triangles * 50:
        raise RuntimeError(f'{item.Name()!r} did not export as binary STL; cannot measure it')
    low = [float('inf')] * 3
    high = [float('-inf')] * 3
    for index in range(triangles):
        base = 84 + index * 50 + 12
        for corner in range(3):
            vertex = struct.unpack('<3f', data[base + corner * 12:base + corner * 12 + 12])
            for axis in range(3):
                low[axis] = min(low[axis], vertex[axis])
                high[axis] = max(high[axis], vertex[axis])
    return low, high


def extent_along(low, high, direction) -> float:
    """How far a box of those bounds reaches along `direction`, which for an axis is that side."""
    return sum(abs(direction[axis]) * (high[axis] - low[axis]) for axis in range(3))


def measure_product_pitch(rdk, prototype, direction) -> float:
    low, high = mesh_bounds(rdk, prototype)
    pitch = round(extent_along(low, high, direction), 1)
    if pitch <= 0.0:
        raise RuntimeError(f'{prototype.Name()!r} measures {pitch:.3f} mm along the lane')
    print(f'queue pitch: {pitch:.1f} mm, the footprint of {prototype.Name()!r} along the lane')
    return pitch


def measure_deck(rdk, prototype, along, lane_height_mm):
    low, high = mesh_bounds(rdk, prototype)
    across = [abs(along[1]), abs(along[0]), 0.0]  # the horizontal direction the lane is not
    span = sum(across)
    across = [component / span for component in across] if span > 1.0e-9 else [0.0, 1.0, 0.0]
    width = extent_along(low, high, across) + 2.0 * DECK_RAIL_MM

    from robodk import robolink

    floor = rdk.Item(FLOOR_NAME, robolink.ITEM_TYPE_OBJECT)
    height = DEFAULT_DECK_HEIGHT_MM
    if floor.Valid():
        floor_low, floor_high = mesh_bounds(rdk, floor)
        top = floor.PoseAbs().Pos()[2] + floor_high[2]
        if lane_height_mm - top > 100.0:
            height = lane_height_mm - top
    width = round(width, 1)
    height = round(height, 1)
    print(f'deck: {width:.1f} mm wide, standing {height:.1f} mm below the lane')
    return width, height


def unit(vector) -> list:
    length = sum(component * component for component in vector) ** 0.5
    if length < 1.0e-9:
        raise RuntimeError(f'{vector} has no direction')
    return [component / length for component in vector]


def cross(a, b) -> list:
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def dot(a, b) -> float:
    return sum(x * y for x, y in zip(a, b))


def workpiece_centre_offset(rdk, prototype) -> list:
    low, high = mesh_bounds(rdk, prototype)
    pose = prototype.PoseAbs()
    centre = pose * [(low[axis] + high[axis]) * 0.5 for axis in range(3)]
    offset = [c - p for c, p in zip(centre, pose.Pos())]
    print(f'workpiece body sits {[round(v, 1) for v in offset]} from its own origin')
    return offset


def add_conveyor(rdk, start_pose, end_pose, deck_height_mm, offset):
    from robodk import robolink

    lane = [e - s for e, s in zip(end_pose.Pos(), start_pose.Pos())]
    length = round(sum(component * component for component in lane) ** 0.5, 4)
    along = unit(lane)
    up = unit([axis - a * dot([0.0, 0.0, 1.0], along) for axis, a in zip([0.0, 0.0, 1.0], along)])
    across = cross(up, along)

    origin = [end_pose.Pos()[axis]
              + across[axis] * dot(offset, across)
              - along[axis] * (length * 0.5)
              - up[axis] * deck_height_mm
              for axis in range(3)]
    pose = robolink.robomath.Mat([
        [along[0], across[0], up[0], origin[0]],
        [along[1], across[1], up[1], origin[1]],
        [along[2], across[2], up[2], origin[2]],
        [0.0, 0.0, 0.0, 1.0]])

    base = rdk.Item(CONVEYOR_BASE, robolink.ITEM_TYPE_FRAME)
    existing = rdk.Item(FEED_CONVEYOR, robolink.ITEM_TYPE_FRAME)
    conveyor = existing if existing.Valid() else rdk.AddFrame(FEED_CONVEYOR, base)
    conveyor.setPoseAbs(pose)
    # Under the conveyor's own base, so a cell that moves the base moves the conveyor with it.
    conveyor.setParentStatic(base)
    print(f'{FEED_CONVEYOR}: standing at {[round(v, 2) for v in conveyor.PoseAbs().Pos()]}, '
          f'{length:.1f} mm along {[round(v, 3) for v in along]}')

    delivered = [origin[axis] + along[axis] * (length * 0.5) + up[axis] * deck_height_mm
                 - across[axis] * dot(offset, across) for axis in range(3)]
    error = separation(delivered, end_pose.Pos())
    if error > 1.0e-6:
        raise RuntimeError(f'the conveyor would deliver to {[round(v, 4) for v in delivered]} rather '
                           f'than {[round(v, 4) for v in end_pose.Pos()]}, {error:.4f} mm out')
    print(f'  delivers a workpiece origin to {[round(v, 3) for v in delivered]}, '
          f'{error:.3f} mm from where the station delivered one')
    return conveyor, length


def declare_the_conveyor(rdk, conveyor, length_mm, pitch_mm, deck_width_mm,
                         deck_height_mm) -> None:
    import json

    parameters = {
        'generator': 'roller_conveyor',
        'lengthMm': length_mm,
        'widthMm': deck_width_mm,
        'heightMm': deck_height_mm,
        'rollerPitchMm': 100.0,
        'startHeightMm': deck_height_mm,
        'endHeightMm': deck_height_mm,
        'startLeftHeightMm': deck_height_mm,
        'startRightHeightMm': deck_height_mm,
        'endLeftHeightMm': deck_height_mm,
        'endRightHeightMm': deck_height_mm,
        'turnAngleDeg': 0.0,
        'curveRadiusMm': 400.0,
        'supportBracesEnabled': True,
        'rollerCoverEnabled': False,
        'endStopEnabled': True,
        'simulationMode': 'logical',
        'role': 'spawner',
        'speedMmS': FEED_SPEED_MM_S,
        'spawnIntervalSeconds': FEED_INTERVAL_S,
        'maxActiveSpawns': FEED_MAX_ACTIVE,
        'spawnObjectId': SPAWN_PROTOTYPE,
        # The queue this lane accumulates into. See FEED_END_INSET_MM: the zero is stated.
        'initialWorkpieceEndInsetMm': FEED_END_INSET_MM,
        'initialWorkpieceSpacingMm': pitch_mm,
        'next': '',
    }
    blob = json.dumps(parameters, sort_keys=True, separators=(',', ':')).encode('utf-8')
    conveyor.setParam('PhysicsConveyor', blob)
    written = bytes(conveyor.getParam('PhysicsConveyor'))
    if written != blob:
        raise RuntimeError(f'RoboDK did not store the conveyor parameters: {written!r}')
    print(f'{conveyor.Name()}: {blob.decode("utf-8")}')


def keep_reset_visible(rdk) -> None:
    from robodk import robolink

    reset = rdk.Item(RESET_PROGRAM, robolink.ITEM_TYPE_PROGRAM)
    if not reset.Valid():
        raise RuntimeError(f'This station has no {RESET_PROGRAM!r} program')
    listing = [reset.Instruction(index)[0].strip() for index in range(reset.InstructionCount())]
    if any(entry.startswith('Pause') for entry in listing):
        print(f'{RESET_PROGRAM}: already paused; left alone')
        return
    reset.InstructionSelect(reset.InstructionCount() - 1)
    reset.Pause(RESET_PAUSE_MS)
    order = [reset.Instruction(index)[0].strip() for index in range(reset.InstructionCount())]
    if not order[-1].startswith('Pause'):
        raise RuntimeError(f'the pause did not land at the end of {RESET_PROGRAM}: {order}')
    print(f'{RESET_PROGRAM}: {" -> ".join(order)}')


def install_the_wait(rdk) -> None:
    """Make the pick wait on the plugin's conveyor-ready signal."""
    from robodk import robolink

    pick = rdk.Item(PICK_PROGRAM, robolink.ITEM_TYPE_PROGRAM)
    removed = 0
    for index in reversed(range(pick.InstructionCount())):
        if pick.Instruction(index)[0].strip() == INDEX_CALL:
            pick.InstructionDelete(index)
            removed += 1
    if removed != 1:
        raise RuntimeError(f'{PICK_PROGRAM} had {removed} {INDEX_CALL!r} calls to remove, not one')
    pick.InstructionSelect(1)
    pick.waitDI(READY_PARAM, 1)
    order = [pick.Instruction(index)[0].strip() for index in range(pick.InstructionCount())]
    if not order[2].startswith('Wait'):
        raise RuntimeError(f'the wait did not land before the approach moves: {order}')
    print(f'{PICK_PROGRAM}: {" -> ".join(order)}')



def delete_the_scripting(rdk) -> None:
    from robodk import robolink

    doomed = set(VENDOR_SCRIPTS) | {INIT_PROGRAM, PICK_12X10_PROGRAM}

    # Every call to something that is about to go, out of every program that survives.
    for program in rdk.ItemList(robolink.ITEM_TYPE_PROGRAM, False):
        if program.Name() in doomed:
            continue  # the whole program goes; its instructions go with it
        removed = []
        for index in reversed(range(program.InstructionCount())):
            called = program.Instruction(index)[0].strip()
            if called in doomed:
                program.InstructionDelete(index)
                removed.append(called)
        if removed:
            order = [program.Instruction(i)[0].strip() for i in range(program.InstructionCount())]
            print(f'{program.Name()}: dropped {", ".join(reversed(removed))}; now '
                  f'{" -> ".join(order) if order else "(empty)"}')

    # Programs, then the targets only the deleted one named.
    for program in rdk.ItemList(robolink.ITEM_TYPE_PROGRAM, False):
        if program.Name() in doomed:
            print(f'deleted program {program.Name()!r}')
            program.Delete()
    for script in rdk.ItemList(robolink.ITEM_TYPE_PROGRAM_PYTHON, False):
        print(f'deleted script {script.Name()!r}')
        script.Delete()
    for name in PICK_12X10_TARGETS:
        target = rdk.Item(name, robolink.ITEM_TYPE_TARGET)
        if target.Valid():
            print(f'deleted target {name!r}')
            target.Delete()

    leftover = [script.Name() for script in rdk.ItemList(robolink.ITEM_TYPE_PROGRAM_PYTHON, False)]
    if leftover:
        raise RuntimeError(f'the station still has Python programs: {leftover}')
    gone = doomed | set(PICK_12X10_TARGETS)
    for program in rdk.ItemList(robolink.ITEM_TYPE_PROGRAM, False):
        for index in range(program.InstructionCount()):
            instruction = program.Instruction(index)[0]
            for name in gone:
                if name in instruction:
                    raise RuntimeError(f'{program.Name()} instruction {index} still names the '
                                       f'deleted {name!r}: {instruction!r}')
    print(f'no Python programs remain, and no instruction names any of the {len(gone)} deleted items')


def delete_the_belt(rdk) -> None:
    from robodk import robolink

    belt = rdk.Item(BELT_MECHANISM, robolink.ITEM_TYPE_ROBOT)
    if not belt.Valid():
        print(f'{BELT_MECHANISM}: already gone')
        return
    carried = [child.Name() for child in belt.Childs()]
    belt.Delete()
    print(f'deleted mechanism {BELT_MECHANISM!r}'
          + (f', and with it {", ".join(repr(name) for name in carried)}' if carried else ''))
    if not rdk.Item(CONVEYOR_BASE, robolink.ITEM_TYPE_FRAME).Valid():
        raise RuntimeError(f'{CONVEYOR_BASE!r} went with the mechanism; every pick target in this '
                           'station is defined against that frame')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('source', type=Path,
                        help='the original showcase .rdk (never modified)')
    parser.add_argument('-o', '--output', type=Path,
                        help='destination .rdk; defaults to "<source stem> physics.rdk" beside it')
    parser.add_argument('--robodk', type=Path, default=DEFAULT_EXE)
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    args = parser.parse_args()

    source: Path = args.source.resolve()
    if not source.is_file():
        print(f'No such station: {source}', file=sys.stderr)
        return 1
    output: Path = args.output or source.with_name(f'{source.stem} physics.rdk')
    output = output.resolve()
    if output == source:
        print('Refusing to overwrite the source station.', file=sys.stderr)
        return 1

    staging = output.with_suffix('.staging.rdk')
    shutil.copy2(source, staging)

    from robodk import robolink

    rdk = connect(args.robodk, staging, args.port)
    try:
        for name, kind in ((TOOL_NAME, robolink.ITEM_TYPE_TOOL),
                           (SPAWN_PROTOTYPE, robolink.ITEM_TYPE_OBJECT),
                           (SPAWN_PARENT, robolink.ITEM_TYPE_FRAME)):
            if not rdk.Item(name, kind).Valid():
                raise RuntimeError(f'This station has no {name!r}')

        for leftover in clear_leftover_workpieces(rdk):
            print(f'deleted {leftover}')

        start_pose, end_pose = derive_lane(rdk)
        lane = [e - s for e, s in zip(end_pose.Pos(), start_pose.Pos())]
        length = separation(end_pose.Pos(), start_pose.Pos())
        along = [component / length for component in lane]
        prototype = rdk.Item(SPAWN_PROTOTYPE, robolink.ITEM_TYPE_OBJECT)
        pitch = measure_product_pitch(rdk, prototype, along)
        deck_width, deck_height = measure_deck(rdk, prototype, along, end_pose.Pos()[2])
        # After the deck, because where the conveyor stands depends on how tall it is.
        conveyor, length = add_conveyor(rdk, start_pose, end_pose, deck_height,
                                        workpiece_centre_offset(rdk, prototype))

        pick = rdk.Item(PICK_PROGRAM, robolink.ITEM_TYPE_PROGRAM)
        if not pick.Valid():
            raise RuntimeError(f'This station has no {PICK_PROGRAM!r} program')
        removed = False
        for index in range(pick.InstructionCount()):
            if pick.Instruction(index)[0].strip() == SPAWN_CALL:
                pick.InstructionDelete(index)
                removed = True
                break
        if not removed:
            raise RuntimeError(f'{PICK_PROGRAM} has no {SPAWN_CALL!r} call to remove')
        print(f'{PICK_PROGRAM}: removed the {SPAWN_CALL} call; the plugin conveyor feeds the line')

        install_the_wait(rdk)
        # Before the deletions, because it is the deletions that make Reset too fast to be noticed.
        keep_reset_visible(rdk)
        delete_the_scripting(rdk)
        delete_the_belt(rdk)
        rdk.setParam(READY_PARAM, '0')

        # The conveyor goes on its own item, so there is one description of it and RoboDK saves it.
        declare_the_conveyor(rdk, conveyor, length, pitch, deck_width, deck_height)

        config = ';'.join([
            'autostart=1',
            'gravity=-9.81',
            f'role={PALLET_NAME}:static',
            f'role={FLOOR_NAME}:static',
            f'role={TOOL_NAME}:kinematic',
        ])
        rdk.setParam(CONFIG_PARAM, config)
        print(f'{CONFIG_PARAM} = {config}')

        rdk.Save(str(output))
        print(f'wrote {output}')
    finally:
        close_robodk(rdk)
        staging.unlink(missing_ok=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
