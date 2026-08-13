from __future__ import annotations

import argparse
import struct
import sys
import tempfile
import time

from pathlib import Path

DEFAULT_EXE = Path(r'C:\RoboDK\bin\RoboDK.exe')
DEFAULT_PORT = 20599

PLUGIN = 'Physics Simulation'
TOOL_NAME = 'RobotiQ EPick Four'
PALLET_NAME = 'Pallet 40inx48in'
FLOOR_NAME = 'Floor'
SPAWN_PROTOTYPE = 'BoxRef_24X16'
FEED_CONVEYOR = 'Box feed'
SPAWN_PARENT = FEED_CONVEYOR
CONVEYOR_ITEM_PARAM = 'PhysicsConveyor'

DROP_HEIGHT_MM = 600.0
SETTLE_SECONDS = 5.0
OFF_PALLET_CLEARANCE_MM = 1000.0


class SmokeFailure(RuntimeError):
    pass


def connect(exe: Path, station: Path, port: int):
    import shutil

    from robodk import robolink

    scratch = Path(tempfile.gettempdir()) / f'physics_smoke_{port}_{station.name}'
    shutil.copyfile(station, scratch)

    rdk = robolink.Robolink(
        port=port,
        args=['-NEWINSTANCE', '-HIDDEN', '-EXIT_LAST_COM'],
        robodk_path=str(exe),
        quit_on_close=False)
    rdk.TIMEOUT = 60
    opened = rdk.AddFile(str(scratch))
    if not opened.Valid():
        close_robodk(rdk)
        raise SmokeFailure(f'RoboDK could not load {scratch}, copied from {station}')
    return rdk


def close_robodk(rdk) -> None:
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


def command(rdk, verb: str, value: str = '') -> str:
    reply = rdk.PluginCommand(PLUGIN, verb, value)
    return '' if reply is None else str(reply).strip()


def require_ok(rdk, verb: str, value: str = '') -> str:
    reply = command(rdk, verb, value)
    if not reply.startswith('OK'):
        raise SmokeFailure(f'{verb} {value!r} answered {reply!r}')
    return reply


def item_rows(rdk) -> dict:
    """`items` parsed into {name: (role, carried, held, body, triangles, status)}."""
    rows = {}
    reply = command(rdk, 'items')
    if not reply:
        return rows
    for record in reply.split(' || '):
        fields = [field.strip() for field in record.split(' | ')]
        if len(fields) < 7:
            continue
        rows[fields[0]] = tuple(fields[1:7])
    return rows


def describe_rows(rows: dict) -> str:
    return '\n'.join(f'    {name}: {" | ".join(values)}' for name, values in sorted(rows.items()))


def conveyor_item(rdk, name: str = FEED_CONVEYOR):
    item = rdk.Item(name)
    if not item.Valid():
        raise SmokeFailure(f'This station has no conveyor called {name!r}')
    return item


def conveyor_placement(rdk, name: str = FEED_CONVEYOR):
    from robodk import robolink, robomath

    values = conveyor_parameters(rdk, name).get('pose')
    if not isinstance(values, list) or len(values) < 12:
        raise SmokeFailure(f'Conveyor {name!r} carries no {"pose"!r} key, so nothing in the station says '
                           'where it stands. That key is the whole of a conveyor\'s placement.')
    local = robomath.Mat([[values[0], values[1], values[2], values[3]],
                          [values[4], values[5], values[6], values[7]],
                          [values[8], values[9], values[10], values[11]],
                          [0.0, 0.0, 0.0, 1.0]])
    parent = conveyor_item(rdk, name).Parent()
    if not parent.Valid() or parent.Type() == robolink.ITEM_TYPE_STATION:
        return local
    return parent.PoseAbs() * local


def conveyor_parameters(rdk, name: str = FEED_CONVEYOR) -> dict:
    import json

    blob = bytes(conveyor_item(rdk, name).getParam(CONVEYOR_ITEM_PARAM))
    if not blob:
        raise SmokeFailure(f'{name!r} carries no {CONVEYOR_ITEM_PARAM!r} parameter, so the station '
                           'declares no conveyor at all')
    return json.loads(blob.decode('utf-8'))


def position_of(item) -> list:
    return list(item.PoseAbs().Pos())


def world_aabb(rdk, item) -> tuple:
    export = Path(tempfile.gettempdir()) / f'physics_smoke_{abs(hash(item.Name())):x}.stl'
    export.unlink(missing_ok=True)
    rdk.Save(str(export), item)
    if not export.is_file():
        raise SmokeFailure(f'RoboDK exported no geometry for {item.Name()!r}')
    data = export.read_bytes()
    export.unlink(missing_ok=True)
    triangles = struct.unpack('<I', data[80:84])[0] if len(data) >= 84 else 0
    if triangles == 0 or len(data) != 84 + triangles * 50:
        raise SmokeFailure(f'{item.Name()!r} did not export as binary STL; cannot measure it')
    low = [float('inf')] * 3
    high = [float('-inf')] * 3
    for index in range(triangles):
        base = 84 + index * 50 + 12
        for corner in range(3):
            vertex = struct.unpack('<3f', data[base + corner * 12:base + corner * 12 + 12])
            for axis in range(3):
                low[axis] = min(low[axis], vertex[axis])
                high[axis] = max(high[axis], vertex[axis])

    pose = item.PoseAbs()
    world_low = [float('inf')] * 3
    world_high = [float('-inf')] * 3
    for corner in range(8):
        local = [high[axis] if corner & (1 << axis) else low[axis] for axis in range(3)]
        for axis in range(3):
            value = sum(pose[axis, k] * local[k] for k in range(3)) + pose[axis, 3]
            world_low[axis] = min(world_low[axis], value)
            world_high[axis] = max(world_high[axis], value)
    return world_low, world_high


def settle(rdk, item, seconds: float) -> list:
    samples = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        time.sleep(0.2)
        samples.append(position_of(item)[2])
    return samples


def drop_and_settle(rdk, station, tool, box, target_xyz: list) -> list:
    box.setParentStatic(tool)
    time.sleep(0.3)
    pose = box.PoseAbs()
    pose.setPos(target_xyz)
    box.setPoseAbs(pose)
    time.sleep(0.3)
    box.setParentStatic(station)
    time.sleep(0.5)
    return settle(rdk, box, SETTLE_SECONDS)


def station_made_box(rdk, parent) -> object:
    from robodk import robolink

    prototype = rdk.Item(SPAWN_PROTOTYPE, robolink.ITEM_TYPE_OBJECT)
    if not prototype.Valid():
        raise SmokeFailure(f'This station has no {SPAWN_PROTOTYPE!r} to clone')
    before = {child.item for child in parent.Childs()}
    rdk.Copy(prototype)
    rdk.Paste(parent)
    fresh = [child for child in parent.Childs()
             if child.item not in before and child.Type() == robolink.ITEM_TYPE_OBJECT]
    if len(fresh) != 1:
        raise SmokeFailure(f'cloning {SPAWN_PROTOTYPE!r} produced {len(fresh)} objects under '
                           f'{parent.Name()!r}, not one')
    # The prototype is hidden station furniture; its clone is a box in the cell.
    fresh[0].setVisible(True)
    fresh[0].setPoseAbs(prototype.PoseAbs())
    return fresh[0]


def station_box_is_adopted(rdk, station, tool, drop_xy: list, drop_z: float,
                           floor_top: float) -> None:
    from robodk import robolink

    print('--- a box the station made, not the plugin')
    parent = conveyor_item(rdk, SPAWN_PARENT)
    box = station_made_box(rdk, parent)
    name = f'{box.Name()} smoke'
    box.setName(name)
    print(f'the station made a box by cloning {SPAWN_PROTOTYPE}, renamed to {name!r}')
    if item_rows(rdk).get(name):
        raise SmokeFailure(f'{name} is already a participant before any tool has touched it; the '
                           'plugin is claiming objects the station merely owns')

    box.setParentStatic(tool)
    # Adoption happens on a step, and the step timer is RoboDK's own.
    time.sleep(0.5)
    row = item_rows(rdk).get(name)
    if not row:
        raise SmokeFailure(
            f'{name} is under {TOOL_NAME!r} and the plugin has never heard of it. This is '
            'adoption in updateCarryStates, not the transport rules.')
    if row[1] != 'carried' or row[2] != 'held':
        raise SmokeFailure(f'{name} was adopted as {row}, not as carried and held. A box a gripper '
                           'is holding must stay the station\'s until it is let go.')
    print(f'adopted {name}: {row}')

    heights = drop_and_settle(rdk, station, tool, box, [drop_xy[0], drop_xy[1], drop_z])
    row = item_rows(rdk).get(name)
    if not row or row[3] != 'body':
        raise SmokeFailure(f'{name} was let go by the tool but never became a body: {row}')
    if not heights:
        raise SmokeFailure('no pose samples were taken')
    recent = heights[-5:]
    print(f'released {name}: {row}; came to rest at {heights[-1]:.1f} mm '
          f'(last {len(recent)} samples span {max(recent) - min(recent):.2f} mm)')
    if drop_z - heights[-1] <= 1.0:
        raise SmokeFailure(f'{name} never fell; it was adopted but nothing simulates it')
    if max(recent) - min(recent) > 2.0:
        raise SmokeFailure(f'{name} had not settled after {SETTLE_SECONDS:.0f} s')
    if heights[-1] < floor_top + 10.0:
        raise SmokeFailure(
            f'{name} came to rest at {heights[-1]:.1f} mm, which is the floor at {floor_top:.1f} mm. '
            'It fell through the stack it should have landed on.')

    resting_low, resting_high = world_aabb(rdk, box)
    box_height = resting_high[2] - resting_low[2]
    spawned = require_ok(rdk, 'spawn', f'{SPAWN_PROTOTYPE}|{SPAWN_PARENT}')
    second = rdk.Item(spawned[len('OK'):].strip(), robolink.ITEM_TYPE_OBJECT)
    stacked = drop_and_settle(rdk, station, tool, second,
                             [drop_xy[0], drop_xy[1], resting_high[2] + DROP_HEIGHT_MM * 0.5])
    lift = stacked[-1] - heights[-1]
    print(f'stacked {second.Name()}: rest {stacked[-1]:.1f} mm, {lift:.1f} mm above the box under '
          f'it, which is {box_height:.1f} mm tall')
    if lift < box_height * 0.75:
        raise SmokeFailure(
            f'the second box came to rest only {lift:.1f} mm above the first, which is {box_height:.1f} '
            'mm tall. Boxes are not resting on each other, so no stack in this cell is real.')
    box.Delete()


def run(rdk) -> None:
    from robodk import robolink

    if not command(rdk, 'status'):
        raise SmokeFailure(
            'The physics plugin did not answer. It is either not installed in RoboDK\\bin\\plugins '
            'or not enabled in RoboDK\'s add-in list.')

    station = rdk.ActiveStation()
    tool = rdk.Item(TOOL_NAME, robolink.ITEM_TYPE_TOOL)
    pallet = rdk.Item(PALLET_NAME, robolink.ITEM_TYPE_OBJECT)
    floor = rdk.Item(FLOOR_NAME, robolink.ITEM_TYPE_OBJECT)
    for name, item in ((TOOL_NAME, tool), (PALLET_NAME, pallet), (FLOOR_NAME, floor)):
        if not item.Valid():
            raise SmokeFailure(f'This station has no {name!r}')
    before = {name: position_of(item)
              for name, item in ((TOOL_NAME, tool), (PALLET_NAME, pallet), (FLOOR_NAME, floor))}

    require_ok(rdk, 'loadconfig')
    require_ok(rdk, 'clearspawners')
    require_ok(rdk, 'clearspawned')
    reply = command(rdk, 'start')
    if reply != 'OK':
        raise SmokeFailure(f'start answered {reply!r}')
    print(f'status: {command(rdk, "status")}')

    rows = item_rows(rdk)
    print('items after start:')
    print(describe_rows(rows))
    for name in (PALLET_NAME, FLOOR_NAME):
        values = rows.get(name)
        if not values:
            raise SmokeFailure(f'{name} is not a participant; the station parameter did not apply')
        triangles = int(values[4].split()[0])
        if triangles <= 0 or not values[5].startswith('Static'):
            raise SmokeFailure(
                f'{name} produced no collision geometry: {values[4]!r}. '
                'This is the STL export path, not the transport rules.')
        print(f'{name}: {triangles} triangles cooked')

    pallet_low, pallet_high = world_aabb(rdk, pallet)
    floor_low, floor_high = world_aabb(rdk, floor)
    print('pallet occupies x {:.0f}..{:.0f}, y {:.0f}..{:.0f}, top z {:.1f}'.format(
        pallet_low[0], pallet_high[0], pallet_low[1], pallet_high[1], pallet_high[2]))
    print('floor occupies  x {:.0f}..{:.0f}, y {:.0f}..{:.0f}, top z {:.1f}'.format(
        floor_low[0], floor_high[0], floor_low[1], floor_high[1], floor_high[2]))

    on_pallet = [(pallet_low[0] + pallet_high[0]) * 0.5, (pallet_low[1] + pallet_high[1]) * 0.5]
    off_everything = [max(pallet_high[0], floor_high[0]) + OFF_PALLET_CLEARANCE_MM, on_pallet[1]]
    drop_z = pallet_high[2] + DROP_HEIGHT_MM

    drops = {}
    for label, target_xy in (('the pallet', on_pallet), ('open air', off_everything)):
        spawned = require_ok(rdk, 'spawn', f'{SPAWN_PROTOTYPE}|{SPAWN_PARENT}')
        name = spawned[len('OK'):].strip()
        box = rdk.Item(name, robolink.ITEM_TYPE_OBJECT)
        if not box.Valid():
            raise SmokeFailure(f'spawn reported {name!r} but RoboDK has no such item')
        row = item_rows(rdk).get(name)
        if not row or row[1] != 'carried':
            raise SmokeFailure(f'a freshly spawned box should be carried by the station, not {row}')
        print(f'spawned {name}: {row}')

        target = [target_xy[0], target_xy[1], drop_z]
        heights = drop_and_settle(rdk, station, tool, box, target)
        row = item_rows(rdk).get(name)
        if not row or row[3] != 'body':
            raise SmokeFailure(
                f'{name} was let go by the tool but never became a body: {row}. '
                'This is release()/createDynamicBody, not the transport rules.')
        print(f'released {name}: {row}')

        if not heights:
            raise SmokeFailure('no pose samples were taken')
        fell = target[2] - heights[-1]
        recent = heights[-5:]
        moved_late = max(recent) - min(recent)
        print(f'drop over {label}: from {target[2]:.1f} to {heights[-1]:.1f} mm '
              f'(fell {fell:.1f}, last {len(recent)} samples span {moved_late:.2f} mm)')
        if fell <= 1.0:
            raise SmokeFailure(f'the box over {label} never fell; gravity did not reach it')
        drops[label] = {'rest': heights[-1], 'settled': moved_late <= 2.0}

    if not drops['the pallet']['settled']:
        raise SmokeFailure(
            f'the box over the pallet had not settled after {SETTLE_SECONDS:.0f} s. The pallet '
            'cooked, but nothing in the scene stopped a box falling onto it.')
    if drops['the pallet']['rest'] < floor_high[2] + 10.0:
        raise SmokeFailure(
            'the box came to rest at {:.1f} mm, which is the floor at {:.1f} mm, not the pallet '
            'whose top is at {:.1f} mm. It fell straight through the pallet.'.format(
                drops['the pallet']['rest'], floor_high[2], pallet_high[2]))
    if drops['open air']['settled']:
        raise SmokeFailure(
            'a box dropped clear of everything also came to rest, at {:.1f} mm. Something is '
            'catching boxes where the cell has nothing, so the pallet drop proves nothing.'.format(
                drops['open air']['rest']))
    print('the pallet stopped its box at {:.1f} mm, clear of the floor at {:.1f} mm; the box with '
          'nothing under it was still falling past {:.1f} mm'.format(
              drops['the pallet']['rest'], floor_high[2], drops['open air']['rest']))

    station_box_is_adopted(rdk, station, tool, on_pallet, drop_z, floor_high[2])

    require_ok(rdk, 'stop')
    leftovers = [name for name in rdk.ItemList(robolink.ITEM_TYPE_OBJECT, True)
                 if name.startswith(f'{SPAWN_PROTOTYPE} #')]
    if leftovers:
        raise SmokeFailure(f'stop left spawned boxes in the station: {leftovers}')
    for name, item in ((TOOL_NAME, tool), (PALLET_NAME, pallet), (FLOOR_NAME, floor)):
        after = position_of(item)
        drift = max(abs(a - b) for a, b in zip(after, before[name]))
        if drift > 1.0e-6:
            raise SmokeFailure(f'stop did not restore {name}: moved {drift:.4f} mm')
    print('stop: no spawned boxes left, every station item back where it was')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('station', type=Path, help='the physics variant .rdk')
    parser.add_argument('--robodk', type=Path, default=DEFAULT_EXE)
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
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
    print('PASS: the plugin spawned, carried, released and settled a box, and stop restored the '
          'station.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
