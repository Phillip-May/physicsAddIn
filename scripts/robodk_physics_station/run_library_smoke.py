from __future__ import annotations

import argparse
import math
import re
import sys

from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_physics_smoke import (SmokeFailure, close_robodk, command, connect, require_ok,
                               conveyor_parameters, conveyor_placement, DEFAULT_EXE)

REPO_ROOT = Path(__file__).resolve().parents[2]
LIBRARY_ROOT = REPO_ROOT / 'library' / 'packages'

PACKAGE = 'pedestal_h12in_type1.zip'
# Clear of the showcase's own cell, so nothing here can be confused with what the station already has.
PLACE_AT = (-3000.0, 2500.0, 0.0)
GENERATED_PACKAGE = 'roller_conveyor.zip'
CONVEYOR_VARIANT = 'straight-890'
SNAP_BASE_AT = (-4200.0, 1800.0, 300.0, 0.0, 0.0, 37.0)

ROBOT_PACKAGE = 'ar4_6dof_robot.zip'
RAIL_PACKAGE = 'gudel_tmf1_6m_gantry.zip'
COPIES_PACKAGE = 'ws2000_gantry.zip'
# Where the first of the pair is put, clear of everything else this file places.
FIRST_COPY_AT = (-6500.0, -2200.0, 0.0)
ROBOT_AT = (-5000.0, 3200.0, 400.0, 15.0, -25.0, 40.0)
ROBOT_JOINTS = (10.0, -20.0, 30.0, -40.0, 50.0, -60.0)
ROBOT_JOINTS_BEYOND_LIMIT = (0.0, 120.0, 0.0, 0.0, 0.0, 0.0)
# The Gudel rail travels 0..6400 mm.
RAIL_AT_MM = 2750.0


def library_rows(rdk) -> list:
    """The plugin's own account of what it has placed, one record per item."""
    report = command(rdk, 'library')
    if report.startswith('no placed library items'):
        return []
    rows = []
    for record in report.split(' || '):
        fields = [field.strip() for field in record.split(' | ')]
        if len(fields) < 7:
            raise SmokeFailure(f'a library record has {len(fields)} fields: {record}')
        triangles = int(fields[3].split()[0])
        at = [float(value) for value in fields[4].removeprefix('at ').split(',')]
        low, high = fields[6].removeprefix('box ').split(' to ')
        row = {'name': fields[0], 'package': fields[1], 'variant': fields[2],
               'triangles': triangles, 'at': at, 'state': fields[5],
               'box': [float(value) for value in low.split(',') + high.split(',')],
               'axes': None, 'axes_unit': None}
        if len(fields) > 7 and fields[7].startswith('axes '):
            numbers, unit = fields[7].removeprefix('axes ').rsplit(' ', 1)
            row['axes'] = [float(value) for value in numbers.split(',')]
            row['axes_unit'] = unit
        rows.append(row)
    return rows


def generic_items(rdk) -> dict:
    """Every generic node in the station, by name. Held as items, never looked up by name."""
    from robodk import robolink

    found = {}
    for item in rdk.ItemList():
        if item.Type() == robolink.ITEM_TYPE_GENERIC:
            found[item.Name()] = item
    return found


def place(rdk, package: str, variant: str, x: float, y: float, z: float,
          rx: float = 0.0, ry: float = 0.0, rz: float = 0.0) -> str:
    argument = '|'.join([package, variant, str(x), str(y), str(z), str(rx), str(ry), str(rz)])
    reply = require_ok(rdk, 'libraryplace', argument)
    return reply.removeprefix('OK').strip()


def snap(rdk, package: str, variant: str, mate_to: str = '', source: str = '',
         target: str = '') -> str:
    reply = require_ok(rdk, 'librarysnap', '|'.join([package, variant, mate_to, source, target]))
    print(f'  {reply}')
    return reply.removeprefix('OK').split('|')[0].strip()


def snapped_conveyor_ends_meet(rdk) -> None:
    from robodk import robomath

    print('--- two conveyor ends brought together, with no cursor in it')
    first = snap(rdk, GENERATED_PACKAGE, CONVEYOR_VARIANT)
    # Somewhere awkward, and turned. A conversion that is out by a quarter turn survives the origin.
    require_ok(rdk, 'moveconveyor', '|'.join([first] + [str(value) for value in SNAP_BASE_AT]))
    second = snap(rdk, GENERATED_PACKAGE, CONVEYOR_VARIANT, first, 'start', 'end')
    if second == first:
        raise SmokeFailure('librarysnap mated the conveyor to itself')

    stands = conveyor_placement(rdk, first)
    mated = conveyor_placement(rdk, second)
    length = float(conveyor_parameters(rdk, first)['lengthMm'])
    if length <= 0.0:
        raise SmokeFailure(f'{first!r} reports a lane length of {length}')
    expected = stands * robomath.transl(length, 0.0, 0.0)
    worst = max(abs(mated[row, col] - expected[row, col]) for row in range(3) for col in range(4))
    if worst > 1.0e-3:
        raise SmokeFailure(
            f'{second!r} was mated onto {first!r} and stands {worst:.6f} out of where its own end grid '
            f'says it should.\n    stands: {stands}\n    expected: {expected}\n    mated: {mated}\n'
            '    A snapped conveyor whose stored pose disagrees with the mate is the frame conversion '
            'between the solver, the accessory frame and the parent-relative pose.')
    print(f'  the two lanes meet at one point, {worst:.6f} mm out, and the second runs on rather than '
          'back over the first')

    ends = {}
    for record in command(rdk, 'conveyors').split(' || '):
        found = re.match(r'c (.+?) \| .*path ([-+\d.,eE]+) -> ([-+\d.,eE]+)', record)
        if found and found.group(1) in (first, second):
            ends[found.group(1)] = [[float(value) for value in found.group(half).split(',')]
                                    for half in (2, 3)]
    if len(ends) != 2:
        raise SmokeFailure(f'the plugin reports lane ends for {sorted(ends)}, not both conveyors')
    apart = min(math.dist(a, b) for a in ends[first] for b in ends[second])
    if apart > 0.01:
        raise SmokeFailure(f'the nearest pair of lane endpoints is {apart:.4f} mm apart, so what is '
                           'drawn and what products ride have not been joined')
    print(f'  and its own lane arithmetic agrees: the nearest endpoints are {apart:.4f} mm apart')

    for conveyor in (second, first):
        require_ok(rdk, 'deleteconveyor', conveyor)


def the_dock_says_what_it_will_not_arm(rdk) -> None:
    print('--- what the dock would list, and what it will not arm')
    rows = []
    for record in command(rdk, 'librarylist').split(' || '):
        fields = [field.strip() for field in record.split(' | ')]
        if len(fields) < 4:
            raise SmokeFailure(f'a catalogue record has {len(fields)} fields: {record}')
        rows.append({'kind': fields[0], 'name': fields[1], 'variant': fields[2], 'note': fields[3]})
    if not rows:
        raise SmokeFailure('the dock would list nothing at all')

    armable = [row for row in rows if row['note'] == 'armable']
    if not armable:
        raise SmokeFailure('the dock would offer nothing armable, so it is a list and not a library')
    stale = [row for row in rows if 'kinematics' in row['note']]
    if stale:
        raise SmokeFailure(f'a row is still refused for having kinematics: {stale}. A placed mechanism is '
                           'driven by the kinematics RobotSimulator poses arms with, so that refusal '
                           'describes a placement path that no longer exists.')
    for kind in ('Robot', 'Linear rail'):
        shown = [row for row in rows if row['kind'] == kind]
        if not shown:
            raise SmokeFailure(f'the dock hides every {kind}, so it disagrees with the library on disk')
        offered = [row for row in shown if row['note'] == 'armable']
        if not offered:
            raise SmokeFailure(f'the dock will arm no {kind} at all: {shown}. Its joints move the drawn '
                               'triangles now, so a library that lists arms and places none is a '
                               'catalogue.')
        for row in shown:
            if row['note'] not in ('armable',) and 'mounting pattern' not in row['note']:
                raise SmokeFailure(f'a {kind} row is refused for an unexpected reason: {row}')
    print(f"  {len([r for r in rows if r['kind'] == 'Robot' and r['note'] == 'armable'])} arm(s) and "
          f"{len([r for r in rows if r['kind'] == 'Linear rail' and r['note'] == 'armable'])} rail(s) "
          'will arm, and nothing is refused for having kinematics')

    presets = [row for row in rows if row['kind'] == 'Accessory' and row['variant'] != 'default']
    if not presets:
        raise SmokeFailure('the dock lists no accessory presets, so it disagrees with the library on disk')
    unarmable = [row for row in presets if row['note'] != 'armable']
    if unarmable:
        raise SmokeFailure(f'an accessory preset will not arm: {unarmable}. Its geometry comes from its '
                           'parameters and commitPlacement delivers it as a conveyor, so there is nothing '
                           'left for it to be refused for.')
    print(f'  {len(rows)} rows, {len(armable)} armable including {len(presets)} generated preset(s); '
          'every robot and rail shown with its reason')


def placed_joints(rdk, name: str, *values: float) -> dict:
    from robodk import robomath

    reply = require_ok(rdk, 'placedjoints', '|'.join([name] + [str(value) for value in values]))
    fields = [field.strip() for field in reply.split(' | ')]
    if len(fields) < 3:
        raise SmokeFailure(f'placedjoints answered {len(fields)} fields: {reply}')

    def mat(text: str, prefix: str):
        cells = [float(value) for value in text.removeprefix(prefix).strip().split(',')]
        if len(cells) != 12:
            raise SmokeFailure(f'{prefix} is {len(cells)} numbers, not 12: {text}')
        return robomath.Mat([cells[0:4], cells[4:8], cells[8:12], [0.0, 0.0, 0.0, 1.0]])

    answered = {'axes': [float(value) for value in fields[1].rsplit(' ', 1)[0].split(',')],
                'unit': fields[1].rsplit(' ', 1)[1]}
    # An arm answers two frames and a rail one, because an arm has two and a rail has one.
    for field in fields[2:]:
        key = field.split(' ', 1)[0]
        if key not in ('tcp', 'base', 'carriage'):
            raise SmokeFailure(f'placedjoints answered an unknown frame {key!r}: {reply}')
        answered[key] = mat(field, key)
    return answered


def tool_pose_from_robot_simulator(package: str, joints) -> object:
    import subprocess

    from robodk import robomath

    exe = REPO_ROOT / 'dist' / 'RobotSimulator' / 'release' / 'RobotSimulator.exe'
    if not exe.is_file():
        raise SmokeFailure(f'no RobotSimulator at {exe}; run scripts\\build_robot_simulator.ps1 first')
    argv = [str(exe), '--tool-pose', str(LIBRARY_ROOT / package)] + [str(value) for value in joints]
    finished = subprocess.run(argv, capture_output=True, text=True)
    if finished.returncode != 0:
        raise SmokeFailure(f'--tool-pose failed: {finished.stderr.strip() or finished.stdout.strip()}')
    parts = finished.stdout.split()
    if len(parts) != 13 or parts[0] != 'tcp':
        raise SmokeFailure(f'--tool-pose answered {finished.stdout.strip()!r}')
    cells = [float(value) for value in parts[1:]]
    return robomath.Mat([cells[0:4], cells[4:8], cells[8:12], [0.0, 0.0, 0.0, 1.0]])


def worst_element(a, b) -> float:
    return max(abs(a[row, col] - b[row, col]) for row in range(3) for col in range(4))


def a_placed_arm_is_posed_by_the_shared_kinematics(rdk) -> str:
    print('--- an arm placed as a drawn item, and posed by the kinematics RobotSimulator uses')
    name = place(rdk, ROBOT_PACKAGE, '', *ROBOT_AT)
    print(f"  placed '{name}' from {ROBOT_PACKAGE}")

    home = [row for row in library_rows(rdk) if row['name'] == name]
    if not home:
        raise SmokeFailure('the plugin does not report the arm it just placed')
    home = home[0]
    if home['axes'] is None:
        raise SmokeFailure(f'the placed arm reports no axes, so nothing drives it: {home}')
    if home['axes_unit'] != 'deg':
        raise SmokeFailure(f"its axes are in {home['axes_unit']!r} rather than degrees")
    if len(home['axes']) != 6:
        raise SmokeFailure(f"a six-axis arm reports {len(home['axes'])} axes: {home['axes']}")
    if home['triangles'] <= 0:
        raise SmokeFailure('the arm draws no triangles, so nothing stands in the cell')
    print(f"  {home['triangles']} triangles, at its package's home {home['axes']}")

    posed = placed_joints(rdk, name, *ROBOT_JOINTS)
    if any(abs(posed['axes'][axis] - ROBOT_JOINTS[axis]) > 1e-6 for axis in range(6)):
        raise SmokeFailure(f"it was asked for {list(ROBOT_JOINTS)} and answers {posed['axes']}")

    # Claim one: the same code, therefore the same answer.
    expected = tool_pose_from_robot_simulator(ROBOT_PACKAGE, ROBOT_JOINTS)
    worst = worst_element(posed['tcp'], expected)
    if worst > 1.0e-6:
        raise SmokeFailure(
            f'the placed arm puts its flange {worst:.9f} mm from where RobotSimulator puts it for the same '
            f'joints.\n    plugin: {posed["tcp"]}\n    RobotSimulator: {expected}\n'
            '    Two answers to one forward kinematics is the whole thing route 2 exists to avoid.')
    print(f'  its flange agrees with RobotSimulator to {worst:.9f} mm, in the arm\'s own base frame')

    bent = [row for row in library_rows(rdk) if row['name'] == name][0]
    if bent['axes'] is None or any(abs(bent['axes'][axis] - ROBOT_JOINTS[axis]) > 1e-6
                                   for axis in range(6)):
        raise SmokeFailure(f"after posing it, the plugin still reports axes {bent['axes']}")
    if any(abs(bent['at'][axis] - home['at'][axis]) > 1e-9 for axis in range(3)):
        raise SmokeFailure('bending the arm moved its placement, which is a different thing entirely')
    if bent['triangles'] != home['triangles']:
        raise SmokeFailure('bending the arm changed how much of it there is')
    moved = max(abs(bent['box'][corner] - home['box'][corner]) for corner in range(6))
    if moved < 1.0:
        raise SmokeFailure(
            f'the arm bent through {list(ROBOT_JOINTS)} and the box its triangles fill moved by '
            f'{moved:.6f} mm. The joints are posing a model beside the geometry rather than the tree the '
            'triangles are flattened out of, which is the frozen arm this was all refused for.')
    print(f'  and the triangles followed: the box they fill moved {moved:.1f} mm without the placement '
          'or the count changing')

    from robodk import robomath

    flange = posed['base'] * posed['tcp']
    step = robomath.transl(700.0, -400.0, 250.0) * robomath.rotz(0.6) * robomath.roty(-0.3)
    frame = rdk.AddFrame('Library smoke arm base')
    frame.setPose(step)
    generic_items(rdk)[name].setParentStatic(frame)

    carried = placed_joints(rdk, name, *ROBOT_JOINTS)
    if worst_element(carried['tcp'], posed['tcp']) > 1.0e-9:
        raise SmokeFailure('moving the item changed the flange in the arm\'s own base frame. A taught '
                           'point is six joint angles and does not mean something else because the cell '
                           'moved, so that frame is one the station cannot reach.')
    worst = worst_element(carried['base'] * carried['tcp'], step * flange)
    if worst > 1.0e-3:
        raise SmokeFailure(
            f'the arm was carried by a known step and its flange ended up {worst:.6f} mm from where that '
            f'step puts it.\n    expected: {step * flange}\n    reported: {carried["base"] * carried["tcp"]}\n'
            '    Composing the arm base, the package tree and the station on the wrong side reads fine '
            'and survives the origin.')
    print(f'  carried by a dragged parent frame, its flange went exactly with it, {worst:.6f} mm out')
    return name


def a_rail_is_one_number_rather_than_six(rdk) -> str:
    print('--- a rail, which is one number rather than six')
    name = place(rdk, RAIL_PACKAGE, '', 4000.0, -3000.0, 0.0)
    row = [entry for entry in library_rows(rdk) if entry['name'] == name][0]
    if row['axes'] is None or len(row['axes']) != 1 or row['axes_unit'] != 'mm':
        raise SmokeFailure(f'a rail reports {row["axes"]} {row["axes_unit"]}, not one number in mm')

    home = placed_joints(rdk, name, row['axes'][0])
    driven = placed_joints(rdk, name, RAIL_AT_MM)
    if abs(driven['axes'][0] - RAIL_AT_MM) > 1e-6:
        raise SmokeFailure(f'the rail was sent to {RAIL_AT_MM} mm and answers {driven["axes"]}')
    if 'carriage' not in driven:
        raise SmokeFailure(f'a rail answers no carriage frame: {sorted(driven)}')

    travelled = math.dist([driven['carriage'][row_, 3] for row_ in range(3)],
                          [home['carriage'][row_, 3] for row_ in range(3)])
    asked = abs(RAIL_AT_MM - home['axes'][0])
    if abs(travelled - asked) > 1.0e-3:
        raise SmokeFailure(f'the carriage was sent {asked} mm along its axis and moved {travelled:.6f} mm')
    # A carriage that rotated is a moving frame being driven as though it were a joint.
    spun = max(abs(driven['carriage'][r, c] - home['carriage'][r, c])
               for r in range(3) for c in range(3))
    if spun > 1.0e-9:
        raise SmokeFailure(f'the carriage turned by {spun:.9f} on its way along a linear axis')
    print(f'  sent {asked:.1f} mm along, and its carriage moved {travelled:.6f} mm without turning')

    beyond = command(rdk, 'placedjoints', f'{name}|9000')
    if beyond.startswith('OK'):
        raise SmokeFailure(f'9000 mm is past the end of this rail and was accepted: {beyond}')
    if 'travel' not in beyond.lower():
        raise SmokeFailure(f'it was refused without naming the travel it has: {beyond}')
    after = [entry for entry in library_rows(rdk) if entry['name'] == name][0]
    if abs(after['axes'][0] - RAIL_AT_MM) > 1e-9:
        raise SmokeFailure(f'a refused position moved the carriage anyway, to {after["axes"]}')
    print(f'  refused: {beyond}')
    return name


def two_copies_of_one_package_are_two_things(rdk) -> None:
    print('--- a second copy of one package leaves the first alone')
    first = place(rdk, COPIES_PACKAGE, '', *FIRST_COPY_AT)
    before = [row for row in library_rows(rdk) if row['name'] == first]
    if len(before) != 1:
        raise SmokeFailure(f'{first!r} is not one placed item: {before}')

    second = snap(rdk, COPIES_PACKAGE, '')
    if second == first:
        raise SmokeFailure(f'both copies are called {first!r}. RoboDK does not uniquify a generic item\'s '
                           'name, so the plugin has to - and two copies sharing one are one item to every '
                           'verb that takes a name and to any code that resolves an item by one.')
    nodes = generic_items(rdk)
    for name in (first, second):
        if name not in nodes:
            raise SmokeFailure(f'{name!r} was placed but there is no generic node called that')

    after = [row for row in library_rows(rdk) if row['name'] == first]
    if len(after) != 1:
        raise SmokeFailure(f'placing a second copy left {len(after)} item(s) called {first!r}')
    if after[0] != before[0]:
        raise SmokeFailure(f'placing {second!r} changed {first!r}: it stood at {before[0]["at"]} filling '
                           f'{before[0]["box"]} and now stands at {after[0]["at"]} filling '
                           f'{after[0]["box"]}. The commit resolved its own new item by a shared name.')

    if before[0]['axes'] is not None:
        moved = placed_joints(rdk, second, before[0]['axes'][0] + 100.0)
        still = [row for row in library_rows(rdk) if row['name'] == first][0]
        if still['axes'] != before[0]['axes']:
            raise SmokeFailure(f'driving {second!r} to {moved["axes"]} moved {first!r} to '
                               f'{still["axes"]}, so the two copies share one controller')
    print(f'  {first!r} and {second!r} are two nodes, two records and two sets of axes')

    for name in (first, second):
        require_ok(rdk, 'librarydelete', name)


def a_pose_the_arm_has_not_got_is_refused(rdk, name: str) -> None:
    print('--- a joint past its limit is refused, and the arm does not move')
    before = [row for row in library_rows(rdk) if row['name'] == name][0]
    reply = command(rdk, 'placedjoints',
                    '|'.join([name] + [str(value) for value in ROBOT_JOINTS_BEYOND_LIMIT]))
    if reply.startswith('OK'):
        raise SmokeFailure(f'J2 = {ROBOT_JOINTS_BEYOND_LIMIT[1]} deg is past this arm\'s limit and was '
                           f'accepted: {reply}')
    if 'J2' not in reply or 'limit' not in reply.lower():
        raise SmokeFailure(f'it was refused without naming the joint or its limits: {reply}')
    after = [row for row in library_rows(rdk) if row['name'] == name][0]
    if any(abs(after['axes'][axis] - before['axes'][axis]) > 1e-9 for axis in range(6)):
        raise SmokeFailure(f'a refused pose moved the arm anyway, from {before["axes"]} to {after["axes"]}')
    if any(abs(after['box'][corner] - before['box'][corner]) > 1e-6 for corner in range(6)):
        raise SmokeFailure('a refused pose left the arm where it was and re-baked it somewhere else')
    print(f'  refused: {reply}')


def a_saved_posed_arm_reopens_posed(rdk, station: Path) -> None:
    print('--- a station saved with a posed arm reopens posed')
    posed = {row['name']: row for row in library_rows(rdk) if row['axes'] is not None}
    if not posed:
        raise SmokeFailure('there is no mechanism placed to save')
    saved = station.parent / f'{station.stem} - library smoke posed.rdk'
    rdk.Save(str(saved))
    try:
        rdk.CloseStation()
        rdk.AddFile(str(saved))
        reopened = {}
        for _ in range(60):
            rdk.Command('Wait', '50')
            reopened = {row['name']: row for row in library_rows(rdk) if row['axes'] is not None}
            if len(reopened) >= len(posed):
                break
        for name, was in posed.items():
            now = reopened.get(name)
            if now is None:
                raise SmokeFailure(f"'{name}' came back from a saved station with no axes at all: "
                                   f"{sorted(reopened)}")
            if len(now['axes']) != len(was['axes']) or any(
                    abs(now['axes'][axis] - was['axes'][axis]) > 1e-6
                    for axis in range(len(was['axes']))):
                raise SmokeFailure(
                    f"'{name}' was saved at {was['axes']} and reopened at {now['axes']}. Its axes are not "
                    'in its own JSON, so the cell changed shape by being saved.')
            if any(abs(now['box'][corner] - was['box'][corner]) > 0.5 for corner in range(6)):
                raise SmokeFailure(
                    f"'{name}' reopened with its axes right and its triangles somewhere else: was "
                    f"{was['box']}, now {now['box']}. Adoption baked it before it stood the axes up.")
            print(f"  '{name}' came back at {[round(value, 3) for value in now['axes']]} "
                  f"{now['axes_unit']}, and its triangles with it")
    finally:
        # Closed before the copy is removed, or Windows will not let it go while RoboDK holds it open.
        try:
            rdk.CloseStation()
        except Exception:
            pass
        try:
            saved.unlink()
        except OSError:
            pass


def run(rdk, station: Path) -> None:
    if not LIBRARY_ROOT.is_dir():
        raise SmokeFailure(f'no library at {LIBRARY_ROOT}')
    require_ok(rdk, 'libraryroot', str(LIBRARY_ROOT))

    print('--- a package placed by name, at a stated pose')
    before = set(generic_items(rdk))
    name = place(rdk, PACKAGE, '', *PLACE_AT)
    if not name:
        raise SmokeFailure('libraryplace named nothing')
    print(f"  placed '{name}' from {PACKAGE}")

    rows = library_rows(rdk)
    if len(rows) != 1:
        raise SmokeFailure(f'expected one placed item, the plugin says {len(rows)}: {rows}')
    row = rows[0]
    if row['name'] != name:
        raise SmokeFailure(f"the plugin calls it '{row['name']}' and libraryplace called it '{name}'")
    if row['state'] != 'generic node':
        raise SmokeFailure(f"it is a '{row['state']}' rather than a generic node")

    added = set(generic_items(rdk)) - before
    if len(added) != 1:
        raise SmokeFailure(f'placing one package added {len(added)} generic items: {sorted(added)}')
    node = generic_items(rdk)[name]
    blob = node.getParam('PhysicsLibraryItem')
    if not blob:
        raise SmokeFailure('the placed node carries no PhysicsLibraryItem description')
    import json

    described = json.loads(blob)
    if Path(described.get('package', '')).name != PACKAGE:
        raise SmokeFailure(f"its description names '{described.get('package')}', not {PACKAGE}")
    if len(described.get('pose', [])) != 12:
        raise SmokeFailure('its description carries no 12-number pose')
    print(f"  it is one generic node carrying {len(blob)} bytes of its own description")

    if row['triangles'] <= 0:
        raise SmokeFailure('it draws no triangles, so nothing stands in the cell')
    stood = row['at']
    if any(abs(stood[axis] - PLACE_AT[axis]) > 1e-6 for axis in range(3)):
        raise SmokeFailure(f'it was placed at {list(PLACE_AT)} and stands at {stood}')
    print(f"  and {row['triangles']} triangles baked at {stood[0]:.1f}, {stood[1]:.1f}, "
          f'{stood[2]:.1f} mm')

    print('--- moved by the RoboDK parent it hangs under, which nothing reports')
    from robodk import robomath

    frame = rdk.AddFrame('Library smoke base')
    frame.setPose(robomath.transl(1000.0, 0.0, 250.0))
    node.setParentStatic(frame)
    moved = None
    for _ in range(40):
        rdk.Command('Wait', '50')
        candidates = [entry for entry in library_rows(rdk) if entry['name'] == name]
        if candidates and any(abs(candidates[0]['at'][axis] - stood[axis]) > 1.0
                              for axis in range(3)):
            moved = candidates[0]
            break
    if moved is None:
        raise SmokeFailure('re-parenting it onto a moved frame never re-baked what is drawn')
    expected = [PLACE_AT[0] + 1000.0, PLACE_AT[1], PLACE_AT[2] + 250.0]
    if any(abs(moved['at'][axis] - expected[axis]) > 1e-6 for axis in range(3)):
        raise SmokeFailure(f'it should stand at {expected} and stands at {moved["at"]}')
    if moved['triangles'] != row['triangles']:
        raise SmokeFailure('moving it changed how much of it there is')
    print(f"  its parent moved and it followed, to {moved['at'][0]:.1f}, {moved['at'][1]:.1f}, "
          f"{moved['at'][2]:.1f} mm, still {moved['triangles']} triangles")

    print('--- a generated accessory is a conveyor, and says so rather than landing wrong')
    reply = command(rdk, 'libraryplace',
                    '|'.join([GENERATED_PACKAGE, 'straight-890', '0', '0', '0', '0', '0', '0']))
    if reply.startswith('OK'):
        raise SmokeFailure('the roller conveyor was placed as a library item rather than refused')
    if 'conveyor' not in reply.lower():
        raise SmokeFailure(f'it was refused without saying to add it as a conveyor: {reply}')
    if len(library_rows(rdk)) != 1:
        raise SmokeFailure('a refused placement left something behind')
    print(f'  refused: {reply}')

    the_dock_says_what_it_will_not_arm(rdk)
    snapped_conveyor_ends_meet(rdk)

    print('--- deleted')
    require_ok(rdk, 'librarydelete', name)
    if library_rows(rdk):
        raise SmokeFailure('the plugin still has a placed item after deleting it')
    if name in generic_items(rdk):
        raise SmokeFailure('its generic node is still in the station')
    print('  no placed item, no node, and nothing left drawn')

    arm = a_placed_arm_is_posed_by_the_shared_kinematics(rdk)
    a_pose_the_arm_has_not_got_is_refused(rdk, arm)
    a_rail_is_one_number_rather_than_six(rdk)
    two_copies_of_one_package_are_two_things(rdk)
    a_saved_posed_arm_reopens_posed(rdk, station)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('station', type=Path)
    parser.add_argument('--robodk', type=Path, default=DEFAULT_EXE)
    parser.add_argument('--port', type=int, default=20510)
    args = parser.parse_args()

    station: Path = args.station.resolve()
    if not station.is_file():
        print(f'No such station: {station}', file=sys.stderr)
        return 1

    rdk = connect(args.robodk, station, args.port)
    try:
        run(rdk, station)
    except SmokeFailure as failure:
        print(f'FAIL: {failure}', file=sys.stderr)
        return 1
    finally:
        try:
            command(rdk, 'stop')
        except Exception:
            pass
        close_robodk(rdk)
    print('PASS: a package out of the library stands up as one generic node, is drawn where it was '
          'put, follows the parent it hangs under, snaps a conveyor end onto another to 0.000 mm, and '
          'goes away completely - and a placed arm is posed by the same kinematics RobotSimulator uses, '
          'moves the triangles that are drawn, refuses a pose it has not got, and reopens posed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
