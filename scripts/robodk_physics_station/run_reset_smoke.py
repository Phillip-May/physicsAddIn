"""Verify that top-level program plays begin clean runs and nested calls do not."""
from __future__ import annotations

import argparse
import re
import sys
import time

from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_physics_smoke import (SmokeFailure, close_robodk, command, connect, require_ok,
                               DEFAULT_EXE)

STATUS = re.compile(r'^(?P<state>\w+)\b.*?(?P<products>\d+) product\(s\), (?P<steps>\d+) step\(s\) '
                    r'over (?P<seconds>[\d.]+) s')
RUN_SECONDS = 6.0


def product_ids(rdk) -> set:
    """Return current conveyor product IDs; IDs are never reused within a session."""
    found = set()
    for record in command(rdk, 'conveyors').split(' || '):
        fields = [field.strip() for field in record.split(' | ')]
        if fields and fields[0].startswith('p '):
            found.add(int(fields[0][2:]))
    return found


def status(rdk) -> dict:
    """`status` parsed, or the raw reply when the run is not going."""
    reply = command(rdk, 'status')
    match = STATUS.match(reply)
    if not match:
        return {'state': reply.split(' ')[0], 'raw': reply}
    return {'state': match.group('state'), 'raw': reply,
            'products': int(match.group('products')), 'steps': int(match.group('steps')),
            'seconds': float(match.group('seconds'))}


def require_running(rdk, why: str) -> dict:
    current = status(rdk)
    if current['state'] != 'Running':
        raise SmokeFailure(f'{why}: the run is {current["raw"]!r}')
    return current


def play(rdk, name: str):
    from robodk import robolink

    program = rdk.Item(name, robolink.ITEM_TYPE_PROGRAM)
    if not program.Valid():
        raise SmokeFailure(f'This station has no {name!r} program')
    program.RunProgram()
    return program


def wait_quiet(rdk, program, seconds: float) -> bool:
    """Until `program` stops being busy. False if it was still going when the time ran out."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if not program.Busy():
            return True
        time.sleep(0.2)
    return False


def reset_is_noticed(rdk) -> None:
    print('--- Reset ends the run it is emptying')
    require_ok(rdk, 'start')
    time.sleep(RUN_SECONDS)
    before = require_running(rdk, 'the run would not stay up long enough to reset it')
    was = product_ids(rdk)
    print(f'  before: {before["raw"]}')
    print(f'  products: {sorted(was)}')
    if before['steps'] == 0:
        raise SmokeFailure('the run took no steps, so a restart could not be told from a stall')
    if not was:
        raise SmokeFailure('the run owned no products, so there is nothing a restart could inherit')

    program = play(rdk, 'Reset')
    if not wait_quiet(rdk, program, 30.0):
        raise SmokeFailure('Reset was still running after 30 s')
    time.sleep(1.0)
    after = require_running(rdk, 'Reset left the cell not running')
    now = product_ids(rdk)
    print(f'  after:  {after["raw"]}')
    print(f'  products: {sorted(now)}')
    if after['seconds'] >= before['seconds']:
        raise SmokeFailure(
            f'the run has still simulated {after["seconds"]:.2f} s, up from {before["seconds"]:.2f}. '
            'Playing Reset did not begin a new run, so the old one is still going with the scene '
            'emptied underneath it.')
    if was & now:
        raise SmokeFailure(f'the new run inherited product(s) {sorted(was & now)} from the old one')
    print('  the run was replaced, not inherited')


def stopped_run_comes_back(rdk) -> None:
    print('--- a play brings back a run the operator stopped')
    require_ok(rdk, 'stop')
    if status(rdk)['state'] != 'Stopped':
        raise SmokeFailure(f'stop left the run {status(rdk)["raw"]!r}')
    program = play(rdk, 'Reset')
    if not wait_quiet(rdk, program, 30.0):
        raise SmokeFailure('Reset was still running after 30 s')
    time.sleep(1.0)
    print(f'  {require_running(rdk, "a play did not restart a stopped run")["raw"]}')


def nested_calls_are_not_plays(rdk) -> None:
    print('--- Main runs to the end without restarting itself')
    require_ok(rdk, 'stop')
    require_ok(rdk, 'start')
    program = play(rdk, 'Main')
    highest = 0.0
    dips = []
    deadline = time.monotonic() + 240.0
    while time.monotonic() < deadline and program.Busy():
        current = status(rdk)
        seconds = current.get('seconds')
        if seconds is None:
            dips.append((round(highest, 2), current['raw']))
            break
        if seconds < highest:
            dips.append((round(highest, 2), round(seconds, 2)))
        highest = max(highest, seconds)
        time.sleep(0.25)
    still_going = program.Busy()
    program.Stop()
    print(f'  Main {"was stopped at the time limit" if still_going else "finished"}, '
          f'{highest:.1f} s simulated')
    if dips:
        raise SmokeFailure(
            f'the simulated time went backwards during Main: {dips}. A nested program call is '
            'being read as a play, so the run restarts part way through and the stack is lost.')
    if highest <= 0.0:
        raise SmokeFailure('the run never advanced during Main, so this proves nothing')
    print('  no nested call was read as a play')


def run(rdk) -> None:
    if not command(rdk, 'status'):
        raise SmokeFailure('The physics plugin did not answer.')
    require_ok(rdk, 'loadconfig')
    reset_is_noticed(rdk)
    stopped_run_comes_back(rdk)
    nested_calls_are_not_plays(rdk)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('station', type=Path, help='the physics variant .rdk')
    parser.add_argument('--robodk', type=Path, default=DEFAULT_EXE)
    parser.add_argument('--port', type=int, default=20601)
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
    print('PASS: a play begins a fresh run, a stopped run comes back, and nested calls are not '
          'plays.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
