from __future__ import annotations

import argparse
import sys

from pathlib import Path

PLUGIN = 'Physics Simulation'
REPO_ROOT = Path(__file__).resolve().parents[2]
LIBRARY_ROOT = REPO_ROOT / 'library' / 'packages'


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('package', nargs='?', help='a .zip in the library, or a full path to one')
    parser.add_argument('--variant', default='', help='which preset, for a package that declares any')
    parser.add_argument('--at', nargs=3, type=float, default=[0.0, 0.0, 0.0],
                        metavar=('X', 'Y', 'Z'), help='millimetres, relative to the station root')
    parser.add_argument('--rx', type=float, default=0.0)
    parser.add_argument('--ry', type=float, default=0.0)
    parser.add_argument('--rz', type=float, default=0.0)
    parser.add_argument('--library', type=Path, default=LIBRARY_ROOT,
                        help=f'where the packages are (default {LIBRARY_ROOT})')
    parser.add_argument('--list', action='store_true', help='what is in the library')
    parser.add_argument('--placed', action='store_true', help='what the plugin has placed so far')
    parser.add_argument('--delete', metavar='NAME', help='remove one placed item by name')
    args = parser.parse_args()

    if args.list:
        if not args.library.is_dir():
            print(f'No library at {args.library}', file=sys.stderr)
            return 1
        for package in sorted(args.library.glob('*.zip')):
            print(package.name)
        print('\nFor what each one *is* - robot, rail, tool, accessory - and the presets each declares,'
              '\nrun tools\\library_catalogue_smoke.exe library\\packages; that is the same list a dock'
              '\nwould show, because both read Common/LibraryCatalogue.')
        return 0

    from robodk import robolink

    rdk = robolink.Robolink()
    if not rdk.Valid():
        print('No running RoboDK to talk to.', file=sys.stderr)
        return 1

    def command(verb: str, value: str = '') -> str:
        reply = rdk.PluginCommand(PLUGIN, verb, value)
        return '' if reply is None else str(reply).strip()

    command('libraryroot', str(args.library))

    if args.placed:
        print(command('library').replace(' || ', '\n'))
        return 0
    if args.delete:
        reply = command('librarydelete', args.delete)
        print(reply)
        return 0 if reply.startswith('OK') else 1
    if not args.package:
        parser.error('name a package, or pass --list, --placed or --delete')

    argument = '|'.join([args.package, args.variant,
                         *(str(value) for value in args.at),
                         str(args.rx), str(args.ry), str(args.rz)])
    reply = command('libraryplace', argument)
    print(reply)
    if not reply.startswith('OK'):
        return 1
    print(command('library').replace(' || ', '\n'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
