#!/usr/bin/env python3

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import struct
import zipfile
from dataclasses import dataclass
from pathlib import Path

from pkglib import ROOT
from pkglib.archive import write_zip
from pkglib.geom import cylinder_mesh
from pkglib.hulls import convex_hull
from pkglib.meshbin import write_meshbin
from pkglib.nodes import (TYPE_MESH_GEOMETRY, TYPE_OPW6_ROBOT, TYPE_ROBOT_LINK,
                          TYPE_ROBOT_TOOL, node)

DEFAULT_PACKAGE_DIR = ROOT / "library" / "sources" / "fanuc_m800ia_60_robot"
DEFAULT_ZIP = ROOT / "library" / "packages" / "fanuc_m800ia_60_robot.zip"

FANUC_YELLOW = [1.0, 0.84, 0.0, 1.0]
BLACK = [0.075, 0.08, 0.075, 1.0]
RED = [0.82, 0.015, 0.005, 1.0]
ALUMINIUM = [0.48, 0.50, 0.50, 1.0]
AXIS_GOLD = [1.0, 0.84, 0.0, 1.0]
AXIS_RED = [1.0, 0.25, 0.16, 1.0]


@dataclass
class Shape:
    vertices: list[float]
    faces: list[tuple[int, int, int]]
    material_faces: dict[str, list[int]]


def u16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<H", data, offset)[0]


def u32(data: bytes, offset: int) -> int:
    return struct.unpack_from("<I", data, offset)[0]


def parse_shape(data: bytes, offset: int) -> Shape:
    shape_end = offset + u32(data, offset + 2)
    cursor = offset + 6
    if data[cursor:cursor + 2] != b"\x10\x41":
        raise ValueError(f"shape at 0x{offset:x}: missing vertex chunk")
    vertex_chunk_length = u32(data, cursor + 2)
    vertex_count = u16(data, cursor + 6)
    expected_vertex_length = 8 + vertex_count * 12
    if vertex_chunk_length != expected_vertex_length:
        raise ValueError(f"shape at 0x{offset:x}: malformed vertex chunk")
    raw_vertices = struct.unpack_from(f"<{vertex_count * 3}f", data, cursor + 8)
    vertices: list[float] = []
    for index in range(0, len(raw_vertices), 3):
        x, y, z = raw_vertices[index:index + 3]
        # Visual Components is Z-up; RobotSimulator is Y-up.
        vertices.extend((x, z, -y))
    cursor += vertex_chunk_length

    while cursor < shape_end and data[cursor:cursor + 2] != b"\x20\x41":
        chunk_length = u32(data, cursor + 2)
        if chunk_length < 6 or cursor + chunk_length > shape_end:
            raise ValueError(f"shape at 0x{offset:x}: malformed child chunk")
        cursor += chunk_length
    if data[cursor:cursor + 2] != b"\x20\x41":
        raise ValueError(f"shape at 0x{offset:x}: missing face chunk")

    face_chunk_end = cursor + u32(data, cursor + 2)
    face_count = u16(data, cursor + 6)
    face_cursor = cursor + 8
    faces: list[tuple[int, int, int]] = []
    for _ in range(face_count):
        a, b, c, _flags = struct.unpack_from("<4H", data, face_cursor)
        if max(a, b, c) >= vertex_count:
            raise ValueError(f"shape at 0x{offset:x}: face index out of range")
        faces.append((a, b, c))
        face_cursor += 8

    material_faces: dict[str, list[int]] = {}
    while face_cursor < face_chunk_end:
        if data[face_cursor:face_cursor + 2] != b"\x30\x41":
            raise ValueError(f"shape at 0x{offset:x}: malformed material group")
        group_length = u32(data, face_cursor + 2)
        payload = data[face_cursor + 6:face_cursor + group_length]
        terminator = payload.index(0)
        material = payload[:terminator].decode("utf-8")
        group_count = u16(payload, terminator + 1)
        ids_offset = terminator + 3
        face_ids = list(struct.unpack_from(f"<{group_count}H", payload, ids_offset))
        if face_ids and max(face_ids) >= face_count:
            raise ValueError(f"shape at 0x{offset:x}: material face index out of range")
        material_faces.setdefault(material, []).extend(face_ids)
        face_cursor += group_length

    assigned = sorted(face_id for ids in material_faces.values() for face_id in ids)
    if assigned != list(range(face_count)):
        raise ValueError(f"shape at 0x{offset:x}: material groups do not cover each face once")
    return Shape(vertices, faces, material_faces)


def parse_geometry_stream(data: bytes, source_name: str) -> list[Shape]:
    if len(data) < 16 or data[:2] != b"MM" or u32(data, 2) != len(data):
        raise ValueError(f"{source_name}: unsupported Visual Components geometry stream")
    shapes: list[Shape] = []
    cursor = 0
    while cursor + 12 <= len(data):
        if data[cursor:cursor + 2] == b"\x00\x41":
            length = u32(data, cursor + 2)
            if (length >= 12 and cursor + length <= len(data) and
                    data[cursor + 6:cursor + 8] == b"\x10\x41"):
                shapes.append(parse_shape(data, cursor))
                cursor += length
                continue
        cursor += 1
    if not shapes:
        raise ValueError(f"{source_name}: no mesh shapes found")
    return shapes


def material_color(name: str) -> list[float]:
    lowered = name.lower()
    if "red" in lowered:
        return RED
    if "black" in lowered:
        return BLACK
    if "aluminium" in lowered or "aluminum" in lowered:
        return ALUMINIUM
    # Several inherited FANUC-yellow parts retain generic eCatalog material names such as
    # vc_default, deep_blue, beige_matte, and light_aqua_matte in the raw stream.  The component
    # preview and material inheritance make them yellow in Visual Components, so do the same here.
    return FANUC_YELLOW


def submesh(shape: Shape, face_ids: list[int]) -> tuple[list[float], list[int]]:
    used = sorted({vertex for face_id in face_ids for vertex in shape.faces[face_id]})
    remap = {old: new for new, old in enumerate(used)}
    vertices = [component for old in used for component in shape.vertices[old * 3:old * 3 + 3]]
    indices = [remap[vertex] for face_id in face_ids for vertex in shape.faces[face_id]]
    return vertices, indices


def build_robot(source: Path, package_dir: Path) -> dict:
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    link_sources: list[list[str]] = [[] for _ in range(7)]
    geometry: dict[str, list[Shape]] = {}
    with zipfile.ZipFile(source) as archive:
        names = set(archive.namelist())
        base_names = sorted(
            (name for name in names if name.startswith("base_0__")),
            key=lambda name: int(name.rsplit("__", 1)[1]),
        )
        if not base_names:
            raise ValueError(f"{source}: no M-800iA base geometry found")
        link_sources[0] = base_names
        for link_index in range(1, 7):
            name = f"geometry_{link_index}"
            if name not in names:
                raise ValueError(f"{source}: missing {name}")
            link_sources[link_index] = [name]
        for name in [item for group in link_sources for item in group]:
            geometry[name] = parse_geometry_stream(archive.read(name), name)

    children: list[dict] = []
    total_faces = 0
    total_shapes = 0
    for link_index, source_names in enumerate(link_sources):
        link = node(f"Link {link_index}" + (" Base" if link_index == 0 else ""),
                    TYPE_ROBOT_LINK, {"geometryPaths": []}, FANUC_YELLOW)
        if link_index == 0:
            link["mountingHoles"] = {
                "comment": (
                    "Eight M-800iA/60 floor-fastener centers derived from the eight 24 mm "
                    "through-hole boundary rings in base_0__6 of M-800iA_60.vcmx (two per "
                    "clipped corner). The neighboring 20 mm and 17.5 mm alignment, jacking, and "
                    "service features are intentionally excluded. The snap centers remain on "
                    "the floor interface while markerOffsetMm places the view guides at the "
                    "visible top openings. Generated by "
                    "scripts/generate_fanuc_m800ia_package.py."
                ),
                "markerOffsetMm": [0.0, 30.0, 0.0],
                "placementSource": True,
                "grids": [
                    {
                        "originMm": [-220.0, 0.0, -170.0],
                        "uStepMm": [440.0, 0.0, 0.0],
                        "vStepMm": [0.0, 0.0, 340.0],
                        "uCount": 2,
                        "vCount": 2,
                    },
                    {
                        "originMm": [-170.0, 0.0, -220.0],
                        "uStepMm": [340.0, 0.0, 0.0],
                        "vStepMm": [0.0, 0.0, 440.0],
                        "uCount": 2,
                        "vCount": 2,
                    },
                ],
            }
        elif link_index == 3:
            link["mountingHoles"] = {
                "comment": (
                    "Four externally accessible J3-casing accessory mounting centers taken from "
                    "the 8.38 mm cylindrical openings in geometry_3. Internal joint and cover "
                    "fasteners are excluded. Generated by "
                    "scripts/generate_fanuc_m800ia_package.py."
                ),
                "grids": [{
                    "originMm": [512.0, 1750.0, -52.05],
                    "uStepMm": [45.0, 0.0, 0.0],
                    "vStepMm": [0.0, 0.0, 100.0],
                    "uCount": 2,
                    "vCount": 2,
                }],
            }
        elif link_index == 4:
            link["mountingHoles"] = {
                "comment": (
                    "Six externally accessible forearm accessory mounting centers taken from "
                    "the 8.38 mm cylindrical openings in geometry_4. Stored as the source 2 by 3 "
                    "pattern. Generated by scripts/generate_fanuc_m800ia_package.py."
                ),
                "grids": [{
                    "originMm": [868.0, 1716.0, -22.5],
                    "uStepMm": [170.0, 0.0, 0.0],
                    "vStepMm": [0.0, 0.0, 22.5],
                    "uCount": 2,
                    "vCount": 3,
                }],
            }
        elif link_index == 6:
            link["mountingHoles"] = {
                "comment": (
                    "J6 faceplate interface at X=1402 mm from geometry_6: ten 6.8 mm threaded-hole "
                    "centers and the two 8 mm locator-hole centers on the 50 mm pitch radius. The "
                    "central pilot and service ports are excluded. Generated by "
                    "scripts/generate_fanuc_m800ia_package.py."
                ),
                "pointsMm": [
                    [1402.0, 1675.0, 0.0],
                    [1402.0, 1668.3, 25.0],
                    [1402.0, 1650.0, 43.3],
                    [1402.0, 1625.0, 50.0],
                    [1402.0, 1600.0, 43.3],
                    [1402.0, 1581.7, 25.0],
                    [1402.0, 1575.0, 0.0],
                    [1402.0, 1581.7, -25.0],
                    [1402.0, 1600.0, -43.3],
                    [1402.0, 1625.0, -50.0],
                    [1402.0, 1650.0, -43.3],
                    [1402.0, 1668.3, -25.0],
                ],
            }
        children.append(link)
        collision_hulls = []
        for source_name in source_names:
            for shape_index, shape in enumerate(geometry[source_name]):
                total_shapes += 1
                total_faces += len(shape.faces)
                collision_hulls.append(convex_hull(shape.vertices, source_name,
                                                   max_vertices=128))
                for material, face_ids in sorted(shape.material_faces.items()):
                    vertices, indices = submesh(shape, face_ids)
                    mesh_name = f"link{link_index}_{source_name}_{shape_index}_{material}"
                    mesh_name = mesh_name.replace("/", "_").replace("\\", "_")
                    mesh_rel = f"meshes/{mesh_name}.meshbin"
                    write_meshbin(package_dir / mesh_rel, vertices, indices)
                    link["data"]["geometryPaths"].append([len(children)])
                    children.append(node(
                        f"{source_name} shape {shape_index + 1} ({material})",
                        TYPE_MESH_GEOMETRY,
                        {"meshSource": mesh_rel},
                        material_color(material),
                    ))
        if len(collision_hulls) > 128:
            raise ValueError(f"Link {link_index}: collision hull hard cap exceeded")
        link["data"]["collisionHulls"] = collision_hulls

    tool_index = len(children)
    children.append(node("J6 TCP", TYPE_ROBOT_TOOL, {
        "geometryPaths": [],
        "tcps": [{
            "name": "Faceplate center",
            "loc": [0.0, 0.0, 1.0, 1402.0, -1.0, 0.0, 0.0, 1625.0,
                    0.0, -1.0, 0.0, 0.0],
        }],
    }, ALUMINIUM))

    axes = [
        ((0.0, 0.0, 0.0), "y"),
        ((312.0, 600.0, 0.0), "z"),
        ((312.0, 1400.0, 0.0), "z"),
        ((312.0, 1625.0, 0.0), "x"),
        ((1212.0, 1625.0, 0.0), "z"),
        ((1212.0, 1625.0, 0.0), "x"),
    ]
    for joint, (origin, axis) in enumerate(axes, 1):
        vertices, indices = cylinder_mesh(origin, axis)
        mesh_rel = f"meshes/joint_axis_{joint}.meshbin"
        write_meshbin(package_dir / mesh_rel, vertices, indices)
        children.append(node(f"Joint {joint} Axis", TYPE_MESH_GEOMETRY,
                             {"meshSource": mesh_rel},
                             AXIS_GOLD if joint % 2 else AXIS_RED))

    speeds_deg_s = [150.0, 150.0, 150.0, 260.0, 260.0, 400.0]
    accelerations_deg_s2 = [600.0, 600.0, 600.0, 1040.0, 1040.0, 1600.0]
    return node("FANUC M-800iA/60", TYPE_OPW6_ROBOT, {
        # Exact home-pose axes from the source model: J2=(312,600), J3=(312,1400),
        # J4=(312,1625), J5/J6=(1212,1625), faceplate=(1402,1625), in viewer XY.
        "dhm": [
            0.0, -math.pi / 2.0, 0.0, 0.0, 0.0, math.pi,
            0.0, 312.0, 800.0, 225.0, 0.0, 0.0,
            0.0, -math.pi / 2.0, 0.0, -math.pi / 2.0, math.pi / 2.0, -math.pi / 2.0,
            600.0, 0.0, 0.0, 900.0, 0.0, 190.0,
        ],
        "qHome": [0.0] * 6,
        "qMin": [math.radians(value) for value in [-185.0, -90.0, -160.0, -360.0, -125.0, -360.0]],
        "qMax": [math.radians(value) for value in [185.0, 135.0, 180.0, 360.0, 125.0, 360.0]],
        "jointZeroOffsetRad": [0.0] * 6,
        "dhmCorrection": [0.0] * 24,
        "toolCalibration": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0],
        "baseCalibration": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0],
        "controlPeriodSec": 0.008,
        "defaultJointSpeedDegPerSec": 30.0,
        "defaultLinearSpeedMmPerSec": 250.0,
        "defaultLinearAccelerationMmSec2": 750.0,
        "defaultLinearJerkMmSec3": 3000.0,
        "defaultToolAngularSpeedRadSec": math.radians(90.0),
        "defaultToolAngularAccelerationRadSec2": math.radians(180.0),
        "defaultToolAngularJerkRadSec3": math.radians(720.0),
        "singularityThresholdRad": math.radians(5.0),
        "jointVelocityMaxRadS": [math.radians(value) for value in speeds_deg_s],
        "jointAccelerationMaxRadS2": [math.radians(value) for value in accelerations_deg_s2],
        "jointJerkMaxRadS3": [math.radians(value * 4.0) for value in accelerations_deg_s2],
        "motionLimitDerivation": {
            "schema": "fanuc_m800ia60_visual_components_v1",
            "generatedUtc": "2026-08-09",
            "generator": "scripts/generate_fanuc_m800ia_package.py",
            "sourceModel": source.name,
            "sourceSha256": source_hash,
            "sourceGeometryShapes": total_shapes,
            "sourceTriangleCount": total_faces,
            "sources": [
                "Visual Components eCatalog M-800iA_60.vcmx, local licensed installation asset",
                "https://www.fanucamerica.com/products/robot/m-800ia-60",
                "https://www.fanuc.co.jp/en/product/robot/model/m800ia60.html",
            ],
            "notes": [
                "Visual meshes and home-pose joint centers are converted from the real articulated Visual Components model, not procedural replacement geometry.",
                "J1, J2, J4, J5, and J6 limits and all acceleration values come from component.rsc in the source model.",
                "The source expresses the J3 limits as functions of J2. The current robot schema has fixed limits, so J3 uses the union of that coupled envelope (-160 to 180 degrees); invalid extreme J2/J3 combinations are not yet excluded.",
                "Jerk limits remain a documented simulator assumption of four times source acceleration because the source does not specify jerk.",
                "Each source shape produces one convex hull capped at 128 vertices; every link remains below the 128-hull hard cap.",
            ],
        },
        "activeToolPath": [tool_index],
        "collisionIgnores": [[0, 1], [1, 2], [2, 3], [3, 4], [4, 5], [5, 6],
                             [0, 2], [2, 4], [3, 5], [4, 6]],
    }, FANUC_YELLOW, children)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True,
                        help="Visual Components M-800iA_60.vcmx source (vendor data, not in the repo)")
    parser.add_argument("--package-dir", type=Path, default=DEFAULT_PACKAGE_DIR)
    parser.add_argument("--zip", type=Path, default=DEFAULT_ZIP)
    args = parser.parse_args()
    if not args.source.is_file():
        raise FileNotFoundError(args.source)
    if args.package_dir.exists():
        shutil.rmtree(args.package_dir)
    args.package_dir.mkdir(parents=True)
    robot = build_robot(args.source, args.package_dir)
    (args.package_dir / "robot.json").write_text(
        json.dumps(robot, indent=2) + "\n", encoding="utf-8")
    write_zip(args.package_dir, args.zip)
    print(f"Wrote package directory: {args.package_dir}")
    print(f"Wrote zip package: {args.zip}")
    print(f"Source SHA-256: {robot['data']['motionLimitDerivation']['sourceSha256']}")
    print(f"Source shapes: {robot['data']['motionLimitDerivation']['sourceGeometryShapes']}")
    print(f"Source triangles: {robot['data']['motionLimitDerivation']['sourceTriangleCount']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
