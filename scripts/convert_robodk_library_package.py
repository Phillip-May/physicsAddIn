#!/usr/bin/env python3

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import shutil
import sys
import time
import urllib.request
import urllib.parse
import zipfile
from collections import defaultdict
from pathlib import Path
from typing import Iterable

from pkglib import ROOT, slug
from pkglib.archive import write_zip
from pkglib.geom import append_box as add_box
from pkglib.hulls import convex_hull
from pkglib.meshbin import bounds_of, merge_bounds, write_meshbin
from pkglib.nodes import (DEFAULT_COLOR, TYPE_CUSTOM, TYPE_DRAG_CHAIN_MECHANISM,
                          TYPE_GANTRY_MECHANISM, TYPE_MESH_GEOMETRY, TYPE_OPW6_ROBOT,
                          TYPE_ROBOT_LINK, TYPE_ROBOT_TOOL, TYPE_TRANSFORM, mesh_node,
                          node as identity_node)

NUMBER_RE = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")


def repo_path(value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else ROOT / path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def download_source(spec: dict, cache_dir: Path) -> Path:
    source = spec["source"]
    if source.get("path"):
        path = repo_path(source["path"])
    else:
        url = source["url"]
        filename = source.get("filename") or Path(urllib.parse.urlparse(url).path).name
        path = cache_dir / filename
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            partial = path.with_suffix(path.suffix + ".partial")
            urllib.request.urlretrieve(url, partial)
            partial.replace(path)
    if not path.is_file():
        raise FileNotFoundError(path)
    expected = source.get("sha256", "").lower()
    actual = sha256(path)
    if expected and actual != expected:
        raise ValueError(f"{path}: SHA-256 {actual} does not match expected {expected}")
    return path


def extract_bracket(text: str, start: int, opening: str, closing: str) -> tuple[str, int]:
    depth = 0
    for index in range(start, len(text)):
        char = text[index]
        if char == opening:
            depth += 1
        elif char == closing:
            depth -= 1
            if depth == 0:
                return text[start + 1:index], index + 1
    raise ValueError(f"unterminated {opening}{closing} block")


def shape_blocks(text: str) -> Iterable[str]:
    cursor = 0
    while True:
        match = re.search(r"\bShape\s*\{", text[cursor:])
        if not match:
            return
        brace = cursor + match.end() - 1
        block, cursor = extract_bracket(text, brace, "{", "}")
        yield block


def bracket_field(block: str, field: str) -> str:
    match = re.search(rf"\b{re.escape(field)}\s*\[", block)
    if not match:
        raise ValueError(f"VRML Shape has no {field} field")
    value, _ = extract_bracket(block, match.end() - 1, "[", "]")
    return value


def parse_vrml(path: Path) -> list[tuple[list[float], list[int], list[float]]]:
    text = path.read_text(encoding="utf-8", errors="strict")
    if not text.startswith("#VRML V2.0"):
        raise ValueError(f"{path}: expected RoboDK VRML V2 output")
    pose_path = path.with_suffix(".pose.json")
    if not pose_path.is_file():
        raise FileNotFoundError(f"{pose_path}: missing RoboDK link-pose sidecar")
    pose = json.loads(pose_path.read_text(encoding="utf-8"))
    if len(pose) != 4 or any(len(row) != 4 for row in pose):
        raise ValueError(f"{pose_path}: expected a 4 by 4 pose matrix")
    groups: dict[tuple[float, float, float, float], tuple[list[float], list[int]]] = {}
    for block in shape_blocks(text):
        point_match = re.search(r"\bpoint\s*\[", block)
        index_match = re.search(r"\bcoordIndex\s*\[", block)
        if not point_match or not index_match:
            continue
        color_match = re.search(
            r"\bdiffuseColor\s+(%s)\s+(%s)\s+(%s)" %
            (NUMBER_RE.pattern, NUMBER_RE.pattern, NUMBER_RE.pattern), block)
        if color_match:
            color = tuple(round(float(color_match.group(i)), 6) for i in range(1, 4)) + (1.0,)
        else:
            color = tuple(DEFAULT_COLOR)
        raw_points, _ = extract_bracket(block, point_match.end() - 1, "[", "]")
        values = [float(value) for value in NUMBER_RE.findall(raw_points)]
        if len(values) % 3:
            raise ValueError(f"{path}: point count is not divisible by three")
        vertices: list[float] = []
        for offset in range(0, len(values), 3):
            x, y, z = values[offset:offset + 3]
            world_x = pose[0][0] * x + pose[0][1] * y + pose[0][2] * z + pose[0][3]
            world_y = pose[1][0] * x + pose[1][1] * y + pose[1][2] * z + pose[1][3]
            world_z = pose[2][0] * x + pose[2][1] * y + pose[2][2] * z + pose[2][3]
            vertices.extend((world_x, world_z, -world_y))

        raw_indices, _ = extract_bracket(block, index_match.end() - 1, "[", "]")
        polygons: list[list[int]] = []
        polygon: list[int] = []
        for value in (int(number) for number in re.findall(r"-?\d+", raw_indices)):
            if value == -1:
                if len(polygon) >= 3:
                    polygons.append(polygon)
                polygon = []
            else:
                polygon.append(value)
        if len(polygon) >= 3:
            polygons.append(polygon)

        merged_vertices, merged_indices = groups.setdefault(color, ([], []))
        base = len(merged_vertices) // 3
        merged_vertices.extend(vertices)
        for face in polygons:
            for index in range(1, len(face) - 1):
                merged_indices.extend((base + face[0], base + face[index], base + face[index + 1]))

    result = [(vertices, indices, list(color))
              for color, (vertices, indices) in groups.items() if vertices and indices]
    if not result:
        raise ValueError(f"{path}: no IndexedFaceSet geometry found")
    return result


def mesh_connected_components(vertices: list[float], indices: list[int],
                              weld_decimals: int = 5) -> list[list[int]]:
    """Return stable triangle components, welding duplicate CAD vertices by position."""
    triangle_count = len(indices) // 3
    parents = list(range(triangle_count))

    def find(value: int) -> int:
        while parents[value] != value:
            parents[value] = parents[parents[value]]
            value = parents[value]
        return value

    def union(a: int, b: int) -> None:
        a, b = find(a), find(b)
        if a != b:
            parents[b] = a

    position_owner: dict[tuple[float, float, float], int] = {}
    for triangle in range(triangle_count):
        for corner in range(3):
            vertex = indices[triangle * 3 + corner] * 3
            position = tuple(round(vertices[vertex + axis], weld_decimals) for axis in range(3))
            owner = position_owner.setdefault(position, triangle)
            union(triangle, owner)

    components: dict[int, list[int]] = defaultdict(list)
    for triangle in range(triangle_count):
        components[find(triangle)].append(triangle)

    def component_key(triangles: list[int]) -> tuple:
        used = [indices[triangle * 3 + corner]
                for triangle in triangles for corner in range(3)]
        component_vertices = [vertices[index * 3 + axis]
                              for index in used for axis in range(3)]
        return (*bounds_of(component_vertices), len(triangles))

    return sorted(components.values(), key=component_key)


def exclude_mesh_components(vertices: list[float], indices: list[int],
                            excluded: list[int], label: str) -> tuple[list[float], list[int]]:
    if not excluded:
        return vertices, indices
    components = mesh_connected_components(vertices, indices)
    invalid = sorted(index for index in set(excluded)
                     if index < 0 or index >= len(components))
    if invalid:
        raise ValueError(f"{label}: connected component indices out of range: {invalid}; "
                         f"mesh has {len(components)} components")
    kept_triangles = [triangle for component_index, component in enumerate(components)
                      if component_index not in set(excluded) for triangle in component]
    if not kept_triangles:
        raise ValueError(f"{label}: component exclusion removed all geometry")
    kept_indices = [indices[triangle * 3 + corner]
                    for triangle in kept_triangles for corner in range(3)]
    used = sorted(set(kept_indices))
    remap = {old: new for new, old in enumerate(used)}
    kept_vertices = [vertices[index * 3 + axis] for index in used for axis in range(3)]
    return kept_vertices, [remap[index] for index in kept_indices]


def robodk_connection(source: Path, port: int, executable: str):
    try:
        from robodk import robolink
    except ImportError as exc:
        raise RuntimeError("Install the RoboDK Python package: py -3 -m pip install robodk") from exc
    rdk = robolink.Robolink(
        port=port,
        args=["-NEWINSTANCE", "-HIDDEN", "-SKIPINI", "-EXIT_LAST_COM"],
        robodk_path=executable,
        quit_on_close=False)
    robot = rdk.AddFile(str(source))
    if not robot.Valid():
        rdk.Disconnect()
        raise RuntimeError(f"RoboDK could not load {source}")
    return robolink, rdk, robot


def close_robodk(rdk) -> None:
    process = getattr(rdk, "NEW_INSTANCE", None)
    try:
        rdk.CloseRoboDK()
    except Exception:
        pass
    if process is not None:
        try:
            process.wait(timeout=8.0)
        except Exception:
            try:
                process.terminate()
                process.wait(timeout=3.0)
            except Exception:
                try:
                    process.kill()
                except Exception:
                    pass


def export_link_vrml(source: Path, link_index: int, output: Path,
                     port: int, executable: str, pose_mode: str) -> None:
    robolink, rdk, robot = robodk_connection(source, port, executable)
    try:
        joint_poses = robot.JointPoses()
        if link_index >= len(joint_poses):
            raise RuntimeError(f"{source.name}: RoboDK has no joint pose for link {link_index}")
        link = robot.ObjectLink(link_index)
        if not link.Valid():
            raise RuntimeError(f"{source.name}: RoboDK has no link {link_index}")
        pointer = link.setParam("Convert", "Object")
        converted = robolink.Item(rdk, pointer)
        if not converted.Valid():
            raise RuntimeError(f"{source.name}: could not convert link {link_index} to an object")
        # Convert/Object exports link-local vertices. RobotSimulator animates visual geometry as
        # deltas from the serialized home pose, so a robot mesh must first be assembled with its
        # RoboDK home joint frame. Gantry geometry stays link-local because its carriage transform
        # supplies the runtime placement.
        if pose_mode == "joint_home":
            pose_values = [[float(joint_poses[link_index][row, col])
                            for col in range(4)] for row in range(4)]
        elif pose_mode == "link_local":
            pose_values = [[1.0 if row == col else 0.0 for col in range(4)] for row in range(4)]
        else:
            raise ValueError(f"unsupported RoboDK geometry pose mode: {pose_mode}")
        output.parent.mkdir(parents=True, exist_ok=True)
        rdk.Save(str(output), converted)
        if not output.is_file() or output.stat().st_size == 0:
            raise RuntimeError(f"{source.name}: RoboDK did not write {output}")
        output.with_suffix(".pose.json").write_text(
            json.dumps(pose_values, indent=2) + "\n", encoding="utf-8")
    finally:
        close_robodk(rdk)


def export_links(source: Path, link_count: int, export_dir: Path,
                 port_base: int, executable: str, reuse_session: bool,
                 pose_mode: str) -> list[Path]:
    export_dir.mkdir(parents=True, exist_ok=True)
    outputs = [export_dir / f"link_{index}.wrl" for index in range(link_count)]
    if not reuse_session:
        for index, output in enumerate(outputs):
            export_link_vrml(source, index, output, port_base + index, executable, pose_mode)
        return outputs

    robolink, rdk, robot = robodk_connection(source, port_base, executable)
    try:
        joint_poses = robot.JointPoses()
        if len(joint_poses) < link_count:
            raise RuntimeError(f"{source.name}: RoboDK returned only {len(joint_poses)} joint poses")
        for index, output in enumerate(outputs):
            link = robot.ObjectLink(index)
            pointer = link.setParam("Convert", "Object")
            converted = robolink.Item(rdk, pointer)
            if pose_mode == "joint_home":
                pose_values = [[float(joint_poses[index][row, col])
                                for col in range(4)] for row in range(4)]
            elif pose_mode == "link_local":
                pose_values = [[1.0 if row == col else 0.0 for col in range(4)] for row in range(4)]
            else:
                raise ValueError(f"unsupported RoboDK geometry pose mode: {pose_mode}")
            output.parent.mkdir(parents=True, exist_ok=True)
            rdk.Save(str(output), converted)
            output.with_suffix(".pose.json").write_text(
                json.dumps(pose_values, indent=2) + "\n", encoding="utf-8")
    finally:
        close_robodk(rdk)
    return outputs


def export_object(source: Path, item_name: str, output: Path, port: int,
                  executable: str, existing_port: int | None) -> Path:
    """Export one named RoboDK object without baking its station pose into its vertices."""
    try:
        from robodk import robolink
    except ImportError as exc:
        raise RuntimeError("Install the RoboDK Python package: py -3 -m pip install robodk") from exc

    owns_process = existing_port is None
    if owns_process:
        rdk = robolink.Robolink(
            port=port,
            args=["-NEWINSTANCE", "-HIDDEN", "-SKIPINI", "-EXIT_LAST_COM"],
            robodk_path=executable,
            quit_on_close=False)
        loaded = rdk.AddFile(str(source))
        if not loaded.Valid():
            close_robodk(rdk)
            raise RuntimeError(f"RoboDK could not load {source}")
    else:
        # This connection belongs to the operator. Disconnect below, but never close their licensed
        # RoboDK process or mutate the item's pose in the open source station.
        rdk = robolink.Robolink(port=existing_port, quit_on_close=False)
    try:
        item = rdk.Item(item_name, robolink.ITEM_TYPE_OBJECT)
        if not item.Valid():
            raise RuntimeError(f"RoboDK station has no object named {item_name!r}")
        output.parent.mkdir(parents=True, exist_ok=True)
        rdk.Save(str(output), item)
        if not output.is_file() or output.stat().st_size == 0:
            raise RuntimeError(f"RoboDK did not write {output}")
        identity = [[1.0 if row == col else 0.0 for col in range(4)] for row in range(4)]
        output.with_suffix(".pose.json").write_text(
            json.dumps(identity, indent=2) + "\n", encoding="utf-8")
        return output
    finally:
        if owns_process:
            close_robodk(rdk)
        else:
            rdk.Disconnect()


def convert_link_meshes(vrml_paths: list[Path], package_dir: Path,
                        link_colors: list[list[float]] | None = None,
                        excluded_geometry_groups: dict[str, list[int]] | None = None,
                        excluded_geometry_components: dict[str, dict[str, list[int]]] | None = None
                        ) -> tuple[list[list[dict]], list[list[dict]], list[list[float]]]:
    meshes_by_link: list[list[dict]] = []
    hulls_by_link: list[list[dict]] = []
    bounds_by_link: list[list[float]] = []
    for link_index, vrml in enumerate(vrml_paths):
        meshes: list[dict] = []
        hulls: list[dict] = []
        link_bounds: list[float] | None = None
        for group_index, (vertices, indices, source_color) in enumerate(parse_vrml(vrml)):
            excluded = (excluded_geometry_groups or {}).get(str(link_index), [])
            if group_index in excluded:
                continue
            excluded_components = ((excluded_geometry_components or {})
                                   .get(str(link_index), {}).get(str(group_index), []))
            vertices, indices = exclude_mesh_components(
                vertices, indices, excluded_components,
                f"RoboDK link {link_index} material {group_index}")
            override = link_colors[link_index] if link_colors and link_index < len(link_colors) else None
            color = override or source_color
            rel = f"meshes/link_{link_index}_{group_index}_{slug('_'.join(str(v) for v in color[:3]))}.meshbin"
            write_meshbin(package_dir / rel, vertices, indices)
            meshes.append(mesh_node(f"Link {link_index} geometry {group_index}", rel, color))
            hulls.append(convex_hull(vertices, f"RoboDK link {link_index} material {group_index}",
                                     round_decimals=5, include_source=True))
            link_bounds = merge_bounds(link_bounds, bounds_of(vertices))
        meshes_by_link.append(meshes)
        hulls_by_link.append(hulls)
        bounds_by_link.append(link_bounds or [0.0] * 6)
    return meshes_by_link, hulls_by_link, bounds_by_link


def dhm_transform(alpha: float, a: float, theta: float, d: float) -> list[list[float]]:
    crx, srx, crz, srz = math.cos(alpha), math.sin(alpha), math.cos(theta), math.sin(theta)
    return [[crz, -srz, 0.0, a],
            [crx * srz, crx * crz, -srx, -d * srx],
            [srx * srz, srx * crz, crx, d * crx],
            [0.0, 0.0, 0.0, 1.0]]


def mat_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[sum(a[row][k] * b[k][col] for k in range(4)) for col in range(4)]
            for row in range(4)]


def viewer_transform(matrix: list[list[float]]) -> list[list[float]]:
    s = [[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]]
    si = [[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]]
    return mat_mul(mat_mul(s, matrix), si)


def flange_matrix(dhm: list[float], q_home: list[float]) -> list[list[float]]:
    pose = [[1.0 if row == col else 0.0 for col in range(4)] for row in range(4)]
    for index in range(6):
        pose = mat_mul(pose, dhm_transform(
            dhm[12 + index], dhm[6 + index], dhm[index] + q_home[index], dhm[18 + index]))
    pose = viewer_transform(pose)
    approach_to_z = [[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]]
    pose = mat_mul(pose, approach_to_z)
    return pose


def matrix_loc(pose: list[list[float]]) -> list[float]:
    return [0.0 if abs(pose[row][col]) < 1.0e-12 else round(pose[row][col], 12)
            for row in range(3) for col in range(4)]


def flange_loc(dhm: list[float], q_home: list[float]) -> list[float]:
    return matrix_loc(flange_matrix(dhm, q_home))


def flange_mounting_interface_loc(dhm: list[float], q_home: list[float]) -> list[float]:
    flange = flange_matrix(dhm, q_home)
    interface = [[0.0] * 4 for _ in range(4)]
    for row in range(3):
        interface[row][0] = flange[row][0]
        interface[row][1] = flange[row][2]
        interface[row][2] = -flange[row][1]
        interface[row][3] = flange[row][3]
    interface[3][3] = 1.0
    return matrix_loc(interface)


def validate_robot_home_frames(dhm: list[float], q_home: list[float],
                               vrml_paths: list[Path], tolerance: float = 1.0e-5) -> None:
    """Prove imported robot geometry is assembled in the same home frames as its DH table."""
    expected = [[1.0 if row == col else 0.0 for col in range(4)] for row in range(4)]
    expected_poses = [expected]
    for index in range(6):
        expected = mat_mul(expected, dhm_transform(
            dhm[12 + index], dhm[6 + index],
            dhm[index] + q_home[index], dhm[18 + index]))
        expected_poses.append(expected)
    if len(vrml_paths) != len(expected_poses):
        raise ValueError(f"robot requires 7 base/link home frames, found {len(vrml_paths)}")
    for index, vrml in enumerate(vrml_paths):
        actual = json.loads(vrml.with_suffix(".pose.json").read_text(encoding="utf-8"))
        maximum_error = max(abs(float(actual[row][col]) - expected_poses[index][row][col])
                            for row in range(3) for col in range(4))
        if maximum_error > tolerance:
            raise ValueError(
                f"RoboDK link {index} home frame disagrees with the robot DH table "
                f"(maximum matrix error {maximum_error:.6g})")


def radians(values: Iterable[float]) -> list[float]:
    return [math.radians(float(value)) for value in values]


def build_robot(spec: dict, source: Path, vrml_paths: list[Path], package_dir: Path) -> dict:
    model = spec["robot"]
    q_home = radians(model["qHomeDeg"])
    validate_robot_home_frames(model["dhm"], q_home, vrml_paths)
    meshes_by_link, hulls_by_link, _ = convert_link_meshes(
        vrml_paths, package_dir, model.get("linkColors"),
        model.get("excludedGeometryGroups"), model.get("excludedGeometryComponents"))
    children: list[dict] = []
    for link_index, meshes in enumerate(meshes_by_link):
        link = identity_node(f"Link {link_index}", TYPE_ROBOT_LINK,
                             {"geometryPaths": [], "collisionHulls": hulls_by_link[link_index]})
        if link_index == 0 and model.get("mountingHoles"):
            link["mountingHoles"] = model["mountingHoles"]
        children.append(link)
        paths = []
        for mesh in meshes:
            paths.append([len(children)])
            children.append(mesh)
        link["data"]["geometryPaths"] = paths

    tool = identity_node("Flange TCP", TYPE_ROBOT_TOOL, {
        "geometryPaths": [],
        "tcps": [{"name": "TCP", "loc": flange_loc(model["dhm"], q_home)}],
    }, [0.45, 0.48, 0.52, 1.0])
    if model.get("flangeMountingHoles"):
        flange_interface = identity_node("Robot flange mounting interface", TYPE_TRANSFORM, {})
        flange_interface["loc"] = flange_mounting_interface_loc(model["dhm"], q_home)
        flange_interface["mountingHoles"] = model["flangeMountingHoles"]
        tool["children"] = [flange_interface]
    active_tool_path = [len(children)]
    children.append(tool)
    derivation = {
        "schema": "robodk_library_import_v1",
        "generator": "scripts/convert_robodk_library_package.py",
        "sourceUrl": spec["source"].get("url", ""),
        "sourceSha256": sha256(source),
        "sourceItem": spec["source"]["itemName"],
        "notes": model.get("notes", []),
    }
    data = {
        "dhm": model["dhm"],
        "qHome": q_home,
        "qMin": radians(model["qMinDeg"]),
        "qMax": radians(model["qMaxDeg"]),
        "jointZeroOffsetRad": [0.0] * 6,
        "dhmCorrection": [0.0] * 24,
        "toolCalibration": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0],
        "baseCalibration": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0],
        "controlPeriodSec": model.get("controlPeriodSec", 0.008),
        "defaultJointSpeedDegPerSec": model.get("defaultJointSpeedDegPerSec", 30.0),
        "defaultLinearSpeedMmPerSec": model.get("defaultLinearSpeedMmPerSec", 250.0),
        "defaultLinearAccelerationMmSec2": model.get("defaultLinearAccelerationMmSec2", 750.0),
        "defaultLinearJerkMmSec3": model.get("defaultLinearJerkMmSec3", 3000.0),
        "defaultToolAngularSpeedRadSec": model.get("defaultToolAngularSpeedRadSec", math.pi / 2),
        "defaultToolAngularAccelerationRadSec2": model.get("defaultToolAngularAccelerationRadSec2", math.pi),
        "defaultToolAngularJerkRadSec3": model.get("defaultToolAngularJerkRadSec3", 4 * math.pi),
        "singularityThresholdRad": math.radians(model.get("singularityThresholdDeg", 5.0)),
        "jointVelocityMaxRadS": radians(model["jointVelocityMaxDegS"]),
        "jointAccelerationMaxRadS2": radians(model["jointAccelerationMaxDegS2"]),
        "jointJerkMaxRadS3": radians(model["jointJerkMaxDegS3"]),
        "motionLimitDerivation": derivation,
        "activeToolPath": active_tool_path,
        "collisionIgnores": [[i, i + 1] for i in range(6)] + model.get("collisionIgnores", []),
    }
    return identity_node(spec["name"], TYPE_OPW6_ROBOT, data,
                         model.get("color", DEFAULT_COLOR), children)


def make_chain_link(path: Path, pitch: float, width: float, height: float) -> None:
    vertices: list[float] = []
    indices: list[int] = []
    side = max(10.0, width * 0.10)
    cross = max(8.0, pitch * 0.10)
    # DragChainPoseController places each frame at its local-zero hinge and points +X toward the
    # next hinge. Keep the complete member in [0, pitch]; centering it around zero creates a
    # half-pitch visual gap at the moving/fixed connector even though all pivots are correctly
    # spaced.
    add_box(vertices, indices, (0.5 * pitch, 0, -0.5 * (width - side)),
            (pitch, height, side))
    add_box(vertices, indices, (0.5 * pitch, 0, 0.5 * (width - side)),
            (pitch, height, side))
    add_box(vertices, indices, (0.5 * cross, 0, 0),
            (cross, height * 0.72, width))
    add_box(vertices, indices, (pitch - 0.5 * cross, 0, 0),
            (cross, height * 0.72, width))
    write_meshbin(path, vertices, indices)


def make_box_mesh(path: Path, center: list[float], size: list[float]) -> None:
    vertices: list[float] = []
    indices: list[int] = []
    add_box(vertices, indices, tuple(center), tuple(size))
    write_meshbin(path, vertices, indices)


def build_gantry(spec: dict, source: Path, vrml_paths: list[Path], package_dir: Path) -> dict:
    model = spec["gantry"]
    meshes_by_link, hulls_by_link, bounds_by_link = convert_link_meshes(
        vrml_paths, package_dir, model.get("linkColors"),
        model.get("excludedGeometryGroups"), model.get("excludedGeometryComponents"))
    base = identity_node("Base", TYPE_CUSTOM, {}, model.get("color", DEFAULT_COLOR), meshes_by_link[0])
    carriage = identity_node("Carriage", TYPE_TRANSFORM, {}, model.get("color", DEFAULT_COLOR), meshes_by_link[1])
    if model.get("baseMountingHoles"):
        base["mountingHoles"] = model["baseMountingHoles"]
    if model.get("carriageMountingHoles"):
        carriage["mountingHoles"] = model["carriageMountingHoles"]

    chain = model["dragChain"]
    bracket_color = chain.get("color", [0.10, 0.11, 0.12, 1.0])
    if chain.get("fixedBracket"):
        rel = "meshes/drag_chain_fixed_bracket.meshbin"
        make_box_mesh(package_dir / rel, chain["fixedBracket"]["centerMm"],
                      chain["fixedBracket"]["sizeMm"])
        base["children"].append(mesh_node("Drag chain fixed bracket", rel, bracket_color))
    if chain.get("movingBracket"):
        rel = "meshes/drag_chain_moving_bracket.meshbin"
        make_box_mesh(package_dir / rel, chain["movingBracket"]["centerMm"],
                      chain["movingBracket"]["sizeMm"])
        carriage["children"].append(mesh_node("Drag chain moving bracket", rel, bracket_color))
    chain_rel = "meshes/drag_chain_link.meshbin"
    make_chain_link(package_dir / chain_rel, chain["pitchMm"], chain["widthMm"], chain["heightMm"])
    prototype = mesh_node("Drag chain link prototype", chain_rel, chain.get("color", [0.10, 0.11, 0.12, 1.0]))
    frame_count = int(chain["linkCount"])
    drag_children = [prototype] + [identity_node(f"Link frame {index + 1}", TYPE_TRANSFORM)
                                   for index in range(frame_count)]
    drag_data = {
        "fixedAnchorMm": chain["fixedAnchorMm"],
        "movingAnchorMm": chain["movingAnchorMm"],
        "travelAxis": model["axisOfTravel"],
        "hingeAxis": chain.get("hingeAxis", [0.0, 0.0, 1.0]),
        "departureAxisSign": chain.get("departureAxisSign", -1.0),
        "pitchMm": chain["pitchMm"],
        "bendRadiusMm": chain["bendRadiusMm"],
        "linkMassKg": chain.get("linkMassKg", 0.35),
        "maxJointRotationDeg": chain.get("maxJointRotationDeg", 20.0),
        "physicsEnabled": chain.get("physicsEnabled", True),
        "reverseFixedEndMember": chain.get("reverseFixedEndMember", True),
        "fixedEndMemberOffsetMm": chain.get("fixedEndMemberOffsetMm", [0.0, 0.0, 0.0]),
        "movingFramePath": [1],
        "prototypeGeometryPath": [2, 0],
        "linkFramePaths": [[2, index] for index in range(1, frame_count + 1)],
    }
    if chain.get("comment"):
        drag_data["comment"] = chain["comment"]
    if chain.get("sourcePartNumber"):
        drag_data["sourcePartNumber"] = chain["sourcePartNumber"]
    drag = identity_node("Drag Chain", TYPE_DRAG_CHAIN_MECHANISM, drag_data,
                         chain.get("color", [0.10, 0.11, 0.12, 1.0]), drag_children)
    bake = {
        "comment": "Single convex hull baked from the RoboDK stationary rail link during import.",
        "generator": "scripts/convert_robodk_library_package.py",
        "sourceUrl": spec["source"].get("url", ""),
        "sourceSha256": sha256(source),
        "sourceBoundsMm": bounds_by_link[0],
    }
    data = {
        "axisOfTravel": model["axisOfTravel"],
        "positionMm": model.get("positionMm", 0.0),
        "homePositionMm": model.get("homePositionMm", 0.0),
        "lowerLimitMm": model["lowerLimitMm"],
        "upperLimitMm": model["upperLimitMm"],
        "velocityMaxMmS": model.get("velocityMaxMmS", 1000.0),
        "baseCollisionHulls": hulls_by_link[0],
        "baseCollisionHullBake": bake,
        "movingFramePath": [1],
    }
    gantry = identity_node(spec["name"], TYPE_GANTRY_MECHANISM, data,
                           model.get("color", DEFAULT_COLOR), [base, carriage, drag])
    # The pattern the whole mechanism is placed *by*, on the root, the way build_accessory takes one and
    # the way convert_ws2000_gantry_package.py writes one. Its absence was not a neutral silence: a package
    # declaring no placement source cannot be armed at all, in either cell, so an imported rail could be
    # listed in the library and put nowhere. baseMountingHoles above is a different thing - holes other
    # things bolt onto the base with - and reclassifying those would make the rail placeable by its own
    # fixture surface.
    if model.get("mountingHoles"):
        gantry["mountingHoles"] = model["mountingHoles"]
    return gantry


def build_accessory(spec: dict, source: Path, vrml_path: Path, package_dir: Path) -> dict:
    model = spec["accessory"]
    meshes_by_link, _, bounds_by_link = convert_link_meshes(
        [vrml_path], package_dir, model.get("linkColors"),
        model.get("excludedGeometryGroups"), model.get("excludedGeometryComponents"))
    children = list(meshes_by_link[0])
    for interface in model.get("mountingInterfaces", []):
        node = identity_node(interface["name"], TYPE_TRANSFORM)
        node["mountingHoles"] = interface["mountingHoles"]
        children.append(node)
    root = identity_node(spec["name"], TYPE_TRANSFORM, {
        "sourceImport": {
            "schema": "robodk_library_import_v1",
            "generator": "scripts/convert_robodk_library_package.py",
            "sourceUrl": spec["source"].get("url", ""),
            "sourceSha256": sha256(source),
            "sourceItem": spec["source"]["itemName"],
            "sourceBoundsMm": bounds_by_link[0],
            "notes": model.get("notes", []),
        }
    }, model.get("color", DEFAULT_COLOR), children)
    if model.get("mountingHoles"):
        root["mountingHoles"] = model["mountingHoles"]
    return root


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("spec", type=Path, help="RoboDK import specification JSON")
    parser.add_argument("--cache-dir", type=Path, default=ROOT / ".cache" / "robodk_import")
    parser.add_argument("--robodk", default=r"C:\RoboDK\bin\RoboDK.exe")
    parser.add_argument("--port-base", type=int, default=20600)
    parser.add_argument("--reuse-session", action="store_true",
                        help="Export all links through one licensed RoboDK process")
    parser.add_argument("--reuse-vrml", action="store_true",
                        help="Skip RoboDK when all cached link_*.wrl files exist")
    parser.add_argument("--existing-port", type=int,
                        help="Use and disconnect from an already-running licensed RoboDK instance")
    args = parser.parse_args()

    spec_path = args.spec if args.spec.is_absolute() else ROOT / args.spec
    spec = json.loads(spec_path.read_text(encoding="utf-8"))
    kind = spec["kind"]
    if kind not in ("robot", "gantry", "accessory"):
        parser.error("spec kind must be robot, gantry or accessory")
    source = download_source(spec, args.cache_dir)
    package_dir = repo_path(spec["output"]["packageDir"])
    output_zip = repo_path(spec["output"]["zip"])
    pose_mode = "joint_home" if kind == "robot" else "link_local"
    # Keep incompatible pose conventions in separate cache folders so --reuse-vrml cannot silently
    # reuse sidecars produced by an older or different asset type.
    export_dir = args.cache_dir / slug(spec["name"]) / pose_mode
    link_count = int(spec[kind].get("linkCount", 1))
    vrml_paths = [export_dir / f"link_{index}.wrl" for index in range(link_count)]
    cached_complete = all(path.is_file() and path.with_suffix(".pose.json").is_file()
                          for path in vrml_paths)
    if not args.reuse_vrml or not cached_complete:
        if kind == "accessory":
            export_object(source, spec["source"]["itemName"], vrml_paths[0], args.port_base,
                          args.robodk, args.existing_port)
        else:
            export_links(source, link_count, export_dir, args.port_base, args.robodk,
                         args.reuse_session, pose_mode)

    if package_dir.exists():
        shutil.rmtree(package_dir)
    package_dir.mkdir(parents=True)
    if kind == "robot":
        root = build_robot(spec, source, vrml_paths, package_dir)
        manifest = "robot.json"
    elif kind == "gantry":
        root = build_gantry(spec, source, vrml_paths, package_dir)
        manifest = "mechanism.json"
    else:
        root = build_accessory(spec, source, vrml_paths[0], package_dir)
        manifest = "accessory.json"
    (package_dir / manifest).write_text(json.dumps(root, indent=2) + "\n", encoding="utf-8")
    write_zip(package_dir, output_zip)
    with zipfile.ZipFile(output_zip) as archive:
        chain_meshes = [name for name in archive.namelist() if name.endswith("drag_chain_link.meshbin")]
        if kind == "gantry" and len(chain_meshes) != 1:
            raise RuntimeError(f"gantry archive must contain exactly one chain mesh, found {len(chain_meshes)}")
    print(f"Built {spec['name']}: {output_zip}")


if __name__ == "__main__":
    main()
