#!/usr/bin/env python3
"""Generate first-party robot tool packages used by RobotSimulator's tool library."""

from __future__ import annotations

import json
import math
import shutil
from pathlib import Path

from pkglib import ROOT
from pkglib.archive import write_zip
from pkglib.geom import append_box, append_cylinder
from pkglib.hulls import convex_hull
from pkglib.meshbin import write_meshbin
from pkglib.nodes import TYPE_ROBOT_TOOL, mesh_node, node


def tcp_transform(axis: tuple[float, float, float], position) -> list[float]:
    # TCP Z follows the finger approach. X is across the gripper fingers.
    x = (0.0, 0.0, 1.0)
    z = axis
    y = (z[1], -z[0], 0.0)
    return [x[0], y[0], z[0], position[0],
            x[1], y[1], z[1], position[1],
            x[2], y[2], z[2], position[2]]


def build_hand_e_dual() -> None:
    package_dir = ROOT / "library" / "sources" / "robotiq_hand_e_dual_tool"
    archive = ROOT / "library" / "packages" / "robotiq_hand_e_dual_tool.zip"
    if package_dir.exists():
        shutil.rmtree(package_dir)
    (package_dir / "meshes").mkdir(parents=True)

    colors = {
        "robotiq_blue": [0.12, 0.48, 0.70, 1.0],
        "adapter": [0.18, 0.21, 0.24, 1.0],
        "metal": [0.64, 0.68, 0.71, 1.0],
        "jaw_static": [0.08, 0.09, 0.10, 1.0],
    }
    meshes = {name: ([], []) for name in colors}
    hulls: list[dict] = []

    def cylinder(group, label, center, axis, radius, length, sides=32):
        vertices, indices = meshes[group]
        primitive = append_cylinder(vertices, indices, center, axis, radius, length, sides)
        hulls.append(convex_hull(primitive, label, round_decimals=5, include_source=True))

    def box(group, label, center, size, basis=None):
        vertices, indices = meshes[group]
        primitive = append_box(vertices, indices, center, size, basis)
        hulls.append(convex_hull(primitive, label, round_decimals=5, include_source=True))

    # Tool-local Y points back out of the mount; geometry extends toward -Y so the mounting
    # solver's opposite-normal half-turn places it outward from the robot flange.
    cylinder("metal", "ISO 63 coupling", (0.0, -12.0, 0.0), (0.0, 1.0, 0.0), 42.0, 24.0)
    cylinder("adapter", "dual adapter hub", (0.0, -29.0, 0.0), (0.0, 1.0, 0.0), 58.0, 10.0)

    approach = 1.0 / math.sqrt(2.0)
    tcp_distance = 214.0
    for side, label in ((-1.0, "Left"), (1.0, "Right")):
        axis = (side * approach, -approach, 0.0)
        cross = (0.0, 0.0, 1.0)
        up = (approach, side * approach, 0.0)
        basis = (cross, axis, up)
        arm_center = tuple(axis[row] * 55.0 for row in range(3))
        box("adapter", f"{label} adapter arm", arm_center, (60.0, 84.0, 18.0), basis)
        coupling_center = tuple(axis[row] * 94.0 for row in range(3))
        cylinder("metal", f"{label} gripper coupling", coupling_center, axis, 31.0, 24.0)
        body_center = tuple(axis[row] * 142.0 for row in range(3))
        cylinder("robotiq_blue", f"{label} Hand-E body", body_center, axis, 37.5, 100.5)
        wrist_center = tuple(axis[row] * 193.0 for row in range(3))
        box("jaw_static", f"{label} finger carriage", wrist_center, (60.0, 16.0, 58.0), basis)
        for finger_side in (-1.0, 1.0):
            finger_center = [axis[row] * 218.0 + cross[row] * finger_side * 30.0
                             for row in range(3)]
            group = f"{label.lower()}_{'negative' if finger_side < 0 else 'positive'}_finger"
            colors[group] = [0.08, 0.09, 0.10, 1.0]
            meshes[group] = ([], [])
            vertices, indices = meshes[group]
            primitive = append_box(vertices, indices, tuple(finger_center),
                                   (14.0, 56.0, 16.0), basis)
            hulls.append(convex_hull(primitive, f"{label} finger {finger_side:+.0f}",
                                     round_decimals=5, include_source=True))

    children = []
    geometry_paths = []
    for name, color in colors.items():
        vertices, indices = meshes[name]
        source = f"meshes/{name}.meshbin"
        write_meshbin(package_dir / source, vertices, indices)
        geometry_paths.append([len(children)])
        children.append(mesh_node(name.replace("_", " ").title(), source, color))

    child_index = {name: index for index, name in enumerate(colors)}
    actuators = []
    for side, label, tcp_index, interaction in (
            (-1.0, "Left", 0, "logical-grasp"),
            (1.0, "Right", 1, "physics-grasp")):
        bindings = []
        for finger_side in (-1.0, 1.0):
            group = f"{label.lower()}_{'negative' if finger_side < 0 else 'positive'}_finger"
            bindings.append({
                "nodePath": [child_index[group]],
                "translationAxis": [0.0, 0.0, -finger_side],
                # Close to a 29 mm clear gap around the 30 mm demo payload. The remaining 0.5 mm
                # per pad supplies contact pressure without a deep kinematic overlap that would
                # eject a light workpiece as PhysX resolves the first manifold.
                "mmPerUnit": 8.5,
            })
        actuators.append({
            "id": f"{label.lower()}-grip",
            "name": f"{label} Hand-E",
            "position": 0.0,
            "lowerLimit": 0.0,
            "upperLimit": 1.0,
            "velocityUnitsPerSecond": 4.0,
            "effortLimit": 130.0,
            "interaction": interaction,
            "tcpIndex": tcp_index,
            "captureRadiusMm": 105.0,
            "bindings": bindings,
        })

    tcp_positions = [(-approach * tcp_distance, -approach * tcp_distance, 0.0),
                     (approach * tcp_distance, -approach * tcp_distance, 0.0)]
    tcp_axes = [(-approach, -approach, 0.0), (approach, -approach, 0.0)]
    root = node("Robotiq Hand-E Dual Gripper", TYPE_ROBOT_TOOL, {
        "geometryPaths": geometry_paths,
        "collisionHulls": hulls,
        "tcps": [
            {"name": "Left Hand-E TCP", "loc": tcp_transform(tcp_axes[0], tcp_positions[0])},
            {"name": "Right Hand-E TCP", "loc": tcp_transform(tcp_axes[1], tcp_positions[1])},
        ],
        "activeTcpIndex": 0,
        "actuators": actuators,
    }, colors["robotiq_blue"], children)
    root["libraryCategory"] = "tool"
    root["mountingHoles"] = {
        "comment": "FR20-compatible four-hole source interface: 4x M6 on a 63 mm pitch circle. The dual Hand-E geometry uses Robotiq's documented 75 mm body diameter, 100.5 mm body height and +/-45 degree dual-gripper TCP orientation. Generated by scripts/generate_tool_packages.py.",
        "placementSource": True,
        "mateOpposite": True,
        "interfaceId": "tool_mount",
        "pointsMm": [[31.5, 0.0, 0.0], [0.0, 0.0, 31.5],
                     [-31.5, 0.0, 0.0], [0.0, 0.0, -31.5]],
    }
    (package_dir / "accessory.json").write_text(json.dumps(root, indent=2) + "\n",
                                                 encoding="utf-8")
    write_zip(package_dir, archive)
    print(f"Built {archive}")


if __name__ == "__main__":
    build_hand_e_dual()
