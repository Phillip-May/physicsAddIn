#!/usr/bin/env python3
"""Convert the WS2000 COLLADA export into a RobotSimulator gantry package."""

from __future__ import annotations

import argparse
import json
import math
import shutil
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from xml.etree import ElementTree as ET

from pkglib import ROOT, slug
from pkglib.archive import write_zip
from pkglib.meshbin import write_meshbin
from pkglib.nodes import (DEFAULT_COLOR, TYPE_CUSTOM, TYPE_DRAG_CHAIN_MECHANISM,
                          TYPE_GANTRY_MECHANISM, TYPE_TRANSFORM, mesh_node, node)

COLLADA_NS = "http://www.collada.org/2005/11/COLLADASchema"
NS = {"c": COLLADA_NS}

DEFAULT_PACKAGE_DIR = ROOT / "library" / "sources" / "ws2000_gantry"
DEFAULT_ZIP = ROOT / "library" / "packages" / "ws2000_gantry.zip"
BASE_COLLISION_MAX_HULLS = 128
MAX_COLLISION_HULLS = 128


def source_values(mesh: ET.Element, source_id: str) -> tuple[list[float], int]:
    source = mesh.find(f"c:source[@id='{source_id}']", NS)
    if source is None:
        raise ValueError(f"COLLADA source not found: {source_id}")
    values_node = source.find("c:float_array", NS)
    accessor = source.find("c:technique_common/c:accessor", NS)
    if values_node is None or not values_node.text:
        raise ValueError(f"COLLADA source has no float array: {source_id}")
    stride = int(accessor.get("stride", "1")) if accessor is not None else 1
    return [float(value) for value in values_node.text.split()], stride


def position_source_id(mesh: ET.Element, vertices_url: str) -> str:
    vertices_id = vertices_url.lstrip("#")
    vertices = mesh.find(f"c:vertices[@id='{vertices_id}']", NS)
    if vertices is None:
        raise ValueError(f"COLLADA vertices not found: {vertices_id}")
    position = vertices.find("c:input[@semantic='POSITION']", NS)
    if position is None:
        raise ValueError(f"COLLADA vertices has no POSITION input: {vertices_id}")
    return position.get("source", "").lstrip("#")


def material_colors(document: ET.Element) -> tuple[dict[str, str], dict[str, list[float]]]:
    effects: dict[str, list[float]] = {}
    for effect in document.findall(".//c:library_effects/c:effect", NS):
        diffuse = effect.find(".//c:diffuse/c:color", NS)
        if diffuse is not None and diffuse.text:
            values = [float(value) for value in diffuse.text.split()]
            if len(values) == 4:
                effects[effect.get("id", "")] = values

    colors: dict[str, list[float]] = {}
    names: dict[str, str] = {}
    for material in document.findall(".//c:library_materials/c:material", NS):
        material_id = material.get("id", "")
        instance = material.find("c:instance_effect", NS)
        effect_id = instance.get("url", "").lstrip("#") if instance is not None else ""
        colors[material_id] = effects.get(effect_id, DEFAULT_COLOR)
        names[material_id] = material.get("name", material_id)

    # Primitive material attributes are symbols. Resolve the visual-scene binding when the symbol
    # differs from the library material id; the WS2000 currently uses identical values for both.
    for binding in document.findall(".//c:instance_material", NS):
        symbol = binding.get("symbol", "")
        target = binding.get("target", "").lstrip("#")
        if target in colors:
            colors[symbol] = colors[target]
            names[symbol] = names.get(target, target)
    return names, colors


def ros_metres_to_viewer_mm(position: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = position
    return 1000.0 * x, 1000.0 * z, -1000.0 * y


def convert_dae(path: Path, package_dir: Path, prefix: str
                ) -> tuple[list[dict], int, list[tuple[list[float], list[int]]]]:
    document = ET.parse(path).getroot()
    unit = document.find("c:asset/c:unit", NS)
    up_axis = document.findtext("c:asset/c:up_axis", namespaces=NS)
    if unit is None or abs(float(unit.get("meter", "0")) - 1.0) > 1.0e-12 or up_axis != "Z_UP":
        raise ValueError(f"{path.name}: expected metre, Z_UP COLLADA input")

    material_names, colors = material_colors(document)
    output_nodes: list[dict] = []
    face_total = 0
    group_index = 0
    collision_meshes: list[tuple[list[float], list[int]]] = []

    for geometry in document.findall(".//c:library_geometries/c:geometry", NS):
        mesh = geometry.find("c:mesh", NS)
        if mesh is None:
            continue
        for primitive in list(mesh):
            if primitive.tag != f"{{{COLLADA_NS}}}triangles":
                continue
            inputs = primitive.findall("c:input", NS)
            vertex_input = next((item for item in inputs if item.get("semantic") == "VERTEX"), None)
            index_node = primitive.find("c:p", NS)
            if vertex_input is None or index_node is None or not index_node.text:
                raise ValueError(f"{path.name}: malformed triangles primitive")

            input_stride = max(int(item.get("offset", "0")) for item in inputs) + 1
            vertex_offset = int(vertex_input.get("offset", "0"))
            positions_id = position_source_id(mesh, vertex_input.get("source", ""))
            positions, position_stride = source_values(mesh, positions_id)
            if position_stride < 3:
                raise ValueError(f"{path.name}: position source stride is less than three")

            raw_indices = [int(value) for value in index_node.text.split()]
            expected_faces = int(primitive.get("count", "0"))
            if len(raw_indices) != expected_faces * 3 * input_stride:
                raise ValueError(f"{path.name}: triangle index count does not match primitive count")

            remap: dict[int, int] = {}
            vertices: list[float] = []
            indices: list[int] = []
            for cursor in range(0, len(raw_indices), input_stride):
                source_index = raw_indices[cursor + vertex_offset]
                output_index = remap.get(source_index)
                if output_index is None:
                    base = source_index * position_stride
                    converted = ros_metres_to_viewer_mm(
                        (positions[base], positions[base + 1], positions[base + 2]))
                    output_index = len(vertices) // 3
                    remap[source_index] = output_index
                    vertices.extend(converted)
                indices.append(output_index)

            material_id = primitive.get("material", "")
            material_name = material_names.get(material_id, material_id or f"material_{group_index}")
            mesh_rel = f"meshes/{prefix}_{group_index:02d}_{slug(material_name)}.meshbin"
            write_meshbin(package_dir / mesh_rel, vertices, indices)
            output_nodes.append(mesh_node(material_name, mesh_rel,
                                          colors.get(material_id, DEFAULT_COLOR)))
            collision_meshes.append((vertices, indices))
            face_total += expected_faces
            group_index += 1

    if not output_nodes:
        raise ValueError(f"{path.name}: no triangle primitives found")
    return output_nodes, face_total, collision_meshes


def bake_base_collision_hulls(meshes: list[tuple[list[float], list[int]]],
                              max_hulls: int = BASE_COLLISION_MAX_HULLS) -> list[dict]:
    if max_hulls < 1 or max_hulls > MAX_COLLISION_HULLS:
        raise ValueError(
            f"base collision hull limit must be between 1 and {MAX_COLLISION_HULLS}")
    try:
        import coacd
        import numpy as np
    except ImportError as exc:
        raise RuntimeError(
            "Offline WS2000 base collision baking requires the coacd and numpy Python packages.") from exc

    vertex_groups = []
    face_groups = []
    vertex_offset = 0
    for vertices, indices in meshes:
        group_vertices = np.asarray(vertices, dtype=np.float64).reshape((-1, 3))
        group_faces = np.asarray(indices, dtype=np.int64).reshape((-1, 3)) + vertex_offset
        vertex_groups.append(group_vertices)
        face_groups.append(group_faces)
        vertex_offset += len(group_vertices)
    vertices = np.vstack(vertex_groups)
    faces = np.vstack(face_groups)

    # The source CAD tessellation has many near-identical vertices and almost 200k triangles.
    # Weld it to a 5 mm collision grid before decomposition. This is fine enough to preserve the
    # chain brackets and cover details, while still removing the source tessellation's duplicate
    # vertices before the deliberately higher-quality CoACD pass.
    grid_mm = 5.0
    quantized = np.rint(vertices / grid_mm).astype(np.int64)
    unique_quantized, inverse = np.unique(quantized, axis=0, return_inverse=True)
    remapped_faces = inverse[faces]
    nondegenerate = ((remapped_faces[:, 0] != remapped_faces[:, 1]) &
                     (remapped_faces[:, 1] != remapped_faces[:, 2]) &
                     (remapped_faces[:, 0] != remapped_faces[:, 2]))
    remapped_faces = remapped_faces[nondegenerate]
    face_keys = np.sort(remapped_faces, axis=1)
    _, first_face_indices = np.unique(face_keys, axis=0, return_index=True)
    remapped_faces = remapped_faces[np.sort(first_face_indices)]
    used_vertices, compact_faces = np.unique(remapped_faces, return_inverse=True)
    simplified_vertices = unique_quantized[used_vertices].astype(np.float64) * grid_mm
    simplified_faces = compact_faces.reshape((-1, 3)).astype(np.int32)

    parts = coacd.run_coacd(
        coacd.Mesh(simplified_vertices, simplified_faces),
        threshold=0.05,
        max_convex_hull=max_hulls,
        preprocess_mode="auto",
        preprocess_resolution=50,
        resolution=1500,
        mcts_nodes=10,
        mcts_iterations=80,
        mcts_max_depth=3,
        merge=True,
        decimate=True,
        max_ch_vertex=64,
        seed=0)
    if not parts:
        raise RuntimeError("CoACD produced no WS2000 base collision hulls.")
    if len(parts) > MAX_COLLISION_HULLS:
        raise RuntimeError(
            f"CoACD produced {len(parts)} hulls; hard cap is {MAX_COLLISION_HULLS}")
    hulls = []
    for part_vertices, part_faces in parts:
        if len(part_vertices) < 4 or len(part_vertices) > 255 or len(part_faces) == 0:
            raise RuntimeError("CoACD produced a malformed WS2000 base collision hull.")
        hulls.append({
            "vertices": np.asarray(part_vertices, dtype=np.float64).tolist(),
            "indices": np.asarray(part_faces, dtype=np.int64).tolist(),
        })
    return hulls


def build_package(source_dir: Path, package_dir: Path, min_chain_links: int = 13,
                  max_chain_joint_deg: float = 35.0,
                  reused_base_collision: tuple[list[dict], dict] | None = None) -> dict:
    metadata_path = source_dir / "ws2000_sim.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    joint = metadata["joint"]
    axis_ros = tuple(float(value) for value in joint["axis"])
    axis_viewer = ros_metres_to_viewer_mm(axis_ros)
    axis_length = sum(value * value for value in axis_viewer) ** 0.5
    axis = [value / axis_length for value in axis_viewer]

    base_nodes, base_faces, base_collision_meshes = convert_dae(
        source_dir / metadata["links"]["base"]["mesh"], package_dir, "base")
    carriage_nodes, carriage_faces, _ = convert_dae(
        source_dir / metadata["links"]["carriage"]["mesh"], package_dir, "carriage")
    if reused_base_collision is None:
        base_collision_hulls = bake_base_collision_hulls(base_collision_meshes)
        base_collision_hull_bake = {
            "comment": (
                "Generated offline during WS2000 package conversion. The simulator loads these "
                "serialized hulls directly and does not run convex decomposition at station load."),
            "generatedAtUtc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "tool": "CoACD Python binding",
            "sourceMesh": metadata["links"]["base"]["mesh"],
            "settings": {
                "maxHullCount": BASE_COLLISION_MAX_HULLS,
                "hardHullCap": MAX_COLLISION_HULLS,
                "sourceWeldGridMm": 5.0,
                "threshold": 0.05,
                "preprocessResolution": 50,
                "resolution": 1500,
                "mctsNodes": 10,
                "mctsIterations": 80,
                "mctsMaxDepth": 3,
                "merge": True,
                "decimate": True,
                "maxVerticesPerHull": 64,
            },
        }
    else:
        base_collision_hulls, base_collision_hull_bake = reused_base_collision
        if not base_collision_hulls:
            raise ValueError("reused base collision bake contains no hulls")
        if len(base_collision_hulls) > MAX_COLLISION_HULLS:
            raise ValueError(
                f"reused base collision bake has {len(base_collision_hulls)} hulls; "
                f"hard cap is {MAX_COLLISION_HULLS}")

    chain = metadata["drag_chain"]
    # One canonical full-width member is intentionally used for every station. This is actual
    # instancing: the archive, decoded scene and GPU each own one mesh, not thirteen files with
    # identical bytes or thirteen MeshGeometry nodes pointing at the same filename.
    legacy_chain_dir = package_dir / "meshes" / "chain"
    if legacy_chain_dir.is_dir():
        shutil.rmtree(legacy_chain_dir)
    for stale_member in (package_dir / "meshes").glob("chain_member_*.meshbin"):
        stale_member.unlink()
    chain_nodes, _, _ = convert_dae(source_dir / chain["links"][0]["mesh"],
                                    package_dir, "chain_member")
    if len(chain_nodes) != 1:
        raise ValueError("canonical drag-chain member must contain exactly one material group")
    prototype = chain_nodes[0]
    prototype["name"] = "Chain Member Prototype (instanced)"
    prototype["visible"] = False

    expected_base_faces = int(metadata["links"]["base"]["triangles"])
    expected_carriage_faces = int(metadata["links"]["carriage"]["triangles"])
    if base_faces != expected_base_faces or carriage_faces != expected_carriage_faces:
        raise ValueError(
            f"triangle totals differ from metadata: base {base_faces}/{expected_base_faces}, "
            f"carriage {carriage_faces}/{expected_carriage_faces}")

    base = node("Base", TYPE_CUSTOM, children=base_nodes)
    base["mountingHoles"] = {
        "comment": (
            "Fixture-table centers fitted from the complete 17 by 9 boundary grid on the large "
            "WS2000 base plate. Linear-rail slider fasteners are intentionally excluded."),
        "grids": [{
            "originMm": [655.1, -114.5, 300.8],
            "uStepMm": [50.0, 0.0, 0.0],
            "vStepMm": [0.0, 0.0, 50.0],
            "uCount": 17,
            "vCount": 9,
        }],
    }
    carriage = node("Carriage", TYPE_TRANSFORM, children=carriage_nodes)
    # Carriage-local snap centers include the main robot plate (the origin bore, its four large
    # mounting holes and the smaller perimeter arrays) plus the available holes on the upper beige
    # mounting extension. Repeated rows remain grids; only the unique origin bore is explicit.
    carriage["mountingHoles"] = {
        "comment": (
            "Carriage mounting centers fitted from the top planar boundaries. Includes the robot "
            "mount plate and upper beige extension; rail-slider fasteners are excluded."),
        "pointsMm": [[0.05, -0.05, -0.04]],
        "grids": [
            {
                "originMm": [-217.45, -0.05, -170.2],
                "uStepMm": [440.0, 0.0, 0.0],
                "vStepMm": [0.0, 0.0, 340.0],
                "uCount": 2,
                "vCount": 2,
            },
            {
                "originMm": [-208.95, -0.05, -265.05],
                "uStepMm": [60.0, 0.0, 0.0],
                "vStepMm": [0.0, 0.0, 30.0],
                "uCount": 2,
                "vCount": 3,
            },
            {
                "originMm": [-208.95, -0.05, 204.95],
                "uStepMm": [60.0, 0.0, 0.0],
                "vStepMm": [0.0, 0.0, 30.0],
                "uCount": 2,
                "vCount": 3,
            },
            {
                "originMm": [152.55, -0.05, -265.05],
                "uStepMm": [60.0, 0.0, 0.0],
                "vStepMm": [0.0, 0.0, 30.0],
                "uCount": 2,
                "vCount": 3,
            },
            {
                "originMm": [152.55, -0.05, 204.95],
                "uStepMm": [60.0, 0.0, 0.0],
                "vStepMm": [0.0, 0.0, 30.0],
                "uCount": 2,
                "vCount": 3,
            },
            {
                "originMm": [-167.5, -0.05, -220.2],
                "uStepMm": [340.05, 0.0, 0.0],
                "vStepMm": [0.0, 0.0, 440.0],
                "uCount": 2,
                "vCount": 2,
            },
            {
                "originMm": [-578.45, 218.34, -87.695],
                "uStepMm": [0.0, 0.0, 0.0],
                "vStepMm": [0.0, 0.0, 35.0],
                "uCount": 1,
                "vCount": 6,
            },
            {
                "originMm": [-445.55, 218.34, -137.7],
                "uStepMm": [0.0, 0.0, 0.0],
                "vStepMm": [0.0, 0.0, 68.75],
                "uCount": 1,
                "vCount": 5,
            },
            {
                "originMm": [-390.5, 218.34, -245.0],
                "uStepMm": [22.1, 0.0, 0.0],
                "vStepMm": [0.0, 0.0, 490.0],
                "uCount": 2,
                "vCount": 2,
            },
            {
                "originMm": [-718.46, 218.34, -101.65],
                "uStepMm": [0.0, 0.0, 0.0],
                "vStepMm": [0.0, 0.0, 202.9],
                "uCount": 1,
                "vCount": 2,
            },
        ],
    }
    # Repeating the canonical member costs no additional mesh payload. Size the runtime chain for
    # the worst endpoint separation plus its 180-degree return bend, while treating the stations
    # present in the exported STEP as a minimum.
    pitch_mm = 1000.0 * float(chain["pitch_m"])
    requested_radius_mm = 200.0
    # Keep the user-confirmed upper pivot on the carriage mount's free rounded end. Its hinge depth
    # matches the attachment metadata; parenting supplies the live carriage displacement.
    exported_fixed = ros_metres_to_viewer_mm(tuple(chain["attachments"]["fixed_end"]["point"]))
    moving_anchor = [-792.038, 158.45, exported_fixed[2]]
    fixed_anchor = list(ros_metres_to_viewer_mm(tuple(chain["attachments"]["moving_end"]["point"])))
    # The lower point becomes base-local and therefore receives the base-only re-datum. The upper
    # point remains in the carriage's original local frame; parenting adds the live displacement.
    base_shift_mm = float(metadata["frame"].get("base_shift_mm", 0.0))
    for component in range(3):
        fixed_anchor[component] += base_shift_mm * axis[component]
    # The fixed hinge is the rounded opening of the lower T-shaped base mount. Registering that
    # mount against the confirmed carriage copy places its hinge 893.350 mm along the rail.
    fixed_anchor[0] = moving_anchor[0] + 893.350
    # The base copy of the end-stop bracket is 19.995 mm farther along the hinge axis than the
    # carriage copy. Offset the terminal member onto the same connector face while leaving the
    # revolute hinge line and the user-confirmed carriage endpoint unchanged.
    fixed_end_member_offset = [0.0, 0.0, 19.995]
    anchor_delta = [fixed_anchor[i] - moving_anchor[i] for i in range(3)]
    axial_at_zero = sum(anchor_delta[i] * axis[i] for i in range(3))
    transverse = [anchor_delta[i] - axial_at_zero * axis[i] for i in range(3)]
    transverse_distance = sum(value * value for value in transverse) ** 0.5
    bend_radius_mm = min(requested_radius_mm, 0.5 * transverse_distance)
    lower_mm = 1000.0 * float(joint["limit_lower_m"])
    upper_mm = 1000.0 * float(joint["limit_upper_m"])
    worst_axial_separation = max(abs(axial_at_zero - lower_mm),
                                 abs(axial_at_zero - upper_mm))
    required_intervals = math.ceil(
        (worst_axial_separation + math.pi * bend_radius_mm) / pitch_mm)
    # The earlier 17-frame chain represented 16 pitches because its far endpoint was the last
    # frame origin. Runtime now includes the last member's +pitch hinge, so 16 members preserve
    # that established contour length without leaving one extra member in the loop.
    link_count = max(min_chain_links, int(chain["link_count"]), required_intervals)
    link_frames = [node(f"Chain Member {index:02d}", TYPE_TRANSFORM)
                   for index in range(link_count)]
    hinge_viewer = ros_metres_to_viewer_mm(tuple(chain["hinge_axis_world"]))
    hinge_length = sum(value * value for value in hinge_viewer) ** 0.5
    hinge_axis = [value / hinge_length for value in hinge_viewer]
    drag_chain = node(
        "Drag Chain",
        TYPE_DRAG_CHAIN_MECHANISM,
        {
            "fixedAnchorMm": fixed_anchor,
            "movingAnchorMm": moving_anchor,
            "travelAxis": axis,
            "hingeAxis": hinge_axis,
            "departureAxisSign": -1.0,
            "pitchMm": pitch_mm,
            "bendRadiusMm": bend_radius_mm,
            "linkMassKg": float(chain["links"][0]["mass_kg"]),
            "maxJointRotationDeg": max_chain_joint_deg,
            "reverseFixedEndMember": True,
            "fixedEndMemberOffsetMm": fixed_end_member_offset,
            "movingFramePath": [1],
            "prototypeGeometryPath": [2, 0],
            "linkFramePaths": [[2, index + 1] for index in range(link_count)],
        },
        children=[prototype, *link_frames],
    )

    gantry = node(
        metadata.get("display_name", "WS2000"),
        TYPE_GANTRY_MECHANISM,
        {
            "axisOfTravel": axis,
            "positionMm": 1000.0 * float(joint["limit_lower_m"]),
            "homePositionMm": 1000.0 * float(joint["limit_lower_m"]),
            "lowerLimitMm": 1000.0 * float(joint["limit_lower_m"]),
            "upperLimitMm": 1000.0 * float(joint["limit_upper_m"]),
            "velocityMaxMmS": 1000.0 * float(joint["velocity"]),
            "baseCollisionHulls": base_collision_hulls,
            "baseCollisionHullBake": base_collision_hull_bake,
            "movingFramePath": [1],
        },
        children=[base, carriage, drag_chain],
    )
    # The WS2000 CAD has no literal floor-fastener bores at this level, but cell layout still needs
    # a compact, stable footprint pattern. Use the four corners of the base wrap's floor envelope as
    # virtual mounting/marking points. Keeping this on the gantry root (and placementSource=true)
    # makes it available when placing the whole mechanism without reclassifying the fixture-table or
    # carriage holes as mechanism-placement anchors.
    gantry["mountingHoles"] = {
        "comment": (
            "Four virtual WS2000 base-corner marking centers derived from the floor envelope of "
            "base_01_external_axis_augmentus_wrap_grey.meshbin. These are placement references for "
            "the complete mechanism, not physical fasteners. Generated by "
            "scripts/convert_ws2000_gantry_package.py."),
        "placementSource": True,
        "grids": [{
            "originMm": [-357.6, -771.344, -290.2],
            "uStepMm": [1828.8, 0.0, 0.0],
            "vStepMm": [0.0, 0.0, 1016.0],
            "uCount": 2,
            "vCount": 2,
        }],
    }
    return gantry


def write_validated_zip(package_dir: Path, output_zip: Path) -> None:
    write_zip(package_dir, output_zip)
    with zipfile.ZipFile(output_zip, "r") as archive:
        chain_members = [name for name in archive.namelist() if name.endswith(".meshbin") and
                         (Path(name).name.startswith("chain_member_") or "/chain/" in name)]
        if len(chain_members) != 1:
            raise ValueError(f"archive must contain one chain-member mesh, found {len(chain_members)}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True,
                        help="WS2000-sim COLLADA export folder (vendor data, not in the repo)")
    parser.add_argument("--package-dir", type=Path, default=DEFAULT_PACKAGE_DIR)
    parser.add_argument("--zip", dest="zip_path", type=Path, default=DEFAULT_ZIP)
    parser.add_argument("--min-chain-links", type=int, default=13)
    parser.add_argument("--max-chain-joint-deg", type=float, default=35.0)
    parser.add_argument(
        "--reuse-base-hulls", action="store_true",
        help="reuse baseCollisionHulls from the existing package manifest")
    parser.add_argument("--no-zip", action="store_true")
    args = parser.parse_args()

    args.package_dir.mkdir(parents=True, exist_ok=True)
    if args.min_chain_links < 2:
        parser.error("--min-chain-links must be at least 2")
    if not 0.0 < args.max_chain_joint_deg < 180.0:
        parser.error("--max-chain-joint-deg must be between 0 and 180")
    manifest = args.package_dir / "mechanism.json"
    reused_base_collision = None
    if args.reuse_base_hulls:
        if not manifest.is_file():
            parser.error(f"--reuse-base-hulls requires an existing {manifest}")
        existing = json.loads(manifest.read_text(encoding="utf-8"))
        existing_data = existing.get("data", {})
        reused_base_collision = (
            existing_data.get("baseCollisionHulls", []),
            existing_data.get("baseCollisionHullBake", {}),
        )
    package = build_package(
        args.source, args.package_dir, args.min_chain_links, args.max_chain_joint_deg,
        reused_base_collision)
    manifest.write_text(json.dumps(package, indent=2) + "\n", encoding="utf-8")
    if not args.no_zip:
        write_validated_zip(args.package_dir, args.zip_path)
    print(f"wrote {manifest}")
    if not args.no_zip:
        print(f"wrote {args.zip_path}")


if __name__ == "__main__":
    main()
