
from __future__ import annotations

import math


def append_box(vertices: list[float], indices: list[int], center, size,
               basis=None) -> list[float]:
    """Append a box (optionally oriented by a 3-column basis); returns its 8 corners flat."""
    basis = basis or ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    half = tuple(value * 0.5 for value in size)
    first = len(vertices) // 3
    primitive: list[float] = []
    for sx, sy, sz in ((-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
                       (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)):
        local = (sx * half[0], sy * half[1], sz * half[2])
        point = [center[row] + sum(basis[col][row] * local[col] for col in range(3))
                 for row in range(3)]
        vertices.extend(point)
        primitive.extend(point)
    for face in ((0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
                 (0, 1, 5), (0, 5, 4), (3, 7, 6), (3, 6, 2),
                 (0, 4, 7), (0, 7, 3), (1, 2, 6), (1, 6, 5)):
        indices.extend(first + value for value in face)
    return primitive


def append_cylinder(vertices: list[float], indices: list[int], center, axis, radius: float,
                    length: float, sides: int = 32) -> list[float]:
    """Capped cylinder about an arbitrary axis; returns its ring/cap points flat."""
    axis_length = math.sqrt(sum(value * value for value in axis))
    a = tuple(value / axis_length for value in axis)
    helper = (0.0, 0.0, 1.0) if abs(a[2]) < 0.9 else (1.0, 0.0, 0.0)
    u_raw = (a[1] * helper[2] - a[2] * helper[1],
             a[2] * helper[0] - a[0] * helper[2],
             a[0] * helper[1] - a[1] * helper[0])
    u_len = math.sqrt(sum(value * value for value in u_raw))
    u = tuple(value / u_len for value in u_raw)
    v = (a[1] * u[2] - a[2] * u[1],
         a[2] * u[0] - a[0] * u[2],
         a[0] * u[1] - a[1] * u[0])
    first = len(vertices) // 3
    primitive: list[float] = []
    for end in (-1.0, 1.0):
        for side in range(sides):
            angle = 2.0 * math.pi * side / sides
            point = [center[row] + a[row] * length * 0.5 * end +
                     radius * (u[row] * math.cos(angle) + v[row] * math.sin(angle))
                     for row in range(3)]
            vertices.extend(point)
            primitive.extend(point)
    bottom_center = len(vertices) // 3
    bottom = [center[row] - a[row] * length * 0.5 for row in range(3)]
    top_center = bottom_center + 1
    top = [center[row] + a[row] * length * 0.5 for row in range(3)]
    vertices.extend(bottom + top)
    primitive.extend(bottom + top)
    for side in range(sides):
        nxt = (side + 1) % sides
        b0, b1 = first + side, first + nxt
        t0, t1 = first + sides + side, first + sides + nxt
        indices.extend((b0, b1, t1, b0, t1, t0))
        indices.extend((bottom_center, b1, b0, top_center, t0, t1))
    return primitive


def append_z_cylinder(vertices: list[float], indices: list[int], center_x: float,
                      center_y: float, z_min: float, z_max: float, radius: float,
                      segments: int = 20) -> None:
    base = len(vertices) // 3
    for z in (z_min, z_max):
        for segment in range(segments):
            angle = 2.0 * math.pi * segment / segments
            vertices.extend((center_x + radius * math.cos(angle),
                             center_y + radius * math.sin(angle), z))
    for segment in range(segments):
        nxt = (segment + 1) % segments
        a, b = base + segment, base + nxt
        c, d = base + segments + segment, base + segments + nxt
        indices.extend((a, b, d, a, d, c))
    low_center = len(vertices) // 3
    vertices.extend((center_x, center_y, z_min))
    high_center = len(vertices) // 3
    vertices.extend((center_x, center_y, z_max))
    for segment in range(segments):
        nxt = (segment + 1) % segments
        indices.extend((low_center, base + nxt, base + segment))
        indices.extend((high_center, base + segments + segment, base + segments + nxt))


def cylinder_mesh(origin, axis: str, length: float = 180.0,
                  radius: float = 7.0) -> tuple[list[float], list[int]]:
    """Uncapped 12-segment marker cylinder along a named axis (joint-axis overlays)."""
    segments = 12
    vertices: list[float] = []
    for end in (-length * 0.5, length * 0.5):
        for index in range(segments):
            angle = math.tau * index / segments
            u, v = radius * math.cos(angle), radius * math.sin(angle)
            if axis == "x":
                point = (origin[0] + end, origin[1] + u, origin[2] + v)
            elif axis == "y":
                point = (origin[0] + u, origin[1] + end, origin[2] + v)
            else:
                point = (origin[0] + u, origin[1] + v, origin[2] + end)
            vertices.extend(point)
    indices: list[int] = []
    for index in range(segments):
        following = (index + 1) % segments
        indices.extend((index, segments + index, segments + following,
                        index, segments + following, following))
    return vertices, indices


def append_perforated_cell(vertices: list[float], indices: list[int], center_x: float,
                           center_z: float, y_bottom: float, y_top: float,
                           cell_size: float, hole_radius: float) -> None:
    half = cell_size * 0.5
    outer = ((-half, -half), (0.0, -half), (half, -half), (half, 0.0),
             (half, half), (0.0, half), (-half, half), (-half, 0.0))
    inner = []
    for x, z in outer:
        length = math.hypot(x, z)
        inner.append((hole_radius * x / length, hole_radius * z / length))

    base = len(vertices) // 3
    for y in (y_bottom, y_top):
        for ring in (outer, inner):
            for x, z in ring:
                vertices.extend((center_x + x, y, center_z + z))
    bottom_outer, bottom_inner = base, base + 8
    top_outer, top_inner = base + 16, base + 24
    for i in range(8):
        nxt = (i + 1) % 8
        # Top and bottom annuli.
        indices.extend((top_outer + i, top_outer + nxt, top_inner + nxt,
                        top_outer + i, top_inner + nxt, top_inner + i))
        indices.extend((bottom_outer + i, bottom_inner + nxt, bottom_outer + nxt,
                        bottom_outer + i, bottom_inner + i, bottom_inner + nxt))
        # Outer and bore walls. Internal cell walls are coincident and remain hidden inside the slab.
        indices.extend((bottom_outer + i, bottom_outer + nxt, top_outer + nxt,
                        bottom_outer + i, top_outer + nxt, top_outer + i))
        indices.extend((bottom_inner + i, top_inner + nxt, bottom_inner + nxt,
                        bottom_inner + i, top_inner + i, top_inner + nxt))
