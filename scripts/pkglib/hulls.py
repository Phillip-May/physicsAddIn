
from __future__ import annotations

import math

from .meshbin import bounds_of


def box_hull(bounds: list[float], minimum_thickness: float = 0.0) -> dict:
    xmin, ymin, zmin, xmax, ymax, zmax = bounds
    values = [[xmin, xmax], [ymin, ymax], [zmin, zmax]]
    for axis in range(3):
        if values[axis][1] - values[axis][0] < minimum_thickness:
            center = sum(values[axis]) * 0.5
            values[axis] = [center - minimum_thickness * 0.5,
                            center + minimum_thickness * 0.5]
    xmin, xmax = values[0]
    ymin, ymax = values[1]
    zmin, zmax = values[2]
    return {
        "vertices": [
            [xmin, ymin, zmin], [xmax, ymin, zmin], [xmax, ymax, zmin], [xmin, ymax, zmin],
            [xmin, ymin, zmax], [xmax, ymin, zmax], [xmax, ymax, zmax], [xmin, ymax, zmax],
        ],
        # Outward winding; PhysX convex cooking forgives inward faces but renderers and ray
        # casts do not.
        "indices": [
            [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
            [0, 1, 5], [0, 5, 4], [3, 7, 6], [3, 6, 2],
            [0, 4, 7], [0, 7, 3], [1, 2, 6], [1, 6, 5],
        ],
    }


def hull_is_valid(hull: dict) -> bool:
    vertices = hull.get("vertices", [])
    indices = hull.get("indices", [])
    if len(vertices) < 4 or len(indices) < 4:
        return False
    for vertex in vertices:
        if len(vertex) != 3 or not all(math.isfinite(float(value)) for value in vertex):
            return False
    for triangle in indices:
        if len(triangle) != 3 or len(set(triangle)) != 3:
            return False
        if any(int(index) < 0 or int(index) >= len(vertices) for index in triangle):
            return False
    return True


def convex_hull(vertices: list[float], label: str = "mesh", *,
                max_vertices: int | None = None, dedup_decimals: int = 5,
                round_decimals: int | None = None, include_source: bool = False,
                fallback_thickness: float = 2.0) -> dict:
    import numpy as np
    from scipy.spatial import ConvexHull, QhullError

    points = np.asarray(vertices, dtype=np.float64).reshape((-1, 3))
    points = np.unique(np.round(points, decimals=dedup_decimals), axis=0)
    hull = None
    if len(points) >= 4 and np.count_nonzero(np.ptp(points, axis=0) > 1.0e-4) >= 3:
        try:
            hull = ConvexHull(points, qhull_options="QJ")
        except QhullError:
            hull = None
    if hull is None:
        return box_hull(bounds_of(vertices), minimum_thickness=fallback_thickness)
    if max_vertices is not None and len(hull.vertices) > max_vertices:
        # Support points in deterministic spherical directions retain the outer silhouette
        # while keeping collision JSON and downstream convex cooking bounded.
        golden = math.pi * (3.0 - math.sqrt(5.0))
        directions = []
        for index in range(max_vertices * 2):
            y = 1.0 - 2.0 * (index + 0.5) / (max_vertices * 2)
            radius = math.sqrt(max(0.0, 1.0 - y * y))
            angle = golden * index
            directions.append((math.cos(angle) * radius, y, math.sin(angle) * radius))
        selected = sorted({int(np.argmax(points @ np.asarray(direction)))
                           for direction in directions})
        points = points[selected]
        hull = ConvexHull(points, qhull_options="QJ")
    used = sorted({int(index) for index in hull.simplices.flat})
    remap = {source: target for target, source in enumerate(used)}
    if round_decimals is not None:
        out_vertices = [[round(float(value), round_decimals) for value in points[index]]
                        for index in used]
    else:
        out_vertices = [points[index].tolist() for index in used]
    result = {
        "vertices": out_vertices,
        "indices": [[remap[int(index)] for index in triangle]
                    for triangle in hull.simplices.tolist()],
    }
    if include_source:
        result["source"] = label
    if not hull_is_valid(result):
        raise RuntimeError(f"{label}: generated malformed convex hull")
    return result
