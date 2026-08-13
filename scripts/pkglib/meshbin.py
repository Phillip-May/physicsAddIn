"""PAMESH01 binary mesh writer shared by every package script."""

from __future__ import annotations

import struct
from pathlib import Path

try:
    import numpy as _np
except ImportError:
    _np = None

MESH_MAGIC = b"PAMESH01"


def bounds_of(vertices) -> list[float]:
    if len(vertices) == 0:
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    xs, ys, zs = vertices[0::3], vertices[1::3], vertices[2::3]
    return [min(xs), min(ys), min(zs), max(xs), max(ys), max(zs)]


def merge_bounds(a: list[float] | None, b: list[float]) -> list[float]:
    if a is None:
        return list(b)
    return [min(a[0], b[0]), min(a[1], b[1]), min(a[2], b[2]),
            max(a[3], b[3]), max(a[4], b[4]), max(a[5], b[5])]


def write_meshbin(path: Path, vertices, indices, bounds: list[float] | None = None) -> None:
    # Callers may pass bounds computed from the full-resolution input so that display-mesh
    # decimation cannot quietly shrink the part's recorded extent.
    if bounds is None:
        bounds = bounds_of(vertices)
    path.parent.mkdir(parents=True, exist_ok=True)
    vertices_np = _np is not None and isinstance(vertices, _np.ndarray)
    indices_np = _np is not None and isinstance(indices, _np.ndarray)
    with path.open("wb") as stream:
        stream.write(MESH_MAGIC)
        stream.write(struct.pack("<II",
                                 (vertices.size if vertices_np else len(vertices)) // 3,
                                 int(indices.size) if indices_np else len(indices)))
        stream.write(struct.pack("<6f", *bounds))
        # tobytes for numpy input: struct.pack with a star-args list of a million floats is
        # both slow and needless once the data is already an array.
        if vertices_np:
            stream.write(_np.ascontiguousarray(vertices, dtype="<f4").tobytes())
        else:
            stream.write(struct.pack(f"<{len(vertices)}f", *vertices))
        if indices_np:
            stream.write(_np.ascontiguousarray(indices, dtype="<u4").tobytes())
        else:
            stream.write(struct.pack(f"<{len(indices)}I", *indices))
