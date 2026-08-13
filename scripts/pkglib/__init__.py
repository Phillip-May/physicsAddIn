"""Shared helpers for the package conversion and generation scripts."""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LIBRARY_SOURCES = ROOT / "library" / "sources"
LIBRARY_PACKAGES = ROOT / "library" / "packages"


def slug(value: str) -> str:
    value = value.lower()
    if value.endswith(".stl"):
        value = value[:-4]
    return re.sub(r"[^a-z0-9]+", "_", value).strip("_") or "mesh"
