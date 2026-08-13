"""Reproducible package ZIP writer."""

from __future__ import annotations

import zipfile
from pathlib import Path
from typing import Iterable


def write_zip(package_dir: Path, output: Path, members: Iterable[str] | None = None,
              compresslevel: int = 9) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        output.unlink()
    if members is None:
        members = [path.relative_to(package_dir).as_posix()
                   for path in sorted(package_dir.rglob("*")) if path.is_file()]
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED,
                         compresslevel=compresslevel) as archive:
        for member in members:
            info = zipfile.ZipInfo(Path(member).as_posix(),
                                   date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o100644 << 16
            archive.writestr(info, (package_dir / member).read_bytes(),
                             compress_type=zipfile.ZIP_DEFLATED, compresslevel=compresslevel)
