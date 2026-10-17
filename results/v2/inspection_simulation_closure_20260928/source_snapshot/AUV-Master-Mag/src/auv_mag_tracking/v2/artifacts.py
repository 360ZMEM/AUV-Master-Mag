"""Small reproducibility manifest helper for v2-only experiment outputs."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import platform
from pathlib import Path
import subprocess
import sys
from typing import Any, Iterable

import numpy as np


def _git(repo: Path, *args: str) -> str | None:
    try:
        result = subprocess.run(
            ["git", "-C", str(repo), *args],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    return result.stdout.strip()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_experiment_manifest(
    output_dir: Path,
    *,
    experiment: str,
    config: dict[str, Any],
    artifact_paths: Iterable[Path],
    repository_root: Path,
) -> Path:
    """Write a manifest after all other v2 artifacts have been finalized."""
    artifacts = []
    for path in sorted({Path(item).resolve() for item in artifact_paths}):
        if not path.is_file():
            continue
        try:
            relative = path.relative_to(output_dir.resolve())
        except ValueError:
            relative = path
        artifacts.append(
            {"path": str(relative), "size_bytes": path.stat().st_size, "sha256": _sha256(path)}
        )
    status = _git(repository_root, "status", "--porcelain")
    payload = {
        "schema_version": 1,
        "experiment": experiment,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "repository": {
            "root": str(repository_root.resolve()),
            "commit": _git(repository_root, "rev-parse", "HEAD"),
            "dirty": None if status is None else bool(status),
        },
        "environment": {
            "python": sys.version,
            "platform": platform.platform(),
            "numpy": np.__version__,
        },
        "config": config,
        "artifacts": artifacts,
    }
    path = output_dir / "manifest.json"
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path
