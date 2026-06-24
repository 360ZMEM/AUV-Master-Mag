"""Build a compact cable map from simulation or replay records."""

from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.routes import build_cable_map_from_record  # noqa: E402
from auv_mag_tracking.viz import RunRecord, simulate_case  # noqa: E402


def _load_record_npz(path: Path) -> RunRecord:
    archive = np.load(path, allow_pickle=True)
    channels = {
        key: np.asarray(archive[key], dtype=float)
        for key in archive.files
        if not key.startswith("__")
    }
    return RunRecord(
        case_name=str(archive["__case_name__"].item()),
        deployment_mode=bool(archive["__deployment_mode__"].item()),
        dt_s=float(archive["__dt_s__"].item()),
        channels=channels,
        modes=[str(item) for item in archive["__modes__"].tolist()],
        sources=[str(item) for item in archive["__sources__"].tolist()],
        cable_route_xy_m=np.asarray(archive["__cable_route_xy_m__"], dtype=float),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Build a compact cable map JSON from cable observations")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--case", help="simulate a scenario and build map from its estimated cable observations")
    source.add_argument("--record", type=Path, help="load an existing results/<run>/<case>/record.npz")
    parser.add_argument("--deployment", action="store_true", help="disable nominal route prior when using --case")
    parser.add_argument("--max-steps", type=int, default=None, help="cap simulation steps when using --case")
    parser.add_argument("--out", type=Path, default=None, help="output JSON path")
    parser.add_argument("--min-confidence", type=float, default=0.45)
    parser.add_argument("--min-spacing-m", type=float, default=2.0)
    parser.add_argument("--simplify-tolerance-m", type=float, default=3.0)
    parser.add_argument("--tolerance-band-m", type=float, default=30.0)
    parser.add_argument(
        "--truth-fallback",
        action="store_true",
        help="use true nearest cable points only when estimated channels are absent; for synthetic smoke tests only",
    )
    args = parser.parse_args()

    if args.record is not None:
        record = _load_record_npz(args.record)
        default_root = args.record.parent
    else:
        record = simulate_case(args.case, deployment_mode=args.deployment, max_steps=args.max_steps)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_root = WORKSPACE_ROOT / "results" / timestamp / args.case
        default_root.mkdir(parents=True, exist_ok=True)
        record.save_npz(default_root / "record.npz")

    try:
        cable_map = build_cable_map_from_record(
            record,
            min_confidence=args.min_confidence,
            min_spacing_m=args.min_spacing_m,
            simplify_tolerance_m=args.simplify_tolerance_m,
            tolerance_band_m=args.tolerance_band_m,
            truth_fallback=args.truth_fallback,
        )
    except ValueError as exc:
        raise SystemExit(
            f"[map] no map generated: {exc}. "
            "Use a longer run, lower --min-confidence, or --truth-fallback for synthetic smoke tests."
        ) from exc
    out_path = args.out if args.out is not None else default_root / "cable_map.json"
    cable_map.save_json(out_path)
    print(
        f"[map] {record.case_name}: "
        f"{cable_map.metadata['raw_observation_count']} observations -> "
        f"{cable_map.metadata['waypoint_count']} waypoints"
    )
    print(f"[map] written to {out_path}")


if __name__ == "__main__":
    main()
