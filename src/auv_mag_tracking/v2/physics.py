"""Physics-reference magnetic field helpers kept separate from the v1 model."""

from __future__ import annotations

from typing import Iterable

import numpy as np

from ..math_utils import MU0_OVER_4PI


def physical_finite_wire_field_nT(
    point_ned_m: Iterable[float],
    segment_start_ned_m: Iterable[float],
    segment_end_ned_m: Iterable[float],
    current_a: float,
) -> np.ndarray:
    """Return the dimensionally consistent field of one finite straight wire.

    This reference intentionally does not replace ``math_utils.finite_wire_field_nT``.
    The extra ``|r1| + |r2|`` factor gives the expected 1/distance dimension and
    the infinite-wire limit ``mu0 I / (2 pi rho)``.
    """
    point = np.asarray(point_ned_m, dtype=float)
    start = np.asarray(segment_start_ned_m, dtype=float)
    end = np.asarray(segment_end_ned_m, dtype=float)
    r1 = point - start
    r2 = point - end
    r1_norm = float(np.linalg.norm(r1))
    r2_norm = float(np.linalg.norm(r2))
    cross = np.cross(r1, r2)
    denominator = r1_norm * r2_norm * (r1_norm * r2_norm + float(np.dot(r1, r2)))
    if r1_norm <= 1e-12 or r2_norm <= 1e-12 or abs(denominator) <= 1e-18:
        return np.zeros(3, dtype=float)
    field_t = (
        MU0_OVER_4PI
        * float(current_a)
        * (r1_norm + r2_norm)
        * cross
        / denominator
    )
    return field_t * 1e9


def physical_polyline_field_nT(
    point_ned_m: Iterable[float],
    segment_starts_ned_m: np.ndarray,
    segment_ends_ned_m: np.ndarray,
    current_a: float,
) -> np.ndarray:
    total = np.zeros(3, dtype=float)
    for start, end in zip(segment_starts_ned_m, segment_ends_ned_m):
        total += physical_finite_wire_field_nT(point_ned_m, start, end, current_a)
    return total

