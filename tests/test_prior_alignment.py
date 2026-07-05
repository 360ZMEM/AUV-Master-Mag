from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np

SRC_ROOT = Path(__file__).resolve().parents[1] / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.perception import PriorAlignmentEstimator


def _tracking_config(**overrides):
    values = dict(
        nominal_route_prior_correction_gain=1.0,
        nominal_route_prior_correction_max_residual_m=10.0,
        nominal_route_prior_correction_max_step_m=0.5,
        nominal_route_prior_correction_max_heading_error_deg=20.0,
        nominal_route_prior_correction_max_translation_m=5.0,
        nominal_route_prior_correction_max_rotation_deg=8.0,
        nominal_route_prior_correction_ekf_enabled=False,
        nominal_route_prior_correction_ekf_translation_process_std_m_per_sqrt_s=0.05,
        nominal_route_prior_correction_ekf_rotation_process_std_deg_per_sqrt_s=0.03,
        nominal_route_prior_correction_ekf_translation_meas_std_m=1.5,
        nominal_route_prior_correction_ekf_rotation_meas_std_deg=2.5,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def test_prior_alignment_limits_single_translation_step():
    estimator = PriorAlignmentEstimator(
        initial_translation_xy_m=np.zeros(2),
        initial_rotation_deg=0.0,
        initial_covariance_diag=np.ones(3),
    )
    state = estimator.update(
        tracking=_tracking_config(),
        observed_point_xy=np.array([3.0, 4.0]),
        prior_point_xy=np.array([0.0, 0.0]),
        prior_tangent_xy=np.array([1.0, 0.0]),
        observed_heading_deg=None,
        confidence=1.0,
        progress_m=12.0,
    )

    assert state.accepted
    assert np.isclose(state.residual_norm_m, 5.0)
    assert np.isclose(state.applied_step_norm_m, 0.5)
    assert np.allclose(state.translation_xy_m, np.array([0.3, 0.4]))


def test_prior_alignment_rejects_out_of_gate_residual():
    estimator = PriorAlignmentEstimator(
        initial_translation_xy_m=np.zeros(2),
        initial_rotation_deg=0.0,
        initial_covariance_diag=np.ones(3),
    )
    state = estimator.update(
        tracking=_tracking_config(nominal_route_prior_correction_max_residual_m=2.0),
        observed_point_xy=np.array([3.0, 4.0]),
        prior_point_xy=np.array([0.0, 0.0]),
        prior_tangent_xy=np.array([1.0, 0.0]),
        observed_heading_deg=None,
        confidence=1.0,
        progress_m=12.0,
    )

    assert not state.accepted
    assert state.reason_code == PriorAlignmentEstimator.REASON_RESIDUAL_TOO_LARGE
    assert np.allclose(state.translation_xy_m, np.zeros(2))
