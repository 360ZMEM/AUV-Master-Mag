"""Opt-in direction-aware rejoin using the existing finite-route geometry.

Only an observed disturbance opposing the measured cable offset selects the
short run-in. Near zero keeps the validated 20 m nominal recipe. The velocity
proxy includes sideslip and is not a direct ocean-current measurement.
"""
from dataclasses import dataclass
import math

import numpy as np

from .flythrough import FlythroughInspectionPipelineV2


@dataclass(frozen=True)
class DirectionalRejoinContract:
    nominal_runin_m: float = 20.0
    opposed_runin_m: float = 5.0
    opposing_proxy_threshold_mps: float = 0.02

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.nominal_runin_m, self.opposed_runin_m, self.opposing_proxy_threshold_mps)):
            raise ValueError('directional rejoin budgets must be positive and finite')

    def choose(self, offset_m, normal_proxy_mps):
        if not math.isfinite(offset_m) or not math.isfinite(normal_proxy_mps):
            raise ValueError('directional rejoin observations must be finite')
        toward = normal_proxy_mps * (1.0 if offset_m >= 0 else -1.0)
        return self.opposed_runin_m if toward < -self.opposing_proxy_threshold_mps else self.nominal_runin_m


class DirectionalRejoinPipelineV2(FlythroughInspectionPipelineV2):
    """Reuse scan, FWHM, track and API; change only upstream turn distance."""

    def __init__(self, *args, rejoin_contract=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.rejoin_contract = rejoin_contract or DirectionalRejoinContract()
        if max(self.rejoin_contract.nominal_runin_m, self.rejoin_contract.opposed_runin_m) >= -self.flythrough_config.end_along_m:
            raise ValueError('directional run-in must fit upstream of the route')

    def _choose_rejoin_runin_m(self):
        return self.rejoin_contract.choose(float(self._observation_prior_offset_m or 0.0),
                                          float(np.dot(self._fly_current_xy, self._fly_normal)))
