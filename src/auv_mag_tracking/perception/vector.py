"""Envelope gradient and magnetic-vector heading analyzers."""

from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np

from ..sensor_model import PoseMeasurement


class EnvelopeGradientTracker:
    """Compute spatial gradient on the RMS envelope using Savitzky-Golay
    filtering.

    Gradient is computed on the *smoothed* envelope to avoid noise from
    raw 50 Hz residual ripple.  The temporal gradient is then converted to
    a spatial gradient (nT / m) using the current vehicle speed so that the
    feature is invariant to velocity changes.
    """

    def __init__(
        self,
        window_size: int = 7,
        polyorder: int = 2,
        buffer_capacity: int = 40,
        min_speed_mps: float = 0.3,
    ) -> None:
        """初始化包络梯度跟踪器的窗口、缓存与速度约束。"""
        self.window_size = max(3, window_size if window_size % 2 == 1 else window_size + 1)
        self.polyorder = min(polyorder, self.window_size - 1)
        self.buffer_capacity = max(4, buffer_capacity)
        self.min_speed_mps = max(min_speed_mps, 0.05)
        self.time_buffer: Deque[float] = deque(maxlen=self.buffer_capacity)
        self.strength_buffer: Deque[float] = deque(maxlen=self.buffer_capacity)
        self.position_buffer: Deque[np.ndarray] = deque(maxlen=self.buffer_capacity)
        self.gradient_nT_per_m: float = 0.0
        self.gradient_heading_deg: Optional[float] = None
        self.gradient_sign: int = 0  # +1 ascending, -1 descending, 0 flat

    def update(
        self,
        strength_nt: float,
        time_s: float,
        position_xy_m: np.ndarray,
        speed_mps: float,
    ) -> None:
        """更新梯度估计并同步计算信号上升/下降方向。"""
        self.time_buffer.append(time_s)
        self.strength_buffer.append(strength_nt)
        self.position_buffer.append(np.asarray(position_xy_m, dtype=float).copy())

        n = len(self.strength_buffer)
        if n < self.window_size:
            self.gradient_nT_per_m = 0.0
            self.gradient_heading_deg = None
            self.gradient_sign = 0
            return

        # Apply Savitzky-Golay filter and compute derivative
        strengths = np.asarray(list(self.strength_buffer), dtype=float)
        times = np.asarray(list(self.time_buffer), dtype=float)
        positions = np.vstack(list(self.position_buffer))

        try:
            from scipy.signal import savgol_filter
            # Derivative order 1 gives us dRMS/dt in units of index-space
            deriv = savgol_filter(strengths, self.window_size, self.polyorder, deriv=1, delta=1.0)
            # The delta=1.0 means deriv is in per-sample units.
            # Convert to temporal gradient using average dt
            avg_dt = float(np.mean(np.diff(times)))
            temporal_gradient = deriv[-1] / max(avg_dt, 1e-6)

            # Convert to spatial gradient using speed
            effective_speed = max(speed_mps, self.min_speed_mps)
            self.gradient_nT_per_m = temporal_gradient / effective_speed
        except Exception:
            self.gradient_nT_per_m = 0.0

        # Gradient sign: positive = signal ascending, negative = descending
        if abs(self.gradient_nT_per_m) > 0.5:
            self.gradient_sign = 1 if self.gradient_nT_per_m > 0 else -1
        else:
            self.gradient_sign = 0

        # Gradient heading: direction of maximum signal increase
        if n >= 2:
            delta_xy = positions[-1] - positions[-2]
            dist_m = float(np.linalg.norm(delta_xy))
            if dist_m > 1e-3:
                movement_heading = float(np.rad2deg(np.arctan2(delta_xy[1], delta_xy[0]))) % 360.0
                if self.gradient_sign < 0:
                    # Signal decreasing → cable is behind us
                    self.gradient_heading_deg = (movement_heading + 180.0) % 360.0
                elif self.gradient_sign > 0:
                    self.gradient_heading_deg = movement_heading
                else:
                    self.gradient_heading_deg = None
            else:
                self.gradient_heading_deg = None
        else:
            self.gradient_heading_deg = None


class StreamingVectorPCAFitter:
    """Extract the principal component direction from a block of AC magnetic
    vector samples using covariance eigen-analysis.

    For 50 Hz AC cables, the magnetic field oscillates rapidly. Instead of
    using a single instantaneous snapshot (which may land on any phase of the
    sine wave), we accumulate XY vector samples in a sliding window and
    compute the dominant oscillation axis via PCA on the 2x2 covariance matrix.
    """

    def __init__(self, buffer_capacity: int = 20) -> None:
        """Initialize the PCA fitter with a fixed-capacity vector buffer."""
        self.buffer_capacity = max(3, buffer_capacity)
        self._buffer_x: Deque[float] = deque(maxlen=self.buffer_capacity)
        self._buffer_y: Deque[float] = deque(maxlen=self.buffer_capacity)

    def add_sample(self, vector_xy: np.ndarray) -> None:
        """Append a single [Bx, By] sample to the sliding window."""
        self._buffer_x.append(float(vector_xy[0]))
        self._buffer_y.append(float(vector_xy[1]))

    def compute_principal_vector(self) -> Tuple[np.ndarray, float]:
        """Return the principal eigenvector and a consistency score.

        Returns
        -------
        principal_vector : np.ndarray of shape (2,)
            The dominant oscillation axis in the XY plane.
        consistency : float
            A value in [0, 1] reflecting how concentrated the samples are
            along the principal axis (based on eigenvalue ratio and circular
            mean resultant length).
        """
        n = len(self._buffer_x)
        if n < 3:
            return np.array([1.0, 0.0], dtype=float), 0.0

        xs = np.asarray(list(self._buffer_x), dtype=float)
        ys = np.asarray(list(self._buffer_y), dtype=float)

        # Build the 2x2 covariance matrix
        data = np.stack([xs, ys], axis=1)  # (n, 2)
        cov_matrix = np.cov(data, rowvar=False)  # (2, 2)

        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        # eigenvalues are sorted ascending; take the largest
        principal_idx = int(np.argmax(eigenvalues))
        principal_vec = eigenvectors[:, principal_idx]
        principal_vec = principal_vec / max(np.linalg.norm(principal_vec), 1e-12)

        # Consistency: ratio of largest eigenvalue to total variance
        total_var = float(np.sum(eigenvalues))
        if total_var < 1e-12:
            return principal_vec, 0.0
        eigen_ratio = float(eigenvalues[principal_idx] / total_var)

        # Also compute circular mean resultant length of the vector angles
        angles = np.arctan2(ys, xs)
        mean_r = float(np.sqrt(np.mean(np.cos(angles)) ** 2 + np.mean(np.sin(angles)) ** 2))
        # Combine eigen_ratio and circular consistency
        consistency = float(np.clip(0.6 * eigen_ratio + 0.4 * mean_r, 0.0, 1.0))

        return principal_vec, consistency

    def clear(self) -> None:
        """Reset the internal buffer."""
        self._buffer_x.clear()
        self._buffer_y.clear()


class MagneticVectorAnalyzer:
    """Extract horizontal magnetic vector direction and infer cable heading.

    Physics constraint: at the cable crossing (peak), the horizontal
    magnetic vector B_xy = [Bx, By] is perpendicular to the cable
    direction.  Therefore cable_heading ≈ vector_heading ± 90°.

    For AC mode (e.g. 50 Hz), instead of using an instantaneous snapshot,
    we accumulate vector samples in a sliding window and extract the
    principal oscillation axis via PCA/SVD on the 2×2 covariance matrix.
    This eliminates aliasing caused by sampling at arbitrary phases of the
    AC waveform.

    A dynamic gating mechanism rejects updates when SNR is too low or when
    AUV attitude (roll/pitch) is unstable, which would cause earth-field
    leakage to dominate the anomaly vector.
    """

    def __init__(
        self,
        buffer_capacity: int = 8,
        pca_buffer_capacity: int = 40,
    ) -> None:
        """Initialize the magnetic vector analyzer with PCA support."""
        self.buffer_capacity = max(1, buffer_capacity)
        self.vector_headings: Deque[float] = deque(maxlen=self.buffer_capacity)
        self.vector_magnitudes: Deque[float] = deque(maxlen=self.buffer_capacity)
        self.magnetic_vector_heading_deg: Optional[float] = None
        self.vector_cable_heading_deg: Optional[float] = None
        self.vector_confidence: float = 0.0

        # PCA fitter for AC mode vector extraction
        self.pca_fitter = StreamingVectorPCAFitter(buffer_capacity=pca_buffer_capacity)
        self._previous_vector_xy: Optional[np.ndarray] = None

        # Diagnostic state
        self.vector_consistency_score: float = 0.0
        self.attitude_leakage_risk: bool = False

    def update(
        self,
        anomaly_ned_nt: np.ndarray,
        tracking_strength_nt: float,
        pose_measurement: Optional["PoseMeasurement"] = None,
        snr_db: float = -120.0,
        signal_mode: str = "dc",
    ) -> None:
        """Estimate cable heading from the NED magnetic anomaly vector.

        Parameters
        ----------
        anomaly_ned_nt : np.ndarray
            3-element magnetic anomaly vector in NED coordinates.
        tracking_strength_nt : float
            Current tracking field strength (RMS or filtered).
        pose_measurement : PoseMeasurement, optional
            IMU-derived pose for attitude-based gating.
        snr_db : float
            Current signal-to-noise ratio in dB.
        signal_mode : str
            Signal mode identifier ("dc", "ac_50hz", etc.).
        """
        bx, by = float(anomaly_ned_nt[0]), float(anomaly_ned_nt[1])
        magnitude_xy = float(np.sqrt(bx * bx + by * by))

        # --- Dynamic gating: reject low-SNR updates ---
        if snr_db < 10.0:
            self.attitude_leakage_risk = False
            return

        # --- Dynamic gating: reject high-attitude-risk updates ---
        if pose_measurement is not None:
            roll_ok = abs(float(pose_measurement.roll_deg)) <= 3.0
            pitch_ok = abs(float(pose_measurement.pitch_deg)) <= 3.0
            if not (roll_ok and pitch_ok):
                self.attitude_leakage_risk = True
                return
        self.attitude_leakage_risk = False

        # --- AC mode: use PCA to extract principal oscillation axis ---
        if signal_mode != "dc":
            self.pca_fitter.add_sample(anomaly_ned_nt[:2])
            principal_vec, pca_consistency = self.pca_fitter.compute_principal_vector()

            if pca_consistency < 0.1:
                return

            # Sign alignment: prevent 180° flip between consecutive frames
            if self._previous_vector_xy is not None:
                if float(np.dot(principal_vec, self._previous_vector_xy)) < 0:
                    principal_vec = -principal_vec
            self._previous_vector_xy = principal_vec.copy()

            vector_heading = float(np.rad2deg(np.arctan2(principal_vec[1], principal_vec[0]))) % 360.0
            vector_magnitude = magnitude_xy
            self.vector_consistency_score = pca_consistency
        else:
            # --- DC mode: use instantaneous vector directly ---
            if magnitude_xy < 1e-3 or tracking_strength_nt < 10.0:
                return
            vector_heading = float(np.rad2deg(np.arctan2(by, bx))) % 360.0
            vector_magnitude = magnitude_xy
            # For DC mode, consistency is not PCA-based
            self.vector_consistency_score = 0.0

        self.vector_headings.append(vector_heading)
        self.vector_magnitudes.append(vector_magnitude)

        n = len(self.vector_headings)
        if n < 1:
            return

        # Circular mean of recent vector headings
        rads = np.array([np.deg2rad(h) for h in self.vector_headings])
        mean_sin = float(np.mean(np.sin(rads)))
        mean_cos = float(np.mean(np.cos(rads)))
        mean_rad = np.arctan2(mean_sin, mean_cos)
        self.magnetic_vector_heading_deg = float(np.rad2deg(mean_rad)) % 360.0

        # Cable heading is perpendicular to B_xy
        self.vector_cable_heading_deg = (self.magnetic_vector_heading_deg + 90.0) % 360.0

        # Confidence based on magnitude consistency (R < 1 → spread)
        resultant_length = float(np.sqrt(mean_sin ** 2 + mean_cos ** 2))
        self.vector_confidence = float(np.clip(resultant_length, 0.0, 1.0))
