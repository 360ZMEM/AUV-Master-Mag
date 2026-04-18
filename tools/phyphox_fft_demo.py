"""Standalone Phyphox demo that reads phone magnetometer data and performs live FFT.

This script stays thin on purpose: it reuses the existing Phyphox transport and
signal-processing helpers, then adds a small outer loop for live time-domain and
time-frequency visualization.
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque
from pathlib import Path
from typing import Deque, Optional

import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.tools.phyphox_adapter import PhyphoxStreamer, SignalProcessor


class FftVisualizer:
    """Live time-domain and time-frequency view of the processed signal."""

    def __init__(self, enabled: bool = True, sample_rate_hz: float = 20.0, history_seconds: float = 20.0) -> None:
        self.enabled = enabled
        self.history_seconds = max(float(history_seconds), 5.0)
        self.sample_rate_hz = max(float(sample_rate_hz), 1e-6)
        self.history_rows = max(32, int(round(self.sample_rate_hz * self.history_seconds)))
        self._plt = None
        self._figure = None
        self._axes = None
        self._time_lines = None
        self._spectrogram_image = None
        self._spectrogram_colorbar = None
        self._peak_trace = None

        self.time_history_s: Deque[float] = deque(maxlen=self.history_rows)
        self.bx_history_uT: Deque[float] = deque(maxlen=self.history_rows)
        self.by_history_uT: Deque[float] = deque(maxlen=self.history_rows)
        self.bz_history_uT: Deque[float] = deque(maxlen=self.history_rows)
        self.magnitude_history_uT: Deque[float] = deque(maxlen=self.history_rows)
        self.spectrogram_time_s: Deque[float] = deque(maxlen=self.history_rows)
        self.spectrogram_rows_db: Deque[np.ndarray] = deque(maxlen=self.history_rows)
        self.freq_axis_hz: np.ndarray = np.zeros(0, dtype=float)
        self.peak_frequency_history_hz: Deque[float] = deque(maxlen=self.history_rows)

        if self.enabled:
            try:
                import matplotlib.pyplot as plt

                plt.ion()
                self._plt = plt
                self._setup()
            except Exception:
                self.enabled = False
                self._plt = None

    def _setup(self) -> None:
        assert self._plt is not None
        self._figure, self._axes = self._plt.subplots(2, 1, figsize=(14, 9), sharex=False)
        self._figure.suptitle("Phyphox Live Magnetometer FFT Demo")
        self._set_initial_window_position()

        time_ax, spectrogram_ax = self._axes
        time_ax.set_title("Magnetic Field Time Domain")
        time_ax.set_xlabel("Time [s]")
        time_ax.set_ylabel("Field [uT]")
        time_ax.grid(True, alpha=0.3)
        (bx_line,) = time_ax.plot([], [], color="tab:blue", lw=1.2, label="Bx")
        (by_line,) = time_ax.plot([], [], color="tab:orange", lw=1.2, label="By")
        (bz_line,) = time_ax.plot([], [], color="tab:green", lw=1.2, label="Bz")
        (mag_line,) = time_ax.plot([], [], color="tab:red", lw=1.8, label="|B|")
        time_ax.legend(loc="upper right", fontsize=8)

        spectrogram_ax.set_title("FFT Spectrogram (Time vs Frequency)")
        spectrogram_ax.set_xlabel("Time [s]")
        spectrogram_ax.set_ylabel("Frequency [Hz]")
        spectrogram_ax.grid(False)
        self._time_lines = (bx_line, by_line, bz_line, mag_line)
        self._peak_trace, = spectrogram_ax.plot([], [], color="white", lw=1.2, alpha=0.9, label="Peak freq")
        spectrogram_ax.legend(loc="upper right", fontsize=8)

    def _set_initial_window_position(self) -> None:
        if self._plt is None:
            return
        manager = getattr(self._figure.canvas, "manager", None)
        if manager is None:
            return
        window = getattr(manager, "window", None)
        if window is None:
            return
        try:
            if hasattr(window, "move"):
                window.move(100, 100)
                return
            if hasattr(window, "SetPosition"):
                window.SetPosition((100, 100))
                return
            if hasattr(window, "wm_geometry"):
                window.wm_geometry("+100+100")
                return
        except Exception:
            return

    @staticmethod
    def _dominant_axis(window: np.ndarray) -> int:
        if window.ndim != 2 or window.shape[1] != 3:
            return 0
        axis_variance = np.var(window, axis=0)
        return int(np.argmax(axis_variance))

    def update_time_domain(self, timestamp_s: float, vector_uT: np.ndarray) -> None:
        self.time_history_s.append(float(timestamp_s))
        self.bx_history_uT.append(float(vector_uT[0]))
        self.by_history_uT.append(float(vector_uT[1]))
        self.bz_history_uT.append(float(vector_uT[2]))
        self.magnitude_history_uT.append(float(np.linalg.norm(vector_uT)))

    def update_spectrogram(self, timestamp_s: float, frequency_axis_hz: np.ndarray, psd_db: np.ndarray, peak_frequency_hz: Optional[float]) -> None:
        self.spectrogram_time_s.append(float(timestamp_s))
        self.spectrogram_rows_db.append(np.asarray(psd_db, dtype=float))
        if peak_frequency_hz is not None:
            self.peak_frequency_history_hz.append(float(peak_frequency_hz))
        else:
            self.peak_frequency_history_hz.append(float("nan"))
        self.freq_axis_hz = np.asarray(frequency_axis_hz, dtype=float)

    def render(
        self,
        fft_max_frequency_hz: float,
    ) -> None:
        if not self.enabled:
            return

        assert self._plt is not None and self._axes is not None and self._time_lines is not None
        time_ax, spectrogram_ax = self._axes
        bx_line, by_line, bz_line, mag_line = self._time_lines

        time_axis = np.asarray(self.time_history_s, dtype=float)
        bx_axis = np.asarray(self.bx_history_uT, dtype=float)
        by_axis = np.asarray(self.by_history_uT, dtype=float)
        bz_axis = np.asarray(self.bz_history_uT, dtype=float)
        mag_axis = np.asarray(self.magnitude_history_uT, dtype=float)

        bx_line.set_data(time_axis, bx_axis)
        by_line.set_data(time_axis, by_axis)
        bz_line.set_data(time_axis, bz_axis)
        mag_line.set_data(time_axis, mag_axis)
        if time_axis.size > 1:
            time_ax.set_xlim(float(time_axis[0]), float(time_axis[-1]))
            combined = np.concatenate([bx_axis, by_axis, bz_axis, mag_axis])
            spread = max(float(np.max(np.abs(combined))), 1.0)
            time_ax.set_ylim(-1.15 * spread, 1.15 * spread)

        if self.freq_axis_hz.size > 1 and self.spectrogram_rows_db:
            freq_mask = self.freq_axis_hz <= max(float(fft_max_frequency_hz), 1.0)
            freq_axis = self.freq_axis_hz[freq_mask]
            spectrogram_matrix = np.vstack(self.spectrogram_rows_db)[:, freq_mask]
            if self.spectrogram_time_s:
                time_start = float(self.spectrogram_time_s[0])
                time_end = float(self.spectrogram_time_s[-1])
                if time_end <= time_start:
                    half_step_s = 0.5 / max(self.sample_rate_hz, 1e-6)
                    time_start -= half_step_s
                    time_end += half_step_s
                if freq_axis.size > 1:
                    freq_start = float(freq_axis[0])
                    freq_end = float(freq_axis[-1])
                else:
                    freq_start = 0.0
                    freq_end = max(float(self.sample_rate_hz) * 0.5, 1.0)
            if self._spectrogram_image is None:
                self._spectrogram_image = spectrogram_ax.imshow(
                    spectrogram_matrix.T,
                    origin="lower",
                    aspect="auto",
                    interpolation="nearest",
                    cmap="magma",
                    extent=[
                        time_start,
                        time_end,
                        freq_start,
                        freq_end,
                    ],
                )
                self._spectrogram_colorbar = self._figure.colorbar(self._spectrogram_image, ax=spectrogram_ax, pad=0.01)
                self._spectrogram_colorbar.set_label("PSD [dB]")
            else:
                self._spectrogram_image.set_data(spectrogram_matrix.T)
                self._spectrogram_image.set_extent([
                    time_start,
                    time_end,
                    freq_start,
                    freq_end,
                ])

            if spectrogram_matrix.size > 0:
                valid_rows = np.isfinite(np.asarray(self.peak_frequency_history_hz, dtype=float))
                peak_times = np.asarray(self.spectrogram_time_s, dtype=float)[valid_rows]
                peak_freqs = np.asarray(self.peak_frequency_history_hz, dtype=float)[valid_rows]
                if peak_times.size > 0:
                    self._peak_trace.set_data(peak_times, peak_freqs)
                else:
                    self._peak_trace.set_data([], [])
                spectrogram_ax.set_xlim(time_start, time_end)
                spectrogram_ax.set_ylim(freq_start, freq_end)

        self._figure.canvas.draw_idle()
        self._figure.canvas.flush_events()
        self._plt.pause(0.001)

    def close(self) -> None:
        if self._plt is not None:
            self._plt.ioff()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live Phyphox magnetometer FFT demo")
    parser.add_argument("--phone-ip", type=str, required=True, help="Phyphox phone IP address")
    parser.add_argument("--port", type=int, default=8080, help="Phyphox HTTP port")
    parser.add_argument("--sample-rate-hz", type=float, default=100.0, help="Polling frequency for the demo loop")
    parser.add_argument("--duration-s", type=float, default=30.0, help="Demo duration in seconds")
    parser.add_argument("--endpoint-path", default="/get?magX&magY&magZ&accX&accY&accZ", help="Phyphox endpoint path")
    parser.add_argument("--timeout-s", type=float, default=1.5, help="HTTP timeout in seconds")
    parser.add_argument("--calibration-seconds", type=float, default=3.0, help="Initial DC calibration window")
    parser.add_argument("--lowpass-window-seconds", type=float, default=0.35, help="Sliding-average window length")
    parser.add_argument("--fft-window-seconds", type=float, default=8.0, help="Rolling FFT window length")
    parser.add_argument("--fft-history-seconds", type=float, default=20.0, help="How much spectrogram history to keep")
    parser.add_argument("--fft-max-frequency-hz", type=float, default=120.0, help="Upper frequency limit for visualization")
    parser.add_argument("--no-viz", action="store_true", help="Disable matplotlib and print FFT summaries instead")
    return parser.parse_args()


def _compute_fft(signal: np.ndarray, sample_rate_hz: float, fft_size: Optional[int] = None) -> tuple[np.ndarray, np.ndarray, Optional[float], float]:
    if signal.size < 4:
        if fft_size is None or fft_size < 4:
            return np.zeros(0, dtype=float), np.zeros(0, dtype=float), None, 0.0
        padded = np.zeros(int(fft_size), dtype=float)
        padded[: signal.size] = signal
        signal = padded
    elif fft_size is not None and fft_size > signal.size:
        padded = np.zeros(int(fft_size), dtype=float)
        padded[: signal.size] = signal
        signal = padded

    centered = signal - np.mean(signal)
    window = np.hanning(signal.size) if signal.size > 1 else np.ones(1, dtype=float)
    spectrum = np.fft.rfft(centered * window)
    frequency_axis_hz = np.fft.rfftfreq(signal.size, d=1.0 / max(sample_rate_hz, 1e-6))
    psd_nt2 = (np.abs(spectrum) ** 2) / max(np.sum(window ** 2), 1e-6)

    if frequency_axis_hz.size <= 1:
        return frequency_axis_hz, psd_nt2, None, 0.0

    spectral_energy = float(np.max(psd_nt2[1:])) if psd_nt2.size > 1 else 0.0
    if spectral_energy <= 0.0:
        return frequency_axis_hz, psd_nt2, None, 0.0

    dominant_index = int(np.argmax(psd_nt2[1:])) + 1
    peak_frequency_hz = float(frequency_axis_hz[dominant_index])
    peak_magnitude_nt = float(np.sqrt(max(psd_nt2[dominant_index], 0.0)))
    return frequency_axis_hz, psd_nt2, peak_frequency_hz, peak_magnitude_nt


def run_demo(
    phone_ip: str,
    port: int,
    sample_rate_hz: float,
    duration_s: float,
    endpoint_path: str,
    timeout_s: float,
    calibration_seconds: float,
    lowpass_window_seconds: float,
    fft_window_seconds: float,
    fft_history_seconds: float,
    fft_max_frequency_hz: float,
    no_viz: bool,
) -> int:
    poll_interval_s = 1.0 / max(sample_rate_hz, 1e-6)
    fft_window_size = max(16, int(round(sample_rate_hz * fft_window_seconds)))
    signal_window: Deque[np.ndarray] = deque(maxlen=fft_window_size)

    streamer = PhyphoxStreamer(
        phone_ip=phone_ip,
        port=port,
        endpoint_path=endpoint_path,
        timeout_s=timeout_s,
    )
    processor = SignalProcessor(
        sample_rate_hz=sample_rate_hz,
        calibration_seconds=calibration_seconds,
        lowpass_window_seconds=lowpass_window_seconds,
    )
    visualizer = FftVisualizer(enabled=not no_viz, sample_rate_hz=sample_rate_hz, history_seconds=fft_history_seconds)

    start_time_s = time.time()
    try:
        while True:
            if duration_s is not None and time.time() - start_time_s >= duration_s:
                break

            sample = streamer.read()
            reading = processor.process(sample)
            vector_uT = reading.as_vector_uT()
            signal_window.append(vector_uT.copy())
            window_matrix = np.asarray(signal_window, dtype=float)
            dominant_axis = FftVisualizer._dominant_axis(window_matrix)
            scalar_window_uT = window_matrix[:, dominant_axis]

            freq_axis_hz, psd_nt2, peak_frequency_hz, peak_magnitude_nt = _compute_fft(scalar_window_uT, sample_rate_hz, fft_size=fft_window_size)
            freq_mask = freq_axis_hz <= fft_max_frequency_hz
            psd_db = 10.0 * np.log10(np.maximum(psd_nt2[freq_mask], 1e-12)) if np.any(freq_mask) else np.zeros(0, dtype=float)

            visualizer.update_time_domain(reading.timestamp_s, vector_uT)
            if freq_axis_hz.size > 0 and np.any(freq_mask):
                visualizer.update_spectrogram(reading.timestamp_s, freq_axis_hz[freq_mask], psd_db, peak_frequency_hz)
            visualizer.render(fft_max_frequency_hz=fft_max_frequency_hz)

            if no_viz:
                magnitude_uT = float(np.linalg.norm(vector_uT))
                if peak_frequency_hz is None:
                    print(f"{reading.timestamp_s:10.3f} s | |B|={magnitude_uT:8.3f} uT | waiting for FFT window... | conf={reading.confidence:.2f}")
                else:
                    print(
                        f"{reading.timestamp_s:10.3f} s | |B|={magnitude_uT:8.3f} uT | axis={dominant_axis} | "
                        f"peak={peak_frequency_hz:7.2f} Hz | amp={peak_magnitude_nt:8.3f} uT | conf={reading.confidence:.2f}"
                    )

            time.sleep(poll_interval_s)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.close()
        streamer.close()

    return 0


def main() -> int:
    args = parse_args()
    return run_demo(
        phone_ip=args.phone_ip,
        port=args.port,
        sample_rate_hz=args.sample_rate_hz,
        duration_s=args.duration_s,
        endpoint_path=args.endpoint_path,
        timeout_s=args.timeout_s,
        calibration_seconds=args.calibration_seconds,
        lowpass_window_seconds=args.lowpass_window_seconds,
        fft_window_seconds=args.fft_window_seconds,
        fft_history_seconds=args.fft_history_seconds,
        fft_max_frequency_hz=args.fft_max_frequency_hz,
        no_viz=args.no_viz,
    )


if __name__ == "__main__":
    raise SystemExit(main())