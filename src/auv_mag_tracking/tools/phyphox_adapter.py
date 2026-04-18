"""Phyphox hardware adapter for the AUV magnetic perception stack."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
from typing import Deque, Optional
from urllib.request import Request, urlopen
import json
import time

import numpy as np

try:
    import requests
except ImportError:  # pragma: no cover - fallback path for minimal environments
    requests = None


def _is_number(value: object) -> bool:
    return isinstance(value, (int, float, np.integer, np.floating)) and not isinstance(value, bool)


def _as_float_sequence(value: object) -> Optional[np.ndarray]:
    if value is None:
        return None
    if _is_number(value):
        return np.asarray([float(value)], dtype=float)
    if isinstance(value, np.ndarray):
        if value.size == 0:
            return None
        return np.asarray(value, dtype=float).reshape(-1)
    if isinstance(value, (list, tuple)):
        collected = []
        for item in value:
            if _is_number(item):
                collected.append(float(item))
            else:
                nested = _as_float_sequence(item)
                if nested is not None:
                    collected.extend(float(entry) for entry in nested.tolist())
        return np.asarray(collected, dtype=float) if collected else None
    if isinstance(value, dict):
        for key in ("buffer", "values", "data", "samples", "value", "reading"):
            if key in value:
                nested = _as_float_sequence(value[key])
                if nested is not None:
                    return nested
        for nested_value in value.values():
            nested = _as_float_sequence(nested_value)
            if nested is not None:
                return nested
    return None


def _extract_scalar(value: object) -> Optional[float]:
    sequence = _as_float_sequence(value)
    if sequence is None or sequence.size == 0:
        return None
    return float(sequence.reshape(-1)[-1])


def _find_key_case_insensitive(payload: object, target_key: str) -> Optional[object]:
    if not isinstance(payload, dict):
        return None

    target_key_lower = target_key.lower()
    for key, value in payload.items():
        if str(key).lower() == target_key_lower:
            return value

    for value in payload.values():
        if isinstance(value, dict):
            found = _find_key_case_insensitive(value, target_key)
            if found is not None:
                return found
    return None


@dataclass
class PhyphoxSample:
    timestamp_s: float
    timestamp_source: str
    mag_x_uT: float
    mag_y_uT: float
    mag_z_uT: float
    acc_x: Optional[float] = None
    acc_y: Optional[float] = None
    acc_z: Optional[float] = None


@dataclass
class MagnetometerReading:
    timestamp_s: float
    bx_uT: float
    by_uT: float
    bz_uT: float
    confidence: float
    timestamp_source: str = "local"

    def as_vector_uT(self) -> np.ndarray:
        return np.asarray([self.bx_uT, self.by_uT, self.bz_uT], dtype=float)

    @property
    def magnitude_uT(self) -> float:
        return float(np.linalg.norm(self.as_vector_uT()))


class PhyphoxStreamer:
    """Polls the Phyphox HTTP endpoint and extracts the latest magnetometer sample."""

    def __init__(
        self,
        phone_ip: str,
        port: int = 8080,
        endpoint_path: str = "/get?magX&magY&magZ&accX&accY&accZ",
        timeout_s: float = 1.5,
        retries: int = 3,
        backoff_s: float = 0.2,
    ) -> None:
        self.phone_ip = phone_ip
        self.port = int(port)
        self.endpoint_path = endpoint_path if endpoint_path.startswith("/") else f"/{endpoint_path}"
        self.timeout_s = max(float(timeout_s), 0.1)
        self.retries = max(int(retries), 1)
        self.backoff_s = max(float(backoff_s), 0.0)
        self._requests_session = requests.Session() if requests is not None else None
        self._consecutive_failures = 0

    def _build_url(self) -> str:
        host = self.phone_ip
        if not host.startswith(("http://", "https://")):
            host = f"http://{host}"
        return f"{host}:{self.port}{self.endpoint_path}"

    def _json_from_response(self, response_text: str) -> object:
        return json.loads(response_text)

    def _request_via_requests(self, url: str) -> object:
        assert self._requests_session is not None
        response = self._requests_session.get(url, timeout=self.timeout_s)
        response.raise_for_status()
        try:
            return response.json()
        except ValueError:
            return self._json_from_response(response.text)

    def _request_via_urllib(self, url: str) -> object:
        request = Request(url, headers={"Accept": "application/json"})
        with urlopen(request, timeout=self.timeout_s) as response:
            return self._json_from_response(response.read().decode("utf-8"))

    def _request_payload(self) -> object:
        url = self._build_url()
        last_error: Optional[Exception] = None
        for attempt in range(self.retries):
            try:
                if self._requests_session is not None:
                    return self._request_via_requests(url)
                return self._request_via_urllib(url)
            except Exception as exc:  # pragma: no cover - network failure path
                last_error = exc
                self._consecutive_failures += 1
                if self._requests_session is not None:
                    self._requests_session.close()
                    self._requests_session = requests.Session() if requests is not None else None
                if attempt + 1 < self.retries and self.backoff_s > 0.0:
                    time.sleep(self.backoff_s * (attempt + 1))
        if last_error is not None:
            raise last_error
        raise RuntimeError("Phyphox request failed without an explicit exception")

    def close(self) -> None:
        if self._requests_session is not None:
            self._requests_session.close()
            self._requests_session = None

    def read(self) -> PhyphoxSample:
        payload = self._request_payload()
        timestamp_value = None
        timestamp_source = "local"
        for candidate_key in ("timestamp", "time", "time_s", "t"):
            candidate_value = _find_key_case_insensitive(payload, candidate_key)
            if candidate_value is not None:
                timestamp_value = _extract_scalar(candidate_value)
                if timestamp_value is not None:
                    timestamp_source = "phone"
                    break

        bx_value = _extract_scalar(_find_key_case_insensitive(payload, "magX"))
        by_value = _extract_scalar(_find_key_case_insensitive(payload, "magY"))
        bz_value = _extract_scalar(_find_key_case_insensitive(payload, "magZ"))

        if bx_value is None or by_value is None or bz_value is None:
            raise ValueError("Phyphox response did not contain magX, magY, and magZ samples")

        acc_x = _extract_scalar(_find_key_case_insensitive(payload, "accX"))
        acc_y = _extract_scalar(_find_key_case_insensitive(payload, "accY"))
        acc_z = _extract_scalar(_find_key_case_insensitive(payload, "accZ"))
        if timestamp_value is None:
            timestamp_value = time.time()

        self._consecutive_failures = 0
        return PhyphoxSample(
            timestamp_s=float(timestamp_value),
            timestamp_source=timestamp_source,
            mag_x_uT=float(bx_value),
            mag_y_uT=float(by_value),
            mag_z_uT=float(bz_value),
            acc_x=acc_x,
            acc_y=acc_y,
            acc_z=acc_z,
        )


class SignalProcessor:
    """Applies DC-offset removal, magnitude computation, and low-pass smoothing."""

    def __init__(self, sample_rate_hz: float = 20.0, calibration_seconds: float = 3.0, lowpass_window_seconds: float = 0.35) -> None:
        self.sample_rate_hz = max(float(sample_rate_hz), 1e-6)
        self.calibration_seconds = max(float(calibration_seconds), 0.1)
        self.lowpass_window_seconds = max(float(lowpass_window_seconds), 0.05)
        self._calibration_samples: Deque[np.ndarray] = deque()
        self._lowpass_samples: Deque[np.ndarray] = deque(maxlen=max(3, int(round(self.sample_rate_hz * self.lowpass_window_seconds))))
        self._first_timestamp_s: Optional[float] = None
        self._bias_uT = np.zeros(3, dtype=float)
        self._calibrated = False

    @property
    def calibrated(self) -> bool:
        return self._calibrated

    @property
    def bias_uT(self) -> np.ndarray:
        return self._bias_uT.copy()

    def process(self, sample: PhyphoxSample, upstream_confidence: float = 1.0) -> MagnetometerReading:
        raw_vector_uT = np.asarray([sample.mag_x_uT, sample.mag_y_uT, sample.mag_z_uT], dtype=float)
        if self._first_timestamp_s is None:
            self._first_timestamp_s = float(sample.timestamp_s)
        self._calibration_samples.append(raw_vector_uT)
        elapsed_s = max(float(sample.timestamp_s) - self._first_timestamp_s, 0.0)

        if elapsed_s >= self.calibration_seconds and not self._calibrated:
            self._bias_uT = np.mean(np.asarray(self._calibration_samples, dtype=float), axis=0)
            self._calibrated = True
        elif not self._calibrated:
            self._bias_uT = np.mean(np.asarray(self._calibration_samples, dtype=float), axis=0)

        dc_corrected_uT = raw_vector_uT - self._bias_uT
        self._lowpass_samples.append(dc_corrected_uT)
        smoothed_uT = np.mean(np.asarray(self._lowpass_samples, dtype=float), axis=0)

        calibration_progress = min(elapsed_s / self.calibration_seconds, 1.0)
        confidence = float(np.clip(upstream_confidence * (0.25 + 0.75 * calibration_progress), 0.0, 1.0))
        if not self._calibrated:
            confidence = min(confidence, 0.75 * calibration_progress)

        return MagnetometerReading(
            timestamp_s=float(sample.timestamp_s),
            bx_uT=float(smoothed_uT[0]),
            by_uT=float(smoothed_uT[1]),
            bz_uT=float(smoothed_uT[2]),
            confidence=confidence,
            timestamp_source=sample.timestamp_source,
        )


class Visualizer:
    """Simple live visualization for the processed magnetometer stream."""

    def __init__(self, enabled: bool = True, sample_rate_hz: float = 20.0, history_seconds: float = 20.0) -> None:
        self.enabled = enabled
        self.sample_rate_hz = max(float(sample_rate_hz), 1e-6)
        self.history_seconds = max(float(history_seconds), 1.0)
        self.max_points = max(20, int(round(self.sample_rate_hz * self.history_seconds)))
        self.timestamps_s: Deque[float] = deque(maxlen=self.max_points)
        self.bx_uT: Deque[float] = deque(maxlen=self.max_points)
        self.by_uT: Deque[float] = deque(maxlen=self.max_points)
        self.bz_uT: Deque[float] = deque(maxlen=self.max_points)
        self.magnitude_uT: Deque[float] = deque(maxlen=self.max_points)
        self._figure = None
        self._axes = None
        self._lines = None
        self._print_counter = 0

        if self.enabled:
            try:
                import matplotlib.pyplot as plt

                plt.ion()
                self._plt = plt
                self._setup_matplotlib()
            except Exception:
                self.enabled = False
                self._plt = None
        else:
            self._plt = None

    def _setup_matplotlib(self) -> None:
        plt = self._plt
        assert plt is not None
        self._figure, self._axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
        self._figure.suptitle("Phyphox Magnetometer Live View")
        top_ax, bottom_ax = self._axes
        top_ax.set_title("Triaxial Magnetic Field")
        top_ax.set_ylabel("Field [uT]")
        top_ax.grid(True, alpha=0.3)
        bottom_ax.set_title("Field Magnitude")
        bottom_ax.set_xlabel("Time [s]")
        bottom_ax.set_ylabel("Magnitude [uT]")
        bottom_ax.grid(True, alpha=0.3)
        (bx_line,) = top_ax.plot([], [], label="Bx", color="tab:blue")
        (by_line,) = top_ax.plot([], [], label="By", color="tab:orange")
        (bz_line,) = top_ax.plot([], [], label="Bz", color="tab:green")
        (magnitude_line,) = bottom_ax.plot([], [], label="|B|", color="tab:red")
        top_ax.legend(loc="upper right", fontsize=8)
        bottom_ax.legend(loc="upper right", fontsize=8)
        self._lines = (bx_line, by_line, bz_line, magnitude_line)

    def update(self, reading: MagnetometerReading) -> None:
        if not self.enabled:
            self._print_counter += 1
            if self._print_counter % 1 == 0:
                print(
                    f"{reading.timestamp_s:10.3f} s | "
                    f"Bx={reading.bx_uT:8.3f} uT | By={reading.by_uT:8.3f} uT | "
                    f"Bz={reading.bz_uT:8.3f} uT | |B|={reading.magnitude_uT:8.3f} uT | "
                    f"conf={reading.confidence:.2f} | src={reading.timestamp_source}"
                )
            return

        self.timestamps_s.append(reading.timestamp_s)
        self.bx_uT.append(reading.bx_uT)
        self.by_uT.append(reading.by_uT)
        self.bz_uT.append(reading.bz_uT)
        self.magnitude_uT.append(reading.magnitude_uT)

        if self._figure is None or self._lines is None or self._axes is None:
            return

        top_ax, bottom_ax = self._axes
        bx_line, by_line, bz_line, magnitude_line = self._lines
        time_axis = np.asarray(self.timestamps_s, dtype=float)
        bx_line.set_data(time_axis, np.asarray(self.bx_uT, dtype=float))
        by_line.set_data(time_axis, np.asarray(self.by_uT, dtype=float))
        bz_line.set_data(time_axis, np.asarray(self.bz_uT, dtype=float))
        magnitude_line.set_data(time_axis, np.asarray(self.magnitude_uT, dtype=float))

        if time_axis.size > 1:
            left_edge = float(time_axis[0])
            right_edge = float(time_axis[-1])
            top_ax.set_xlim(left_edge, right_edge)
            bottom_ax.set_xlim(left_edge, right_edge)

            axis_values = np.concatenate([np.asarray(self.bx_uT), np.asarray(self.by_uT), np.asarray(self.bz_uT)])
            magnitude_values = np.asarray(self.magnitude_uT, dtype=float)
            field_min = float(np.min(axis_values))
            field_max = float(np.max(axis_values))
            mag_min = float(np.min(magnitude_values))
            mag_max = float(np.max(magnitude_values))
            field_padding = max(1.0, 0.08 * max(abs(field_min), abs(field_max), 1.0))
            magnitude_padding = max(1.0, 0.08 * max(abs(mag_min), abs(mag_max), 1.0))
            top_ax.set_ylim(field_min - field_padding, field_max + field_padding)
            bottom_ax.set_ylim(max(0.0, mag_min - magnitude_padding), mag_max + magnitude_padding)

        self._figure.canvas.draw_idle()
        self._figure.canvas.flush_events()
        self._plt.pause(0.001)

    def close(self) -> None:
        if self._plt is not None:
            self._plt.ioff()


class PhyphoxMagnetometerAdapter:
    """High-level facade that hides transport and signal processing details."""

    def __init__(
        self,
        phone_ip: str,
        port: int = 8080,
        sample_rate_hz: float = 20.0,
        endpoint_path: str = "/get?magX&magY&magZ&accX&accY&accZ",
        timeout_s: float = 1.5,
        retries: int = 3,
        backoff_s: float = 0.2,
        calibration_seconds: float = 3.0,
        lowpass_window_seconds: float = 0.35,
        enable_visualizer: bool = True,
        history_seconds: float = 20.0,
    ) -> None:
        self.streamer = PhyphoxStreamer(
            phone_ip=phone_ip,
            port=port,
            endpoint_path=endpoint_path,
            timeout_s=timeout_s,
            retries=retries,
            backoff_s=backoff_s,
        )
        self.processor = SignalProcessor(
            sample_rate_hz=sample_rate_hz,
            calibration_seconds=calibration_seconds,
            lowpass_window_seconds=lowpass_window_seconds,
        )
        self.visualizer = Visualizer(enabled=enable_visualizer, sample_rate_hz=sample_rate_hz, history_seconds=history_seconds)
        self._latest_reading: Optional[MagnetometerReading] = None

    def get_latest_reading(self) -> Optional[MagnetometerReading]:
        try:
            sample = self.streamer.read()
            reading = self.processor.process(sample)
            self._latest_reading = reading
            self.visualizer.update(reading)
            return reading
        except Exception:
            if self._latest_reading is None:
                return None

            stale_confidence = max(self._latest_reading.confidence * 0.90, 0.0)
            stale_reading = replace(self._latest_reading, confidence=stale_confidence, timestamp_source="cache")
            self._latest_reading = stale_reading
            self.visualizer.update(stale_reading)
            return stale_reading

    def close(self) -> None:
        self.streamer.close()
        self.visualizer.close()

    def __enter__(self) -> "PhyphoxMagnetometerAdapter":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


def run_demo(
    phone_ip: str,
    port: int = 8080,
    sample_rate_hz: float = 20.0,
    duration_s: Optional[float] = 30.0,
    endpoint_path: str = "/get?magX&magY&magZ&accX&accY&accZ",
    no_viz: bool = False,
    timeout_s: float = 1.5,
    calibration_seconds: float = 3.0,
    lowpass_window_seconds: float = 0.35,
) -> int:
    start_time_s = time.time()
    poll_interval_s = 1.0 / max(sample_rate_hz, 1e-6)
    with PhyphoxMagnetometerAdapter(
        phone_ip=phone_ip,
        port=port,
        sample_rate_hz=sample_rate_hz,
        endpoint_path=endpoint_path,
        timeout_s=timeout_s,
        calibration_seconds=calibration_seconds,
        lowpass_window_seconds=lowpass_window_seconds,
        enable_visualizer=not no_viz,
    ) as adapter:
        try:
            while True:
                if duration_s is not None and time.time() - start_time_s >= duration_s:
                    break
                reading = adapter.get_latest_reading()
                if reading is None and no_viz:
                    print("Waiting for the first Phyphox sample...")
                time.sleep(poll_interval_s)
        except KeyboardInterrupt:
            pass
    return 0
