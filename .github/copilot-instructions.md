# Project Guidelines

## Code Style
- This is a Python simulation project. Prefer small, explicit modules with module docstrings, dataclasses, type hints, and NumPy-first numeric code.
- Keep the current CLI style in `main_demo.py`: `argparse`, a `main()` function, and `if __name__ == "__main__"`.
- Inside the package, prefer relative imports and keep `sys.path` manipulation only in entrypoints and tests.
- Match the existing naming style: config objects in `src/auv_mag_tracking/config/__init__.py`, runtime logic in `controller.py`, `perception.py`, `perception_driver.py`, and `main_viz.py`.

## Architecture
- The project is a layered AUV magnetic cable tracking demo: config → environment/sensor model → perception driver → perception → behavior tree/controller → visualization.
- `src/auv_mag_tracking/config/__init__.py` defines the scenario dataclasses and canned cases; treat `build_default_scenarios()` as the source of truth for supported demo modes.
- `src/auv_mag_tracking/main_viz.py` is the orchestration layer that connects environment, sensing, perception, control, and plotting.
- `src/auv_mag_tracking/controller.py` implements behavior-tree-backed zig-zag guidance and constrained vehicle kinematics.

## Build and Test
- Install dependencies from `requirements.txt` (`numpy`, `matplotlib`, `scipy`, `tqdm`).
- Run tests with: `python -m unittest discover -s tests`.
- List available scenarios with: `python main_demo.py --list-cases`.
- Run a headless smoke test with: `python main_demo.py --case case1 --no-viz`.
- The optional hardware demo uses Phyphox HTTP polling via `main_demo.py --phyphox-ip ...`.

## Project Conventions
- Default tracking mode is 50 Hz AC; the docs also preserve a lower-frequency demo mode for comparison.
- Perception is feature-based, not raw-signal-only: `perception_driver.py` extracts spectral and reliability features before `perception.py` fuses state.
- Burial depth is shown as truth plus an auxiliary estimate from the simulated survey channel; do not describe it as pure magnetic inversion.
- Visual feedback uses confidence-driven uncertainty ellipses and estimated centerlines; preserve that semantics when extending the UI.
- Scenario names and parameters are intentionally curated for experiments such as straight tracking, turns, high-noise operation, and attitude disturbance.

## Integration Points
- `main_demo.py` supports a simulator connector (`none`/`mock`) and a Phyphox adapter; keep both paths working when changing startup logic.
- `src/auv_mag_tracking/sensor_model.py` is the main integration boundary for magnetometer, IMU, sonar, and burial-depth observations.
- `src/auv_mag_tracking/perception_driver.py` is the adapter between raw magnetometer blocks and semantic features consumed by perception.
- `src/auv_mag_tracking/main_viz.py` is the best place to inspect end-to-end data flow when debugging.

## Security
- No authentication or secret-management layer is present in the repo.
- Any external network access is opt-in and user-supplied, mainly the Phyphox HTTP endpoint.
- Do not hardcode device IPs, credentials, tokens, or calibration data that should stay local to the operator.

## Reference Docs
- Primary project overview: [README.md](../README.md)
- Detailed implementation notes: [原理与代码详解.md](../原理与代码详解.md)
- Tuning and experiment log: [调参与实施详尽记录.md](../调参与实施详尽记录.md)
- Standards mapping and operating assumptions live under [标准文档/](../标准文档/)
