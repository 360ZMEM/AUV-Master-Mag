import sys
sys.path.insert(0, ".")
from tools.diagnose_heading_error import run_diagnostic
import numpy as np

def analyze():
    scenario, history, positions, true_cable_x, true_cable_y, offset_distance = run_diagnostic("case1")
    t = np.array(history["time_s"])
    y = positions[:, 1]
    x = positions[:, 0]
    hdg = np.array(history["fused_heading_deg"])
    mode = np.array(history["mode"])
    src = np.array(history["guidance_source"])
    peaks = np.array(history["peak_detected"])
    
    print("Events:")
    last_src = None
    last_mode = None
    for i in range(len(t)):
        if src[i] != last_src or mode[i] != last_mode or peaks[i]:
            print(f"t={t[i]:.1f}s: x={x[i]:.1f}, y={y[i]:.1f}, fused={hdg[i]}, mode={mode[i]}, src={src[i]}, peak={peaks[i]}")
            last_src = src[i]
            last_mode = mode[i]

analyze()