import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from auv_mag_tracking.config import build_default_scenarios
from auv_mag_tracking.main_viz import AuvCableTrackingSimulation
import copy
scenario = build_default_scenarios()["case1"]
scenario = copy.deepcopy(scenario)
scenario.tracking.use_nominal_route_prior = False
sim = AuvCableTrackingSimulation(scenario)
history = sim.run()
for i in range(0, len(history), 100):
    h = history[i]
    print(f"t={h.time_s:.1f}, pos=[{h.pose.position_ned_m[0]:.1f}, {h.pose.position_ned_m[1]:.1f}], hdg={h.pose.heading_deg:.1f}, fused={h.perception_state.fused_heading_deg}, z_width={h.perception_state.zigzag_width_m:.1f}, src={h.perception_state.guidance_source}")
