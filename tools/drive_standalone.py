import os, sys, time
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import mujoco, mujoco.viewer
from sphero_simulation.sim_core import SpheroSim, model_path

model = sys.argv[1] if len(sys.argv) > 1 else "sphere"
variant = sys.argv[2] if len(sys.argv) > 2 else "drive"
world = sys.argv[3] if len(sys.argv) > 3 else "arena"
sim = SpheroSim(model_path(model, variant), world)
cmd = {"vx": 0.0, "vy": 0.0}


def key(k):
    if k == 265: cmd["vx"] += 0.01
    elif k == 264: cmd["vx"] -= 0.01
    elif k == 262: cmd["vy"] += 0.05
    elif k == 263: cmd["vy"] -= 0.05
    elif k == 32: cmd["vy"] = 0.0
    elif k == 48: cmd["vx"] = cmd["vy"] = 0.0
    cmd["vx"], cmd["vy"] = float(np.clip(cmd["vx"], -1, 1)), float(np.clip(cmd["vy"], -1, 1))


with mujoco.viewer.launch_passive(sim.m, sim.d, key_callback=key) as v:
    last = 0
    while v.is_running():
        t0 = time.time()
        sim.set_vx(cmd["vx"]); sim.set_vy(cmd["vy"]); sim.step(10); v.sync()
        if sim.time - last > 0.5:
            tel = sim.telemetry(); last = sim.time
            extra = f"  v {sim.pose()[3]:+.2f} m/s" if sim.has_shell else ""
            print(f"t={sim.time:6.1f}  vx {cmd['vx']:+.2f}  vy {cmd['vy']:+.2f}  roll_rel {tel['main_roll'] - tel['pend_roll']:+6.1f} deg{extra}", end="\r")
        time.sleep(max(0.0, 10 * sim.dt - (time.time() - t0)))