import os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from sphero_simulation.sim_core import SpheroSim, model_path
from sphero_simulation.control import SpeedPID, RollPID, KalmanCV


def gamma_deg(sim):
    return np.degrees(sim.d.qpos[sim.j_servo.qposadr[0]])


def test_servo_is_velocity_servo():
    sim = SpheroSim(model_path("pendulum", "drive"))
    assert abs(sim.m.body_subtreemass[sim.m.body("hub").id] - 4.50) < 0.01
    sim.set_vy(-0.2); sim.step(1000)                      # 1 s (servo_sign = -1: vy < 0 -> kut raste)
    rate = gamma_deg(sim)                                 # deg nakon 1 s ~ deg/s
    assert abs(rate - 58 * (0.2 - 0.01)) < 1.5            # 58 deg/s po jedinici, mrtva zona 0.01
    g = gamma_deg(sim); sim.set_vy(0.0); sim.step(2000)
    assert abs(gamma_deg(sim) - g) < 0.5                  # vy = 0 -> drzi kut


def test_bench_shaft_spins_pendulum_fixed():
    sim = SpheroSim(model_path("pendulum", "bench"))
    sim.set_omega(1.566); sim.step(6000)               # ESC ima kasnjenje (esc_tau = 1 s)
    _, pos, vel, _ = sim.joint_state()
    assert abs(vel[0] - 1.566) < 0.02
    assert np.allclose(sim.d.xquat[sim.m.body("hub").id], [1, 0, 0, 0])


def test_sphere_rolls_forward_x():
    sim = SpheroSim(model_path("sphere", "drive"), "empty")
    sim.set_vx(0.09); sim.step(8000)
    x, y, psi, v = sim.pose()
    assert x > 1.0 and abs(y) < 0.3                       # pozitivan gas -> naprijed u +x
    assert abs(np.degrees(psi)) < 10


def test_sign_conventions_like_robot():
    """vy < 0 -> roll_rel raste (vy = -RollPID je negativna povratna veza);
    roll_rel > 0 -> skretanje ulijevo, yaw raste (0 = +x, +90 = +y)."""
    sim = SpheroSim(model_path("sphere", "drive"), "empty")
    t0 = sim.telemetry()
    sim.set_vy(-0.3); sim.step(300); sim.set_vy(0.0)
    t1 = sim.telemetry()
    rr = lambda t: t["main_roll"] - t["pend_roll"]
    assert rr(t1) > rr(t0) + 3
    sim.set_vx(0.09); sim.step(8000)
    assert sim.telemetry()["main_yaw"] > 20               # skrenuo ulijevo


def test_world_switch():
    names = lambda sim: {sim.m.geom(i).name for i in range(sim.m.ngeom)}
    assert "wall_xp" in names(SpheroSim(model_path("sphere", "drive"), "arena"))
    assert "wall_xp" not in names(SpheroSim(model_path("sphere", "drive"), "empty"))
    SpheroSim(model_path("pendulum", "drive"), "arena")      # pendulum: world se ignorira


def test_control_classes_match_robot_bridge():
    p = SpeedPID(0.15, 0.1, 0.0, out_lim=0.15, out_min=-0.05)
    assert abs(p.update(0.25, 0.0, now=0.0) - 0.0375) < 1e-12
    assert abs(p.update(0.25, 0.1, now=0.05) - 0.02375) < 1e-12
    r = RollPID(kp=0.012, ki=0.0, kd=0.001)
    assert abs(r.update(10.0, 0.0, now=0.0) - 0.12) < 1e-12
    k = KalmanCV(); k.update(0, 0)
    for i in range(1, 40):
        k.predict(0.03); k.update(0.3 * 0.03 * i, 0)
    assert abs(k.speed() - 0.3) < 0.02