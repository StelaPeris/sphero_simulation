import os
import numpy as np
import mujoco


def models_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("sphero_simulation"), "models")
    except Exception:
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models")


def model_path(model, variant="drive"):
    """model='pendulum', variant='drive' -> models/pendulum/pendulum_drive.xml"""
    name = f"{model}_{variant}.xml" if variant else f"{model}.xml"
    return os.path.join(models_dir(), model, name)


def world_path(world):
    """world='arena' -> models/worlds/arena.xml (ili puni put do .xml)."""
    return world if world.endswith(".xml") else os.path.join(models_dir(), "worlds", f"{world}.xml")


def load_model(xml_path, world=None):
    """Ucita model; ako model ukljucuje svijet (<include file="../worlds/...">), world ga zamijeni.
    Ostale relativne putanje (meshdir, include) pretvaraju se u apsolutne, pa se model ucita iz privremene kopije."""
    if not world:
        return mujoco.MjModel.from_xml_path(xml_path)
    import re, tempfile
    base = os.path.dirname(os.path.abspath(xml_path))
    txt = open(xml_path).read()
    if not re.search(r'<include file="[^"]*worlds/[^"]+"', txt):
        return mujoco.MjModel.from_xml_path(xml_path)     # model bez svijeta (npr. pendulum): world se ignorira
    wp = os.path.abspath(world_path(world))
    if not os.path.isfile(wp):
        raise FileNotFoundError(f"Nema svijeta '{world}': {wp}")
    txt = re.sub(r'<include file="[^"]*worlds/[^"]+"', f'<include file="{wp}"', txt)
    txt = re.sub(r'meshdir="([^"/][^"]*)"', lambda mm: f'meshdir="{os.path.join(base, mm.group(1))}"', txt)
    with tempfile.NamedTemporaryFile("w", suffix=".xml", delete=False) as f:
        f.write(txt)
    try:
        return mujoco.MjModel.from_xml_path(f.name)
    finally:
        os.unlink(f.name)


def deadband(u, db):
    """Mrtva zona: |u| <= db -> 0, inace u pomaknut prema nuli za db."""
    return float(np.sign(u) * max(abs(u) - db, 0.0))


class SpheroSim:

    # zadane vrijednosti (mogu se promijeniti parametrima cvora)
    ESC_GAIN = 28.0          # rad/s po jedinici vx iznad mrtve zone - PROCJENA: gas 0.09 -> 0.26 m/s (1.41 rad/s)
    ESC_DEADBAND = 0.04      # = 20 us; robot_bridge kompenzira s U0_SPEED = 0.05 - PROCJENA
    SERVO_GAIN = np.radians(58.0)   # rad/s po jedinici vy; iz baga MSU_test_cet_006 (R^2 = 0.70)
    SERVO_DEADBAND = 0.01    # = 5 us; servo se gotovo ne mice ispod ~10 us
    SERVO_SIGN = -1.0        # smjer vrtnje servoa: vy > 0 -> kut zgloba pada (kao na robotu, vidi telemetry)
    ESC_TAU = 1.0            # s, kasnjenje ESC-a + motora (prvi red) - PROCJENA iz skoka gasa u bagu

    def __init__(self, xml_path, world=None):
        self.m = load_model(xml_path, world)
        self.d = mujoco.MjData(self.m)
        self.dt = self.m.opt.timestep
        self.a_esc = self.m.actuator("esc").id
        self.a_servo = self.m.actuator("servo").id
        self.j_drive = self.m.joint("drive")
        self.j_servo = self.m.joint("servo")
        self.esc_gain, self.esc_deadband = self.ESC_GAIN, self.ESC_DEADBAND
        self.servo_gain, self.servo_deadband = self.SERVO_GAIN, self.SERVO_DEADBAND
        self.esc_tau = self.ESC_TAU
        self.servo_sign = self.SERVO_SIGN
        self.vx = self.vy = 0.0
        self.omega_cmd = 0.0        # izlaz ESC-a nakon kasnjenja [rad/s]
        self.omega_ref = 0.0        # zadana brzina glavcine [rad/s]
        self.servo_rate_ref = 0.0   # zadana brzina nagiba [rad/s]
        mujoco.mj_forward(self.m, self.d)

    # ---- naredbe -------------------------------------------------------
    def _clip(self, a, v):
        lo, hi = self.m.actuator_ctrlrange[a]
        return float(np.clip(v, lo, hi))

    def set_vx(self, vx):
        """Naredba ESC-u kao na robotu, [-1, 1]."""
        self.vx = float(np.clip(vx, -1.0, 1.0))
        self.omega_ref = self._clip(self.a_esc, self.esc_gain * deadband(self.vx, self.esc_deadband))

    def set_vy(self, vy):
        """Naredba servu kao na robotu, [-1, 1] (servo za brzinu)."""
        self.vy = float(np.clip(vy, -1.0, 1.0))
        self.servo_rate_ref = self._clip(self.a_servo, self.servo_sign * self.servo_gain *
                                         deadband(self.vy, self.servo_deadband))

    def set_omega(self, w):
        """Izravno brzina glavcine [rad/s] (zaobilazi model ESC-a)."""
        self.omega_ref = self._clip(self.a_esc, w)

    def set_initial_pose(self, x=0.0, y=0.0, yaw_deg=0.0):
        """Pocetna poza kugle (samo sphere): srediste (x, y) [m] i smjer voznje psi [deg] u svijetu
        (0 = +x, 90 = +y). Upisuje se u qpos0, pa vrijedi i nakon reset()."""
        if not self.has_shell:
            return
        j = self.m.body("shell").jntadr[0]; a = self.m.jnt_qposadr[j]
        if not hasattr(self, "_q_base"):
            self._q_base = self.m.qpos0[a + 3:a + 7].copy()        # orijentacija za psi = 0 (iz XML-a)
        qz = np.array([np.cos(np.radians(yaw_deg) / 2), 0, 0, np.sin(np.radians(yaw_deg) / 2)])
        q = np.zeros(4); mujoco.mju_mulQuat(q, qz, self._q_base)
        self.m.qpos0[a:a + 2] = [x, y]
        self.m.qpos0[a + 3:a + 7] = q
        self.reset()

    # ---- simulacija ----------------------------------------------------
    def step(self, n=1):
        self.d.ctrl[self.a_servo] = self.servo_rate_ref
        if self.esc_tau <= 0:
            self.omega_cmd = self.omega_ref
            self.d.ctrl[self.a_esc] = self.omega_cmd
            mujoco.mj_step(self.m, self.d, nstep=n)
            return
        a = self.dt / (self.esc_tau + self.dt)
        for _ in range(n):              # ESC: prvi red prema omega_ref
            self.omega_cmd += a * (self.omega_ref - self.omega_cmd)
            self.d.ctrl[self.a_esc] = self.omega_cmd
            mujoco.mj_step(self.m, self.d)

    def reset(self):
        mujoco.mj_resetData(self.m, self.d)   # qpos <- qpos0 (ukljucuje pocetnu pozu)
        self.vx = self.vy = self.omega_ref = self.servo_rate_ref = self.omega_cmd = 0.0
        mujoco.mj_forward(self.m, self.d)

    @property
    def time(self):
        return self.d.time

    # ---- stanje --------------------------------------------------------
    def joint_state(self):
        """(imena, pozicije, brzine, momenti) za drive i servo."""
        jd, js = self.j_drive, self.j_servo
        pos = [self.d.qpos[jd.qposadr[0]], self.d.qpos[js.qposadr[0]]]
        vel = [self.d.qvel[jd.dofadr[0]], self.d.qvel[js.dofadr[0]]]
        eff = [self.d.actuator_force[self.a_esc], self.d.actuator_force[self.a_servo]]
        return ["drive", "servo"], pos, vel, eff

    @property
    def has_shell(self):
        return mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "shell") >= 0

    def pose(self):
        """Kugla: (x, y, psi, v). psi = smjer voznje iz osi osovine s (os x ljuske):
        smjer naprijed f = (-s_y, s_x), psi = atan2(f_y, f_x); v = brzina sredista u smjeru f."""
        b = self.m.body("shell").id
        s = self.d.xmat[b].reshape(3, 3)[:, 0]
        f = np.array([-s[1], s[0]]); f /= np.linalg.norm(f) + 1e-12
        x, y = self.d.xpos[b][:2]
        v = float(np.dot(self.d.qvel[:2], f))
        return float(x), float(y), float(np.arctan2(f[1], f[0])), v

    def telemetry(self, roll_sign=1.0, yaw_offset_deg=0.0, yaw_sign=1.0):
        """Telemetrija kao TelemetryData s robota, u stupnjevima.
        roll_rel = main_roll - pend_roll = roll_sign * kut zgloba servo.
        main_roll = bocni nagib glavcine (osovine) prema horizontali; main_pitch = zakret glavcine oko osovine
        od vertikale; main_yaw = smjer voznje psi (samo s ljuskom) * yaw_sign + yaw_offset."""
        hub = self.d.xmat[self.m.body("hub").id].reshape(3, 3)
        sx, zb = hub[:, 0], hub[:, 2]                         # os osovine i "gore" glavcine u svijetu
        main_roll = np.degrees(np.arcsin(np.clip(sx[2], -1, 1)))
        f = np.cross([0.0, 0.0, 1.0], sx); n = np.linalg.norm(f)
        f = f / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])  # smjer naprijed (vodoravno, okomito na osovinu)
        main_pitch = np.degrees(np.arctan2(np.dot(zb, f), zb[2]))
        gamma = np.degrees(self.d.qpos[self.j_servo.qposadr[0]])
        roll_rel = roll_sign * gamma
        yaw = np.degrees(self.pose()[2]) * yaw_sign + yaw_offset_deg if self.has_shell else 0.0
        yaw = (yaw + 180.0) % 360.0 - 180.0
        return dict(main_roll=main_roll, pend_roll=main_roll - roll_rel, main_pitch=main_pitch,
                    pend_pitch=main_pitch, main_yaw=yaw, rod_speed=float(self.d.qvel[self.j_drive.dofadr[0]]))

    def sensor(self, name):
        return self.d.sensor(name).data.copy()

    def imu(self, which):
        """which='main' ili 'pend': (quat wxyz, kutna brzina, linearna akceleracija) u okviru IMU-a."""
        q = self.sensor(f"{which}_q")
        site = self.m.site(f"{which}_imu").id
        res = np.zeros(6)
        mujoco.mj_objectVelocity(self.m, self.d, mujoco.mjtObj.mjOBJ_SITE, site, res, 1)
        acc = np.zeros(6)
        mujoco.mj_rnePostConstraint(self.m, self.d)
        mujoco.mj_objectAcceleration(self.m, self.d, mujoco.mjtObj.mjOBJ_SITE, site, acc, 1)
        # MuJoCo u cacc vec ukljucuje gravitaciju (kao akcelerometar: mirno tijelo daje +9.81 prema gore)
        return q, res[:3], acc[3:]


def quat_to_rpy(q):
    """MuJoCo quat (w,x,y,z) -> roll, pitch, yaw [rad]."""
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw