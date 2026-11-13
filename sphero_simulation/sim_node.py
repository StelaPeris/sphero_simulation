import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32, Float32MultiArray
from sensor_msgs.msg import JointState, Imu
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped

from .sim_core import SpheroSim, model_path


class SimNode(Node):
    def __init__(self):
        super().__init__("sphero_sim")
        p = self.declare_parameter
        p("model", "pendulum")
        p("variant", "drive")
        p("model_file", "")          # puni put do XML-a, ako se ne koristi models/
        p("world", "arena")          # models/worlds/<world>.xml (samo za sphere); "" = kako pise u modelu
        p("publish_rate", 50.0)
        p("realtime_factor", 1.0)    # 0 = što brže
        p("viewer", True)
        p("publish_clock", False)
        p("frame_prefix", "")
        # modeli aktuatora (vidi SpheroSim): vx/vy [-1, 1] -> brzina [rad/s]
        p("esc_gain", SpheroSim.ESC_GAIN)
        p("esc_deadband", SpheroSim.ESC_DEADBAND)
        p("servo_gain_deg", float(np.degrees(SpheroSim.SERVO_GAIN)))
        p("servo_deadband", SpheroSim.SERVO_DEADBAND)
        p("esc_tau", SpheroSim.ESC_TAU)
        p("servo_sign", SpheroSim.SERVO_SIGN)   # -1: vy < 0 -> roll_rel raste, roll_rel > 0 -> skretanje ulijevo
        # telemetrija i kamera kao na robotu
        p("telemetry_rate", 50.0)
        p("roll_sign", 1.0)          # roll_rel = roll_sign * kut servo zgloba
        p("yaw_sign", 1.0)           # +1: kao na robotu, 0 = +x, +90 = +y (suprotno od kazaljke)
        p("yaw_offset_deg", 0.0)     # IMU yaw - smjer u okviru kamere (u bagu ~70 deg)
        p("init_x", 0.0)             # pocetna poza kugle u svijetu [m]
        p("init_y", 0.0)
        p("init_yaw_deg", 0.0)       # pocetni smjer voznje u svijetu [deg] (0 = +x)
        p("camera_topic", "/b0/pose")
        p("camera_rate", 33.0)
        p("camera_noise", 0.0)       # sigma [m] suma pozicije
        p("camera_frame", "world")
        g = lambda n: self.get_parameter(n).value

        xml = g("model_file") or model_path(g("model"), g("variant"))
        self.sim = SpheroSim(xml, g("world"))
        self.get_logger().info(f"Model: {xml}  svijet: {g('world') or '(iz modela)'}")
        self.sim.esc_gain, self.sim.esc_deadband = g("esc_gain"), g("esc_deadband")
        self.sim.servo_gain, self.sim.servo_deadband = np.radians(g("servo_gain_deg")), g("servo_deadband")
        self.sim.esc_tau = g("esc_tau")
        self.sim.servo_sign = g("servo_sign")
        self.sim.set_initial_pose(g("init_x"), g("init_y"), g("init_yaw_deg"))
        self.tel_args = dict(roll_sign=g("roll_sign"), yaw_offset_deg=g("yaw_offset_deg"), yaw_sign=g("yaw_sign"))
        self.tel_dt, self.cam_dt = 1.0 / g("telemetry_rate"), 1.0 / g("camera_rate")
        self.cam_noise, self.cam_frame = g("camera_noise"), g("camera_frame")
        self.t_tel = self.t_cam = -1e9
        self.rng = np.random.default_rng()
        self.rate = g("publish_rate")
        self.rtf = g("realtime_factor")
        self.use_viewer = g("viewer")
        self.pub_clock = g("publish_clock")
        self.prefix = g("frame_prefix")

        self.create_subscription(Float32MultiArray, "sim/cmd", self._on_cmd, 10)
        self.create_subscription(Float32, "cmd/vx", lambda m: self.sim.set_vx(m.data), 10)
        self.create_subscription(Float32, "cmd/vy", lambda m: self.sim.set_vy(m.data), 10)
        self.create_subscription(Float64, "cmd/omega", lambda m: self.sim.set_omega(m.data), 10)
        self.pub_tel = self.create_publisher(Float32MultiArray, "sim/telemetry", 10)
        self.pub_js = self.create_publisher(JointState, "joint_states", 10)
        self.pub_imu = {w: self.create_publisher(Imu, f"imu/{w}", 10) for w in ("main", "pend")}
        self.pub_roll = self.create_publisher(Float64, "roll_rel", 10)
        self.pub_clk = self.create_publisher(Clock, "/clock", 10) if self.pub_clock else None
        self.create_service(Trigger, "reset", self._reset)
        if self.sim.has_shell:
            self.pub_pose = self.create_publisher(PoseStamped, "pose", 10)
            self.pub_head = self.create_publisher(Float64, "heading_deg", 10)
            self.pub_speed = self.create_publisher(Float64, "speed", 10)
            self.pub_cam = self.create_publisher(PoseStamped, g("camera_topic"), 10)

    def _on_cmd(self, msg):
        if len(msg.data) >= 2:
            self.sim.set_vx(msg.data[0]); self.sim.set_vy(msg.data[1])

    def _reset(self, req, res):
        self.sim.reset(); res.success = True; res.message = "reset"
        return res

    def _stamp(self):
        t = self.sim.time
        return Time(sec=int(t), nanosec=int((t % 1) * 1e9)) if self.pub_clock else self.get_clock().now().to_msg()

    def publish(self):
        stamp = self._stamp()
        if self.pub_clk:
            self.pub_clk.publish(Clock(clock=Time(sec=int(self.sim.time), nanosec=int((self.sim.time % 1) * 1e9))))
        names, pos, vel, eff = self.sim.joint_state()
        js = JointState(name=names, position=list(map(float, pos)), velocity=list(map(float, vel)), effort=list(map(float, eff)))
        js.header.stamp = stamp
        self.pub_js.publish(js)
        for w, pub in self.pub_imu.items():
            q, gyro, acc = self.sim.imu(w)
            m = Imu(); m.header.stamp = stamp; m.header.frame_id = f"{self.prefix}{w}_imu"
            m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = map(float, q)
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = map(float, gyro)
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(float, acc)
            pub.publish(m)
        tel = self.sim.telemetry(**self.tel_args)
        self.pub_roll.publish(Float64(data=float(tel["main_roll"] - tel["pend_roll"])))
        t = self.sim.time
        if t - self.t_tel >= self.tel_dt - 1e-9:
            self.t_tel = max(self.t_tel + self.tel_dt, t - self.tel_dt)   # stalni ritam, bez gomilanja
            self.pub_tel.publish(Float32MultiArray(data=[float(tel[k]) for k in
                ("main_roll", "pend_roll", "main_pitch", "pend_pitch", "main_yaw", "rod_speed")]))
        if self.sim.has_shell and t - self.t_cam >= self.cam_dt - 1e-9:
            self.t_cam = max(self.t_cam + self.cam_dt, t - self.cam_dt)
            x, y, _, _ = self.sim.pose()
            cam = PoseStamped(); cam.header.stamp = stamp; cam.header.frame_id = self.cam_frame
            cam.pose.position.x = float(x + self.cam_noise * self.rng.standard_normal())
            cam.pose.position.y = float(y + self.cam_noise * self.rng.standard_normal())
            cam.pose.orientation.w = 1.0
            self.pub_cam.publish(cam)
        if self.sim.has_shell:
            x, y, psi, v = self.sim.pose()
            ps = PoseStamped(); ps.header.stamp = stamp; ps.header.frame_id = "world"
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w, ps.pose.orientation.z = float(np.cos(psi / 2)), float(np.sin(psi / 2))
            self.pub_pose.publish(ps)
            self.pub_head.publish(Float64(data=float(np.degrees(psi))))
            self.pub_speed.publish(Float64(data=v))

    def run(self):
        n_sub = max(1, int(round(1.0 / (self.rate * self.sim.dt))))
        viewer = None
        if self.use_viewer:
            try:
                import mujoco, mujoco.viewer
                viewer = mujoco.viewer.launch_passive(self.sim.m, self.sim.d)
                if self.sim.has_shell:   # kamera prati kuglu
                    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                    viewer.cam.trackbodyid = self.sim.m.body("shell").id
                    viewer.cam.distance, viewer.cam.elevation = 1.4, -25
            except Exception as e:
                self.get_logger().warn(f"Viewer nije dostupan ({e}); nastavljam bez prozora.")
        t0_wall, t0_sim = time.time(), self.sim.time
        while rclpy.ok() and (viewer is None or viewer.is_running()):
            rclpy.spin_once(self, timeout_sec=0.0)
            self.sim.step(n_sub)
            self.publish()
            if viewer is not None:
                viewer.sync()
            if self.rtf > 0:
                lag = (self.sim.time - t0_sim) / self.rtf - (time.time() - t0_wall)
                if lag > 0:
                    time.sleep(lag)
        if viewer is not None:
            viewer.close()


def main():
    rclpy.init()
    node = SimNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()