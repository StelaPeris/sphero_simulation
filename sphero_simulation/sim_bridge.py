#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Float32MultiArray

from .control import YawPID, RollPID, SpeedPID, SpeedEstimator, wrap180

TEL_KEYS = ("main_roll", "pend_roll", "main_pitch", "pend_pitch", "main_yaw", "rod_speed")


class SimBridge(Node):
    def __init__(self):
        super().__init__("sim_bridge")
        p = self.declare_parameter
        p("robot_id", 5)
        p("ball_id", 0)
        p("rate_hz", 50.0)
        p("cmd_timeout", 1.0)            # s bez cmd_vx -> stani
        p("pose_timeout", 0.5)           # s bez poze -> brzina nepouzdana
        p("telem_timeout", 0.5)          # s bez telemetrije -> vx = vy = 0
        p("speed_target_timeout", 0.5)   # s bez speed_target -> natrag na cmd_vx
        p("roll_target_timeout", 0.5)    # s bez roll_target -> yaw petlja / nagib 0
        # petlja brzine (kao robot_bridge): gas = U0 + K_ff * v_ref + PI(v_ref - v), u [0, VX_MAX]
        # zadano: stabilno u simulaciji (vidi config/sim_bridge.yaml); na robotu kp 0.15, ki 0.1, k_ff 0
        p("speed_kp", 0.01); p("speed_ki", 0.03); p("speed_kd", 0.0)
        p("k_ff_speed", 0.15)
        p("u0_speed", 0.05)
        p("vx_max", 0.20)
        # nagib
        p("roll_kp", 0.012); p("roll_ki", 0.0); p("roll_kd", 0.001)
        p("max_roll_rate", 20.0)         # deg/s, rampa za rucni roll_target
        p("roll_filter", 0.15)           # roll_rel_filt = (1-a)*filt + a*raw, kao na robotu
        # smjer
        p("yaw_loop", False)
        p("yaw_kp", 0.45); p("yaw_ki", 0.0); p("yaw_kd", 0.08)
        p("roll_limit", 25.0)
        p("yaw_invert", False)
        g = lambda n: self.get_parameter(n).value

        self.rid, self.ball = g("robot_id"), g("ball_id")
        self.rate = g("rate_hz")
        self.cmd_timeout, self.pose_timeout = g("cmd_timeout"), g("pose_timeout")
        self.telem_timeout = g("telem_timeout")
        self.speed_target_timeout, self.roll_target_timeout = g("speed_target_timeout"), g("roll_target_timeout")
        self.k_ff, self.u0, self.vx_max = g("k_ff_speed"), g("u0_speed"), g("vx_max")
        self.max_roll_rate, self.roll_alpha = g("max_roll_rate"), g("roll_filter")
        self.yaw_loop = g("yaw_loop")

        self.speed_pid = SpeedPID(kp=g("speed_kp"), ki=g("speed_ki"), kd=g("speed_kd"),
                                  out_lim=self.vx_max - self.u0, out_min=-self.u0)
        self.roll_pid = RollPID(kp=g("roll_kp"), ki=g("roll_ki"), kd=g("roll_kd"))
        self.yaw_pid = YawPID(kp=g("yaw_kp"), ki=g("yaw_ki"), kd=g("yaw_kd"),
                              roll_limit=g("roll_limit"), invert=g("yaw_invert"))
        self.est = SpeedEstimator(q=1.0, r=1e-4, pose_timeout=self.pose_timeout)

        # stanje
        self.tel = None; self.last_telem_t = None
        self.roll_rel_filt = None
        self.cmd_vx_ext, self.t_cmd_vx = 0.0, -1e9
        self.speed_cl_target, self.t_speed_target = 0.0, -1e9
        self.roll_target, self.roll_target_ramped, self.t_roll_target = 0.0, 0.0, -1e9
        self.yaw_target, self.yaw_target_received = 0.0, False

        T = lambda name: f"/robot{self.rid}/{name}"
        self.create_subscription(Float32MultiArray, T("sim/telemetry"), self.on_telemetry, 10)
        self.create_subscription(PoseStamped, f"/b{self.ball}/pose", self.on_pose, 10)
        self.create_subscription(Float32, T("cmd_vx"), self.on_cmd_vx, 10)
        self.create_subscription(Float32, T("speed_target"), self.on_speed_target, 10)
        self.create_subscription(Float32, T("roll_target"), self.on_roll_target, 10)
        self.create_subscription(Float32, T("yaw_target"), self.on_yaw_target, 10)
        self.pub_cmd = self.create_publisher(Float32MultiArray, T("sim/cmd"), 10)
        # isti debug topici kao robot_bridge (Foxglove)
        names = ["roll_rel", "roll_ref", "roll_resp", "roll_tgt", "speed_meas", "speed_ref", "speed_fwd",
                 "yaw", "yaw_err_pub", "yaw_ref", "yaw_resp", "vx_out", "vy_out", "rod_speed"]
        self.dbg = {n: self.create_publisher(Float32, T(n), 10) for n in names}

        self.get_logger().info(f"sim_bridge: robot{self.rid}, kugla b{self.ball}, yaw_loop={self.yaw_loop}")
        self.timer = self.create_timer(1.0 / self.rate, self.tick)

    # ---------- callbacki ----------
    def now(self):
        return time.time()

    def on_telemetry(self, msg):
        if len(msg.data) < len(TEL_KEYS):
            return
        self.tel = dict(zip(TEL_KEYS, msg.data))
        if self.last_telem_t is None and not self.yaw_target_received:
            self.yaw_target = wrap180(self.tel["main_yaw"])   # ne okreci se naglo na startu
        self.last_telem_t = self.now()

    def on_pose(self, msg):
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.est.update(msg.pose.position.x, msg.pose.position.y, stamp, self.now())

    def on_cmd_vx(self, msg):
        self.cmd_vx_ext, self.t_cmd_vx = float(msg.data), self.now()

    def on_speed_target(self, msg):
        self.speed_cl_target, self.t_speed_target = float(msg.data), self.now()

    def on_roll_target(self, msg):
        self.roll_target, self.t_roll_target = float(msg.data), self.now()

    def on_yaw_target(self, msg):
        self.yaw_target, self.yaw_target_received = wrap180(float(msg.data)), True

    # ---------- glavna petlja ----------
    def send(self, vx, vy):
        self.pub_cmd.publish(Float32MultiArray(data=[float(vx), float(vy)]))

    def _pub(self, name, val):
        self.dbg[name].publish(Float32(data=float(val)))

    def tick(self):
        now = self.now()
        telem_ok = self.last_telem_t is not None and (now - self.last_telem_t) < self.telem_timeout
        if not telem_ok:
            self.send(0.0, 0.0)
            self.roll_pid.reset(); self.yaw_pid.reset()
            return
        t = self.tel
        pose_ok = self.est.pose_ok(now)
        if not pose_ok:
            self.speed_pid.integral = 0.0

        # --- Y os: (yaw PID) -> roll PID ---
        world_yaw = wrap180(t["main_yaw"])
        err_yaw = wrap180(self.yaw_target - world_yaw)
        roll_rel_raw = t["main_roll"] - t["pend_roll"]
        self.roll_rel_filt = roll_rel_raw if self.roll_rel_filt is None else \
            (1.0 - self.roll_alpha) * self.roll_rel_filt + self.roll_alpha * roll_rel_raw
        roll_rel = self.roll_rel_filt

        max_step = self.max_roll_rate / self.rate
        diff = self.roll_target - self.roll_target_ramped
        self.roll_target_ramped = self.roll_target_ramped + math.copysign(max_step, diff) \
            if abs(diff) > max_step else self.roll_target

        if (now - self.t_roll_target) < self.roll_target_timeout:      # rucni nagib
            target_roll = self.roll_target_ramped
            self.yaw_pid.reset()
        else:
            self.roll_target = 0.0
            self.roll_target_ramped = 0.0
            target_roll = self.yaw_pid.update(self.yaw_target, world_yaw, now) if self.yaw_loop else 0.0

        vy_out = -self.roll_pid.update(target_roll, roll_rel, now)

        # --- X os ---
        speed_cl = (now - self.t_speed_target) < self.speed_target_timeout
        if speed_cl:
            v_ref = self.speed_cl_target
            if v_ref <= 0.0 or not pose_ok:
                self.speed_pid.reset()
                vx_out = 0.0
            else:
                vx_out = max(0.0, min(self.vx_max, self.u0 + self.k_ff * v_ref +
                                      self.speed_pid.update(v_ref, self.est.speed_fwd_filtered, now)))
        else:
            self.speed_pid.reset()
            vx_out = self.cmd_vx_ext if (now - self.t_cmd_vx) < self.cmd_timeout else 0.0

        self.send(vx_out, vy_out)

        # --- debug (isti topici kao robot_bridge) ---
        self._pub("roll_rel", roll_rel_raw); self._pub("roll_resp", roll_rel)
        self._pub("roll_ref", self.roll_target); self._pub("roll_tgt", target_roll)
        self._pub("speed_meas", self.est.speed_meas_filtered); self._pub("speed_fwd", self.est.speed_fwd_filtered)
        self._pub("speed_ref", self.speed_cl_target if speed_cl else 0.0)
        self._pub("yaw", world_yaw); self._pub("yaw_resp", world_yaw); self._pub("yaw_ref", self.yaw_target)
        self._pub("yaw_err_pub", err_yaw)
        self._pub("vx_out", vx_out); self._pub("vy_out", vy_out); self._pub("rod_speed", t["rod_speed"])

    def destroy_node(self):
        try:
            for _ in range(3):
                self.send(0.0, 0.0)
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = SimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()