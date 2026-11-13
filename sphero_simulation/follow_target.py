#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker


def wrap180(deg):
    return math.degrees(math.atan2(math.sin(math.radians(deg)), math.cos(math.radians(deg))))


class FollowTarget(Node):
    def __init__(self):
        super().__init__("follow_target")
        p = self.declare_parameter
        p("robot_id", 10)
        p("ball_id", 0)
        p("yaw_offset", 0.0)         # deg, body(IMU) = world + offset (kao u robot_config.json)
        p("target_topic", "/clicked_point")
        p("follow_distance", 1.0)    # m, zeljena udaljenost od tocke
        p("stop_band", 0.10)         # m, stane kad je d < follow_distance + stop_band
        p("start_band", 0.30)        # m, krene kad je d > follow_distance + start_band
        p("gas_min", 0.07)           # gas pri sporoj voznji (~0.17 m/s u simulaciji)
        p("gas_max", 0.12)           # najveci gas (~0.4 m/s u simulaciji)
        p("k_gas", 0.03)             # gas po metru viska udaljenosti
        p("slow_angle", 60.0)        # deg, iznad ove greske smjera vozi s gas_min
        p("target_timeout", 0.0)     # s, 0 = tocka vrijedi zauvijek (klik); za covjeka npr. 0.5
        p("pose_timeout", 0.5)       # s, bez poze kugle -> stani
        p("rate_hz", 20.0)
        g = lambda n: self.get_parameter(n).value
        for n in ("yaw_offset", "follow_distance", "stop_band", "start_band", "gas_min", "gas_max",
                  "k_gas", "slow_angle", "target_timeout", "pose_timeout"):
            setattr(self, n, float(g(n)))
        rid, ball = g("robot_id"), g("ball_id")

        self.target = None; self.t_target = None
        self.pos = None; self.t_pose = None
        self.driving = False
        self.yaw_ref = None
        self.heading = None          # trenutni smjer robota u svijetu [deg] (IMU yaw - offset)

        self.create_subscription(PointStamped, g("target_topic"), self.on_target, 10)
        self.create_subscription(PoseStamped, f"/b{ball}/pose", self.on_pose, 10)
        self.create_subscription(Float32, f"/robot{rid}/yaw", self.on_yaw, 10)
        self.pub_yaw = self.create_publisher(Float32, f"/robot{rid}/yaw_target", 10)
        self.pub_vx = self.create_publisher(Float32, f"/robot{rid}/cmd_vx", 10)
        self.pub_marker = self.create_publisher(Marker, "/follow/target_marker", 10)
        self.create_timer(1.0 / g("rate_hz"), self.tick)
        self.get_logger().info(f"follow_target: robot{rid}, b{ball}, cilj s {g('target_topic')}, "
                               f"udaljenost {self.follow_distance} m")

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_target(self, msg):
        first = self.target is None
        self.target = (msg.point.x, msg.point.y)
        self.t_target = self.now()
        self.frame = msg.header.frame_id or "world"
        if first:
            self.get_logger().info(f"Cilj: ({self.target[0]:.2f}, {self.target[1]:.2f})")

    def on_pose(self, msg):
        self.pos = (msg.pose.position.x, msg.pose.position.y)
        self.t_pose = self.now()

    def on_yaw(self, msg):
        self.heading = wrap180(float(msg.data) - self.yaw_offset)

    def stop(self):
        self.pub_vx.publish(Float32(data=0.0))
        if self.yaw_ref is not None:
            self.pub_yaw.publish(Float32(data=float(self.yaw_ref)))

    def tick(self):
        now = self.now()
        pose_ok = self.pos is not None and (now - self.t_pose) < self.pose_timeout
        target_ok = self.target is not None and (
            self.target_timeout <= 0 or (now - self.t_target) < self.target_timeout)
        if not (pose_ok and target_ok):
            if self.driving:
                self.get_logger().info("Nema cilja ili poze -> STOP")
            self.driving = False
            self.stop()
            return

        ex, ey = self.target[0] - self.pos[0], self.target[1] - self.pos[1]
        d = math.hypot(ex, ey)
        psi = math.degrees(math.atan2(ey, ex))                    # smjer prema tocki (svijet)
        self.yaw_ref = wrap180(psi + self.yaw_offset)             # u okviru IMU-a, kao bouncing_driver

        # histereza voznja/stop
        if self.driving and d < self.follow_distance + self.stop_band:
            self.driving = False
            self.get_logger().info(f"Na udaljenosti {d:.2f} m -> STOP")
        elif not self.driving and d > self.follow_distance + self.start_band:
            self.driving = True
            self.get_logger().info(f"Udaljenost {d:.2f} m -> VOZIM")

        if self.driving:
            gas = self.gas_min + self.k_gas * (d - self.follow_distance)
            gas = max(self.gas_min, min(self.gas_max, gas))
            # tocka iza / sa strane: polako, robot skrece samo dok se kotrlja
            if self.heading is not None and abs(wrap180(psi - self.heading)) > self.slow_angle:
                gas = self.gas_min
        else:
            gas = 0.0

        self.pub_yaw.publish(Float32(data=float(self.yaw_ref)))
        self.pub_vx.publish(Float32(data=float(gas)))
        self.publish_marker()

    def publish_marker(self):
        m = Marker()
        m.header.frame_id = getattr(self, "frame", "world")
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = "follow_target", 0
        m.type, m.action = Marker.CYLINDER, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.target[0], self.target[1], 0.25
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 0.3; m.scale.z = 0.5
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 0.2, 0.8
        self.pub_marker.publish(m)


def main():
    rclpy.init()
    node = FollowTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub_vx.publish(Float32(data=0.0))
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()