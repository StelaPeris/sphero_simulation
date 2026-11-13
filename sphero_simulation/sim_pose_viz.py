#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker


def yaw_quat(deg):
    h = math.radians(deg) / 2.0
    return 0.0, 0.0, math.sin(h), math.cos(h)


class SimPoseViz(Node):
    def __init__(self):
        super().__init__("sim_pose_viz")
        p = self.declare_parameter
        p("robot_id", 10)
        p("ball_id", 0)
        p("yaw_offset", 0.0)        # deg; body(IMU) = world + offset
        p("ball_diameter", 0.37)    # m
        g = lambda n: self.get_parameter(n).value
        rid, b = g("robot_id"), g("ball_id")
        self.offset, self.diam = g("yaw_offset"), g("ball_diameter")
        self.yaw = 0.0
        self.yaw_target = None

        self.create_subscription(PoseStamped, f"/b{b}/pose", self.on_pose, 10)
        self.create_subscription(Float32, f"/robot{rid}/yaw", self.on_yaw, 10)
        self.create_subscription(Float32, f"/robot{rid}/yaw_target", self.on_yaw_target, 10)
        self.pub_oriented = self.create_publisher(PoseStamped, f"/b{b}/pose_oriented", 10)
        self.pub_target = self.create_publisher(PoseStamped, f"/b{b}/pose_target", 10)
        self.pub_marker = self.create_publisher(Marker, f"/b{b}/marker", 10)
        self.get_logger().info(f"sim_pose_viz: b{b} / robot{rid}, yaw_offset {self.offset}")

    def on_yaw(self, msg):
        self.yaw = float(msg.data)

    def on_yaw_target(self, msg):
        self.yaw_target = float(msg.data)

    def _pose(self, msg, yaw_deg):
        out = PoseStamped()
        out.header = msg.header
        out.pose.position = msg.pose.position
        q = out.pose.orientation
        q.x, q.y, q.z, q.w = yaw_quat(yaw_deg - self.offset)
        return out

    def on_pose(self, msg):
        self.pub_oriented.publish(self._pose(msg, self.yaw))
        if self.yaw_target is not None:
            self.pub_target.publish(self._pose(msg, self.yaw_target))

        m = Marker()
        m.header = msg.header
        m.ns, m.id = "ball", 0
        m.type, m.action = Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y = msg.pose.position.x, msg.pose.position.y
        m.pose.position.z = self.diam / 2.0          # kugla lezi na podu
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = self.diam
        m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.1, 0.1, 0.8
        self.pub_marker.publish(m)


def main():
    rclpy.init()
    node = SimPoseViz()
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