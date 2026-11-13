#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class MovingPoint(Node):
    def __init__(self):
        super().__init__("moving_point")
        p = self.declare_parameter
        p("mode", "circle"); p("cx", 0.0); p("cy", 0.0); p("radius", 1.5); p("speed", 0.2)
        p("topic", "/clicked_point"); p("frame", "world"); p("rate_hz", 10.0)
        g = lambda n: self.get_parameter(n).value
        self.mode, self.cx, self.cy = g("mode"), g("cx"), g("cy")
        self.r, self.v, self.frame = g("radius"), g("speed"), g("frame")
        self.pub = self.create_publisher(PointStamped, g("topic"), 10)
        self.t0 = self.now()
        self.create_timer(1.0 / g("rate_hz"), self.tick)
        self.get_logger().info(f"moving_point: {self.mode}, r={self.r} m, v={self.v} m/s -> {g('topic')}")

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def point(self, t):
        if self.mode == "circle":
            a = self.v * t / self.r
            return self.cx + self.r * math.cos(a), self.cy + self.r * math.sin(a)
        if self.mode == "line":
            s = (self.v * t) % (4 * self.r)                    # tamo-amo, ukupni put 4r
            u = s if s < 2 * self.r else 4 * self.r - s
            return self.cx - self.r + u, self.cy
        return self.cx, self.cy

    def tick(self):
        x, y = self.point(self.now() - self.t0)
        m = PointStamped()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.point.x, m.point.y = float(x), float(y)
        self.pub.publish(m)


def main():
    rclpy.init()
    node = MovingPoint()
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