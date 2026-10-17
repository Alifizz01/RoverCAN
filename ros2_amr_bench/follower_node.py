"""Waypoint follower: /odom -> /cmd_vel over a list of waypoints in the odom frame."""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .core.follower import follow
from .core.kinematics import Pose


class Follower(Node):
    def __init__(self):
        super().__init__("waypoint_follower")
        p = self.declare_parameters("", [
            ("waypoints", [2.5, 0.0, 2.5, 1.2, 0.0, 1.2, 0.0, 0.0]),
            ("loop", True), ("v_max", 0.4), ("w_max", 1.5), ("tolerance", 0.15),
        ])
        flat, self.loop, self.v_max, self.w_max, self.tol = (x.value for x in p)
        self.wps = list(zip(flat[0::2], flat[1::2]))
        self.idx = 0
        self.pose = None
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.create_timer(0.05, self.tick)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.pose = Pose(msg.pose.pose.position.x, msg.pose.pose.position.y,
                         2 * math.atan2(q.z, q.w))

    def tick(self):
        cmd = Twist()
        if self.pose is not None and self.idx < len(self.wps):
            v, w, reached = follow(self.pose, self.wps[self.idx], self.v_max, self.w_max, tol=self.tol)
            if reached:
                self.get_logger().info(f"reached waypoint {self.idx}: {self.wps[self.idx]}")
                self.idx += 1
                if self.loop and self.idx == len(self.wps):
                    self.idx = 0
            cmd.linear.x, cmd.angular.z = v, w
        self.pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(Follower())


if __name__ == "__main__":
    main()
