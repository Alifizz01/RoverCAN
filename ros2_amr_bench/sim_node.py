"""Kinematic simulator: integrates measured wheel speeds.

Publishes /odom + TF odom->base_link (wheel odometry), /imu (true yaw rate),
/scan (ray cast against the grid map from the true pose).
slip_factor < 1 scales the true yaw rate after slip_start to emulate wheel slip,
so wheel odometry and IMU disagree.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, LaserScan
from tf2_ros import TransformBroadcaster

from .core.gridmap import GridMap
from .core.kinematics import Pose, integrate, wheels_to_twist


def quat_z(theta):
    return 0.0, 0.0, math.sin(theta / 2), math.cos(theta / 2)


class Sim(Node):
    def __init__(self):
        super().__init__("amr_sim")
        p = self.declare_parameters("", [
            ("wheel_track", 0.4), ("wheel_radius", 0.08), ("rate", 50.0), ("scan_rate", 5.0),
            ("start_x", 1.0), ("start_y", 1.0), ("slip_factor", 1.0), ("slip_start", 0.0),
        ])
        (self.track, self.radius, rate, scan_rate, sx, sy,
         self.slip_factor, self.slip_start) = (x.value for x in p)
        self.map = GridMap()
        self.odom = Pose()
        self.true = Pose(sx, sy, 0.0)
        self.wheels = (0.0, 0.0)
        self.dt = 1.0 / rate
        self.t0 = self.get_clock().now()
        self.tf = TransformBroadcaster(self)
        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.pub_imu = self.create_publisher(Imu, "imu", 10)
        self.pub_scan = self.create_publisher(LaserScan, "scan", 10)
        self.create_subscription(JointState, "wheel_states", self.on_wheels, 10)
        self.create_timer(self.dt, self.tick)
        self.create_timer(1.0 / scan_rate, self.publish_scan)

    def on_wheels(self, msg):
        self.wheels = tuple(msg.velocity[:2])

    def tick(self):
        now = self.get_clock().now()
        v, w = wheels_to_twist(*self.wheels, self.track, self.radius)
        elapsed = (now - self.t0).nanoseconds * 1e-9
        w_true = w * (self.slip_factor if elapsed >= self.slip_start else 1.0)
        self.odom = integrate(self.odom, v, w, self.dt)
        nxt = integrate(self.true, v, w_true, self.dt)
        if not self.map.occupied(nxt.x, nxt.y):  # ponytail: no collision dynamics, robot just stops at walls
            self.true = nxt

        stamp = now.to_msg()
        q = quat_z(self.odom.theta)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, "odom", "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = self.odom.x, self.odom.y
        (o.pose.pose.orientation.x, o.pose.pose.orientation.y,
         o.pose.pose.orientation.z, o.pose.pose.orientation.w) = q
        o.twist.twist.linear.x, o.twist.twist.angular.z = v, w
        self.pub_odom.publish(o)

        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, "odom", "base_link"
        t.transform.translation.x, t.transform.translation.y = self.odom.x, self.odom.y
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        self.tf.sendTransform(t)

        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, "base_link"
        imu.orientation_covariance[0] = -1.0  # orientation not provided
        imu.angular_velocity.z = w_true
        self.pub_imu.publish(imu)

    def publish_scan(self):
        ranges, amin, inc = self.map.scan(self.true.x, self.true.y, self.true.theta)
        s = LaserScan()
        s.header.stamp, s.header.frame_id = self.get_clock().now().to_msg(), "base_link"
        s.angle_min, s.angle_increment = amin, inc
        s.angle_max = amin + inc * (len(ranges) - 1)
        s.range_min, s.range_max = 0.05, 8.0
        s.ranges = [float(r) for r in ranges]
        self.pub_scan.publish(s)


def main():
    rclpy.init()
    rclpy.spin(Sim())


if __name__ == "__main__":
    main()
