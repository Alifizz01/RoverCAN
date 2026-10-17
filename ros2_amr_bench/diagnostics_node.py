"""Publishes diagnostic_msgs/DiagnosticArray on /diagnostics from FaultMonitor."""
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState

from .core.faults import FaultConfig, FaultMonitor
from .core.kinematics import wheels_to_twist


class Diagnostics(Node):
    def __init__(self):
        super().__init__("amr_diagnostics")
        names = list(FaultConfig.__dataclass_fields__)
        p = self.declare_parameters("", [("wheel_track", 0.4), ("wheel_radius", 0.08)]
                                    + [(n, getattr(FaultConfig, n)) for n in names])
        self.track, self.radius = p[0].value, p[1].value
        self.mon = FaultMonitor(FaultConfig(**{n: x.value for n, x in zip(names, p[2:])}))
        self.cmd = (0.0, 0.0)
        self.meas = (0.0, 0.0)
        self.imu_yaw = 0.0
        self.last_levels = {}
        self.pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)
        self.create_subscription(JointState, "wheel_cmd", self.on_cmd, 10)
        self.create_subscription(JointState, "wheel_states", self.on_meas, 10)
        self.create_subscription(Imu, "imu", self.on_imu, 10)
        self.create_timer(0.1, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, m):
        self.cmd = tuple(m.velocity[:2])

    def on_meas(self, m):
        self.meas = tuple(m.velocity[:2])
        self.mon.encoder_rx(self.now())

    def on_imu(self, m):
        self.imu_yaw = m.angular_velocity.z

    def tick(self):
        _, wheel_yaw = wheels_to_twist(*self.meas, self.track, self.radius)
        result = self.mon.update(self.now(), self.cmd, self.meas, wheel_yaw, self.imu_yaw)
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for name, (level, text) in result.items():
            if self.last_levels.get(name, 0) != level and level:
                self.get_logger().warn(f"{name}: {text}")
            self.last_levels[name] = level
            arr.status.append(DiagnosticStatus(
                level=bytes([level]), name=f"amr/{name}", message=text, hardware_id="sim_amr",
                values=[KeyValue(key="cmd_lr", value=f"{self.cmd[0]:.2f},{self.cmd[1]:.2f}"),
                        KeyValue(key="meas_lr", value=f"{self.meas[0]:.2f},{self.meas[1]:.2f}")]))
        self.pub.publish(arr)


def main():
    rclpy.init()
    rclpy.spin(Diagnostics())


if __name__ == "__main__":
    main()
