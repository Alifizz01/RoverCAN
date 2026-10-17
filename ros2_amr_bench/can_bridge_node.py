"""/cmd_vel -> 0x101 wheel command frames; 0x181 encoder frames -> /wheel_states.

Runs a simulated motor ECU on the same python-can virtual channel so the
bridge talks to something over a real python-can Bus API.
"""
import can
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .core.can_codec import ENCODER_ID, WHEEL_CMD_ID, FrameError, decode_encoder, encode_cmd
from .core.kinematics import twist_to_wheels
from .core.motor_ecu import MotorEcu


class CanBridge(Node):
    def __init__(self):
        super().__init__("can_bridge")
        p = self.declare_parameters("", [
            ("wheel_track", 0.4), ("wheel_radius", 0.08), ("rate", 50.0),
            ("can_channel", "amr"), ("fault", "none"), ("fault_start", 0.0),
        ])
        self.track, self.radius, rate, channel, self.fault, self.fault_start = (x.value for x in p)
        self.bus = can.Bus(interface="virtual", channel=channel, receive_own_messages=False)
        self.ecu = MotorEcu(can.Bus(interface="virtual", channel=channel))
        self.cmd = (0.0, 0.0)
        self.counter = 0
        self.dt = 1.0 / rate
        self.t0 = self.get_clock().now()
        self.pub_meas = self.create_publisher(JointState, "wheel_states", 10)
        self.pub_cmd = self.create_publisher(JointState, "wheel_cmd", 10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.create_timer(self.dt, self.tick)

    def on_cmd(self, msg):
        self.cmd = twist_to_wheels(msg.linear.x, msg.angular.z, self.track, self.radius)

    def _js(self, vel):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = ["left_wheel", "right_wheel"]
        m.velocity = [float(v) for v in vel]
        return m

    def tick(self):
        elapsed = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        if self.fault != "none" and elapsed >= self.fault_start and self.fault not in self.ecu.faults:
            self.get_logger().warn(f"injecting fault: {self.fault}")
            self.ecu.faults.add(self.fault)

        self.bus.send(can.Message(arbitration_id=WHEEL_CMD_ID, is_extended_id=False,
                                  data=encode_cmd(*self.cmd, self.counter)))
        self.counter = (self.counter + 1) & 0xFF
        self.pub_cmd.publish(self._js(self.cmd))
        self.ecu.step(self.dt)

        while (msg := self.bus.recv(timeout=0)) is not None:
            if msg.arbitration_id != ENCODER_ID:
                continue
            try:
                left, right, _, _ = decode_encoder(msg.data)
            except FrameError as e:
                self.get_logger().warn(f"bad encoder frame: {e}")
                continue
            self.pub_meas.publish(self._js((left, right)))


def main():
    rclpy.init()
    rclpy.spin(CanBridge())


if __name__ == "__main__":
    main()
