"""Simulated motor controller ECU on a python-can bus, with fault injection.

Faults (strings in MotorEcu.faults):
    encoder_dropout      stop sending encoder frames
    stuck_left/right     that wheel stays at 0 rad/s despite the command
"""
import can

from .can_codec import ENCODER_ID, WHEEL_CMD_ID, FrameError, decode_cmd, encode_encoder


class MotorEcu:
    def __init__(self, bus, accel=20.0):
        self.bus = bus
        self.accel = accel  # rad/s^2 slew limit
        self.target = [0.0, 0.0]
        self.speed = [0.0, 0.0]
        self.counter = 0
        self.faults = set()
        self.bad_frames = 0

    def poll(self):
        while (msg := self.bus.recv(timeout=0)) is not None:
            if msg.arbitration_id != WHEEL_CMD_ID:
                continue
            try:
                left, right, _ = decode_cmd(msg.data)
            except FrameError:
                self.bad_frames += 1
                continue
            self.target = [left, right]

    def step(self, dt):
        self.poll()
        for i, side in enumerate(("left", "right")):
            tgt = 0.0 if f"stuck_{side}" in self.faults else self.target[i]
            self.speed[i] += max(-self.accel * dt, min(self.accel * dt, tgt - self.speed[i]))
        if "encoder_dropout" in self.faults:
            return
        status = (1 if "stuck_left" in self.faults else 0) | (2 if "stuck_right" in self.faults else 0)
        self.bus.send(can.Message(arbitration_id=ENCODER_ID, is_extended_id=False,
                                  data=encode_encoder(*self.speed, self.counter, status)))
        self.counter = (self.counter + 1) & 0xFF
