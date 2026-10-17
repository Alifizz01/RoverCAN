"""Core tests: run with plain pytest, no ROS installation needed."""
import math

import can
import pytest

from ros2_amr_bench.core import can_codec as cc
from ros2_amr_bench.core.faults import ERROR, OK, WARN, FaultMonitor
from ros2_amr_bench.core.follower import follow
from ros2_amr_bench.core.gridmap import GridMap
from ros2_amr_bench.core.kinematics import Pose, integrate, twist_to_wheels, wheels_to_twist
from ros2_amr_bench.core.motor_ecu import MotorEcu

TRACK, R = 0.4, 0.08


# ---- kinematics
@pytest.mark.parametrize("v,w", [(0.0, 0.0), (0.5, 0.0), (0.0, 1.0), (0.3, -0.7)])
def test_twist_wheel_roundtrip(v, w):
    assert wheels_to_twist(*twist_to_wheels(v, w, TRACK, R), TRACK, R) == pytest.approx((v, w))


def test_pure_rotation_wheels_opposite():
    wl, wr = twist_to_wheels(0.0, 1.0, TRACK, R)
    assert wl == pytest.approx(-wr) and wr > 0


def test_straight_line_integration():
    p = integrate(Pose(0, 0, math.pi / 2), 1.0, 0.0, 2.0)
    assert (p.x, p.y) == pytest.approx((0.0, 2.0))


def test_full_circle_returns_to_start():
    p = Pose()
    for _ in range(1000):
        p = integrate(p, 0.5, 2 * math.pi / 10, 0.01)  # 10 s per circle
    assert (p.x, p.y, p.theta) == pytest.approx((0, 0, 0), abs=1e-6)


def test_quarter_arc_endpoint():
    # radius 1 m, quarter turn left from origin facing +x ends at (1, 1)
    p = integrate(Pose(), 1.0, 1.0, math.pi / 2)
    assert (p.x, p.y, p.theta) == pytest.approx((1.0, 1.0, math.pi / 2))


# ---- CAN codec
def test_cmd_roundtrip():
    data = cc.encode_cmd(1.234, -5.678, 42)
    assert len(data) == 8
    assert cc.decode_cmd(data) == pytest.approx((1.234, -5.678, 42))


def test_encoder_roundtrip_with_status():
    left, right, cnt, status = cc.decode_encoder(cc.encode_encoder(-0.5, 12.0, 255, 0x02))
    assert (left, right, cnt, status) == (-0.5, 12.0, 255, 2)


def test_known_byte_layout():
    # 1.0 rad/s = 1000 mrad/s = 0x03E8 little endian; -1.0 = 0xFC18
    data = cc.encode_cmd(1.0, -1.0, 7)
    assert data[:7] == bytes([0xE8, 0x03, 0x18, 0xFC, 7, 0, 0])
    assert data[7] == 0xE8 ^ 0x03 ^ 0x18 ^ 0xFC ^ 7


def test_saturation_and_counter_wrap():
    left, right, cnt = cc.decode_cmd(cc.encode_cmd(100.0, -100.0, 256 + 3))
    assert (left, right, cnt) == (32.767, -32.767, 3)


def test_bad_checksum_rejected():
    data = bytearray(cc.encode_cmd(1.0, 1.0, 0))
    data[0] ^= 0x01
    with pytest.raises(cc.FrameError, match="checksum"):
        cc.decode_cmd(data)


def test_bad_length_rejected():
    with pytest.raises(cc.FrameError, match="8 bytes"):
        cc.decode_encoder(b"\x00" * 7)


# ---- motor ECU over python-can virtual bus
@pytest.fixture
def buses():
    host = can.Bus(interface="virtual", channel="test_amr")
    ecu = MotorEcu(can.Bus(interface="virtual", channel="test_amr"))
    yield host, ecu
    host.shutdown()
    ecu.bus.shutdown()


def _send(host, left, right):
    host.send(can.Message(arbitration_id=cc.WHEEL_CMD_ID, is_extended_id=False,
                          data=cc.encode_cmd(left, right, 0)))


def _last_encoder(host):
    last = None
    while (m := host.recv(timeout=0)) is not None:
        if m.arbitration_id == cc.ENCODER_ID:
            last = cc.decode_encoder(m.data)
    return last


def test_ecu_tracks_command(buses):
    host, ecu = buses
    _send(host, 5.0, -3.0)
    for _ in range(50):
        ecu.step(0.02)
    left, right, _, status = _last_encoder(host)
    assert (left, right, status) == pytest.approx((5.0, -3.0, 0))


def test_ecu_stuck_and_dropout(buses):
    host, ecu = buses
    ecu.faults.add("stuck_left")
    _send(host, 5.0, 5.0)
    for _ in range(50):
        ecu.step(0.02)
    left, right, _, status = _last_encoder(host)
    assert left == 0.0 and right == pytest.approx(5.0) and status == 0x01
    ecu.faults.add("encoder_dropout")
    ecu.step(0.02)
    assert _last_encoder(host) is None


def test_ecu_ignores_corrupt_frame(buses):
    host, ecu = buses
    host.send(can.Message(arbitration_id=cc.WHEEL_CMD_ID, is_extended_id=False, data=b"\x01" * 7 + b"\x00"))
    ecu.step(0.02)
    assert ecu.bad_frames == 1 and ecu.target == [0.0, 0.0]


# ---- fault monitor
def _run(mon, t_end, cmd, meas, wheel_yaw=0.0, imu_yaw=0.0, enc=True, dt=0.02, t0=0.0):
    t, out = t0, None
    while t < t_end:
        if enc:
            mon.encoder_rx(t)
        out = mon.update(t, cmd, meas, wheel_yaw, imu_yaw)
        t += dt
    return out


def test_healthy_all_ok():
    out = _run(FaultMonitor(), 2.0, (5, 5), (5, 5), 0.3, 0.3)
    assert all(level == OK for level, _ in out.values())


def test_encoder_dropout_detected():
    mon = FaultMonitor()
    _run(mon, 1.0, (5, 5), (5, 5))
    assert mon.update(1.1, (5, 5), (5, 5), 0, 0)["encoder"][0] == OK
    assert mon.update(1.5, (5, 5), (5, 5), 0, 0)["encoder"][0] == ERROR


def test_stuck_motor_needs_persistence():
    mon = FaultMonitor()
    assert _run(mon, 0.3, (5, 5), (0.0, 5))["motor_left"][0] == OK
    out = _run(mon, 1.0, (5, 5), (0.0, 5), t0=0.32)
    assert out["motor_left"][0] == ERROR and out["motor_right"][0] == OK


def test_stuck_not_flagged_when_idle():
    assert _run(FaultMonitor(), 2.0, (0, 0), (0, 0))["motor_left"][0] == OK


def test_slip_detected_from_yaw_mismatch():
    out = _run(FaultMonitor(), 1.0, (5, 6), (5, 6), wheel_yaw=0.5, imu_yaw=0.1)
    assert out["slip"][0] == WARN


def test_no_false_alarms_during_dropout():
    mon = FaultMonitor()
    _run(mon, 1.0, (5, 5), (5, 5))
    out = _run(mon, 3.0, (5, 5), (0, 0), wheel_yaw=0.0, imu_yaw=0.5, enc=False, t0=1.0)
    assert out["encoder"][0] == ERROR
    assert out["motor_left"][0] == OK and out["slip"][0] == OK


# ---- map / follower
def test_scan_distance_to_wall():
    m = GridMap()
    ranges, amin, inc = m.scan(1.0, 1.0, 0.0, n=4)
    # beams at -pi, -pi/2, 0, +pi/2: walls at x=0.5, y=0.5, pillar x=5.5, y=4.5
    assert ranges == pytest.approx([0.5, 0.5, 4.5, 3.5], abs=0.06)


def test_follower_reaches_waypoints_closed_loop():
    """Follower + kinematics + ECU over virtual CAN, no ROS."""
    host = can.Bus(interface="virtual", channel="loop")
    ecu = MotorEcu(can.Bus(interface="virtual", channel="loop"))
    pose, wps, idx, dt, meas = Pose(), [(2.5, 0.0), (2.5, 1.2), (0.0, 1.2)], 0, 0.02, (0.0, 0.0)
    for _ in range(3000):
        v, w, reached = follow(pose, wps[idx])
        if reached:
            idx += 1
            if idx == len(wps):
                break
            continue
        _send(host, *twist_to_wheels(v, w, TRACK, R))
        ecu.step(dt)
        meas = _last_encoder(host)[:2] or meas
        pose = integrate(pose, *wheels_to_twist(*meas, TRACK, R), dt)
    host.shutdown()
    ecu.bus.shutdown()
    assert idx == len(wps)
