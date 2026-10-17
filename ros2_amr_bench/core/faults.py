"""Fault detection from wheel command, encoder feedback and IMU yaw rate."""
from dataclasses import dataclass

OK, WARN, ERROR = 0, 1, 2


@dataclass
class FaultConfig:
    encoder_timeout: float = 0.2   # s without encoder frame -> dropout
    stuck_cmd_min: float = 1.0     # rad/s commanded before the stuck check applies
    stuck_meas_max: float = 0.1    # rad/s measured below this counts as not turning
    stuck_time: float = 0.5        # s persistence
    slip_yaw_thresh: float = 0.15  # rad/s wheel-odometry vs IMU yaw rate disagreement
    slip_time: float = 0.3         # s persistence


class FaultMonitor:
    def __init__(self, cfg=None):
        self.cfg = cfg or FaultConfig()
        self.last_enc = None
        self.stuck_since = [None, None]
        self.slip_since = None

    def encoder_rx(self, t):
        self.last_enc = t

    def update(self, t, cmd, meas, wheel_yaw, imu_yaw):
        """cmd, meas: (left, right) rad/s. Returns {name: (level, message)}."""
        c = self.cfg
        out = {}
        if self.last_enc is None or t - self.last_enc > c.encoder_timeout:
            age = "never" if self.last_enc is None else f"{t - self.last_enc:.2f} s ago"
            out["encoder"] = (ERROR, f"encoder frames missing (last {age})")
        else:
            out["encoder"] = (OK, "encoder frames ok")
        enc_ok = out["encoder"][0] == OK

        for i, side in enumerate(("left", "right")):
            stuck_now = enc_ok and abs(cmd[i]) >= c.stuck_cmd_min and abs(meas[i]) <= c.stuck_meas_max
            if not stuck_now:
                self.stuck_since[i] = None
            elif self.stuck_since[i] is None:
                self.stuck_since[i] = t
            stuck = self.stuck_since[i] is not None and t - self.stuck_since[i] >= c.stuck_time
            out[f"motor_{side}"] = (
                (ERROR, f"{side} motor stuck: cmd {cmd[i]:.2f} rad/s, meas {meas[i]:.2f} rad/s")
                if stuck else (OK, f"{side} motor ok"))

        err = abs(wheel_yaw - imu_yaw)
        if enc_ok and err > c.slip_yaw_thresh:
            self.slip_since = t if self.slip_since is None else self.slip_since
        else:
            self.slip_since = None
        slip = self.slip_since is not None and t - self.slip_since >= c.slip_time
        out["slip"] = ((WARN, f"wheel slip suspected: yaw rate error {err:.2f} rad/s")
                       if slip else (OK, f"yaw rate error {err:.2f} rad/s"))
        return out
