"""Waypoint follower: proportional heading controller that slows while misaligned."""
import math

from .kinematics import wrap


def follow(pose, wp, v_max=0.4, w_max=1.5, k_ang=2.0, tol=0.15):
    """Returns (v, w, reached)."""
    dx, dy = wp[0] - pose.x, wp[1] - pose.y
    d = math.hypot(dx, dy)
    if d < tol:
        return 0.0, 0.0, True
    err = wrap(math.atan2(dy, dx) - pose.theta)
    w = max(-w_max, min(w_max, k_ang * err))
    v = v_max * max(0.0, math.cos(err)) * min(1.0, d)
    return v, w, False
