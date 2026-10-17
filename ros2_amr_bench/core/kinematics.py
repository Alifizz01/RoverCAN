"""Differential-drive kinematics and pose integration."""
import math
from dataclasses import dataclass


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def twist_to_wheels(v, w, track, radius):
    """Body twist (m/s, rad/s) -> wheel speeds (rad/s) left, right."""
    return (v - w * track / 2) / radius, (v + w * track / 2) / radius


def wheels_to_twist(wl, wr, track, radius):
    """Wheel speeds (rad/s) -> body twist (m/s, rad/s)."""
    return radius * (wl + wr) / 2, radius * (wr - wl) / track


def integrate(pose, v, w, dt):
    """Exact arc integration (straight line for w ~ 0)."""
    if abs(w) < 1e-9:
        return Pose(pose.x + v * dt * math.cos(pose.theta),
                    pose.y + v * dt * math.sin(pose.theta), pose.theta)
    th2 = pose.theta + w * dt
    r = v / w
    return Pose(pose.x + r * (math.sin(th2) - math.sin(pose.theta)),
                pose.y - r * (math.cos(th2) - math.cos(pose.theta)), wrap(th2))
