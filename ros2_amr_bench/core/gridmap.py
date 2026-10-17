"""Tiny occupancy grid and ray-cast laser scan."""
import math

DEFAULT_MAP = [
    "####################",
    "#..................#",
    "#..................#",
    "#.....####.........#",
    "#.....####.........#",
    "#..................#",
    "#..........#.......#",
    "#..........#.......#",
    "#..................#",
    "####################",
]


class GridMap:
    def __init__(self, rows=DEFAULT_MAP, resolution=0.5):
        self.res = resolution
        self.rows = list(reversed(rows))  # row 0 is y = 0
        self.h, self.w = len(self.rows), len(self.rows[0])

    def occupied(self, x, y):
        i, j = math.floor(y / self.res), math.floor(x / self.res)
        if not (0 <= i < self.h and 0 <= j < self.w):
            return True
        return self.rows[i][j] == "#"

    def scan(self, x, y, theta, n=180, max_range=8.0):
        """360 deg scan. Returns (ranges, angle_min, angle_increment) in the robot frame."""
        # ponytail: fixed-step ray march (res/10 accuracy), switch to DDA if scans get slow
        step = self.res / 10
        inc = 2 * math.pi / n
        ranges = []
        for k in range(n):
            a = theta - math.pi + k * inc
            dx, dy = math.cos(a) * step, math.sin(a) * step
            r, px, py = 0.0, x, y
            while r < max_range and not self.occupied(px, py):
                px += dx
                py += dy
                r += step
            ranges.append(min(r, max_range))
        return ranges, -math.pi, inc
