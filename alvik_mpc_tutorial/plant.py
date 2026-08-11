"""Ground-truth Alvik model: differential-drive motion + synthetic 5-zone ToF.

Port of ../corridor_mpc_navigator/sim/AlvikPlant.h. Used by fake_robot.py to stand in for
the real robot, and by the perception self-test to generate known scenes.

Deliberately mismatched with the MPC's internal model (own integration rule, ray-cast ToF
vs. fitted disks) — if they agreed exactly the simulator would be flattering the controller.
"""

import math
import random

from geometry import N_SENSORS, SENSOR_ANGLES_RAD

# A zone that hits nothing within this range reports it. Deliberately above MAX_RANGE_M
# (1.20) so the navigator's validity test reads it as "clear".
SENSE_CAP = 2.0


def ray_circle(ox, oy, dx, dy, cx, cy, r):
    """Distance along the unit ray (ox,oy)+(t)(dx,dy) to the circle, or -1 for a miss."""
    fx, fy = ox - cx, oy - cy
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - r * r
    disc = b * b - 4.0 * c              # a = 1 because the direction is a unit vector
    if disc < 0.0:
        return -1.0
    sq = math.sqrt(disc)
    t1 = (-b - sq) * 0.5
    t2 = (-b + sq) * 0.5
    if t1 >= 0.0:
        return t1
    if t2 >= 0.0:
        return t2
    return -1.0


class AlvikPlant:
    def __init__(self, start=(0.0, 0.0, 0.0), obstacles=(), sensor_noise_m=0.004, seed=1):
        self.pose = list(start)
        self.obstacles = [list(o) for o in obstacles]    # each [cx, cy, r]
        self.noise = sensor_noise_m
        self.rng = random.Random(seed)

    # ── ground-truth scene editing (the real robot has no equivalent) ────────
    def add_obstacle(self, cx, cy, r):
        self.obstacles.append([cx, cy, r])

    def clear_obstacles(self):
        self.obstacles.clear()

    def remove_near(self, px, py, max_d):
        best, bd = -1, max_d * max_d
        for i, (cx, cy, _) in enumerate(self.obstacles):
            d2 = (cx - px) ** 2 + (cy - py) ** 2
            if d2 < bd:
                bd, best = d2, i
        if best >= 0:
            self.obstacles.pop(best)

    def reset(self, pose=(0.0, 0.0, 0.0)):
        self.pose = list(pose)

    # ── dynamics ────────────────────────────────────────────────────────────
    def step(self, v, omega, dt):
        """Advance the true pose under a commanded (v, omega), midpoint integration."""
        th_mid = self.pose[2] + 0.5 * omega * dt
        self.pose[0] += v * math.cos(th_mid) * dt
        self.pose[1] += v * math.sin(th_mid) * dt
        self.pose[2] += omega * dt

    # ── sensing ─────────────────────────────────────────────────────────────
    def sense(self):
        """Return the 5 zone ranges [m] this pose would produce."""
        out = []
        for i in range(N_SENSORS):
            ang = self.pose[2] + SENSOR_ANGLES_RAD[i]
            dx, dy = math.cos(ang), math.sin(ang)
            best = SENSE_CAP
            for cx, cy, r in self.obstacles:
                t = ray_circle(self.pose[0], self.pose[1], dx, dy, cx, cy, r)
                if 0.0 <= t < best:
                    best = t
            if best < SENSE_CAP and self.noise > 0.0:
                best += self.rng.gauss(0.0, self.noise)
            out.append(max(best, 0.0))
        return out
