"""Obstacle perception: 5 ToF zones -> fitted circular obstacles in the world frame.

Port of the perception half of ../obstacle_mpc_navigator/ObstacleNavigator.cpp; knows
nothing about sockets, the MPC, or the dashboard.

Pipeline per update: filter (median + EMA per zone) -> project to world-frame hit points
-> cluster contiguous zones (XY-gap + range-jump tests) -> fit each cluster to a disk ->
associate to the map (match/take/evict a slot) -> age out anything unseen too long.
Operates in the odometry frame the pose is reported in.
"""

import math

from geometry import (
    N_SENSORS, SENSOR_ANGLES_RAD, MAX_RANGE_M, TOF_MEDIAN_WIN, TOF_EMA_ALPHA,
    N_OBS, MARGIN_M, STOP_KEEPOUT_M, ASSUMED_RADIUS_M, R_MAX_M, CLUSTER_GAP_M,
    RANGE_JUMP_ABS_M, RANGE_JUMP_REL, MATCH_DIST_M, MATCH_SLACK_M, MAP_LOWPASS,
    OBST_TIMEOUT,
)


class Obstacle:
    """A fitted circle in the world frame."""

    __slots__ = ("cx", "cy", "r", "active", "unseen")

    def __init__(self, cx=0.0, cy=0.0, r=0.0, active=False, unseen=0):
        self.cx, self.cy, self.r = cx, cy, r
        self.active = active
        self.unseen = unseen

    def clearance(self, px, py):
        """Distance from (px, py) to this obstacle's NEAR FACE (not its centre)."""
        return math.hypot(px - self.cx, py - self.cy) - self.r

    def as_tuple(self):
        return (self.cx, self.cy, self.r)

    def __repr__(self):
        return (f"Obstacle(cx={self.cx:.3f}, cy={self.cy:.3f}, r={self.r:.3f}, "
                f"active={self.active}, unseen={self.unseen})")


class TofFilter:
    """Per-zone median-of-N followed by an EMA.

    Preferred over a Kalman filter: range steps at object edges (a KF would smear that),
    and the median rejects outliers a least-squares KF would be pulled by.

    A no-return sample feeds the median window as a large sentinel rather than wiping the
    window, so the median stays a true majority vote and a lone outlier (real or
    spurious) needs to repeat before it can flip the state. The EMA reseeds (not blends)
    only on a confirmed flip, so a real transition snaps instead of fading through a fake
    intermediate range. Trade-off: a new obstacle takes ~2 cycles (~200ms) to confirm
    instead of 1 — a good trade against flickering on single-frame noise.
    """

    # Sorts above every real reading (< MAX_RANGE_M), so "no return" only wins the median
    # vote when a majority of the window actually agrees nothing is there.
    _NO_RETURN_SENTINEL = MAX_RANGE_M + 1.0

    def __init__(self):
        self._hist = [[self._NO_RETURN_SENTINEL] * TOF_MEDIAN_WIN for _ in range(N_SENSORS)]
        self._ema = [None] * N_SENSORS
        self._was_valid = [False] * N_SENSORS   # last MEDIAN's state, not the raw sample's

    def reset(self):
        self._hist = [[self._NO_RETURN_SENTINEL] * TOF_MEDIAN_WIN for _ in range(N_SENSORS)]
        self._ema = [None] * N_SENSORS
        self._was_valid = [False] * N_SENSORS

    def __call__(self, dist_m):
        out = [MAX_RANGE_M] * N_SENSORS
        for i in range(N_SENSORS):
            d = dist_m[i]
            raw = d if (d > 0.0 and d < MAX_RANGE_M) else self._NO_RETURN_SENTINEL

            h = self._hist[i]
            h.append(raw)
            if len(h) > TOF_MEDIAN_WIN:
                h.pop(0)
            med = sorted(h)[len(h) // 2]
            valid = med < MAX_RANGE_M

            if valid != self._was_valid[i]:
                # Median flipped state — a confirmed transition, so reseed rather than blend.
                self._ema[i] = med if valid else None
                self._was_valid[i] = valid
            elif valid:
                self._ema[i] = TOF_EMA_ALPHA * med + (1.0 - TOF_EMA_ALPHA) * self._ema[i]

            out[i] = self._ema[i] if valid else MAX_RANGE_M
        return out


def fit_cluster(hx, hy, clipped_sides, pose):
    """Fit one obstacle disk to a cluster of world-frame hit points.

    Not a circle fit — with only 2-5 points over ~22 deg spacing a least-squares fit
    wanders wildly. Instead: near-face distance = closest measured range (conservative);
    bearing = mean of the cluster's unit view directions (immune to the nearest ray
    happening to sit at one edge of the cluster, which would otherwise swing the disk off
    to that side); radius = widest chord, corrected for FoV clipping; centre placed one
    radius behind the near-face point along that same mean bearing, so near-face point,
    centre and bearing are all self-consistent.
    """
    n = len(hx)
    if n == 0:
        return None

    # Mean of unit view directions (not mean position, which a farther hit would skew).
    ux_sum = uy_sum = 0.0
    min_range, min_i = float("inf"), 0
    for i in range(n):
        dx, dy = hx[i] - pose[0], hy[i] - pose[1]
        d = math.hypot(dx, dy)
        if d > 1e-9:
            ux_sum += dx / d
            uy_sum += dy / d
        if d < min_range:
            min_range, min_i = d, i

    mean_norm = math.hypot(ux_sum, uy_sum)
    if mean_norm < 1e-6:
        # Degenerate guard; can't actually happen within a 90 deg fan, but cheap to guard.
        ux, uy = math.cos(pose[2]), math.sin(pose[2])
    else:
        ux, uy = ux_sum / mean_norm, uy_sum / mean_norm

    # Widest chord across the cluster's hit points.
    span = 0.0
    for i in range(n):
        for j in range(i + 1, n):
            s = math.hypot(hx[i] - hx[j], hy[i] - hy[j])
            if s > span:
                span = s

    # A cluster touching zone 0 or 4 was cut off by the fan edge, not the object ending — so the visible span understates true width.
    if clipped_sides >= 2:
        r = R_MAX_M                 # object fills the whole fan: assume the ceiling
    elif clipped_sides == 1:
        r = span                    # visible span is a HALF chord
    else:
        r = 0.5 * span              # full chord visible
    r = max(r, ASSUMED_RADIUS_M)    # floor: a single-zone hit carries no width at all
    r = min(r, R_MAX_M)             # ceiling: a wall must not swallow the goal

    # Near-face point placed on the mean bearing, not on whichever ray reported it.
    ax, ay = pose[0] + min_range * ux, pose[1] + min_range * uy

    return Obstacle(cx=ax + r * ux, cy=ay + r * uy, r=r, active=True, unseen=0)


class ObstacleMap:
    """The tracked obstacle map: association, smoothing, eviction and age-out."""

    def __init__(self, n_obs=N_OBS):
        self.n_obs = n_obs
        self.slots = [Obstacle() for _ in range(n_obs)]
        self.filter = TofFilter()
        self.filtered = [MAX_RANGE_M] * N_SENSORS
        self.raw = [MAX_RANGE_M] * N_SENSORS

    def clear(self):
        """Wipe the map — call on odometry reset, since it invalidates every stored obstacle."""
        for s in self.slots:
            s.active = False
            s.unseen = 0
        self.filter.reset()

    def active(self):
        return [s for s in self.slots if s.active]

    def update(self, pose, dist_m):
        """One perception cycle. pose = (x, y, theta); dist_m = 5 raw ranges [m]."""
        self.raw = list(dist_m)
        d = self.filter(dist_m)
        self.filtered = d

        # Everything ages by one cycle; a match below resets its counter to 0.
        for s in self.slots:
            if s.active:
                s.unseen += 1

        ct, st = math.cos(pose[2]), math.sin(pose[2])
        valid = [0.0 < d[i] < MAX_RANGE_M for i in range(N_SENSORS)]

        # Project each valid zone to a world-frame hit point.
        wx = [0.0] * N_SENSORS
        wy = [0.0] * N_SENSORS
        for i in range(N_SENSORS):
            if not valid[i]:
                continue
            a = SENSOR_ANGLES_RAD[i]
            bx, by = d[i] * math.cos(a), d[i] * math.sin(a)   # body frame
            wx[i] = pose[0] + bx * ct - by * st               # world frame
            wy[i] = pose[1] + bx * st + by * ct

        # Cluster contiguous valid zones.
        i = 0
        while i < N_SENSORS:
            if not valid[i]:
                i += 1
                continue
            k = i + 1
            while k < N_SENSORS and valid[k]:
                # (a) XY proximity of successive hit points.
                if math.hypot(wx[k] - wx[k - 1], wy[k] - wy[k - 1]) > CLUSTER_GAP_M:
                    break
                # (b) Range compatibility — XY proximity alone can merge two surfaces at very different depths.
                lo = min(d[k], d[k - 1])
                if abs(d[k] - d[k - 1]) > RANGE_JUMP_ABS_M + RANGE_JUMP_REL * lo:
                    break
                k += 1

            clipped = (1 if i == 0 else 0) + (1 if k == N_SENSORS else 0)
            det = fit_cluster(wx[i:k], wy[i:k], clipped, pose)
            if det is not None:
                self._insert(det, pose)
            i = k

        # Age out anything not re-seen for a while.
        for s in self.slots:
            if s.active and s.unseen > OBST_TIMEOUT:
                s.active = False
                s.unseen = 0

    def _insert(self, det, pose):
        """Associate a fresh detection with the map: match, take a free slot, or evict."""
        # 1. Match the nearest active obstacle, with a radius-aware threshold (larger
        #    obstacles tolerate more centre wander; small ones must not be swallowed).
        best, best_d = -1, float("inf")
        for j, s in enumerate(self.slots):
            if not s.active:
                continue
            dist = math.hypot(det.cx - s.cx, det.cy - s.cy)
            lim = min(MATCH_DIST_M, 0.5 * (det.r + s.r) + MATCH_SLACK_M)
            if dist < lim and dist < best_d:
                best_d, best = dist, j

        if best >= 0:
            s = self.slots[best]
            a = MAP_LOWPASS
            s.cx = a * det.cx + (1.0 - a) * s.cx
            s.cy = a * det.cy + (1.0 - a) * s.cy
            s.r = a * det.r + (1.0 - a) * s.r
            s.unseen = 0
            return

        # 2. Free slot.
        for s in self.slots:
            if not s.active:
                s.cx, s.cy, s.r = det.cx, det.cy, det.r
                s.active, s.unseen = True, 0
                return

        # 3. Map full: evict the weakest (stalest first, then least threatening); the new
        #    detection only wins if it's itself more threatening than the incumbent.
        weak, weak_key = -1, None
        for j, s in enumerate(self.slots):
            key = (-s.unseen, -s.clearance(pose[0], pose[1]))
            if weak_key is None or key < weak_key:
                weak_key, weak = key, j
        if weak < 0:
            return
        incumbent = self.slots[weak]
        if det.clearance(pose[0], pose[1]) < incumbent.clearance(pose[0], pose[1]):
            incumbent.cx, incumbent.cy, incumbent.r = det.cx, det.cy, det.r
            incumbent.active, incumbent.unseen = True, 0


class BlockedLatch:
    """Latching 'no avoidance solution -> stop' guard.

    The MPC's hard constraints only bind on FUTURE nodes, so if an obstacle is first seen
    already inside its keep-out, the solver would happily plan straight through it — the
    navigator has to catch that itself. Latches (rather than toggling every cycle, which
    would let the robot creep through since reverse is forbidden); clearing needs the
    wider MARGIN_M vs. STOP_KEEPOUT_M for hysteresis.
    """

    def __init__(self):
        self.blocked = False

    def reset(self):
        self.blocked = False

    def update(self, pose, obstacles, solver_failed):
        raw_block = bool(solver_failed)
        if not raw_block:
            for o in obstacles:
                keep = o.r + STOP_KEEPOUT_M
                if (pose[0] - o.cx) ** 2 + (pose[1] - o.cy) ** 2 < keep * keep:
                    raw_block = True
                    break

        if raw_block:
            self.blocked = True
        elif self.blocked:
            clear_all = True
            for o in obstacles:
                keep = o.r + MARGIN_M          # wider than STOP_KEEPOUT_M -> hysteresis
                if (pose[0] - o.cx) ** 2 + (pose[1] - o.cy) ** 2 < keep * keep:
                    clear_all = False
                    break
            self.blocked = not clear_all
        return self.blocked
