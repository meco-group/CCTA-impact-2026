"""Geometry / tuning constants for the remote (laptop-side) Alvik navigator.

Python port of ../obstacle_mpc_navigator/AlvikGeometry.h; single source of truth for
every constant shared by perception, mpc.py, plant.py and the dashboard.
"""

import math

# ── Front 5-zone ToF array ───────────────────────────────────────────────────
# alvik.get_distance(...) maps to indices 0..4, body frame (x forward, y left, CCW+).
# Index 0 (LEFT) is +45 deg, NOT -45 — flipping the sign mirrors every obstacle and
# makes the MPC dodge INTO them (a real bug once, confirmed via a logged hand sweep).
N_SENSORS = 5
SENSOR_ANGLES_DEG = (45.0, 22.0, 0.0, -22.0, -45.0)
SENSOR_ANGLES_RAD = tuple(math.radians(a) for a in SENSOR_ANGLES_DEG)

# A zone reading above this range is treated as "no obstacle" (open space).
MAX_RANGE_M = 1.20

# ── ToF measurement filter (per zone) ────────────────────────────────────────
# Median-of-N (spike rejection) followed by an EMA (residual jitter); see perception.TofFilter.
# WIN=5 needs a 3-of-5 majority to flip state, rejecting bursts up to 2 frames long at the
# cost of ~300 ms confirmation latency (<10 cm of travel at normal speeds) — worth it.
TOF_MEDIAN_WIN = 5
TOF_EMA_ALPHA = 0.5

# ── Obstacle map ─────────────────────────────────────────────────────────────
# Max obstacles tracked by the MPC. MUST equal N_OBS in mpc.py. Off-robot there's no
# RAM/solve-time limit, so the map can hold a small cluster instead of evicting constantly.
N_OBS = 2

# Safety margin the MPC keeps around every obstacle radius [m]. MUST equal MARGIN in mpc.py.
MARGIN_M = 0.10

# Hard-stop keep-out [m]. A receding-horizon plan only constrains FUTURE nodes, so if an
# obstacle is first detected already inside MARGIN_M the MPC can't retroactively avoid it.
# Within (r + STOP_KEEPOUT_M) the navigator declares "no solution" and stops outright.
STOP_KEEPOUT_M = 0.05

# Radius floor for a fitted obstacle [m], also used for a single-zone hit (no width info).
ASSUMED_RADIUS_M = 0.025

# Radius ceiling [m] — caps a wide surface (e.g. a wall) from producing a disk that swallows the goal.
R_MAX_M = 0.1

# Two adjacent zone hits belong to the same obstacle if their world-frame hit points are within this [m].
CLUSTER_GAP_M = 0.1

# ── Range-discontinuity split ────────────────────────────────────────────────
# XY proximity alone isn't enough (22-deg-apart zones can land close in XY at different
# ranges); also require |d_k - d_k-1| <= RANGE_JUMP_ABS_M + RANGE_JUMP_REL * min(d_k, d_k-1).
RANGE_JUMP_ABS_M = 0.03
RANGE_JUMP_REL = 0.25

# A fresh detection associates with an existing map obstacle if centers are within this [m] (ceiling on the radius-aware threshold below).
MATCH_DIST_M = 0.22

# Association is also gated on size (centre sits one radius behind the near face, so
# uncertainty scales with radius): effective threshold is
#   min(MATCH_DIST_M, 0.5 * (r_a + r_b) + MATCH_SLACK_M)
MATCH_SLACK_M = 0.06

# Exponential smoothing for updating a matched obstacle (0..1, higher = trust new more).
MAP_LOWPASS = 0.6

# An obstacle not re-seen for this many update cycles is dropped from the map.
OBST_TIMEOUT = 25

# ── Goal ─────────────────────────────────────────────────────────────────────
# "Arrived" tolerance [m] — kept at cm (not mm) so odometry noise doesn't cause endless nudging.
GOAL_TOLERANCE_M = 0.05

# ── Actuation limits ─────────────────────────────────────────────────────────
# Default MPC ceilings. v_max/omega_max are live parameters (re-solved every cycle), so
# these are only the STARTING values; V_MAX_CEILING/OMEGA_MAX_CEILING below are the real caps.
V_MAX_DEFAULT = 0.10       # m/s
OMEGA_MAX_DEFAULT = 1.5    # rad/s

# Dashboard slider ceilings (matches dashboard_html.py).
V_MAX_CEILING = 0.30       # m/s
OMEGA_MAX_CEILING = 3.0    # rad/s

# ── Control timing ───────────────────────────────────────────────────────────
CONTROL_DT = 0.1           # s -> 10 Hz; the MPC discretisation derives from this too

# ── Units ────────────────────────────────────────────────────────────────────
M_PER_CM = 0.01
CM_PER_M = 100.0
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
