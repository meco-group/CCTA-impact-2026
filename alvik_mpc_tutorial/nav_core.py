"""Transport-agnostic navigation core: perception -> MPC -> blocked-latch.

Given a pose and range readings, decides (v, omega). Knows nothing about sockets,
delay compensation, missions or dashboards -- those are the caller's job. Shared by
Controller (real robot / fake_robot.py over UDP) and run_mpc.py (in-process, no
network) so this logic exists exactly once.
"""

import math

from geometry import N_OBS, GOAL_TOLERANCE_M, V_MAX_DEFAULT, OMEGA_MAX_DEFAULT
from mpc import MPCSolver
from perception import ObstacleMap, BlockedLatch


class NavCore:
    def __init__(self, n_obs=N_OBS):
        self.map = ObstacleMap(n_obs)
        self.latch = BlockedLatch()
        self.solver = MPCSolver(n_obs=n_obs)
        self.arrived_latched = False
        self.plan_x, self.plan_y = [], []
        self._last_goal = None

    def reset(self):
        """Full reset -- e.g. after an odometry reset or a manual takeover."""
        self.map.clear()
        self.latch.reset()
        self.solver.reset_guess()
        self.arrived_latched = False

    def sense(self, pose, dist_m):
        """Update the obstacle map from a fresh measurement (measured pose, not predicted)."""
        self.map.update(pose, dist_m)

    def plan(self, pose, goal, v_max=V_MAX_DEFAULT, omega_max=OMEGA_MAX_DEFAULT, running=True):
        """Decide (v, omega) for this cycle.

        pose    : where the robot will BE when the command lands (delay-compensated by
                  the caller if needed; run_mpc.py just passes the current pose).
        goal    : (gx, gy)
        running : gates driving -- False forces a stop without touching the map/latch.
        """
        if goal != self._last_goal:
            self.arrived_latched = False
            self.solver.reset_guess()
            self._last_goal = goal

        dist_goal = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
        if dist_goal < GOAL_TOLERANCE_M:
            self.arrived_latched = True

        obstacles = self.map.active()
        should_drive = running and not self.arrived_latched
        status, solve_ms, used_soft = 0, 0.0, False

        if should_drive:
            r = self.solver.solve(pose, goal, [o.as_tuple() for o in obstacles], v_max, omega_max)
            v, omega = r["v"], r["omega"]
            status, solve_ms, used_soft = r["status"], r["solve_ms"], r["used_soft"]
            self.plan_x, self.plan_y = r["x_pred"], r["y_pred"]
        else:
            v, omega = 0.0, 0.0
            self.plan_x, self.plan_y = [], []

        blocked = self.latch.update(pose, obstacles, should_drive and status != 0)
        if blocked:
            v, omega = 0.0, 0.0

        return {
            "v": v, "omega": omega,
            "status": status, "solve_ms": solve_ms, "used_soft": used_soft,
            "blocked": blocked, "arrived": self.arrived_latched, "dist_goal": dist_goal,
            "obstacles": obstacles,
            "ocp_obs": [o.as_tuple() for o in obstacles[:self.solver.n_obs]],
        }
