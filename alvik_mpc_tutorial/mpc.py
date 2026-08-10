"""Assignment: build an obstacle-avoiding MPC for the Alvik with Impact.

The robot is a differential-drive unicycle. Fill in the TODOs in build_mpc() below to
get a working receding-horizon controller: kinematics, initial condition, control
bounds, obstacle avoidance and the objective. Everything else (parameter/solver
plumbing, the MPCSolver wrapper, warm-starting) is given.

Check your work:
    python mpc.py --benchmark      # solve-time statistics over many cycles
    python mpc.py --plot           # sanity plot of one solve (mpc_solution.png)
    python run_mpc.py              # live animation driving to a goal around obstacles
"""

import argparse
import os
import time

import numpy as np
import casadi as ca
import impact as im

# Required workaround for CMake >= 3.27 with the bundled blasfeo. Set before any import
# that might trigger cmake.
os.environ.setdefault("CMAKE_POLICY_VERSION_MINIMUM", "3.5")

from geometry import (
    N_OBS, MARGIN_M, V_MAX_DEFAULT, OMEGA_MAX_DEFAULT, CONTROL_DT,
)

# ── Horizon ──────────────────────────────────────────────────────────────────
# Discretisation, NOT the control period — a fresh command goes out every CONTROL_DT and
# only the plan's first control is used; T/N set the lookahead length/resolution.
T = 10.0                    # horizon length [s]
N = 100                      # horizon steps
MPC_DT = T / N

V_MIN, V_MAX = 0.0, V_MAX_DEFAULT   # V_MIN=0 forbids reverse: the robot turns in place, then drives forward
OMEGA_MAX = OMEGA_MAX_DEFAULT
MARGIN = MARGIN_M                   # standoff from the obstacle surface [m]

# Cost weights.
W_POS = 15.0        # stage position error
W_U = 0.05          # control effort
W_TERM = 300.0      # terminal position error (commit to reaching the goal)
W_HEAD = 3.0        # heading alignment (see below)
HEAD_GATE_R = 0.20  # [m] fade the heading term off within this radius of the goal

# Parked value for an unused obstacle slot: far away with zero radius.
PARKED_OBS = (1e3, 1e3, 0.0)


def build_mpc(n_obs=N_OBS, horizon=N, T_horizon=T):
    """Build the Impact MPC. Returns (mpc, symbols dict)."""
    mpc = im.MPC(T=T_horizon)

    # States: world pose
    px = mpc.state("px")
    py = mpc.state("py")
    theta = mpc.state("theta")

    # Controls: body linear + angular velocity
    v = mpc.control("v")
    omega = mpc.control("omega")

    # Parameters, set online every cycle
    x_current = mpc.parameter("x_current", 3)      # [px, py, theta] initial pose
    goal = mpc.parameter("goal", 2)                # [gx, gy] target point
    obs = mpc.parameter("obs", 3, n_obs)           # columns: [cx, cy, r]
    v_max = mpc.parameter("v_max")                 # linear speed limit  [m/s]
    omega_max = mpc.parameter("omega_max")         # angular speed limit [rad/s]

    gx, gy = goal[0], goal[1]

    # ── TODO 1: kinematics ───────────────────────────────────────────────────
    # Differential-drive unicycle: dpx/dt = v*cos(theta), dpy/dt = v*sin(theta),
    # dtheta/dt = omega. Set each with mpc.set_der(state, expression).
    raise NotImplementedError("TODO 1: set the kinematics with mpc.set_der(...)")

    # ── TODO 2: initial condition ────────────────────────────────────────────
    # Pin the state at t=0 to the measured pose: mpc.subject_to(mpc.at_t0(state) == x_current[i])
    # for each of px, py, theta.
    raise NotImplementedError("TODO 2: pin the state at t=0 to x_current")

    # ── TODO 3: control bounds ───────────────────────────────────────────────
    # v in [V_MIN, v_max], omega in [-omega_max, omega_max] (v_max/omega_max are the
    # PARAMETERS above, not the module constants — that's what lets the dashboard
    # tighten them live). Use mpc.subject_to(...) for each bound.
    raise NotImplementedError("TODO 3: bound v and omega with mpc.subject_to(...)")

    # ── TODO 4: obstacle avoidance ───────────────────────────────────────────
    # Each obstacle is a column of obs: (cx, cy, r) = obs[0,j], obs[1,j], obs[2,j] for
    # j in range(n_obs). The robot must stay outside a keep-out disk of radius r + MARGIN
    # around every obstacle. An unused slot is parked far away with r=0 (see PARKED_OBS),
    # so your formulation should leave it harmless rather than special-casing it.
    #
    # How you enforce this is your design decision: a hard constraint via
    # mpc.subject_to(...), a penalty term added to the objective in TODO 5, or a
    # combination of both.
    raise NotImplementedError("TODO 4: implement obstacle avoidance for all n_obs obstacles")

    # Heading term (given): with reverse forbidden the robot must turn to face the goal
    # first. head_align is 0 pointing at the goal, 2 pointing away, gated off near the
    # goal so its bearing-dependent gradient doesn't pull the equilibrium off-target.
    to_goal_x, to_goal_y = gx - px, gy - py
    goal_norm = ca.sqrt(to_goal_x ** 2 + to_goal_y ** 2 + 1e-3)
    head_align = 1.0 - (ca.cos(theta) * to_goal_x + ca.sin(theta) * to_goal_y) / goal_norm
    dist2goal = to_goal_x ** 2 + to_goal_y ** 2
    head_gate = dist2goal / (dist2goal + HEAD_GATE_R ** 2)

    # ── TODO 5: objective ────────────────────────────────────────────────────
    # Stage cost, summed over the horizon with mpc.sum(...):
    #   W_POS * squared distance to goal  +  W_U * squared control effort
    #   +  W_HEAD * head_gate * head_align
    #   + any penalty term from your TODO 4 obstacle-avoidance approach, if applicable
    # Terminal cost, added separately with mpc.at_tf(...):
    #   W_TERM * squared distance to goal at the final node
    # Add both with mpc.add_objective(...).
    raise NotImplementedError("TODO 5: assemble and add the stage + terminal objective")

    # Nominal values — used for the first solve and as the initial guess seed.
    obs_nom = np.tile(np.array([[PARKED_OBS[0]], [PARKED_OBS[1]], [PARKED_OBS[2]]]), (1, n_obs))
    obs_nom[:, 0] = [0.75, 0.10, 0.15]
    mpc.set_value(x_current, [0.0, 0.0, 0.0])
    mpc.set_value(goal, [1.5, 0.0])
    mpc.set_value(obs, obs_nom)
    mpc.set_value(v_max, V_MAX)
    mpc.set_value(omega_max, OMEGA_MAX)

    mpc.method(im.MultipleShooting(N=horizon, M=1, intg="rk"))

    mpc.solver(
        "fatrop",
        {
            "expand": True,
            "structure_detection": "auto",
            "verbose": False,
            "print_time": False,
            "error_on_fail": False,
            "fatrop": {"tol": 1e-3, "max_iter": 150, "print_level": 0},
        },
    )

    syms = dict(px=px, py=py, theta=theta, v=v, omega=omega,
                x_current=x_current, goal=goal, obs=obs,
                v_max=v_max, omega_max=omega_max)
    return mpc, syms


class MPCSolver:
    """Re-solvable wrapper around the Impact MPC.

    backend="function" compiles the OCP once via to_function() and warm-starts from the
    previous solution (cheap, receding-horizon). backend="solve" calls set_value()+solve()
    every cycle (simple, more per-call work). "auto" tries "function", falls back to "solve".
    """

    def __init__(self, n_obs=N_OBS, backend="auto"):
        self.n_obs = n_obs
        self.mpc, self.syms = build_mpc(n_obs)
        self.backend = None
        self._fun = None
        self._guess = None          # (X, U) from the previous solve, for warm starting
        self.last_status = -1
        self.last_solve_ms = 0.0
        self.last_iters = 0

        if backend in ("auto", "function"):
            try:
                self._fun = self._build_function(self.mpc, self.syms)
                self.backend = "function"
            except Exception as exc:                    # noqa: BLE001 - report and fall back
                if backend == "function":
                    raise
                print(f"[mpc] to_function() unavailable ({type(exc).__name__}: {exc}); "
                      f"falling back to per-cycle solve()")
        if self.backend is None:
            self._prime_solve()
            self.backend = "solve"

    # ── backend: to_function ────────────────────────────────────────────────
    @staticmethod
    def _build_function(stage, s):
        # grid="control" repeats the last column (no control at the terminal node), which
        # makes to_function's inputs non-independent; "control-" is the N genuine columns.
        def xs(sym):
            return stage.sample(sym, grid="control")[1]      # 1 x (N+1)

        def us(sym):
            return stage.sample(sym, grid="control-")[1]     # 1 x N

        return stage.to_function(
            "mpc_step",
            [stage.value(s["x_current"]), stage.value(s["goal"]), stage.value(s["obs"]),
             stage.value(s["v_max"]), stage.value(s["omega_max"]),
             us(s["v"]), us(s["omega"]),
             xs(s["px"]), xs(s["py"]), xs(s["theta"])],
            [xs(s["v"]), xs(s["omega"]), xs(s["px"]), xs(s["py"]), xs(s["theta"])],
            ["x_current", "goal", "obs", "v_max", "omega_max",
             "v_init", "omega_init", "px_init", "py_init", "theta_init"],
            ["v", "omega", "px", "py", "theta"],
        )

    # ── backend: repeated solve ─────────────────────────────────────────────
    def _prime_solve(self):
        self._sol = self.mpc.solve()

    @staticmethod
    def _pack_obs(obstacles, n_obs):
        """obstacles: iterable of (cx, cy, r). Unused slots are parked far away."""
        arr = np.tile(np.array([[PARKED_OBS[0]], [PARKED_OBS[1]], [PARKED_OBS[2]]],
                               dtype=float), (1, n_obs))
        for j, (cx, cy, r) in enumerate(obstacles):
            if j >= n_obs:
                break
            arr[:, j] = (cx, cy, r)
        return arr

    def solve(self, pose, goal, obstacles, v_max=V_MAX, omega_max=OMEGA_MAX):
        """Solve one cycle.

        pose      : (x, y, theta)
        goal      : (gx, gy)
        obstacles : iterable of (cx, cy, r) for ACTIVE obstacles only
        returns   : dict with v, omega, x_pred, y_pred, status, solve_ms
        """
        obs_arr = self._pack_obs(obstacles, self.n_obs)
        t0 = time.perf_counter()

        if self.backend == "function":
            if self._guess is None:
                self._reset_guess(pose, goal, v_max)

            gv, gw, gx_, gy_, gth_ = self._guess
            out = self._fun(x_current=np.asarray(pose, dtype=float).reshape(3, 1),
                            goal=np.asarray(goal, dtype=float).reshape(2, 1),
                            obs=obs_arr, v_max=v_max, omega_max=omega_max,
                            v_init=gv, omega_init=gw,
                            px_init=gx_, py_init=gy_, theta_init=gth_)
            st = self._fun.stats()
            ok = bool(st.get("success", False))
            iters = int(st.get("iter_count", 0))

            vs = np.asarray(out["v"]).ravel()
            ws = np.asarray(out["omega"]).ravel()
            xs = np.asarray(out["px"]).ravel()
            ys = np.asarray(out["py"]).ravel()
            ths = np.asarray(out["theta"]).ravel()

            self.last_status = 0 if ok else 1
            self.last_iters = iters

            if ok:
                # Shift the warm start by one step (consecutive problems differ by exactly
                # one dt), repeating the last entry to fill the horizon.
                self._guess = (
                    np.r_[vs[1:N], vs[N - 1]].reshape(1, N),
                    np.r_[ws[1:N], ws[N - 1]].reshape(1, N),
                    np.r_[xs[1:], xs[-1]].reshape(1, N + 1),
                    np.r_[ys[1:], ys[-1]].reshape(1, N + 1),
                    np.r_[ths[1:], ths[-1]].reshape(1, N + 1),
                )
            else:
                # Solve failed — never seed the next solve from a failed iterate, or one
                # bad cycle poisons every cycle after it.
                self._reset_guess(pose, goal, v_max)
        else:
            m, s = self.mpc, self.syms
            m.set_value(s["x_current"], list(pose))
            m.set_value(s["goal"], list(goal))
            m.set_value(s["obs"], obs_arr)
            m.set_value(s["v_max"], v_max)
            m.set_value(s["omega_max"], omega_max)
            sol = m.solve()
            self._sol = sol
            _, xs = sol.sample(s["px"], grid="control")
            _, ys = sol.sample(s["py"], grid="control")
            _, ths = sol.sample(s["theta"], grid="control")
            _, vs = sol.sample(s["v"], grid="control")
            _, ws = sol.sample(s["omega"], grid="control")
            self.last_status = 0

        self.last_solve_ms = (time.perf_counter() - t0) * 1000.0

        v = float(vs[0]) if len(vs) else 0.0
        w = float(ws[0]) if len(ws) else 0.0
        v = min(max(v, 0.0), v_max)      # clamp: an iterate near max_iter can be slightly out of bounds
        w = min(max(w, -omega_max), omega_max)
        if not (np.isfinite(v) and np.isfinite(w)):
            v, w = 0.0, 0.0
            self.last_status = 2

        return {
            "v": v, "omega": w,
            "x_pred": xs.tolist(), "y_pred": ys.tolist(), "theta_pred": ths.tolist(),
            "status": self.last_status, "solve_ms": self.last_solve_ms,
            "used_soft": False,   # kept for interface compatibility with nav_core.py/controller.py
        }

    def _reset_guess(self, pose, goal=None, v_max=V_MAX):
        """Cold initial guess: standing still at the current pose.

        A straight-line-to-goal guess was tried and measured WORSE near an obstacle's
        keep-out (0-iteration failures vs. 13-49 for the stationary guess) — a dynamically
        inconsistent trajectory is a worse starting point than simply not moving.
        `goal`/`v_max` are accepted for interface compatibility but unused.
        """
        self._guess = (np.zeros((1, N)), np.zeros((1, N)),
                       np.full((1, N + 1), pose[0]),
                       np.full((1, N + 1), pose[1]),
                       np.full((1, N + 1), pose[2]))

    def reset_guess(self):
        """Drop the warm start (e.g. after an odometry reset or a big goal jump)."""
        self._guess = None


# ── standalone checks ────────────────────────────────────────────────────────
def _benchmark(n_cycles=200, backend="auto"):
    print(f"Building MPC (N_OBS={N_OBS}, N={N}, T={T}s, dt={T/N}s) …")
    t0 = time.perf_counter()
    solver = MPCSolver(backend=backend)
    print(f"  built in {time.perf_counter()-t0:.2f} s, backend = {solver.backend}")

    # A representative receding-horizon run: drive toward a goal past two obstacles.
    obstacles = [(0.85, 0.06, 0.12), (1.40, -0.28, 0.10)]
    pose = [0.0, 0.0, 0.0]
    times, statuses = [], []
    for _ in range(n_cycles):
        r = solver.solve(pose, (2.0, 0.0), obstacles)
        times.append(r["solve_ms"])
        statuses.append(r["status"])
        # Advance the pose with the commanded control (midpoint integration).
        th_mid = pose[2] + 0.5 * r["omega"] * CONTROL_DT
        pose = [pose[0] + r["v"] * np.cos(th_mid) * CONTROL_DT,
                pose[1] + r["v"] * np.sin(th_mid) * CONTROL_DT,
                pose[2] + r["omega"] * CONTROL_DT]

    t = np.array(times)
    budget = CONTROL_DT * 1000.0
    print(f"\nSolve time over {n_cycles} cycles [ms]")
    print(f"  mean {t.mean():7.2f}   median {np.median(t):7.2f}")
    print(f"  min  {t.min():7.2f}   p95    {np.percentile(t,95):7.2f}   max {t.max():7.2f}")
    print(f"  budget at {1/CONTROL_DT:.0f} Hz is {budget:.0f} ms -> "
          f"p95 uses {np.percentile(t,95)/budget*100:.1f}% of it")
    print(f"  non-zero status: {sum(1 for s in statuses if s != 0)}/{n_cycles}")
    print(f"  final pose: x={pose[0]:.3f} y={pose[1]:.3f} theta={pose[2]:.3f}")
    return t


def _plot():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    solver = MPCSolver(backend="solve")
    obstacles = [(0.75, 0.10, 0.15)]
    r = solver.solve([0.0, 0.0, 0.0], (1.5, 0.0), obstacles)

    fig, (axp, axc) = plt.subplots(1, 2, figsize=(11, 4.5))
    axp.plot(r["x_pred"], r["y_pred"], "-o", ms=3, label="path")
    axp.plot(0, 0, "bs", label="start")
    axp.plot(1.5, 0.0, "g*", ms=14, label="goal")
    for cx, cy, rr in obstacles:
        axp.add_patch(plt.Circle((cx, cy), rr, color="r", alpha=0.4))
        axp.add_patch(plt.Circle((cx, cy), rr + MARGIN, color="r", alpha=0.12))
    axp.set_aspect("equal"); axp.grid(True); axp.legend()
    axp.set_xlabel("x [m]"); axp.set_ylabel("y [m]"); axp.set_title("MPC path")

    ts = np.arange(len(r["x_pred"])) * CONTROL_DT
    axc.step(ts[:-1], np.array(r["x_pred"][:-1]) * 0 + 0, alpha=0)   # keep axes sane
    axc.plot(ts[:len(r["x_pred"])], r["theta_pred"], label="theta [rad]")
    axc.grid(True); axc.legend(); axc.set_xlabel("t [s]"); axc.set_title("heading")

    fig.tight_layout()
    fig.savefig("mpc_solution.png", dpi=110)
    print(f"Saved mpc_solution.png (solve {r['solve_ms']:.1f} ms, status {r['status']})")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--benchmark", action="store_true", help="measure solve time")
    ap.add_argument("--plot", action="store_true", help="write mpc_solution.png")
    ap.add_argument("--cycles", type=int, default=200)
    ap.add_argument("--backend", choices=("auto", "function", "solve"), default="auto")
    args = ap.parse_args()

    if args.benchmark:
        _benchmark(args.cycles, args.backend)
    if args.plot:
        _plot()
    if not (args.benchmark or args.plot):
        _benchmark(50, args.backend)
