"""Run your MPC: in-process dev loop, no UDP link, no dashboard, no fake_robot.py process.

Runs AlvikPlant + NavCore directly in one process with a live animation, so mpc.py can
be iterated on without the network stack. For full-stack/network testing use
fake_robot.py + controller.py (+ dashboard.py) instead.

Run:
    python run_mpc.py
    python run_mpc.py --obstacles "0.7,0.3,0.14 1.6,0.05,0.12" --goal 1.5,0
    python run_mpc.py --no-gui --max-steps 300   # headless, prints status instead
"""

import argparse

from geometry import CONTROL_DT, MARGIN_M, V_MAX_DEFAULT, OMEGA_MAX_DEFAULT
from nav_core import NavCore
from obstacles import DEFAULT_OBSTACLES, parse_obstacles
from plant import AlvikPlant

START = (0.0, 0.0, 0.0)


def parse_xy(s):
    x, y = s.split(",")
    return float(x), float(y)


def run_headless(plant, core, goal, v_max, omega_max, max_steps):
    """Print one status line per cycle -- for quick checks without a GUI."""
    for step in range(max_steps):
        dist = plant.sense()
        core.sense(plant.pose, dist)
        r = core.plan(plant.pose, goal, v_max, omega_max, running=True)
        plant.step(r["v"], r["omega"], CONTROL_DT)
        if step % 10 == 0 or r["arrived"]:
            print(f"t={step*CONTROL_DT:5.1f}s  pose=({plant.pose[0]:6.3f},{plant.pose[1]:6.3f}) "
                  f"v={r['v']:.3f} w={r['omega']:+.3f}  status={r['status']} "
                  f"dist_goal={r['dist_goal']:.3f}")
        if r["arrived"]:
            print(f"arrived at t={step*CONTROL_DT:.1f}s")
            return
    print(f"did not arrive within {max_steps} steps")


def run_animated(plant, core, goal, obstacles, v_max, omega_max):
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect("equal")
    ax.grid(True)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")

    for cx, cy, r in obstacles:
        ax.add_patch(plt.Circle((cx, cy), r, color="r", alpha=0.4))
        ax.add_patch(plt.Circle((cx, cy), r + MARGIN_M, color="r", alpha=0.12))
    ax.plot(*goal, "g*", ms=16, label="goal")

    robot_dot, = ax.plot([], [], "bo", ms=8, label="robot")
    trail_x, trail_y = [], []
    trail_line, = ax.plot([], [], "b-", lw=1, alpha=0.6)
    plan_line, = ax.plot([], [], "c--", lw=1, label="MPC plan")
    fitted = [plt.Circle((0, 0), 1e-3, color="orange", fill=False, lw=2, visible=False)
              for _ in range(core.map.n_obs)]
    for c in fitted:
        ax.add_patch(c)
    status_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", fontsize=9,
                          family="monospace")
    ax.legend(loc="lower right")

    pad = 0.5
    xs = [o[0] for o in obstacles] + [plant.pose[0], goal[0]]
    ys = [o[1] for o in obstacles] + [plant.pose[1], goal[1]]
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)

    def update(frame):
        dist = plant.sense()
        core.sense(plant.pose, dist)
        r = core.plan(plant.pose, goal, v_max, omega_max, running=True)
        plant.step(r["v"], r["omega"], CONTROL_DT)

        robot_dot.set_data([plant.pose[0]], [plant.pose[1]])
        trail_x.append(plant.pose[0]); trail_y.append(plant.pose[1])
        trail_line.set_data(trail_x, trail_y)
        plan_line.set_data(core.plan_x, core.plan_y)

        active = core.map.active()
        for c, o in zip(fitted, active):
            c.center = (o.cx, o.cy)
            c.radius = o.r + MARGIN_M
            c.set_visible(True)
        for c in fitted[len(active):]:
            c.set_visible(False)

        status_text.set_text(
            f"t={frame*CONTROL_DT:5.1f}s  v={r['v']:.3f}  w={r['omega']:+.3f}\n"
            f"status={r['status']}  soft={int(r['used_soft'])}  blocked={int(r['blocked'])}\n"
            f"solve={r['solve_ms']:5.1f} ms  dist_goal={r['dist_goal']:.3f} m"
        )

        if r["arrived"]:
            anim.event_source.stop()
            print(f"arrived at t={frame*CONTROL_DT:.1f}s")
        return (robot_dot, trail_line, plan_line, status_text, *fitted)

    anim = animation.FuncAnimation(fig, update, interval=int(CONTROL_DT * 1000), blit=False)
    plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--obstacles", default=DEFAULT_OBSTACLES,
                    help="'cx,cy,r cx,cy,r ...' (default: two obstacles)")
    ap.add_argument("--goal", default="2.0,0.0")
    ap.add_argument("--v-max", type=float, default=V_MAX_DEFAULT)
    ap.add_argument("--omega-max", type=float, default=OMEGA_MAX_DEFAULT)
    ap.add_argument("--no-gui", action="store_true", help="print status instead of animating")
    ap.add_argument("--max-steps", type=int, default=300, help="--no-gui only")
    args = ap.parse_args()

    obstacles = parse_obstacles(args.obstacles)
    goal = parse_xy(args.goal)
    plant = AlvikPlant(START, obstacles)
    core = NavCore()

    if args.no_gui:
        run_headless(plant, core, goal, args.v_max, args.omega_max, args.max_steps)
    else:
        run_animated(plant, core, goal, obstacles, args.v_max, args.omega_max)


if __name__ == "__main__":
    main()
