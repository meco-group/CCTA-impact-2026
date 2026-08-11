"""Run your MPC: in-process dev loop, no UDP link, no dashboard, no fake_robot.py process.

Runs AlvikPlant + NavCore directly in one process, saving the run as a GIF next to this
script (no live window), so mpc.py can be iterated on without the network stack. For
full-stack/network testing use fake_robot.py + controller.py (+ dashboard.py) instead.

Run:
    python run_mpc.py                            # saves run_mpc.gif in this folder
    python run_mpc.py --obstacles "0.7,0.3,0.14 1.6,0.05,0.12" --goal 1.5,0
    python run_mpc.py --out my_run.gif
    python run_mpc.py --no-gui --max-steps 300   # headless, prints status instead
"""

import argparse
from pathlib import Path

from geometry import CONTROL_DT, MARGIN_M, V_MAX_DEFAULT, OMEGA_MAX_DEFAULT
from nav_core import NavCore
from obstacles import DEFAULT_OBSTACLES, parse_obstacles
from plant import AlvikPlant

START = (0.0, 0.0, 0.0)
OUT_DIR = Path(__file__).resolve().parent


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


def run_animated(plant, core, goal, obstacles, v_max, omega_max, max_steps, out_path):
    # Manual blitting: matplotlib's Animation.save() re-renders the WHOLE figure (axes,
    # gridlines, tick labels, legend) via fig.savefig() on every single frame, no matter
    # what `blit=` is passed to FuncAnimation -- that only speeds up on-screen display, not
    # file output. Static content is drawn once into a cached background here, and only the
    # handful of moving artists (robot dot, trail, plan line, obstacle fits, status text)
    # are re-rendered per step, then composited straight into PIL frames for the GIF.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect("equal")
    ax.grid(True)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")

    for cx, cy, r in obstacles:
        ax.add_patch(plt.Circle((cx, cy), r, color="r", alpha=0.4))
        ax.add_patch(plt.Circle((cx, cy), r + MARGIN_M, color="r", alpha=0.12))
    ax.plot(*goal, "g*", ms=16, label="goal")

    robot_dot, = ax.plot([], [], "bo", ms=8, label="robot", animated=True)
    trail_x, trail_y = [], []
    trail_line, = ax.plot([], [], "b-", lw=1, alpha=0.6, animated=True)
    plan_line, = ax.plot([], [], "c--", lw=1, label="MPC plan", animated=True)
    fitted = [plt.Circle((0, 0), 1e-3, color="orange", fill=False, lw=2, visible=False,
                          animated=True)
              for _ in range(core.map.n_obs)]
    for c in fitted:
        ax.add_patch(c)
    status_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", fontsize=9,
                          family="monospace", animated=True)
    ax.legend(loc="lower right")

    pad = 0.5
    xs = [o[0] for o in obstacles] + [plant.pose[0], goal[0]]
    ys = [o[1] for o in obstacles] + [plant.pose[1], goal[1]]
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)

    dynamic_artists = (robot_dot, trail_line, plan_line, status_text, *fitted)

    fig.canvas.draw()
    background = fig.canvas.copy_from_bbox(fig.bbox)
    size = fig.canvas.get_width_height()

    frames = []
    for step in range(max_steps):
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
            f"t={step*CONTROL_DT:5.1f}s  v={r['v']:.3f}  w={r['omega']:+.3f}\n"
            f"status={r['status']}  soft={int(r['used_soft'])}  blocked={int(r['blocked'])}\n"
            f"solve={r['solve_ms']:5.1f} ms  dist_goal={r['dist_goal']:.3f} m"
        )

        fig.canvas.restore_region(background)
        for artist in dynamic_artists:
            ax.draw_artist(artist)
        fig.canvas.blit(fig.bbox)

        buf = fig.canvas.buffer_rgba()
        frames.append(Image.frombuffer("RGBA", size, buf, "raw", "RGBA", 0, 1).convert("RGB"))

        if r["arrived"]:
            print(f"arrived at t={step*CONTROL_DT:.1f}s")
            break

    plt.close(fig)

    fps = int(round(1 / CONTROL_DT))
    frames[0].save(out_path, save_all=True, append_images=frames[1:],
                   duration=int(1000 / fps), loop=0)
    print(f"saved animation to {out_path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--obstacles", default=DEFAULT_OBSTACLES,
                    help="'cx,cy,r cx,cy,r ...' (default: two obstacles)")
    ap.add_argument("--goal", default="2.0,0.0")
    ap.add_argument("--v-max", type=float, default=V_MAX_DEFAULT)
    ap.add_argument("--omega-max", type=float, default=OMEGA_MAX_DEFAULT)
    ap.add_argument("--no-gui", action="store_true", help="print status instead of animating")
    ap.add_argument("--max-steps", type=int, default=300, help="max sim steps (both modes)")
    ap.add_argument("--out", default=str(OUT_DIR / "run_mpc.gif"),
                    help="where to save the animation (--no-gui mode ignores this)")
    args = ap.parse_args()

    obstacles = parse_obstacles(args.obstacles)
    goal = parse_xy(args.goal)
    plant = AlvikPlant(START, obstacles)
    core = NavCore()

    if args.no_gui:
        run_headless(plant, core, goal, args.v_max, args.omega_max, args.max_steps)
    else:
        run_animated(plant, core, goal, obstacles, args.v_max, args.omega_max,
                     args.max_steps, args.out)


if __name__ == "__main__":
    main()
