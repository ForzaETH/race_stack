#!/usr/bin/env python3
"""Plot one benchmark race: both paths on the track, then lead, distance and speed over time.

Needs only numpy and matplotlib, so it runs on the Mac without ROS or cvxpy.

Usage:  python3 bench_plot.py run.jsonl global_waypoints.json [-o run.png]
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ATTACKER, DEFENDER = "#2a78d6", "#eb6834"
SURFACE, INK, INK_2 = "#fcfcfb", "#0b0b0b", "#52514e"
GRID, WALL = "#e6e5e1", "#a9a8a2"
MARGIN = 3.0       # [m] of track shown around the race
EVERY = 2.0        # [s] between the time dots on the paths


def load_race(path):
    with open(path) as f:
        rows = [json.loads(line) for line in f]
    race = [r for r in rows if "t" in r]
    t = np.array([r["t"] for r in race])
    ego = np.array([r["ego"] for r in race])      # columns: x, y, v, s
    opp = np.array([r["opp"] for r in race])
    dist = np.array([r["dist"] for r in race])
    return t, ego, opp, dist, rows[-1]


def load_map(path):
    with open(path) as f:
        data = json.load(f)
    L = data["global_traj_wpnts_iqp"]["wpnts"][-1]["s_m"]
    walls = [m["pose"]["position"] for m in data["trackbounds_markers"]["markers"]]
    return L, np.array([[w["x"], w["y"]] for w in walls])


def style(ax, title):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, loc="left", fontsize=10, color=INK_2)
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    ax.tick_params(colors=INK_2, labelsize=9, length=0)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(WALL)


def draw_path(ax, car, t, name, color):
    ax.plot(car[:, 0], car[:, 1], color=color, linewidth=2, label=name, solid_capstyle="round")
    dots = [int(np.argmin(np.abs(t - k))) for k in np.arange(EVERY, t[-1], EVERY)]
    ax.plot(car[dots, 0], car[dots, 1], "o", color=color, markersize=5,
            markeredgecolor=SURFACE, markeredgewidth=1)
    ax.plot(car[0, 0], car[0, 1], "o", color=SURFACE, markersize=8,
            markeredgecolor=color, markeredgewidth=2)
    ax.annotate(name, car[-1, :2], xytext=(7, 7), textcoords="offset points", fontsize=9, color=INK)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("log", help="jsonl written by bench_referee.py")
    ap.add_argument("map", help="stack_master/maps/<map>/global_waypoints.json")
    ap.add_argument("-o", "--out", help="image to write; default: the log's name with .png")
    args = ap.parse_args()

    t, ego, opp, dist, end = load_race(args.log)
    L, walls = load_map(args.map)
    lead = (ego[:, 3] - opp[:, 3] + L / 2) % L - L / 2      # wrapped like Race.lead()

    fig = plt.figure(figsize=(11, 6.2), facecolor=SURFACE)
    grid = fig.add_gridspec(3, 2, width_ratios=[1.2, 1], left=0.05, right=0.97, top=0.84,
                            bottom=0.09, hspace=0.5, wspace=0.14)
    outcome = end.get("outcome", "unfinished")
    closest = dist.min()
    fig.suptitle(f"{outcome} after {t[-1]:.1f} s", x=0.05, ha="left", fontsize=15, color=INK,
                 fontweight="bold")
    fig.text(0.05, 0.895, f"{Path(args.log).stem}, closest approach {closest:.2f} m",
             fontsize=10, color=INK_2)

    ax = fig.add_subplot(grid[:, 0])
    ax.set_facecolor(SURFACE)
    ax.scatter(walls[:, 0], walls[:, 1], s=1, color=WALL, linewidths=0)
    draw_path(ax, ego, t, "Attacker", ATTACKER)
    draw_path(ax, opp, t, "Defender", DEFENDER)
    both = np.vstack([ego[:, :2], opp[:, :2]])
    ax.set_xlim(both[:, 0].min() - MARGIN, both[:, 0].max() + MARGIN)
    ax.set_ylim(both[:, 1].min() - MARGIN, both[:, 1].max() + MARGIN)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_title(f"Paths, open circle = start, dots every {EVERY:.0f} s", loc="left", fontsize=10,
                 color=INK_2)
    ax.legend(loc="lower right", frameon=False, fontsize=9, labelcolor=INK)
    ax.set_xticks([])
    ax.set_yticks([])
    for side in ax.spines.values():
        side.set_visible(False)

    ax = fig.add_subplot(grid[0, 1])
    style(ax, "Attacker lead along the raceline [m]")
    ax.axhline(0.0, color=INK_2, linewidth=1)
    ax.plot(t, lead, color=INK_2, linewidth=2)
    shared = ax

    ax = fig.add_subplot(grid[1, 1], sharex=shared)
    style(ax, "Distance between car centres [m]")
    ax.plot(t, dist, color=INK_2, linewidth=2)
    k = int(np.argmin(dist))
    late = t[k] > 0.85 * t[-1]                              # keep the label inside the axes
    ax.annotate(f"{closest:.2f} m", (t[k], closest), xytext=(-7, -4) if late else (0, 7),
                textcoords="offset points", ha="right" if late else "center", fontsize=9,
                color=INK)
    ax.set_ylim(0.0, dist.max() * 1.15)

    ax = fig.add_subplot(grid[2, 1], sharex=shared)
    style(ax, "Speed [m/s]")
    ax.plot(t, ego[:, 2], color=ATTACKER, linewidth=2, label="Attacker")
    ax.plot(t, opp[:, 2], color=DEFENDER, linewidth=2, label="Defender")
    ax.legend(loc="upper right", ncol=2, frameon=False, fontsize=9, labelcolor=INK)
    ax.set_ylim(0.0, max(ego[:, 2].max(), opp[:, 2].max()) * 1.3)
    ax.set_xlabel("Race time [s]", fontsize=9, color=INK_2)

    out = args.out or str(Path(args.log).with_suffix(".png"))
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print(out)


if __name__ == "__main__":
    main()
