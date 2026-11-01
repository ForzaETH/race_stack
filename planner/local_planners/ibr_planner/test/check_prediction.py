#!/usr/bin/env python3
"""Offline test: did the cars do what the recorded SE-IBR game predicted?

Each tick the game predicts both cars' raceline progress over the horizon T = N*dt. Compare
that with how far the cars really got T seconds later:

    pred_i = s(theta_i^N) - s(theta_i^0)
    real_i = s_i(t + T) - s_i(t)
    diff   = progress of player 0 - progress of player 1

Player 0 is the car whose planner wrote the recording. The horizon is measured in seconds
from the odometry stamps, not in ticks: rate_hz is not 1/dt and a slow solve stretches a tick.

Usage:  python3 check_prediction.py run.jsonl global_waypoints.json [--settle 3.0] [--every 20]
"""
import argparse

import numpy as np

from check_gne import load_run, progress
from ibr_planner.inner_ocp import Track


def measured(ticks, gb, i):
    """Time and unwrapped raceline s of player i, so crossing the lap line is harmless."""
    t = np.array([r["t"][i] for r in ticks])
    s = np.unwrap([r["s0g"][i] for r in ticks], period=gb.L)
    return t, s


def predicted(tick, gb, head, i):
    win = max(head["proj_window"] or 0.0, tick["vmax"][i] * head["dt"] * head["N"] + 1.0)
    return progress(gb, np.array(tick["theta"][i]), tick["s0g"][i], win)


def compare(ticks, gb, head, settle, interaction):
    """One row per testable tick: t, pred0, real0, pred1, real1, close."""
    T = head["N"] * head["dt"]
    meas = [measured(ticks, gb, i) for i in (0, 1)]
    t_end = min(t[-1] for t, _ in meas) - T
    t_start = max(t[0] for t, _ in meas) + settle
    rows = []
    for k, tick in enumerate(ticks):
        if not tick["ok"] or not t_start <= meas[0][0][k] <= t_end:
            continue
        pred = [predicted(tick, gb, head, i) for i in (0, 1)]
        real = [np.interp(t[k] + T, t, s) - s[k] for t, s in meas]
        dist = np.linalg.norm(np.array(tick["theta"][0]) - np.array(tick["theta"][1]), axis=1)
        rows.append((meas[0][0][k] - meas[0][0][0], pred[0], real[0], pred[1], real[1],
                     dist.min() < head["d_min"] + interaction))
    return np.array(rows)


def summarise(name, rows):
    print(f"{name}: {len(rows)} ticks")
    if len(rows) == 0:
        return
    pred0, real0, pred1, real1 = rows[:, 1], rows[:, 2], rows[:, 3], rows[:, 4]
    print("                       predicted   actual   mean err   rmse")
    for label, p, r in (("player 0 progress [m]", pred0, real0),
                        ("player 1 progress [m]", pred1, real1),
                        ("diff  0 - 1       [m]", pred0 - pred1, real0 - real1)):
        e = p - r
        print(f"  {label}  {p.mean():8.3f} {r.mean():8.3f}  {e.mean():+8.3f} {np.sqrt(np.mean(e ** 2)):7.3f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run", help="jsonl written by the ibr_planner node (record_path)")
    ap.add_argument("map", help="stack_master/maps/<map>/global_waypoints.json")
    ap.add_argument("--settle", type=float, default=3.0,
                    help="skip this many seconds of launch from standstill")
    ap.add_argument("--interaction", type=float, default=1.0,
                    help="a tick is 'close' when the plans come within d_min + this [m]")
    ap.add_argument("--every", type=int, default=20, help="print every n-th tick")
    args = ap.parse_args()

    head, ticks = load_run(args.run)
    # Solo ticks store the ego's s in both s0g slots, so they cannot be used for player 1.
    ticks = [r for r in ticks if r["game"]]
    if "t" not in ticks[0]:
        raise SystemExit("no odometry stamps in this recording; record it again with t and v")
    gb = Track.from_json(args.map, key="global_traj_wpnts_iqp")

    T = head["N"] * head["dt"]
    t0 = np.array([r["t"][0] for r in ticks])
    period = np.median(np.diff(t0))
    print(f"{len(ticks)} game ticks over {t0[-1] - t0[0]:.1f} s; tick period median "
          f"{period * 1e3:.0f} ms, max {np.diff(t0).max() * 1e3:.0f} ms; "
          f"T = {T:.2f} s spans about {T / period:.0f} ticks")
    # Raceline distance / integral of odom speed. Below 1 means the sim runs slower than the
    # wall clock that stamps the odometry, and every "actual" number here is then too small.
    for i in (0, 1):
        t, s = measured(ticks, gb, i)
        v = np.abs([r["v"][i] for r in ticks])
        print(f"clock check player {i}: {(s[-1] - s[0]) / np.sum(v[:-1] * np.diff(t)):.3f} (expect ~1)")

    rows = compare(ticks, gb, head, args.settle, args.interaction)
    print("\n   t   pred0  real0  pred1  real1   diff pred  diff real  close")
    for t, p0, r0, p1, r1, c in rows[::args.every]:
        print(f"{t:5.1f} {p0:6.2f} {r0:6.2f} {p1:6.2f} {r1:6.2f}  {p0 - p1:9.2f}  {r0 - r1:9.2f}  {bool(c)}")

    print()
    summarise("all", rows)
    if len(rows):
        summarise("cars interacting", rows[rows[:, 5] == 1])
        summarise("cars apart", rows[rows[:, 5] == 0])


if __name__ == "__main__":
    main()
