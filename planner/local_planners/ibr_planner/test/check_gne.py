#!/usr/bin/env python3
"""Offline test: is a recorded SE-IBR plan a generalized Nash equilibrium?

Holding theta_j fixed, re-solve player i's problem much harder than the planner can in real
time and measure how much it could still gain. That gain is the Nash gap of player i:

    gap_i = max J_i(theta, theta_j) - J_i(theta_i, theta_j)    theta in C_i(theta_j)

The plan is an (eps, delta) equilibrium when it violates the constraints by at most eps and
gap_i <= delta for both players, with each best response allowed that same eps.

Compare costs, not trajectories: progress only depends on p^N, so many different theta share
the same optimum and ||theta - BR(theta_j)|| can be large at a true equilibrium.

Usage:  python3 check_gne.py run.jsonl global_waypoints.json [--max-gap 0.05] [--every 10]
"""
import argparse
import json
from dataclasses import replace

import numpy as np
from scipy.optimize import minimize

from ibr_planner.inner_ocp import Params, Track, keep_out, true_violations

LAT_FRACS = (-0.8, -0.4, 0.0, 0.4, 0.8)      # warm starts: fraction of the usable half-width
SPEED_FRACS = (1.0, 0.6)                     # warm starts: fraction of vmax
FEAS_TOL = 1e-3                              # slack on the SLSQP result before it is discarded


def load_run(path):
    with open(path) as f:
        head = json.loads(f.readline())
        ticks = [json.loads(line) for line in f]
    return head, ticks


def progress(gb, p, s0g, win):
    s, _, _, _, _, _, _, _ = gb.frenet(p[-1:], s0g, win)
    return gb.ds_along(s0g, float(s[0]))


def cost(gb, p, s0g, win, w):
    """J_i to maximise: progress of p^N plus the frozen sensitivity term of eq. (16)."""
    return progress(gb, p, s0g, win) + float(np.sum(w * p[1:]))


def starts(track, x, s0, prm):
    """A spread of initial guesses, because the collision constraint makes the problem
    non-convex: passing left and passing right are separate local optima, and a single SCP
    run from the planner's own plan can only ever see the one it is already in."""
    k = np.arange(prm.N + 1)
    lat0 = track.lateral(x, s0, 1.0)
    fade = np.maximum(0.0, 1.0 - k / 5.0)
    for fv in SPEED_FRACS:
        ss = s0 + fv * prm.vmax * prm.dt * k
        wl, wr = track.width(ss)
        for fl in LAT_FRACS:
            lat = (wl if fl >= 0 else wr) * fl
            p = track.pos(ss) + (lat0 * fade + lat * (1.0 - fade))[:, None] * track.normal(ss)
            p[0] = x
            yield p


def best_response(track, gb, q, x, s0g, prm, w, theta, eps, multistart=True):
    """Solve the TRUE non-convex problem of player i with theta_j = q held fixed. No
    linearisation is shared with the planner's SCP, so a bug or an early stop in the SCP
    cannot hide itself here.

    eps relaxes the track and collision constraints by the violation theta itself carries.
    Without it the comparison is unfair: 1 cm of violation can open a gap between the rival
    and the wall that no strictly feasible plan can use. theta is feasible for the relaxed
    problem, so it is the incumbent and the returned value can only match or beat it.
    """
    N, um2, lim = prm.N, prm.umax ** 2, (1.0 - eps / prm.d_min) ** 2
    A = keep_out(q, prm)
    bend2 = (prm.kappa_max * prm.umax ** 2 + eps) ** 2
    s0 = track.project(x)
    win = max(prm.proj_window or 0.0, prm.umax * N + 1.0)

    def unpack(z):
        return np.vstack([x, z.reshape(N, 2)])

    def f(z):
        return -cost(gb, unpack(z), s0g, win, w)

    def df(z):
        p, g = unpack(z), np.zeros((N, 2))
        _, _, t, _, k, lat, _, _ = gb.frenet(p[-1:], s0g, win)
        g[-1] = t[0] / max(1.0 - k[0] * lat[0], 0.1)     # eq. (18)
        return -(g + w).ravel()

    def cons(z):
        p = unpack(z)
        step = np.sum(np.diff(p, axis=0) ** 2, axis=1)
        _, _, _, _, _, lat, wl, wr = track.frenet(p[1:], s0, win)
        turn = p[2:] - 2 * p[1:-1] + p[:-2]
        d = p[1:] - q[1:]
        return np.r_[um2 - step, wl + eps - lat, wr + eps + lat,
                     np.einsum('ki,kij,kj->k', d, A, d) - lim,
                     bend2 - np.sum(turn ** 2, axis=1)]

    def dcons(z):
        p = unpack(z)
        _, _, _, n, _, _, _, _ = track.frenet(p[1:], s0, win)
        J = np.zeros((5 * N - 1, 2 * N))
        du = p[1:] - p[:-1]
        dq = p[1:] - q[1:]
        for k in range(N):
            J[k, 2 * k:2 * k + 2] = -2.0 * du[k]
            if k > 0:
                J[k, 2 * k - 2:2 * k] = 2.0 * du[k]
            J[N + k, 2 * k:2 * k + 2] = -n[k]
            J[2 * N + k, 2 * k:2 * k + 2] = n[k]
            J[3 * N + k, 2 * k:2 * k + 2] = 2.0 * A[k] @ dq[k]
        turn = p[2:] - 2 * p[1:-1] + p[:-2]
        for k in range(N - 1):
            J[4 * N + k, 2 * k:2 * k + 2] = 4.0 * turn[k]
            J[4 * N + k, 2 * k + 2:2 * k + 4] = -2.0 * turn[k]
            if k > 0:
                J[4 * N + k, 2 * k - 2:2 * k] = -2.0 * turn[k]
        return J

    best, best_j = theta, cost(gb, theta, s0g, win, w)
    for p0 in [theta, *(starts(track, x, s0, prm) if multistart else ())]:
        r = minimize(f, p0[1:].ravel(), jac=df, method="SLSQP", tol=1e-7,
                     constraints={"type": "ineq", "fun": cons, "jac": dcons},
                     options={"maxiter": 300})
        if np.min(cons(r.x)) < -FEAS_TOL * prm.umax:     # SLSQP gave up outside the feasible set
            continue
        j = -f(r.x)
        if j > best_j:
            best, best_j = unpack(r.x), j
    return best, best_j


def nash_gap(track, gb, tick, i, prm, alpha):
    """Best-response gap of player i with the other player held fixed, for one objective."""
    th, q = np.array(tick["theta"][i]), np.array(tick["theta"][1 - i])
    mu_j = np.array(tick["mu"][1 - i])
    d = q[1:] - th[1:]
    Ad = np.einsum('kij,kj->ki', keep_out(q, prm), d)
    beta = Ad / np.maximum(np.linalg.norm(Ad, axis=1, keepdims=True), 1e-9)   # ellipse normal
    w = alpha * mu_j[:, None] * beta
    x, s0g = th[0], tick["s0g"][i]
    win = max(prm.proj_window or 0.0, prm.umax * prm.N + 1.0)
    tv, cv, _ = true_violations(track, th, q, prm, track.project(x), win)
    eps = max(tv, cv, 0.0)
    # The sensitivity term is linear in p, so it only means something near theta: far away it
    # pays a plan for sitting on the wrong side of the opponent. alpha>0 is tested locally.
    br, j_br = best_response(track, gb, q, x, s0g, prm, w, th, eps, multistart=alpha == 0.0)
    return dict(gap=j_br - cost(gb, th, s0g, win, w), eps=eps, br=br)


def check_tick(track, gb, head, tick):
    """Both players, both objectives. alpha=0 is the pure racing game; alpha>0 is the game
    SE-IBR actually optimised, with the duals and beta frozen at the final plan."""
    out = {}
    for i in (0, 1):
        prm = replace(head_params(head), vmax=tick["vmax"][i])
        out[i, "se"] = nash_gap(track, gb, tick, i, prm, head["alpha"][i])
        out[i, "pure"] = nash_gap(track, gb, tick, i, prm, 0.0)
    return out


def head_params(head):
    return Params(N=head["N"], dt=head["dt"], d_min=head["d_min"], proj_window=head["proj_window"],
                  kappa_max=head.get("kappa_max", 1e3), d_lat=head.get("d_lat"))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run", help="jsonl written by the ibr_planner node (record_path)")
    ap.add_argument("map", help="stack_master/maps/<map>/global_waypoints.json")
    ap.add_argument("--max-gap", type=float, default=0.05, help="delta: largest gap of an equilibrium [m]")
    ap.add_argument("--max-viol", type=float, default=0.05,
                    help="largest constraint violation of an equilibrium [m]; 0.05 matches "
                         "the node's viol_tolerance")
    ap.add_argument("--every", type=int, default=10, help="test every n-th tick")
    ap.add_argument("--interaction", type=float, default=1.0,
                    help="keep ticks whose closest approach is within d_min + this [m]")
    args = ap.parse_args()

    head, ticks = load_run(args.run)
    if head.get("mode") == "mpc":
        raise SystemExit("naive MPC plays no game, so there is no equilibrium to check")
    track = Track.from_json(args.map, margin=head["margin"])
    gb = Track.from_json(args.map, key="global_traj_wpnts_iqp")

    def close(t):
        gap = np.linalg.norm(np.array(t["theta"][0]) - np.array(t["theta"][1]), axis=1)
        return gap.min() < head["d_min"] + args.interaction

    keep = [t for t in ticks if t["game"] and t["ok"] and close(t)][::args.every]
    print(f"{len(ticks)} ticks recorded, testing {len(keep)}")
    print("tick conv   viol | SE game: gap0  gap1 | pure race: gap0  gap1 | equilibrium")
    n_se = n_pure = 0
    for t in keep:
        o = check_tick(track, gb, head, t)
        eps = max(o[0, "se"]["eps"], o[1, "se"]["eps"])
        se = eps <= args.max_viol and max(o[0, "se"]["gap"], o[1, "se"]["gap"]) <= args.max_gap
        pure = eps <= args.max_viol and max(o[0, "pure"]["gap"], o[1, "pure"]["gap"]) <= args.max_gap
        n_se, n_pure = n_se + se, n_pure + pure
        print(f"{t['tick']:4d} {str(t['converged']):5s} {eps:5.3f} |"
              f"       {o[0, 'se']['gap']:6.3f} {o[1, 'se']['gap']:6.3f} |"
              f"         {o[0, 'pure']['gap']:6.3f} {o[1, 'pure']['gap']:6.3f} |"
              f" SE={'yes' if se else 'NO'} pure={'yes' if pure else 'NO'}")
    print(f"(eps={args.max_viol} m, delta={args.max_gap} m): SE game {n_se}/{len(keep)}, "
          f"pure race {n_pure}/{len(keep)}")


if __name__ == "__main__":
    main()
