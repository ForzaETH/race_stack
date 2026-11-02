#!/usr/bin/env python3
"""
Game-theoretic planner of Spica et al. 2018, "A Real-Time Game Theoretic Planner for
Autonomous Two-Player Drone Racing" -- inner OCP (eq. 16/18) and the IBR loop (Fig. 2).

Inner OCP, player i, opponent trajectory q FIXED:

    max   s_i(p^N) + alpha * sum_k mu_j^k beta_ij^kT p^k                        (16)
    s.t.  p^k = p^{k-1} + u^k,  ||u^k|| <= vmax*dt                              (9b,9e)
          -w_r <= n(p^k)^T (p^k - tau(p^k)) <= w_l                              (9d)
          ||p^k - q^k|| >= d_min                                                (9c)

solved by sequential QCLP, convexified around the iterate p_m:

    collision : beta^kT (q^k - p^k) >= d_min,  beta^k = (q^k - p_m^k)/||q^k - p_m^k||
    track     : n_m^kT (p^k - tau_m^k) in [-w_r, w_l]
    progress  : sigma_m^T p^N,   sigma = t^T / (1 - kappa (p_m^N - tau)^T n)    (18)

Differences from the paper, needed to make the solve reliable:
  * L1-penalised slacks on the track and collision constraints -> the QCLP is ALWAYS
    feasible, even when the measured start state already violates them,
  * trust region + accept/reject on the true (non-linearised) merit function,
  * all track geometry precomputed on a dense arclength grid (no spline calls in the loop),
  * one DPP-compiled cvxpy problem per horizon length, re-solved with new parameter values.

Usage:  python3 inner_ocp.py global_waypoints.json      (offline demo + plots)
        from a ROS node: see ibr_planner.py
"""
import json
import sys
import time
from dataclasses import dataclass, field

import cvxpy as cp
import numpy as np
from scipy.interpolate import CubicSpline


# --------------------------------------------------------------------------- track
class Track:
    """Closed centerline, arclength parameterised, with everything the OCP needs
    precomputed on a dense grid (ds_grid) so the planning loop does array lookups only."""

    def __init__(self, x, y, d_left, d_right, margin=0.0, ds_grid=0.02):
        x, y = np.asarray(x, float), np.asarray(y, float)
        d_left, d_right = np.asarray(d_left, float), np.asarray(d_right, float)
        pts = np.c_[x, y]
        if len(pts) > 3 and np.linalg.norm(pts[0] - pts[-1]) < 1e-7:   # repeated last point
            pts, d_left, d_right = pts[:-1], d_left[:-1], d_right[:-1]
        seg = np.linalg.norm(np.diff(np.vstack([pts, pts[:1]]), axis=0), axis=1)
        s_knot = np.r_[0.0, np.cumsum(seg)]
        self.L = float(s_knot[-1])
        spl = CubicSpline(s_knot, np.vstack([pts, pts[:1]]), bc_type="periodic")
        wl = CubicSpline(s_knot, np.r_[d_left, d_left[:1]], bc_type="periodic")
        wr = CubicSpline(s_knot, np.r_[d_right, d_right[:1]], bc_type="periodic")

        # ---- dense grid: positions, tangents, left normals, curvature, half-widths
        self.sg = np.arange(0.0, self.L, ds_grid)
        self.ds = float(self.sg[1] - self.sg[0])
        self.M = len(self.sg)
        self.cg = spl(self.sg)
        d1, d2 = spl(self.sg, 1), spl(self.sg, 2)
        self.tg = d1 / np.linalg.norm(d1, axis=1, keepdims=True)
        self.ng = np.c_[-self.tg[:, 1], self.tg[:, 0]]                 # LEFT normal
        self.kg = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / \
                  np.linalg.norm(d1, axis=1) ** 3
        self.wlg = np.maximum(np.asarray(wl(self.sg), float) - margin, 0.0)
        self.wrg = np.maximum(np.asarray(wr(self.sg), float) - margin, 0.0)
        self.margin = margin

    # ---- grid lookups -----------------------------------------------------------
    def _idx(self, s):
        return np.mod(np.round(np.asarray(s, float) / self.ds).astype(int), self.M)

    def pos(self, s):
        return self.cg[self._idx(s)]

    def tangent(self, s):
        return self.tg[self._idx(s)]

    def normal(self, s):
        return self.ng[self._idx(s)]

    def curvature(self, s):
        return self.kg[self._idx(s)]

    def width(self, s):
        i = self._idx(s)
        return self.wlg[i], self.wrg[i]

    # ---- projection -------------------------------------------------------------
    def frenet(self, P, s_hint=None, window=None):
        """Project points P (K,2) onto the centerline.

        s_hint/window restrict the search to an arclength window around s_hint, which stops
        the projection jumping to another part of the track that happens to run close by --
        the usual silent failure on tight F1TENTH maps.

        Returns s (K,), tau (K,2), t (K,2), n (K,2), kappa (K,), lat (K,), wl (K,), wr (K,).
        """
        P = np.atleast_2d(np.asarray(P, float))
        if s_hint is not None and window is not None and 2.0 * window < self.L:
            # Search a slice of the grid rather than all of it: cheaper, and it stops the
            # projection snapping onto another part of the track that runs close by.
            i0 = int(np.floor((s_hint - window) / self.ds))
            idx = np.mod(i0 + np.arange(int(2.0 * window / self.ds) + 1), self.M)
            d2 = np.sum((P[:, None, :] - self.cg[None, idx, :]) ** 2, axis=2)
            i = idx[np.argmin(d2, axis=1)]
        else:
            d2 = np.sum((P[:, None, :] - self.cg[None, :, :]) ** 2, axis=2)    # (K, M)
            i = np.argmin(d2, axis=1)
        c, t, n = self.cg[i], self.tg[i], self.ng[i]
        ds = np.clip(np.sum(t * (P - c), axis=1), -self.ds, self.ds)           # sub-grid refine
        s = np.mod(self.sg[i] + ds, self.L)
        tau = c + t * ds[:, None]
        lat = np.sum(n * (P - tau), axis=1)
        return s, tau, t, n, self.kg[i], lat, self.wlg[i], self.wrg[i]

    def project(self, p, s_hint=None, window=None):
        """Scalar arclength of the closest centerline point to p."""
        return float(self.frenet(np.atleast_2d(p), s_hint, window)[0][0])

    def lateral(self, p, s_hint=None, window=None):
        return float(self.frenet(np.atleast_2d(p), s_hint, window)[5][0])

    def ds_along(self, s_from, s_to):
        """Signed arclength gained, unwrapped onto (-L/2, L/2]."""
        return float((s_to - s_from + self.L / 2) % self.L - self.L / 2)

    @staticmethod
    def from_json(path, margin=0.0, key="centerline_waypoints"):
        w = json.load(open(path))[key]["wpnts"]
        return Track([q["x_m"] for q in w], [q["y_m"] for q in w],
                     [q["d_left"] for q in w], [q["d_right"] for q in w], margin=margin)

    @staticmethod
    def from_wpnts(wpnts, margin=0.0):
        """From a list of f110_msgs/Wpnt (ROS): x_m, y_m, d_left, d_right."""
        return Track([w.x_m for w in wpnts], [w.y_m for w in wpnts],
                     [w.d_left for w in wpnts], [w.d_right for w in wpnts], margin=margin)


# --------------------------------------------------------------------------- params
@dataclass
class Params:
    N: int = 20                  # horizon steps
    dt: float = 0.1              # step [s]
    # vmax: float = 3.2            # max speed [m/s] -> step bound vmax*dt
    vmax: float = 1.6
    d_min: float = 0.5           # min centre-to-centre distance along the rival's travel [m]
    d_lat: float = None          # same across its travel [m]; None = circle of radius d_min
    track_margin: float = 0.15   # car half-width, removed from d_left/d_right [m]
    max_iter: int = 8           # max SCP iterations per best response
    tol: float = 1e-3            # ||theta^{m+1} - theta^m|| stopping threshold
    trust: float = 1.0           # max trust-region radius per point [m] (None = off)
    trust0: float = 0.4          # initial radius [m]; the vmax*dt scale works well
    slack_w: float = 1e3         # L1 penalty on the QCLP slacks
    merit_w: float = 100.0       # penalty in the accept/reject merit (must exceed |sigma| ~ 1)
    mu_max: float = 2.0          # clip on the duals handed to the opponent (see ibr())
    kappa_max: float = 1.2       # sharpest bend a plan may ask for at vmax [1/m]; car limit ~1.35
    proj_window: float = None    # arclength window for projections [m]; None = global
    solver: str = "CLARABEL"
    verbose: bool = False

    @property
    def umax(self):
        return self.vmax * self.dt


@dataclass
class Result:
    p: np.ndarray                # (N+1, 2) positions, p[0] = start
    u: np.ndarray                # (N, 2) steps; speed = ||u||/dt
    s_progress: float            # arclength gained over the horizon [m]
    status: str
    iters: int
    max_track_viol: float        # true violations of the returned plan [m]  (<= 0 is OK)
    max_coll_viol: float
    mu: np.ndarray = None        # (N,) duals of the collision constraints (>= 0)
    beta: np.ndarray = None      # (N,2) unit vectors ego -> opponent
    solve_time: float = 0.0
    log: list = field(default_factory=list)
    s0g: float = 0.0


# --------------------------------------------------------------------------- QCLP
class _QCLP:
    """The convexified subproblem, compiled ONCE per horizon length and re-solved with new
    parameter values. Rebuilding it every SCP iteration costs ~35 ms; this costs ~3 ms."""

    def __init__(self, N, solver="CLARABEL"):
        self.N, self.solver = N, solver
        # The decision variable is the positions themselves. Keeping p0 OUT of it is what
        # makes the problem DPP: with p = p0 + cumsum(u), a term like n^T p contains the
        # parameter product n^T p0 and cvxpy re-canonicalises on every solve.
        P = self.P = cp.Variable((N, 2), name="p")
        self.st = cp.Variable(N, nonneg=True, name="track_slack")
        self.sc = cp.Variable(N, nonneg=True, name="collision_slack")
        self.sb = cp.Variable(N, nonneg=True, name="bend_slack")
        self.p0 = cp.Parameter(2)                      # measured start position
        self.umax = cp.Parameter(nonneg=True)          # vmax*dt
        self.bend = cp.Parameter(nonneg=True)          # largest change of step, kappa_max*umax^2
        self.n = cp.Parameter((N, 2))                  # track normals at the iterate
        self.ntau = cp.Parameter(N)                    # n^T tau
        self.wl = cp.Parameter(N)
        self.wr = cp.Parameter(N)
        self.beta = cp.Parameter((N, 2))               # collision half-plane normals
        self.bq = cp.Parameter(N)                      # d_min - beta^T q
        self.sigma = cp.Parameter(2)                   # progress gradient, eq. (18)
        self.wobj = cp.Parameter((N, 2))               # alpha * mu_opp * beta, eq. (16)
        self.pm = cp.Parameter((N, 2))                 # trust-region centre
        self.r = cp.Parameter(nonneg=True)
        self.sw = cp.Parameter(nonneg=True)

        lat = cp.sum(cp.multiply(self.n, P), axis=1) - self.ntau
        # beta^T (q - p) >= d_min - sc   <=>   (d_min - beta^T q) + beta^T p <= sc.
        # beta is a unit vector, so ||q - p|| >= beta^T (q - p): satisfying the half-plane
        # guarantees the true distance constraint whenever the slack sc is zero.
        self.coll = self.bq + cp.sum(cp.multiply(self.beta, P), axis=1) <= self.sc
        cons = [cp.norm(P[0] - self.p0, 2) <= self.umax,
                cp.norm(P[1:] - P[:-1], 2, axis=1) <= self.umax,
                lat <= self.wl + self.st,
                -lat <= self.wr + self.st,
                self.coll,
                cp.norm(P[1] - 2 * P[0] + self.p0, 2) <= self.bend + self.sb[1],
                cp.norm(P[2:] - 2 * P[1:-1] + P[:-2], 2, axis=1) <= self.bend + self.sb[2:],
                cp.norm(P - self.pm, 2, axis=1) <= self.r]
        obj = cp.Minimize(-self.sigma @ P[N - 1] - cp.sum(cp.multiply(self.wobj, P))
                          + self.sw * (cp.sum(self.st) + cp.sum(self.sc) + cp.sum(self.sb)))
        self.prob = cp.Problem(obj, cons)
        if not self.prob.is_dpp():
            raise RuntimeError("QCLP is not DPP; it would be re-canonicalised every solve")

    def solve(self, verbose=False):
        self.prob.solve(solver=self.solver, warm_start=True, verbose=verbose)
        return self.prob.status


_QCLP_CACHE = {}


def get_qclp(N, solver="CLARABEL"):
    """Compiled subproblem for horizon N (first call ~0.3 s, afterwards free). Call once at
    node start-up so the first planning tick is not slowed by canonicalisation."""
    key = (N, solver)
    if key not in _QCLP_CACHE:
        _QCLP_CACHE[key] = _QCLP(N, solver)
    return _QCLP_CACHE[key]


# --------------------------------------------------------------------------- helpers
def predict_opponent(track, q0, prm, v_opp=None):
    """Constant speed along the centerline, keeping the current lateral offset.
    (The paper's MPC-baseline opponent model.)"""
    v = prm.vmax if v_opp is None else v_opp
    s = track.project(q0)
    lat = track.lateral(q0, s, 1.0)
    ss = s + v * prm.dt * np.arange(prm.N + 1)
    q = track.pos(ss) + lat * track.normal(ss)
    q[0] = np.asarray(q0, float).ravel()
    return q


def warm_start(track, p0, prm, frac=0.8, blend=5):
    """Follow the centerline at frac*vmax, blending the current lateral offset out over
    `blend` steps. The QCLP has slacks, so this need not be feasible."""
    p0 = np.asarray(p0, float).ravel()
    s = track.project(p0)
    lat = track.lateral(p0, s, 1.0)
    ss = s + frac * prm.vmax * prm.dt * np.arange(prm.N + 1)
    fade = np.maximum(0.0, 1.0 - np.arange(prm.N + 1) / max(blend, 1))
    p = track.pos(ss) + (lat * fade)[:, None] * track.normal(ss)
    p[0] = p0
    return p


def clip_steps(p, umax):
    """Pull a plan back inside ||p^k - p^{k-1}|| <= umax, walking forward from p[0].

    A car lagging its last plan leaves p[1] too far from the measured p[0]. If that gap
    exceeds umax + trust radius, the step bound and trust region cannot both hold."""
    p = p.copy()
    for k in range(1, len(p)):
        step = p[k] - p[k - 1]
        p[k] = p[k - 1] + step * min(1.0, umax / max(np.linalg.norm(step), 1e-9))
    return p





def shift_plan(plan, track, prm, p0=None):
    """Receding-horizon warm start: drop the first point of the previous plan and extend it
    by one step along the centerline. Better than `warm_start` once the loop is running."""
    plan = np.asarray(plan, float)
    s_end = track.project(plan[-1], track.project(plan[-2]), 2.0)
    lat = track.lateral(plan[-1], s_end, 1.0)
    s_new = s_end + prm.vmax * prm.dt
    nxt = track.pos(s_new) + lat * track.normal(s_new)
    out = np.vstack([plan[1:], nxt])
    out[0] = plan[1] if p0 is None else np.asarray(p0, float).ravel()
    return clip_steps(out, prm.umax)

def keep_out(q, prm):
    """(N, 2, 2) matrices A with d^T A d = 1 on the ellipse around each rival point q^k.

    A car is about three times longer than it is wide, so one circle either lets a car follow
    close enough to touch or keeps it too far out to the side to ever pass. The ellipse has
    half-axes d_min along the rival's direction of travel and d_lat across it."""
    circle = np.eye(2) / prm.d_min ** 2
    if prm.d_lat is None:
        return np.broadcast_to(circle, (len(q) - 1, 2, 2))
    step = np.diff(q, axis=0)
    size = np.linalg.norm(step, axis=1, keepdims=True)
    t = step / np.maximum(size, 1e-6)
    n = np.c_[-t[:, 1], t[:, 0]]
    A = (t[:, :, None] * t[:, None, :]) / prm.d_min ** 2 + (n[:, :, None] * n[:, None, :]) / prm.d_lat ** 2
    return np.where(size[:, None] > 1e-6, A, circle)          # a parked rival has no direction


def true_violations(track, p, q, prm, s_hint=None, window=None):
    """Violations of the ORIGINAL (non-linearised) constraints. Positive = violated [m]."""
    s, _, _, _, _, lat, wl, wr = track.frenet(p[1:], s_hint, window)
    tv = float(np.max(np.maximum(lat - wl, -lat - wr)))
    d = p[1:] - q[1:]
    dist = np.linalg.norm(d, axis=1)
    scale = np.sqrt(np.einsum('ki,kij,kj->k', d, keep_out(q, prm), d))
    cv = float(np.max(dist / np.maximum(scale, 1e-6) - dist))

    return tv, cv, s


def _merit(track, p, q, prm, s0, w_obj, window=None, gb=None, s0g=None):
    """L1 merit of the TRUE problem: -progress - sensitivity bonus + w * violations."""
    tv, cv, s = true_violations(track, p, q, prm, s0, window)
    ds = gb.ds_along(s0g, gb.project(p[-1], s0g, window))
    bonus = float(np.sum(w_obj * p[1:])) if w_obj is not None else 0.0
    return -ds - bonus + prm.merit_w * (max(tv, 0.0) + max(cv, 0.0)), ds


# --------------------------------------------------------------------------- inner OCP
def solve_inner_ocp(track, p_ego, q_traj, prm=Params(), theta0=None, alpha=0.0, mu=None, gb=None, s_hint=None):
    """One best response: problem (16) for a single player, opponent trajectory fixed.

    p_ego  : (2,)      measured start position of this player
    q_traj : (N+1, 2)  opponent positions over the horizon (q_traj[0] = now)
    theta0 : (N+1, 2)  initial guess; default = centerline warm start. In IBR / receding
                       horizon, pass this player's previous plan (see shift_plan).
    alpha  : aggressiveness gain of eq. (16); 0 = plain OCP against a fixed opponent.
    mu     : (N,) duals mu_j of the OPPONENT's collision constraints from its last solve
             (Result.mu of the opponent). None -> zeros.

    beta in the OBJECTIVE is frozen at theta0 for the whole solve, as in the paper;
    beta in the collision CONSTRAINT is re-linearised at every SCP iteration.
    """
    t_start = time.perf_counter()
    N = prm.N
    p0 = np.asarray(p_ego, float).ravel()
    q = np.asarray(q_traj, float)
    if q.shape != (N + 1, 2):
        raise ValueError(f"q_traj must be {(N + 1, 2)}, got {q.shape}")
    gb = track if gb is None else gb
    s0 = track.project(p0)
    s0g = gb.project(p0) if s_hint is None else gb.project(p0, s_hint, 3.0)
    # The window must reach past the far end of the horizon, or the projection of p^N gets
    # clamped to the window edge and the reported progress saturates.
    win = None if prm.proj_window is None else max(prm.proj_window, prm.umax * N + 1.0)
    p_m = warm_start(track, p0, prm) if theta0 is None else np.asarray(theta0, float).copy()
    p_m[0] = p0
    r = min(prm.trust0, prm.trust) if prm.trust is not None else None

    mu_in = np.zeros(N) if mu is None else np.asarray(mu, float).ravel()
    d0 = q[1:] - p_m[1:]
    A = keep_out(q, prm)
    Ad0 = np.einsum('kij,kj->ki', A, d0)
    beta_obj = Ad0 / np.maximum(np.linalg.norm(Ad0, axis=1, keepdims=True), 1e-9)   # ellipse normal
    w_obj = alpha * mu_in[:, None] * beta_obj          # frozen objective gradient
    w_merit = w_obj if alpha != 0.0 else None

    
    Q = get_qclp(N, prm.solver)
    Q.p0.value, Q.umax.value = p0, prm.umax
    Q.bend.value = prm.kappa_max * prm.umax ** 2
    Q.wobj.value, Q.sw.value = w_obj, prm.slack_w

    cur, _ = _merit(track, p_m, q, prm, s0, w_merit, win, gb, s0g)
    mu_out, beta_out, log, status = np.zeros(N), beta_obj, [], "not_run"

    for m in range(prm.max_iter):
        # ---- linearise at p_m
        _, tau, _, nrm, _, _, wl, wr = track.frenet(p_m[1:], s0, win)
        _, _, tN, _, kN, latN, _, _ = gb.frenet(p_m[N:N + 1], s0g, win)
        sigma = tN[0] / max(1.0 - kN[0] * latN[0], 0.1)     # eq. (18), clipped at the pole
        diff = q[1:] - p_m[1:]
        beta = diff / np.maximum(np.linalg.norm(diff, axis=1, keepdims=True), 1e-6)
        # n·(p − τ) = n·p − n·τ
        Q.n.value, Q.ntau.value = nrm, np.sum(nrm * tau, axis=1)
        Q.wl.value, Q.wr.value = wl, wr
        # Half-plane tangent to the ellipse: stays outside it, so the plan is safe when sc = 0.
        Ad = np.einsum('kij,kj->ki', A, diff)
        normal = Ad / np.maximum(np.linalg.norm(Ad, axis=1, keepdims=True), 1e-9)
        surface = diff / np.maximum(np.sqrt(np.sum(diff * Ad, axis=1, keepdims=True)), 1e-6)
        Q.beta.value = normal
        Q.bq.value = np.sum(normal * (surface - q[1:]), axis=1)

        Q.sigma.value = sigma
        Q.pm.value = p_m[1:]
        Q.r.value = r if r is not None else 1e4

        try:
            st_ = Q.solve(verbose=prm.verbose)
        except cp.SolverError as e:          # should not happen: the slacks keep it feasible
            status = f"solver_error: {e}"
            log.append(dict(it=m, status="solver_error"))
            break
        if st_ not in ("optimal", "optimal_inaccurate") or Q.P.value is None:
            log.append(dict(it=m, status=st_, trust=r))
            if r is not None and r > 1e-3:
                r *= 0.5                               # shrink and retry
                continue
            status = f"qclp_{st_}"
            break

        p_new = np.vstack([p0, np.asarray(Q.P.value)])
        new, ds = _merit(track, p_new, q, prm, s0, w_merit, win, gb, s0g)
        step = float(np.linalg.norm(p_new - p_m))
        if r is not None and new > cur + 1e-9 and step > prm.tol:
            r *= 0.5                                   # reject: the linearisation overshot
            log.append(dict(it=m, status="rejected", merit=new, cur=cur, trust=r))
            if r < 1e-2 * prm.umax:
                status = "stationary"
                break
            continue

        gain = cur - new
        p_m, cur = p_new, new
        dual = Q.coll.dual_value
        mu_out = np.zeros(N) if dual is None else np.maximum(np.asarray(dual).ravel(), 0.0)
        beta_out = beta
        if r is not None:
            r = min(2.0 * r, prm.trust)                # grow back after a good step
        log.append(dict(it=m, status=st_, ds=ds, step=step, trust=r, merit=new,
                        slack=float(np.sum(Q.st.value) + np.sum(Q.sc.value) + np.sum(Q.sb.value))))
        if step < prm.tol or (m > 0 and abs(gain) < 1e-2):
            status = "converged"
            break
    else:
        status = "max_iter"

    tv, cv, s = true_violations(track, p_m, q, prm, s0, win)
    return Result(p_m, np.diff(p_m, axis=0), gb.ds_along(s0g, gb.project(p_m[-1], s0g, win)), status, len(log),
                  tv, cv, mu_out, beta_out, time.perf_counter() - t_start, log, s0g)


# --------------------------------------------------------------------------- IBR
@dataclass
class IBRResult:
    p: list                      # [p1, p2], each (N+1, 2)
    mu: list                     # [mu1, mu2], each (N,)
    converged: bool
    solve_time: float = 0.0
    history: list = field(default_factory=list)
    res: list = field(default_factory=list)    # [Result, Result] from the last sweep


def ibr(track, x1, x2, prm=Params(), alpha=(0.0, 0.0), n_ibr=2, tol=2e-2,
        theta=(None, None), prm2=None, mu0=(None, None), gb=None, s_hint=(None, None)):
    """Iterated best response (Fig. 2): alternate the two best responses.

        theta_1^l = argmax s_1 + alpha_1 sum_k mu_2^{k,l-1} beta_12^kT p_1^k   (theta_2 fixed)
        theta_2^l = argmax s_2 + alpha_2 sum_k mu_1^{k,l}   beta_21^kT p_2^k   (theta_1 fixed)

    Player 1 is solved first, so player 1 is the ego car. Pass the previous tick's plans as
    `theta` for a receding-horizon warm start, and `prm2` if the opponent has other limits.

    The duals are clipped to prm.mu_max: with slack variables a binding constraint can
    return a dual as large as slack_w, which would swamp the progress term in (16).
    """
    t0 = time.perf_counter()
    prm2 = prm if prm2 is None else prm2
    P = [prm, prm2]
    x = [np.asarray(x1, float).ravel(), np.asarray(x2, float).ravel()]
    th = [warm_start(track, x[i], P[i]) if theta[i] is None else np.asarray(theta[i], float)
          for i in (0, 1)]
    mu = [np.zeros(prm.N) if mu0[i] is None else np.asarray(mu0[i], float) for i in (0, 1)]
    hist, res, conv = [], [None, None], False

    for l in range(n_ibr):
        d = [0.0, 0.0]
        for i in (0, 1):
            j = 1 - i
            r = solve_inner_ocp(track, x[i], th[j], P[i], theta0=th[i],
                                alpha=alpha[i], mu=mu[j], gb=gb, s_hint=s_hint[i])
            d[i] = float(np.linalg.norm(r.p - th[i]))
            th[i], mu[i], res[i] = r.p, np.clip(r.mu, 0.0, P[i].mu_max), r
        hist.append(dict(l=l, d1=d[0], d2=d[1], s1=res[0].s_progress, s2=res[1].s_progress,
                         st1=res[0].status, st2=res[1].status,
                         coll=max(res[0].max_coll_viol, res[1].max_coll_viol),
                         track=max(res[0].max_track_viol, res[1].max_track_viol),
                         t=res[0].solve_time + res[1].solve_time))
        if max(d) < tol:
            conv = True
            break
    return IBRResult(th, mu, conv, time.perf_counter() - t0, hist, res)


# --------------------------------------------------------- receding horizon wrapper
class RecedingHorizonIBR:
    """One planning tick of the receding-horizon loop: warm-start bookkeeping, the IBR
    sweep, and the feasibility check that decides whether the plan may be published.

    Deliberately free of ROS so it can be exercised offline (see sim_ibr.py).
    """

    def __init__(self, track, prm, prm_opp=None, alpha=(1.0, 1.0), n_ibr=1, ibr_tol=2e-2,
                 viol_tol=0.05, reset_dist=1.0, gb=None):
        self.track, self.prm = track, prm
        self.gb = gb
        self.prm_opp = prm if prm_opp is None else prm_opp
        self.alpha, self.n_ibr, self.ibr_tol = alpha, n_ibr, ibr_tol
        self.viol_tol = viol_tol            # extra violation allowed before refusing [m]
        self.reset_dist = reset_dist        # drop the warm start if the car jumped [m]
        self.prev = [None, None]            # last published plans
        self.mu = [None, None]
        self.s_gb = [None, None]
        self.x_last = [None, None]
        self.ticks = self.rejects = 0

    def reset(self):
        self.prev, self.mu = [None, None], [None, None]

    def _hint(self, i, x):
        """Last tick's raceline s, or None if the car moved further than the hint can reach."""
        if self.s_gb[i] is None or np.linalg.norm(x - self.x_last[i]) > 3.0:
            return None
        return self.s_gb[i]


    def _theta(self, i, x):
        """Previous plan shifted one step, or None if it is missing or stale."""
        prm = self.prm if i == 0 else self.prm_opp
        if self.prev[i] is None:
            return None
        if np.linalg.norm(np.asarray(x) - self.prev[i][1]) > self.reset_dist:
            return None                     # the car is not where we predicted: start fresh
        return shift_plan(self.prev[i], self.track, prm, p0=x)

    def step(self, x_ego, x_opp, q_opp=None):
        """Plan one tick. Returns a dict; `ok` says whether the ego plan is safe to publish.

        q_opp : optional (N+1,2) opponent prediction from somewhere else (a perception or
                prediction stack). Given one, this runs a SINGLE best response against it
                -- the paper's MPC baseline -- instead of an IBR sweep, and no game is
                played. Leave it None to run IBR with the opponent modelled as a player.
        """
        self.ticks += 1
        x = [np.asarray(x_ego, float).ravel(), np.asarray(x_opp, float).ravel()]
        if q_opp is not None:
            r = solve_inner_ocp(self.track, x[0], np.asarray(q_opp, float), self.prm,
                                theta0=self._theta(0, x[0]), alpha=0.0, gb=self.gb, s_hint=self._hint(0, x[0]))
            R = IBRResult([r.p, np.asarray(q_opp, float)], [r.mu, np.zeros(self.prm.N)],
                          True, r.solve_time, [], [r, r])
        else:
            R = ibr(self.track, x[0], x[1], self.prm, alpha=self.alpha, n_ibr=self.n_ibr,
                    tol=self.ibr_tol, theta=(self._theta(0, x[0]), self._theta(1, x[1])),
                    prm2=self.prm_opp, mu0=tuple(self.mu), gb=self.gb, s_hint=(self._hint(0, x[0]), self._hint(1, x[1])))

        # R.res = [r, r] in the solo branch, so the opponent keeps no hint there.
        self.s_gb = [R.res[0].s0g, None if q_opp is not None else R.res[1].s0g]
        self.x_last = x

        # How badly does the MEASURED state already violate the constraints? A plan is not
        # rejected for inheriting a violation it cannot undo within one horizon.
        gap0 = float(np.linalg.norm(x[0] - x[1]))
        lat0 = self.track.lateral(x[0], self.track.project(x[0]), 2.0)
        wl0, wr0 = self.track.width(self.track.project(x[0]))
        start_viol = max(self.prm.d_min - gap0, lat0 - float(wl0), -lat0 - float(wr0), 0.0)

        r0 = R.res[0]
        worst = max(r0.max_coll_viol, r0.max_track_viol)
        ok = worst <= max(self.viol_tol, start_viol)
        if ok:
            self.prev, self.mu = list(R.p), list(R.mu)
        else:
            self.rejects += 1
            self.reset()                    # do not warm-start from a plan we rejected
        return dict(ok=ok, plan=R.p[0], plan_opp=R.p[1], res=R.res, ibr=R,
                    start_viol=start_viol, worst_viol=worst, time=R.solve_time,
                    status=r0.status, progress=r0.s_progress)


# --------------------------------------------------------------------------- demo
def demo(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    prm = Params(N=15, dt=0.1, vmax=4.0, d_min=0.58, track_margin=0.29, proj_window=4.0)
    track = Track.from_json(path, margin=prm.track_margin)
    print(f"track length {track.L:.2f} m, grid {track.M} pts")
    get_qclp(prm.N)

    s1, s2 = 5.0, 5.7
    x1 = track.pos(s1)
    x2 = track.pos(s2) + 0.25 * track.normal(s2)

    q = predict_opponent(track, x2, prm, v_opp=3.0)
    ra = solve_inner_ocp(track, x1, q, prm)
    print(f"\n(a) inner OCP, fixed opponent : {ra.status:10s} {ra.iters:2d} it  "
          f"{ra.solve_time * 1e3:5.1f} ms  progress {ra.s_progress:.2f} m  "
          f"viol {ra.max_track_viol:+.3f}/{ra.max_coll_viol:+.3f}")

    out = {"(a) inner OCP, fixed opp": (ra.p, q)}
    for name, al in {"(b) IBR alpha=(0,0)": (0.0, 0.0),
                     "(c) IBR alpha=(1,1)": (1.0, 1.0),
                     "(d) IBR alpha=(3,3)": (3.0, 3.0)}.items():
        R = ibr(track, x1, x2, prm, alpha=al, n_ibr=4)
        out[name] = (R.p[0], R.p[1])
        print(f"\n{name}: converged={R.converged} in {R.solve_time * 1e3:.0f} ms")
        for h in R.history:
            print("   l={l} |d1|={d1:.3f} |d2|={d2:.3f}  s1={s1:5.2f} s2={s2:5.2f}  "
                  "viol {track:+.3f}/{coll:+.3f}  {t:.3f}s  [{st1}/{st2}]".format(**h))

    fig, axs = plt.subplots(1, 4, figsize=(22, 5.5))
    sg = np.linspace(0, track.L, 1200)
    c, nn = track.pos(sg), track.normal(sg)
    wl, wr = track.width(sg)
    for ax, (name, (a, b)) in zip(axs, out.items()):
        ax.plot(*c.T, "k--", lw=.4)
        ax.plot(*(c + nn * wl[:, None]).T, "g", lw=1)
        ax.plot(*(c - nn * wr[:, None]).T, "g", lw=1)
        ax.plot(*a.T, "b.-", label="player 1 (ego)")
        ax.plot(*b.T, "r.-", label="player 2" if "IBR" in name else "opp prediction")
        ax.plot(*a[0], "bo", ms=9)
        ax.plot(*b[0], "ro", ms=9)
        ax.set_aspect("equal")
        ax.set_title(name, fontsize=10)
        ax.legend(fontsize=8)
        mid = (x1 + x2) / 2
        ax.set_xlim(mid[0] - 3.5, mid[0] + 3.5)
        ax.set_ylim(mid[1] - 3.5, mid[1] + 3.5)
    plt.tight_layout()
    plt.savefig("ibr_demo.png", dpi=110)
    print("\nsaved ibr_demo.png")


if __name__ == "__main__":
    demo(sys.argv[1] if len(sys.argv) > 1 else "global_waypoints.json")
