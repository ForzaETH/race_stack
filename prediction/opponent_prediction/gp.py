"""
Defines the Gaussian Process models to estimate the opponent trajectory,
both for the deviation off the raceline d(s) as well as for the velocity v(s).

Pure Python (no ROS). Based on https://github.com/ForzaETH/predictive-spliner.
"""

from dataclasses import dataclass
from typing import cast
import numpy as np
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import Kernel, Matern, WhiteKernel, ConstantKernel


@dataclass
class OpponentPrediction:
    """Predicted opponent trajectory at track positions s."""
    s: np.ndarray                # track position [m]
    d_mean: np.ndarray           # lateral offset from our raceline [m]
    d_std: np.ndarray            # total std: epistemic + lap-to-lap [m] (the planning band)
    d_std_epistemic: np.ndarray  # epistemic std only [m] (shrinks with data)
    vs_median: np.ndarray        # speed [m/s]
    vs_lower: np.ndarray         # speed band, n_std below [m/s]
    vs_upper: np.ndarray         # speed band, n_std above [m/s]


def _predict_with_std(gp: GaussianProcessRegressor, X: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """GaussianProcessRegressor.predict with return_std=True, typed as (mean, std)."""
    return cast(tuple[np.ndarray, np.ndarray], gp.predict(X, return_std=True))


class OpponentTrajectoryGP:
    def __init__(self, track_length: float, s_ref: np.ndarray, v_ref: np.ndarray) -> None:
        """
        s_ref, v_ref: the ego's raceline track positions [m] and speed profile [m / s]
                      (obtained from /global_waypoints)
        track_length: length of the closed track
        """

        self.track_length = track_length
        self.s_ref = np.asarray(s_ref)
        self.v_ref = np.asarray(v_ref)

        # Define the kernels for the two GPs
        self.K_d = self._make_kernel_d()
        self.K_v = self._make_kernel_v()

        self.gp_d = None
        self.gp_v = None
        self.alpha_pace = None

    def _make_kernel_d(self) -> Kernel:
        """
        Kernel for the opponent's lateral offset d(s) from the ego (our) raceline.

        d(s) = smooth line (Constant * Matern) + lap-to-lap variance of the opp car (White)
        """

        # How far the opponent's line can be from ours, as a variance [m²] (can use track width as an upper bound)
        line_variance = ConstantKernel(0.1, constant_value_bounds=(1e-2, 0.5))

        # nu=2.5: smooth path (C2 continuous)
        # length_scale: track distance [m] over which the line changes substantially
        line_shape = Matern(length_scale=2.0, length_scale_bounds=(0.2, 10.0), nu=2.5)

        # How differently the opponent drives the same spot each lap [m^2]
        lap_variation = WhiteKernel(noise_level=1e-3, noise_level_bounds=(1e-4, 1e-1))

        return line_variance * line_shape + lap_variation

    def _make_kernel_v(self) -> Kernel:
        """
        Kernel for the log residual of the opponent's speed, y(s) = log(vs) - log(α·v_ref(s)).
        The idea is that the opponent's speed is simply scaled by the ego speed: v(s) ≈ α·v_ref(s)
        This allows us to encode much better priors.

        y(s) = log(vs) - log(α·v_ref(s)).

        We can then recover vs as

        vs(s) = α·v_ref(s)·exp(y).
        """

        # How much the opponent's pace deviates from our scaled profile (log-variance)
        pace_variance = ConstantKernel(0.02, constant_value_bounds=(1e-3, 0.1))

        # nu=2.5: smooth speed profile (C2 continuous)
        # length_scale: track distance [m] over which the pace deviation changes substantially
        pace_shape = Matern(length_scale=3.0, length_scale_bounds=(0.5, 15.0), nu=2.5)

        # How differently the opponent's speed is at the same spot each lap (log-variance)
        lap_variation = WhiteKernel(noise_level=1e-3, noise_level_bounds=(1e-5, 1e-2))

        return pace_variance * pace_shape + lap_variation

    def embed_on_circle(self, s: np.ndarray) -> np.ndarray:
        """
        Map track position s [m] onto a circle with circumference track_length,
        so that the GP input wraps around at the start/finish line (s=0 and s=L coincide).

        The kernel still uses the straight line distance, which doesn't change much at close range.
        Far apart points are pulled closer, but their correlation is anyways 0.
        """

        radius = self.track_length / (2 * np.pi)
        angle = 2 * np.pi * np.asarray(s) / self.track_length
        return np.column_stack([radius * np.cos(angle), radius * np.sin(angle)])

    def fit(
            self,
            s: np.ndarray,
            d: np.ndarray,
            vs: np.ndarray,
            d_noise_var: float | np.ndarray,
            vs_noise_var: float | np.ndarray) -> None:
        """
        Fit both GPs to opponent observations.

        s, d, vs: opponent position, lateral offset [m] and speed [m/s]
        d_noise_var, vs_noise_var: measurement noise variance [m²], [m²/s²]
        """

        s = np.asarray(s, dtype=float) % self.track_length
        d = np.asarray(d, dtype=float)
        vs = np.asarray(vs, dtype=float)

        # Do not forget to embed onto a circle
        X = self.embed_on_circle(s)

        # Lateral offset GP: prior mean => opponent drives on the same raceline as the ego
        self.gp_d = GaussianProcessRegressor(kernel=self.K_d, alpha=d_noise_var, normalize_y=False)
        self.gp_d.fit(X, d)

        # Use interpolation to handle the start / finish
        v_ref_at_s = np.interp(s, self.s_ref, self.v_ref, period=self.track_length)

        log_ratio = np.log(vs) - np.log(v_ref_at_s)             # log(vs / v_ref)
        log_alpha = np.mean(log_ratio)                          # the constant offset, average log speed ratio
        self.alpha_pace = float(np.exp(log_alpha))
        y = log_ratio - log_alpha                               # what's left after removing the offset (mean 0)

        # Var[log vs] ≈ Var[vs] / vs²
        log_vs_noise_var = np.asarray(vs_noise_var) / vs**2
        self.gp_v = GaussianProcessRegressor(kernel=self.K_v, alpha=log_vs_noise_var, normalize_y=False)
        self.gp_v.fit(X, y)

    def predict(self, s: np.ndarray, n_std: float = 2.0) -> OpponentPrediction:
        """
        Predict the opponent's lateral offset and speed at track positions s [m].

        n_std: width of the speed band in standard deviations (log space)
        """
        if self.gp_d is None or self.gp_v is None or self.alpha_pace is None:
            raise RuntimeError("call fit() before predict()")

        s = np.asarray(s, dtype=float) % self.track_length
        X = self.embed_on_circle(s)

        # Lateral offset. return_std includes the lap-to-lap White term but not the sensor noise (alpha)
        d_mean, d_std = _predict_with_std(self.gp_d, X)

        # Epistemic part: remove the fitted lap-to-lap variance (WhiteKernel part)
        lap_var_d = self.gp_d.kernel_.get_params()["k2__noise_level"]
        d_std_epistemic = np.sqrt(np.clip(d_std**2 - lap_var_d, 0.0, None))

        # Speed: predict the log residual y, then map back with vs = α·v_ref·exp(y)
        y_mean, y_std = _predict_with_std(self.gp_v, X)
        v_ref_at_s = np.interp(s, self.s_ref, self.v_ref, period=self.track_length)
        vs_scale = self.alpha_pace * v_ref_at_s

        return OpponentPrediction(
            s=s,
            d_mean=d_mean,
            d_std=d_std,
            d_std_epistemic=d_std_epistemic,
            vs_median=vs_scale * np.exp(y_mean),
            vs_lower=vs_scale * np.exp(y_mean - n_std * y_std),
            vs_upper=vs_scale * np.exp(y_mean + n_std * y_std),
        )
