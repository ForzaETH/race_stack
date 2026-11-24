"""
Defines the Gaussian Process models to estimate the opponent trajectory,
both for the deviation off the raceline d(s) as well as for the velocity v(s).

Pure Python (no ROS). Based on https://github.com/ForzaETH/predictive-spliner.
"""

from dataclasses import dataclass
from typing import cast
from linear_operator.operators import LinearOperator
import numpy as np
import torch
import gpytorch
from gpytorch.distributions import MultivariateNormal
from gpytorch.kernels import Kernel, MaternKernel, ScaleKernel, AdditiveKernel
from gpytorch.likelihoods import FixedNoiseGaussianLikelihood
from gpytorch.constraints import Interval
from gpytorch.mlls import ExactMarginalLogLikelihood
from gpytorch.priors import LogNormalPrior


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


NEXT_LAP = -1  # lap index never used in training: stands for "the next lap"


def _lap_deviation_prior(typical_std: float) -> LogNormalPrior:
    """
    Prior on a lap-deviation variance: typically typical_std², within a factor of ~2.7 in std (95 %).
    With only a few laps, the lap-to-lap spread can't be estimated from the data alone: this keeps it
    from collapsing to 0 (laps that happen to agree) or absorbing the whole line (laps sharing an offset).
    """
    return LogNormalPrior(loc=2 * np.log(typical_std), scale=1.0)


class SameLapKernel(Kernel):
    """1 for two points from the same lap, 0 otherwise. Input: the lap index."""

    def forward(self, x1: torch.Tensor, x2: torch.Tensor, diag: bool = False, **params) -> torch.Tensor:
        lap1, lap2 = x1[..., 0], x2[..., 0]
        if diag:  # only the variance at each point is needed
            return (lap1 == lap2).to(x1.dtype)
        return (lap1.unsqueeze(-1) == lap2.unsqueeze(-2)).to(x1.dtype)


class TrackGP(gpytorch.models.ExactGP):
    """Exact (non sparse) GP over track position s (embedded on a circle). """

    def __init__(
        self,
        X: torch.Tensor,
        y: torch.Tensor,
        kernel: Kernel,
        likelihood: FixedNoiseGaussianLikelihood,
    ) -> None:
        super().__init__(X, y, likelihood)

        self.mean_module = gpytorch.means.ZeroMean()
        self.covar_module = kernel

    def forward(self, *inputs: torch.Tensor, **_kwargs) -> MultivariateNormal:
        X = inputs[0]
        mean = cast(torch.Tensor, self.mean_module(X))
        covariance = cast(LinearOperator, self.covar_module(X))
        return MultivariateNormal(mean, covariance)


class OpponentTrajectoryGP:
    """GP model of the opponent's line d(s) and speed vs(s) on a closed track, with per-lap deviations."""

    def __init__(self, track_length: float, s_ref: np.ndarray, v_ref: np.ndarray) -> None:
        """
        s_ref, v_ref: the ego's raceline track positions [m] and speed profile [m/s]
                      (obtained from /global_waypoints)
        track_length: length of the closed track [m]
        """
        self.track_length = track_length
        self.s_ref = np.asarray(s_ref)
        self.v_ref = np.asarray(v_ref)

        # Built in fit() once data is known
        self.gp_d: TrackGP | None = None
        self.gp_v: TrackGP | None = None
        self.alpha_pace: float | None = None

    @staticmethod
    def _make_kernel_d() -> Kernel:
        """
        Kernel for the opponent's lateral offset d(s) from the ego (our) raceline:
        the opponent's mean line + each lap's own smooth deviation from it.
        """

        # nu=2.5: smooth path (C2 continuous)
        # length_scale: track distance [m] over which the line changes substantially
        line_shape = MaternKernel(nu=2.5, active_dims=(0, 1), lengthscale_constraint=Interval(0.2, 10.0))
        line_shape.lengthscale = torch.tensor(2.0)

        # How far the opponent's line can be from ours, as a variance [m²]
        line = ScaleKernel(line_shape, outputscale_constraint=Interval(1e-2, 0.5))
        line.outputscale = torch.tensor(0.1)

        # Each lap deviates smoothly from the line (e.g. a corner run wide), shared within the lap only.
        # length_scale: track distance [m] over which one lap's deviation persists
        lap_shape = MaternKernel(nu=2.5, active_dims=(0, 1), lengthscale_constraint=Interval(1.0, 50.0))
        lap_shape.lengthscale = torch.tensor(5.0)

        # How much a lap deviates from the line, as a variance [m²]
        # Prior: typically ~5 cm (2–14 cm).
        lap_deviation = ScaleKernel(
            lap_shape * SameLapKernel(active_dims=(2,)),
            outputscale_prior=_lap_deviation_prior(typical_std=0.05),
            outputscale_constraint=Interval(1e-4, 1e-1),
        )
        lap_deviation.outputscale = torch.tensor(0.05**2)

        return AdditiveKernel(line, lap_deviation)

    @staticmethod
    def _make_likelihood_d(sensor_noise_var: torch.Tensor) -> FixedNoiseGaussianLikelihood:
        """Noise model for d(s): known sensor noise per point [m²]. Lap-to-lap variation is in the kernel."""
        return FixedNoiseGaussianLikelihood(noise=sensor_noise_var, learn_additional_noise=False)

    @staticmethod
    def _make_kernel_v() -> Kernel:
        """
        Kernel for the log residual of the opponent's speed, y(s) = log(vs) - log(α·v_ref(s)).
        Speed is recovered as vs(s) = α·v_ref(s)·exp(y).

        We assume the opponent's speed is roughly our speed profile scaled by its pace α,
        which gives a good prior on the speed profile.

        y(s) = smooth pace deviation + each lap's own smooth pace deviation.
        """

        # nu=2.5: smooth speed profile (C2 continuous)
        # length_scale: track distance [m] over which the pace deviation changes substantially
        pace_shape = MaternKernel(nu=2.5, active_dims=(0, 1), lengthscale_constraint=Interval(0.5, 15.0))
        pace_shape.lengthscale = torch.tensor(3.0)

        # How much the opponent's pace deviates from our scaled profile (log-variance)
        pace = ScaleKernel(pace_shape, outputscale_constraint=Interval(1e-3, 0.1))
        pace.outputscale = torch.tensor(0.02)

        # Each lap's own smooth pace deviation (e.g. a slower corner exit), shared within the lap only
        lap_shape = MaternKernel(nu=2.5, active_dims=(0, 1), lengthscale_constraint=Interval(1.0, 50.0))
        lap_shape.lengthscale = torch.tensor(5.0)

        # How much a lap's pace deviates (log-variance)
        # Prior: typically ~16 % of speed, at most 30 %. Tuned on the real ROS1 bags (train on 2 laps, predict
        # the 3rd): with 2 laps of history the lap-to-lap spread is barely identifiable from data, and an 8 %
        # prior left the band covering only ~75 % of the real next laps (now ~96 % when trailing).
        lap_deviation = ScaleKernel(
            lap_shape * SameLapKernel(active_dims=(2,)),
            outputscale_prior=_lap_deviation_prior(typical_std=0.16),
            outputscale_constraint=Interval(1e-5, 0.3**2),
        )
        lap_deviation.outputscale = torch.tensor(0.16**2)

        return AdditiveKernel(pace, lap_deviation)

    @staticmethod
    def _make_likelihood_v(sensor_noise_var: torch.Tensor) -> FixedNoiseGaussianLikelihood:
        """Noise model for the speed residual: known sensor noise (log-variance). Lap-to-lap variation is in the kernel."""
        return FixedNoiseGaussianLikelihood(noise=sensor_noise_var, learn_additional_noise=False)

    def embed_on_circle(self, s: np.ndarray) -> torch.Tensor:
        """
        Map track position s [m] onto a circle with circumference track_length,
        so that the GP input wraps around at the start/finish line (s=0 and s=L coincide).
        """
        radius = self.track_length / (2 * np.pi)
        angle = 2 * np.pi * np.asarray(s, dtype=float) / self.track_length
        points = np.column_stack([radius * np.cos(angle), radius * np.sin(angle)])
        return torch.as_tensor(points, dtype=torch.float64)

    def _inputs(self, s: np.ndarray, lap: np.ndarray) -> torch.Tensor:
        """GP input: s on the circle (columns 0, 1) and the lap index (column 2)."""
        lap_column = torch.as_tensor(np.asarray(lap, dtype=float), dtype=torch.float64).reshape(-1, 1)
        return torch.cat([self.embed_on_circle(s), lap_column], dim=1)

    @staticmethod
    def _train(model: TrackGP, X: torch.Tensor, y: torch.Tensor, max_iter: int = 100) -> None:
        """
        Fit the kernel hyperparameters by maximising the marginal likelihood,
        what sklearn's fit() did internally. Uses L-BFGS, as sklearn did.
        """
        model.train()  # set training mode
        mll = ExactMarginalLogLikelihood(model.likelihood, model)
        optimizer = torch.optim.LBFGS(model.parameters(), max_iter=max_iter, line_search_fn="strong_wolfe")

        def closure() -> torch.Tensor:
            optimizer.zero_grad()
            loss = -cast(torch.Tensor, mll(model(X), y))
            loss.backward()
            return loss

        optimizer.step(closure)
        model.eval()  # prediction mode

    def fit(
            self,
            s: np.ndarray,
            d: np.ndarray,
            vs: np.ndarray,
            lap: np.ndarray,
            d_noise_var: float | np.ndarray,
            vs_noise_var: float | np.ndarray) -> None:
        """
        Fit both GPs to opponent observations.

        s, d, vs: opponent track position [m], lateral offset [m] and speed [m/s] per observation
        lap: lap index per observation (integers >= 0); points of the same lap share one deviation
        d_noise_var, vs_noise_var: sensor noise variance per observation [m²], [m²/s²] (scalar or array)
        """
        s = np.asarray(s, dtype=float) % self.track_length
        d = np.asarray(d, dtype=float)
        vs = np.asarray(vs, dtype=float)
        if np.any(vs <= 0):
            raise ValueError("vs must be positive (log speed model)")

        # Track position on the circle + lap index
        X = self._inputs(s, lap)
        n = len(s)

        # Lateral offset GP: zero prior mean => the opponent drives on our raceline
        d_noise = torch.as_tensor(np.broadcast_to(d_noise_var, (n,)).copy(), dtype=torch.float64)
        y_d = torch.as_tensor(d)
        self.gp_d = TrackGP(X, y_d, self._make_kernel_d(), self._make_likelihood_d(d_noise)).double()
        self._train(self.gp_d, X, y_d)

        # Use interpolation to handle the start / finish
        v_ref_at_s = np.interp(s, self.s_ref, self.v_ref, period=self.track_length)

        log_ratio = np.log(vs) - np.log(v_ref_at_s)    # log(vs / v_ref)
        log_alpha = np.mean(log_ratio)                 # the constant offset, average log speed ratio
        self.alpha_pace = float(np.exp(log_alpha))
        y_v = torch.as_tensor(log_ratio - log_alpha)   # what's left after removing the offset (mean 0)

        # Var[log vs] ≈ Var[vs] / v², with v the expected speed α·v_ref at s, not the measured one:
        # dividing by the measured speed would give outliers with a too-high speed the least noise
        expected_vs = self.alpha_pace * v_ref_at_s
        log_vs_noise = torch.as_tensor(np.broadcast_to(vs_noise_var, (n,)) / expected_vs**2, dtype=torch.float64)
        self.gp_v = TrackGP(X, y_v, self._make_kernel_v(), self._make_likelihood_v(log_vs_noise)).double()
        self._train(self.gp_v, X, y_v)

    def _predict_line(self, model: TrackGP, s: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
        """
        For the next lap at s: mean, epistemic variance of the line, and the lap-deviation variance.
        The next lap's deviation is unknown (a fresh draw), so it adds its full variance.
        Sensor noise is not included: it is about our measurements, not where the opponent will be.
        """
        X = self._inputs(s, np.full(len(s), NEXT_LAP))
        with torch.no_grad():  # no gradients needed for prediction
            prediction = cast(MultivariateNormal, model(X))
            mean, total_var = prediction.mean.numpy(), prediction.variance.numpy()
        lap_deviation = cast(ScaleKernel, cast(AdditiveKernel, model.covar_module).kernels[1])
        lap_var = float(lap_deviation.outputscale.detach())
        return mean, np.clip(total_var - lap_var, 0.0, None), lap_var

    def predict(self, s: np.ndarray, n_std: float = 2.0) -> OpponentPrediction:
        """
        Predict the opponent's lateral offset and speed on its next lap, at track positions s [m].

        n_std: width of the speed band in standard deviations (log space)
        """
        if self.gp_d is None or self.gp_v is None or self.alpha_pace is None:
            raise RuntimeError("call fit() before predict()")

        s = np.asarray(s, dtype=float) % self.track_length

        # Lateral offset: band = line uncertainty (epistemic) + lap-to-lap variation
        d_mean, d_var_line, d_var_lap = self._predict_line(self.gp_d, s)

        # Speed: predict the log residual y, then map back with vs = α·v_ref·exp(y)
        y_mean, y_var_line, y_var_lap = self._predict_line(self.gp_v, s)
        y_std = np.sqrt(y_var_line + y_var_lap)
        v_ref_at_s = np.interp(s, self.s_ref, self.v_ref, period=self.track_length)
        vs_scale = self.alpha_pace * v_ref_at_s

        return OpponentPrediction(
            s=s,
            d_mean=d_mean,
            d_std=np.sqrt(d_var_line + d_var_lap),
            d_std_epistemic=np.sqrt(d_var_line),
            vs_median=vs_scale * np.exp(y_mean),
            vs_lower=vs_scale * np.exp(y_mean - n_std * y_std),
            vs_upper=vs_scale * np.exp(y_mean + n_std * y_std),
        )
