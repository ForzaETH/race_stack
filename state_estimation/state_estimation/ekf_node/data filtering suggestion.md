# Data Filtering Suggestion: Innovation Gating for IMU and VIO Updates

Context: single-track EKF with wheel-speed-driven Pacejka forces (`single_track_wheel_model`), fusing IMU (gyro + accelerometer) and VIO body-frame velocity.

## 1. Why gating is needed

A Kalman filter assumes every measurement is the truth plus small Gaussian noise of known size R. The gain K depends only on the nominal uncertainties P and R — **not on whether a specific measurement is plausible**. A measurement off by 5 m/s pulls the state as hard, proportionally, as one off by 5 cm/s.

Consequences of not gating:

- **Overconfidence.** Every update shrinks P, so after absorbing a bad measurement the filter is *more* confident in a wrong state, and later good measurements pull it back more slowly.
- **Persistence.** Slow states (IMU biases, wheel-speed scale `s_w`, disturbances `d_x, d_y, d_psi`) have small process noise; errors absorbed there last a long time.

## 2. The test

Before each update, compute how surprising the measurement is relative to the expected uncertainty:

$$
\mathbf{y} = \mathbf{z} - h(\mathbf{x}), \qquad
S = HPH^\top + R, \qquad
d^2 = \mathbf{y}^\top S^{-1}\mathbf{y}
$$

- **y**: innovation (measurement minus prediction).
- **S**: expected spread of the innovation (filter uncertainty + sensor noise).
- **d²**: Mahalanobis distance — "how many standard deviations off", in multiple dimensions.

If the model and noise assumptions hold, d² ~ χ² with m degrees of freedom (m = measurement dimension). Reject if d² exceeds the chosen quantile:

| Measurement | m | 99% threshold |
|---|---|---|
| Gyro z | 1 | 6.63 |
| Accelerometer (a_x, a_y) | 2 | 9.21 |
| VIO velocity (v_x, v_y) | 2 | 9.21 |
| Full IMU (theta, gyro, a_x, a_y) | 4 | 13.28 |

The normalization by S is essential: a 0.5 m/s disagreement is normal when P is large (after coasting) and alarming when the filter is tightly converged. A fixed threshold on the raw error cannot distinguish these.

## 3. Failure modes per sensor

### VIO (sensor failures — the measurement itself is wrong)

- Tracking loss or degradation: motion blur in fast corners, featureless walls, glare, repetitive textures (barriers, track tiles).
- Moving objects (other cars, people) treated as static scenery.
- Re-initialization jumps: velocity/pose discontinuities after a VIO reset.
- Delay mismatch: a measurement applied at the wrong timestamp during a hard transient shows up as a large innovation.

### IMU

- Impacts: curb strikes, bumps, contact with other cars → acceleration spikes far outside nominal noise.
- **Model violations:** the accelerometer is compared against the *tire model's* predicted force. During wheelspin, lockup, or a sudden grip change, the measurement is correct but the model is wrong. Without a gate, the filter "fixes" the disagreement by corrupting `v_x`, `s_w`, or `b_ax`.

The gate tests consistency between measurement and model, not sensor correctness — so it also catches moments where the model assumptions break.

## 4. Implementation notes

- **Gate components that fail independently separately.** The gyro is rarely an outlier; the accelerometer often is. Testing the 4-D IMU block jointly discards a good gyro reading together with a bad accelerometer spike. Gate gyro and accelerometer separately.
- **Prevent permanent lockout.** If the filter drifts, every measurement looks surprising and all get rejected, so it never recovers. Count consecutive rejections per sensor; after N (e.g. 0.5 s worth), inflate P or force-accept the measurement to re-anchor.
- **Soft gating (alternative).** Instead of rejecting, inflate R in proportion to the surprise (Huber-style weighting). Moderate outliers are down-weighted rather than discarded, giving smoother behaviour near the threshold.

Minimal sketch:

```python
import numpy as np
from scipy.stats import chi2

def gated_update(ekf, z, h, H, R, alpha=0.99, max_reject=50, state={'n': 0}):
    y = z - h
    S = H @ ekf.P @ H.T + R
    d2 = float(y @ np.linalg.solve(S, y))
    if d2 > chi2.ppf(alpha, df=len(z)) and state['n'] < max_reject:
        state['n'] += 1
        return False, d2                      # rejected
    state['n'] = 0
    K = ekf.P @ H.T @ np.linalg.inv(S)
    ekf.x = ekf.x + K @ y
    I_KH = np.eye(len(ekf.x)) - K @ H
    ekf.P = I_KH @ ekf.P @ I_KH.T + K @ R @ K.T   # Joseph form
    return True, d2
```

(Use a separate rejection counter per sensor; the mutable default here is only for brevity.)

## 5. Using d² as a diagnostic

Log d² (the normalized innovation squared, NIS) for every update. For a well-tuned filter its mean over a run should be ≈ m (2 for VIO, 4 for full IMU).

- **Mean well above m:** R or Q too small, tire fit off, or a timing error. Fix the cause, not the threshold.
- **Mean well below m:** noise set too large; the filter is ignoring information.
- **Rejections clustered in specific corners:** usually the tire model in that operating regime — feed this back into the parameter fitting.
