# TODO: Understanding and Using the Single-Track Wheel-Speed EKF

You will work with `single_track_wheel_model`, an extension of the existing `single_track_model` in
`state_estimation/state_estimation/ekf_node/models.py`. The new model drives the longitudinal tire
force from the **measured wheel speed** (VESC) instead of the commanded acceleration, and fuses IMU
and VIO.

Work through the stages in order. Each stage lists a few suggested outputs: they are optional, but
plots and short notes along the way make it much easier to discuss progress with your supervisor.

**Background reading (keep at hand):**
- R. Rajamani, *Vehicle Dynamics and Control*, Ch. 2 (bicycle model) and Ch. 13 (tire models).
- H. Pacejka, *Tire and Vehicle Dynamics*, Ch. 1 and 4 (slip definitions, Magic Formula).
- S. Thrun et al., *Probabilistic Robotics*, Ch. 3 (Kalman filter, EKF) — refresher if needed.
- `data filtering suggestion.md` (gating) — Stage 4.

**Setup**
- [ ] Clone the race stack, check out `ros2-humble`, apply `single-track-wheel-ekf.patch` (`git am`), build.
- [ ] Run the stack once with the *existing* `single_track_model` (`custom_ekf:=true model_type:=single_track_model`) to see the baseline.
- [ ] Record a rosbag on the track with at least: `/imu`, `/odom` (VESC), `/basalt/odom`, `/drive`, `/tf_static`. Drive a few laps with straights, corners, and some hard accelerations and braking. You will reuse this bag in every stage.

---

## Stage 1 — Understand the model and the measurements

This stage focuses on **what the model predicts** and **what each sensor measures**. The
filter-specific parts (covariances, gating) come later.

### 1.1 Start from what you know
- [ ] Read `single_track_model` and write down, in your own notation: states, inputs, and the three equations of motion (v̇_x, v̇_y, ṙ).
- [ ] Explain where the commanded acceleration enters that model and why that is a weak point (hint: what happens if the car is commanded to accelerate but the wheels spin?).

### 1.2 What is new
- [ ] Read `single_track_wheel_model` (docstring first). List every difference from `single_track_model`: state vector, input vector, how F_x is computed, combined slip, CoG vs. `base_link`.
- [ ] **Slip ratio.** Derive κ = (r_e·ω − v_x)/v_x from the definition of free rolling. Explain what κ > 0 and κ < 0 mean physically. In the code, find where `v_w` and `s_w` appear and explain why `s_w · v_w` plays the role of r_e·ω.
- [ ] **Slip angles.** Derive α_f and α_r from the velocity of each axle. Check the signs against the code.
- [ ] **Magic Formula.** Plot the lateral curves (front and rear, using the stack's `pacejka/<floor>/default.yaml`) and a longitudinal curve for some plausible values (e.g. B=10, C=1.6, D=1, E=0). Mark the linear region and the peak. What does each of B, C, D, E change?
- [ ] **Friction ellipse.** Explain in two sentences why a tire cannot deliver its full lateral and full longitudinal force at the same time. Find the saturation in the code.
- [ ] **Load transfer.** Explain why braking increases the front normal load. Identify `ax_lt` in the input and find where it is computed (`ImuSensor` in `sensors.py`). Why is it low-pass filtered?
- [ ] **Kinematic blend.** Below ~2 m/s the model blends to a kinematic model. Explain why the dynamic model is unreliable at low speed (hint: look at the denominators).

### 1.3 Measurements
- [ ] For the IMU measurement `Hx_imu`, write each of the four rows as an equation and explain each term: why does the accelerometer row contain −p_x·r² and the ṙ term? What are `b_gz`, `b_ax`, `b_ay`?
- [ ] For `Hx_vio`, explain the mapping from CoG to `base_link`. At v_x = 4 m/s and r = 2 rad/s, compute the difference between v_y at the CoG and at the rear axle. Is it negligible?
- [ ] Explain why the VESC speed is an **input** in this model and why it must **not** also be fused as a measurement.

### 1.4 Open-loop simulation (no filter)
`models.py` only needs `casadi` and `numpy`, so you can use it outside ROS.
- [ ] Write a script that reads the bag, builds u = [steer, v_w, ax_lt] at each time step, and integrates the model with `model.f(x, u, dt)` starting from the first VIO velocity.
- [ ] Plot simulated vs. VIO v_x, v_y and yaw rate vs. IMU gyro. Expect drift — explain where and why the model diverges (which parts of the lap, which states).

*Suggested output (optional):* your derivations, the Magic Formula plots, and the open-loop comparison with a few lines of interpretation.

---

## Stage 2 — Run the filter with hardcoded longitudinal tire parameters

The longitudinal Magic Formula parameters `Bx, Cx, Dx, Ex` have **not been identified yet**. The node
refuses to start without them, so for now you hardcode plausible values and study how much they matter.

- [ ] Add placeholder values to `stack_master/config/<car>/pacejka/<floor>/default.yaml`, e.g. `Bx: 10.0, Cx: 1.6, Dx: 1.0, Ex: 0.0`, with a comment marking them as **not identified**.
- [ ] Fill in the remaining TODOs in `launch/ekf_node_launch.py`: `imu_px`/`imu_py` (measure where the VESC IMU sits relative to the CoG) and check in the URDF/TF that `base_link` is at the rear axle.
- [ ] Set `gate_prob: 0.0` (gating off) for this stage.
- [ ] Replay your bag with `custom_ekf:=true model_type:=single_track_wheel_model`. Record `/state_estimation/odom` and plot it against VIO and the old `single_track_model`.
- [ ] Log and plot the extra states: `s_w`, the biases, and the disturbances `d_x, d_y, d_psi` (you will need to publish or log `ekf.x` — add this to the node).
- [ ] **Sensitivity study.** Rerun with `Bx` × 0.5 and × 2. How do `d_x` and the accelerometer innovations change? Explain why `d_x` absorbs the error of a wrong tire model.
- [ ] Relate `s_w` to the VESC `speed_to_erpm_gain`. Does the converged `s_w` suggest the gain is off? By how much?

*Suggested output (optional):* plots of the filter output and the extra states for the three `Bx` settings, and a few lines on what the disturbance states tell you about the model quality.

---

## Stage 3 — Design a longitudinal system identification experiment

Now replace the placeholders with identified values. **You design the experiment.** Write a protocol
first, discuss it with your supervisor, then drive it.

- [ ] Write down what you need to fit F_x = MF(κ): pairs of (κ, F_x) covering both the linear region and the peak, for accelerating and braking.
- [ ] For each quantity, state which sensor or computation provides it and how accurate it is.
- [ ] Write the experiment protocol: maneuvers, speeds, number of repetitions, surface, tire condition, safety (space, run-off, who holds the kill switch).
- [ ] Implement a fitting script (put it in `system_identification/`) using `scipy.optimize.least_squares` with a robust loss.
- [ ] Write the identified values into `default.yaml` (remove the "not identified" comment) and include the fit residuals and parameter uncertainties in your report.
- [ ] **Validation:** rerun Stage 2 with the identified parameters. Did `|d_x|` shrink? Did the a_x innovations shrink?

> **Hints**
> - Identify things in order: first `s_w` (free rolling at constant speed, compare wheel speed with VIO), then rolling resistance and drag (coast-downs with **zero motor current**, not zero duty — zero duty brakes), and only then the Magic Formula. Otherwise the resistance and radius errors end up inside your tire parameters.
> - F_x is not measured directly: use F_x = m·a_x + F_resistance, with a_x from the IMU (lever-arm corrected, bias removed at standstill).
> - κ needs the true ground speed v_x, which the wheel speed alone cannot give you during slip. Use VIO (or motion capture if available), smoothed offline.
> - Normal acceleration on rubber tires stays at small κ. To reach the peak you need deliberate wheelspin (hard launches) and hard braking. The VESC runs in speed-control mode, so speed **steps** in the command are the easiest way to excite large slip.
> - Most samples will be in the linear region. Bin or weight the data by κ so the peak is not ignored by the fit. Start with E = 0 and only free it if the residuals show structure.

*Suggested output (optional):* the experiment protocol, the fitting script, the identified parameters with uncertainties, and the validation plots.

---

## Stage 4 — The filter and the gating

### 4.1 What changes for the EKF
- [ ] Map the predict and update steps you already know onto the new code: `models.py: predict` (note the substeps) and `filterpy`'s `ExtendedKalmanFilter.update`, now called through `Sensor.update` in `sensors.py`.
- [ ] Explain the roles of `Q` and `R`. Why should Q on v_x, v_y and r be **small** in this model, with uncertainty entering through `d_x, d_y, d_psi` instead? (Hint: look at ∂a_x/∂v_x in `HJacobian_imu`.)

### 4.2 Gating
- [ ] Read `data filtering suggestion.md`. Explain in your own words: innovation, innovation covariance S, Mahalanobis distance d², and why d² follows a χ² distribution.
- [ ] Find `Sensor._gate` in `sensors.py`. Explain why the IMU is gated in three groups (yaw / gyro / accelerometer) and VIO in two (pose / velocity).
- [ ] Explain the force-accept mechanism (`gate_max_reject`). What would happen without it if the filter drifted?
- [ ] Set `gate_prob: 0.99` and replay your bag. Where (which parts of the lap) are measurements rejected? Relate each cluster to a cause (curb, wheelspin, VIO blur, …).
- [ ] **Outlier injection.** In a replay script, inject an accelerometer spike and a VIO velocity jump. Compare the state estimate with gating on and off.
- [ ] **Consistency check.** Log d² for every update (the normalized innovation squared, NIS). Compute its mean per sensor group and compare with the group dimension. Is the filter over- or under-confident? Adjust R accordingly.

*Suggested output (optional):* rejections along the track, the outlier-injection comparison, and the NIS statistics before and after your R tuning.

---

## Wrapping up
- [ ] Commit your scripts and the updated parameter files.
- [ ] Archive the bags you used, with a short README describing them.
- [ ] If you write a report, the suggested outputs above are a natural structure for it.
