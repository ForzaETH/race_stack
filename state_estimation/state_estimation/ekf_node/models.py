import casadi as ca
import numpy as np


class point_mass_model:
    """
    omnidirectional, three-dimensional point-mass motion model

    State x = [x, y, theta, vx, vy, omega]
    Control inputs u = [] (not used)
    """

    def __init__(self, Q):
        ################
        # System Model #
        ################
        self.dim_x = 6
        self.dim_u = 2  # not used here but keep for generic structure
        self.Q = Q

        # Initial Values
        self.x_0 = np.zeros(self.dim_x)
        self.P_0 = np.eye(self.dim_x) * 10
        self.P_0[4][4] = 0.0001

        # CasADi Symbols
        x_sym = ca.MX.sym('x', self.dim_x)
        u_sym = ca.MX.sym('u', self.dim_u)
        dt = ca.MX.sym('dt')

        x = x_sym[0]
        y = x_sym[1]
        psi = x_sym[2]
        vx = x_sym[3]
        vy = x_sym[4]
        vpsi = x_sym[5]
        accel = u_sym[0]
        steer_agnle = u_sym[1]

        # nonlinear transition functions
        fxu = ca.vertcat(
            x + (vx * ca.cos(psi) - vy * ca.sin(psi)) * dt,
            y + (vx * ca.sin(psi) + vy * ca.cos(psi)) * dt,
            psi + vpsi * dt,
            vx,
            vy,
            vpsi
        )
        self.f = ca.Function('f', [x_sym, u_sym, dt], [fxu])
        self.F = ca.Function('F', [x_sym, u_sym, dt], [ca.jacobian(fxu, x_sym)])

        #####################
        # Measurement Model #
        #####################
        H_imu = np.array([[0, 0, 1, 0, 0, 0],
                          [0, 0, 0, 0, 0, 1],
                          [0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0]])
        self.Hx_imu = lambda x, u: H_imu @ np.array(x).flatten()
        self.HJacobian_imu = lambda x, u: H_imu

        H_vesc = np.eye(6)
        self.Hx_vesc = lambda x: H_vesc @ np.array(x).flatten()
        self.HJacobian_vesc = lambda x: H_vesc

    def predict(self, ekf, dt, u):
        x_flat = np.array(ekf.x).flatten()

        x_new = np.array(self.f(x_flat, u, dt)).flatten()

        F = np.array(self.F(x_flat, u, dt)).reshape(self.dim_x, self.dim_x)

        ekf.x = x_new
        ekf.P = F @ ekf.P @ F.T + self.Q

    def get_odom_state(self, x):
        return x


class kinematic_bicycle_model:
    """
    Kinematic Bicycle Model Class. Uses model defined in the vehicle_dynamics_update_k function of f1tenth_pysim.dynamic_models_ros

    State vector x = [x, y, steer_angle, velocity, theta, angular_velocity, slip_angle]
    Control input u = [accel, steer_angle]

    """

    def __init__(self, config, Q):
        self.p = config

        ################
        # System Model #
        ################
        self.dim_x = 7
        self.dim_u = 2

        self.Q = Q

        # Initial Values
        self.x_0 = np.zeros(self.dim_x)
        self.P_0 = np.eye(self.dim_x) * 10

        x_sym = ca.MX.sym('x', self.dim_x)
        u_sym = ca.MX.sym('u', self.dim_u)
        dt = ca.MX.sym('dt')

        # for readability
        x = x_sym[0]
        y = x_sym[1]
        steer = x_sym[2]
        v = x_sym[3]
        theta = x_sym[4]
        omega = x_sym[5]
        slip = x_sym[6]

        accel = u_sym[0]
        steer_cmd = u_sym[1]

        fxu = ca.vertcat(
            x + v * ca.cos(theta) * dt,
            y + v * ca.sin(theta) * dt,
            steer_cmd,
            v + accel * dt,
            theta + v * ca.tan(steer) / self.p['wheelbase'] * dt,
            v * ca.tan(steer) / self.p['wheelbase'],
            ca.MX(0.0)
        )

        self.f = ca.Function('f', [x_sym, u_sym, dt], [fxu])
        self.F = ca.Function('F', [x_sym, u_sym, dt], [ca.jacobian(fxu, x_sym)])

        #####################
        # Measurement Model #
        #####################
        h_imu_sym = ca.vertcat(
            theta,
            omega,
            accel,
            0,
        )
        _h_imu = ca.Function
        _h_imu = ca.Function('h_imu', [x_sym, u_sym], [h_imu_sym])
        _H_imu = ca.Function('H_imu', [x_sym, u_sym], [ca.jacobian(h_imu_sym, x_sym)])
        self.Hx_imu = lambda x, u: np.array(_h_imu(x, u)).flatten()
        self.HJacobian_imu = lambda x, u: np.array(_H_imu(x, u)).reshape(4, self.dim_x)

        h_vesc_sym = ca.vertcat(
            x,
            y,
            theta,
            v,
            ca.MX(0.0),
            omega,
        )
        _h_vesc = ca.Function('h_vesc', [x_sym], [h_vesc_sym])
        _H_vesc = ca.Function('H_vesc', [x_sym], [ca.jacobian(h_vesc_sym, x_sym)])
        self.Hx_vesc = lambda x: np.array(_h_vesc(x)).flatten()
        self.HJacobian_vesc = lambda x: np.array(_H_vesc(x)).reshape(6, self.dim_x)

    def predict(self, ekf, dt, u):
        x_flat = np.array(ekf.x).flatten()
        x_new = np.array(self.f(x_flat, u, dt)).flatten()

        F = np.array(self.F(x_flat, u, dt)).reshape(self.dim_x, self.dim_x)

        ekf.x = x_new
        ekf.P = F @ ekf.P @ F.T + self.Q

    def get_odom_state(self, x):
        # x_kin = [x, y, steer_angle, velocity, theta, angular_velocity, slip_angle]
        # state = [x, y, psi, vx, vy, vpsi]
        state = np.zeros(6)
        state[0] = x[0]
        state[1] = x[1]
        state[2] = x[4]

        state[3] = x[3]
        state[4] = 0.0

        state[5] = x[5]

        return state


class single_track_model:
    """
    EKF Model for the single-track model. Uses model defined in the vehicle_dynamics_st_update_pacejka function of f1tenth_pysim.dynamic_models_ros

    State vector x = [x, y, theta, vx, vy, omega]
    Control input u = [accel, steer_angle]

    Threshold value for mixing logic v_b is tuned empirically to 2.05

    """

    def __init__(self, config, Q):
        self.p = config

        ################
        # System Model #
        ################
        self.dim_x = 6
        self.dim_u = 2

        self.Q = Q

        # Initial Values
        self.x_0 = np.zeros(self.dim_x)
        self.P_0 = np.eye(self.dim_x) * 10

        # CasADi symbols
        x_sym = ca.MX.sym('x', self.dim_x)
        u = ca.MX.sym('u', self.dim_u)
        dt = ca.MX.sym('dt')

        # for readability
        x = x_sym[0]
        y = x_sym[1]
        theta = x_sym[2]
        vx = x_sym[3]
        vy = x_sym[4]
        omega = x_sym[5]
        accel = u[0]
        steer = u[1]

        # Compute Forces used in Model (math coppied from original model definition)
        F_zf = self.p['m'] * (-accel * self.p['h_cg'] + 9.81 * self.p['l_r']) / (self.p['l_f'] + self.p['l_r'])
        F_zr = self.p['m'] * (accel * self.p['h_cg'] + 9.81 * self.p['l_f']) / (self.p['l_f'] + self.p['l_r'])

        eps = 1e-4
        v_safe = vx + ca.if_else(vx >= 0, eps, -eps)  # prevent division by 0 later

        alpha_f = ca.atan2((-vy - self.p['l_f'] * omega), v_safe) + steer
        alpha_r = ca.atan2((-vy + self.p['l_r'] * omega), v_safe)

        Fyf = self.p['mu'] * self.p['Df'] * F_zf * ca.sin(
            self.p['Cf'] * ca.atan(
                self.p['Bf'] * alpha_f - self.p['Ef'] * (self.p['Bf'] * alpha_f - ca.atan(
                    self.p['Bf'] * alpha_f))))
        Fyr = self.p['mu'] * self.p['Dr'] * F_zr * ca.sin(
            self.p['Cr'] * ca.atan(
                self.p['Br'] * alpha_r - self.p['Er'] * (self.p['Br'] * alpha_r - ca.atan(
                    self.p['Br'] * alpha_r))))

        Fxr = accel * self.p['m']
        Fxf = ca.MX(0.0)

        fxu_dyn = ca.vertcat(
            x + dt * (vx * ca.cos(theta) - vy * ca.sin(theta)),
            y + dt * (vx * ca.sin(theta) + vy * ca.cos(theta)),
            theta + dt * omega,
            vx + dt * ((Fxr + Fxf * ca.cos(steer) - Fyf * ca.sin(steer) + self.p['m'] * vy * omega) / self.p['m']),
            vy + dt * ((Fyr + Fxf * ca.sin(steer) + Fyf * ca.cos(steer) - self.p['m'] * vx * omega) / self.p['m']),
            omega + dt * ((Fyf * self.p['l_f'] * ca.cos(steer) + Fxf * self.p['l_f']
                          * ca.sin(steer) - Fyr * self.p['l_r']) / self.p['I_z'])
        )

        # kinematic bicycle model uses overall velocity not vx and vy (1e-6 to prevent division by 0 later)
        speed_mag = ca.sqrt(vx**2 + vy**2 + 1e-6)
        speed = ca.sign(vx) * speed_mag
        fxu_kin = ca.vertcat(
            x + vx * ca.cos(theta) * dt,
            y + vx * ca.sin(theta) * dt,
            theta + (speed * ca.tan(steer) / self.p['wheelbase']) * dt,
            vx + accel * dt,
            ca.MX(0.0),
            speed * ca.tan(steer) / self.p['wheelbase']
        )

        # mixing weights
        v_b = 2.05  # empirically tuned variable
        v_s = 1
        w_std = 0.5 * (1 + ca.tanh((speed_mag - v_b) / v_s))
        w_std = ca.if_else(speed_mag < (v_b - 2 * v_s), 0.0, w_std)

        f_mixed = w_std * fxu_dyn + (1 - w_std) * fxu_kin

        self.f = ca.Function('f', [x_sym, u, dt], [f_mixed])
        self.F = ca.Function('F', [x_sym, u, dt], [ca.jacobian(f_mixed, x_sym)])

        #####################
        # Measurement Model #
        #####################
        ax_dyn = (Fxr + Fxf * ca.cos(steer) - Fyf * ca.sin(steer)) / self.p['m']
        ay_dyn = (Fyr + Fxf * ca.sin(steer) + Fyf * ca.cos(steer)) / self.p['m']
        ax_kin = accel
        ay_kin = 0.0

        ax_mixed = w_std * ax_dyn + (1 - w_std) * ax_kin
        ay_mixed = w_std * ay_dyn + (1 - w_std) * ay_kin

        h_imu_sym = ca.vertcat(
            theta,
            omega,
            ax_mixed,
            ay_mixed,
        )

        # Note: h_imu now depends on x_sym AND u_sym
        self._h_imu = ca.Function('h_imu', [x_sym, u], [h_imu_sym])
        self._H_imu = ca.Function('H_imu', [x_sym, u], [ca.jacobian(h_imu_sym, x_sym)])
        self.Hx_imu = lambda x, u: np.array(self._h_imu(x, u)).flatten()
        self.HJacobian_imu = lambda x, u: np.array(self._H_imu(x, u)).reshape(4, self.dim_x)

        H_vesc = np.eye(6)
        self.Hx_vesc = lambda x: H_vesc @ np.array(x).flatten()
        self.HJacobian_vesc = lambda x: H_vesc

    def predict(self, ekf, dt, u):
        x_flat = np.array(ekf.x).flatten()
        x_new = np.array(self.f(x_flat, u, dt)).flatten()

        F = np.array(self.F(x_flat, u, dt)).reshape(self.dim_x, self.dim_x)

        ekf.x = x_new
        ekf.P = F @ ekf.P @ F.T + self.Q

    def get_odom_state(self, x):
        # x_model = [x, y, psi, vx, vy, vpsi]
        # state = [x, y, psi, vx, vy, vpsi]
        return x


class single_track_wheel_model:
    """
    Single-track EKF model with Pacejka longitudinal and lateral forces, where the
    longitudinal force is driven by the measured wheel speed (VESC) instead of the
    commanded acceleration.

    Simplifications:
      - shared longitudinal slip for both axles (4WD with spool)
      - quasi-static slip angles and slip ratio (no relaxation lengths)
      - dynamic model above ~v_blend, tanh-blended with a kinematic model below
      - friction-ellipse saturation for combined slip

    States are expressed at the CoG. Measurements and the published odometry are
    mapped to base_link through the offset (base_px, base_py) of base_link from
    the CoG (F1TENTH convention: base_link on the rear axle -> base_px = -l_r).

    State  x = [x, y, theta, vx, vy, omega, s_w, b_gz, b_ax, b_ay, d_x, d_y, d_psi]
        s_w             : wheel-speed scale (effective / nominal radius), ~1
        b_gz            : gyro z bias
        b_ax, b_ay      : accelerometer biases
        d_x, d_y, d_psi : lumped force / moment disturbances (model mismatch)

    Input  u = [steer, v_w, ax_lt]
        steer : steering angle [rad] (same sign convention as single_track_model)
        v_w   : nominal wheel surface speed [m/s] = (ERPM - offset) / speed_to_erpm_gain
        ax_lt : bias-corrected, low-passed longitudinal accel, load transfer only

    Measurements
        imu : [theta, gyro_z, acc_x, acc_y]                  (h depends on x and u)
        vio : [x, y, theta, vx, vy, omega] at base_link      (same layout as OdomSensor)

    VESC odometry must NOT also be fused as a measurement with this model: its
    speed already enters through v_w.
    """

    IDX = dict(x=0, y=1, theta=2, vx=3, vy=4, omega=5, s_w=6,
               b_gz=7, b_ax=8, b_ay=9, d_x=10, d_y=11, d_psi=12)

    def __init__(self, config, Q):
        self.p = config

        ################
        # System Model #
        ################
        self.dim_x = 13
        self.dim_u = 3
        self.Q = Q

        # Initial Values
        self.x_0 = np.zeros(self.dim_x)
        self.x_0[self.IDX['s_w']] = 1.0
        self.P_0 = np.eye(self.dim_x) * 10
        self.P_0[6, 6] = 1e-3            # s_w
        self.P_0[7:10, 7:10] *= 1e-4     # IMU biases (calibrate at standstill)
        self.P_0[10:13, 10:13] *= 1e-2   # disturbances

        x_sym = ca.SX.sym('x', self.dim_x)
        u_sym = ca.SX.sym('u', self.dim_u)
        dt = ca.SX.sym('dt')

        # Explicit Euler with substeps: the slip-ratio coupling is stiff,
        # eigenvalue ~ sum_i(Bx*Cx*mu*Dx*F_zi) / (m*vx). Keep dt/n_sub well below its inverse.
        n_sub = int(self.p.get('n_substeps', 4))
        h = dt / n_sub
        x_next = x_sym
        for _ in range(n_sub):
            xdot, _, _, _ = self._dynamics(x_next, u_sym)
            x_next = x_next + h * xdot

        self.f = ca.Function('f', [x_sym, u_sym, dt], [x_next])
        self.F = ca.Function('F', [x_sym, u_sym, dt], [ca.jacobian(x_next, x_sym)])

        #####################
        # Measurement Model #
        #####################
        x, y, theta = x_sym[0], x_sym[1], x_sym[2]
        vx, vy, omega = x_sym[3], x_sym[4], x_sym[5]
        b_gz, b_ax, b_ay = x_sym[7], x_sym[8], x_sym[9]

        _, ax, ay, omega_dot = self._dynamics(x_sym, u_sym)

        # IMU: specific force at the sensor location (lever arm from CoG)
        px, py = self.p.get('imu_px', 0.0), self.p.get('imu_py', 0.0)
        h_imu_sym = ca.vertcat(
            theta,
            omega + b_gz,
            ax - px * omega**2 - py * omega_dot + b_ax,
            ay - py * omega**2 + px * omega_dot + b_ay,
        )
        self._h_imu = ca.Function('h_imu', [x_sym, u_sym], [h_imu_sym])
        self._H_imu = ca.Function('H_imu', [x_sym, u_sym], [ca.jacobian(h_imu_sym, x_sym)])
        self.Hx_imu = lambda x, u: np.array(self._h_imu(x, u)).flatten()
        self.HJacobian_imu = lambda x, u: np.array(self._H_imu(x, u)).reshape(4, self.dim_x)

        # VIO / odometry: CoG state mapped to base_link
        bx, by = self.p.get('base_px', 0.0), self.p.get('base_py', 0.0)
        h_base_sym = ca.vertcat(
            x + ca.cos(theta) * bx - ca.sin(theta) * by,
            y + ca.sin(theta) * bx + ca.cos(theta) * by,
            theta,
            vx - omega * by,
            vy + omega * bx,
            omega,
        )
        self._h_base = ca.Function('h_base', [x_sym], [h_base_sym])
        self._H_base = ca.Function('H_base', [x_sym], [ca.jacobian(h_base_sym, x_sym)])
        self.Hx_vio = lambda x: np.array(self._h_base(x)).flatten()
        self.HJacobian_vio = lambda x: np.array(self._H_base(x)).reshape(6, self.dim_x)

        # Kept for interface compatibility; do not fuse VESC odometry with this model.
        self.Hx_vesc = self.Hx_vio
        self.HJacobian_vesc = self.HJacobian_vio

    @staticmethod
    def _mf(s, B, C, E, D):
        """Pacejka Magic Formula."""
        Bs = B * s
        return D * ca.sin(C * ca.atan(Bs - E * (Bs - ca.atan(Bs))))

    def _dynamics(self, xs, u):
        """
        Continuous-time dynamics, blended dynamic/kinematic.
        Returns (xdot [dim_x], ax, ay, omega_dot) where ax, ay are body-frame
        specific forces at the CoG, used by the IMU measurement model.
        """
        p = self.p
        g = 9.81
        m, Iz = p['m'], p['I_z']
        lf, lr, h_cg = p['l_f'], p['l_r'], p['h_cg']
        L = lf + lr
        mu = p['mu']

        theta, vx, vy, omega = xs[2], xs[3], xs[4], xs[5]
        s_w = xs[6]
        d_x, d_y, d_psi = xs[10], xs[11], xs[12]

        steer, v_w, ax_lt = u[0], u[1], u[2]

        # Normal loads (longitudinal load transfer)
        F_zf = m * (g * lr - h_cg * ax_lt) / L
        F_zr = m * (g * lf + h_cg * ax_lt) / L

        # Shared slip ratio; denominator floored (dynamic branch is blended out at low speed)
        v_den = ca.fmax(ca.fabs(vx), p.get('v_min_slip', 0.5))
        kappa = (s_w * v_w - vx) / v_den

        # Slip angles (same convention as single_track_model)
        eps = 1e-4
        v_safe = vx + ca.if_else(vx >= 0, eps, -eps)
        alpha_f = ca.atan2(-vy - lf * omega, v_safe) + steer
        alpha_r = ca.atan2(-vy + lr * omega, v_safe)

        # Pure-slip forces
        Dxf, Dxr = mu * p['Dx'] * F_zf, mu * p['Dx'] * F_zr
        Dyf, Dyr = mu * p['Df'] * F_zf, mu * p['Dr'] * F_zr
        Fxf0 = self._mf(kappa, p['Bx'], p['Cx'], p['Ex'], Dxf) * p.get('front_drive', 1.0)
        Fxr0 = self._mf(kappa, p['Bx'], p['Cx'], p['Ex'], Dxr)
        Fyf0 = self._mf(alpha_f, p['Bf'], p['Cf'], p['Ef'], Dyf)
        Fyr0 = self._mf(alpha_r, p['Br'], p['Cr'], p['Er'], Dyr)

        # Friction-ellipse saturation, smooth max(1, rho) to keep Jacobians continuous
        k = 20.0
        rho_f = ca.sqrt((Fxf0 / Dxf)**2 + (Fyf0 / Dyf)**2 + 1e-9)
        rho_r = ca.sqrt((Fxr0 / Dxr)**2 + (Fyr0 / Dyr)**2 + 1e-9)
        sf = 1.0 / (1.0 + ca.log(1 + ca.exp(k * (rho_f - 1.0))) / k)
        sr = 1.0 / (1.0 + ca.log(1 + ca.exp(k * (rho_r - 1.0))) / k)
        Fxf, Fyf = sf * Fxf0, sf * Fyf0
        Fxr, Fyr = sr * Fxr0, sr * Fyr0

        # Resultants
        F_res = p.get('C_roll', 0.0) * ca.tanh(vx / 0.1) + p.get('C_drag', 0.0) * vx * ca.fabs(vx)
        F_X = Fxr + Fxf * ca.cos(steer) - Fyf * ca.sin(steer) - F_res
        F_Y = Fyr + Fxf * ca.sin(steer) + Fyf * ca.cos(steer)
        M_Z = lf * (Fxf * ca.sin(steer) + Fyf * ca.cos(steer)) - lr * Fyr

        ax_dyn = (F_X + d_x) / m
        ay_dyn = (F_Y + d_y) / m
        omega_dot_dyn = (M_Z + d_psi) / Iz

        f_dyn = ca.vertcat(
            vx * ca.cos(theta) - vy * ca.sin(theta),
            vx * ca.sin(theta) + vy * ca.cos(theta),
            omega,
            ax_dyn + vy * omega,
            ay_dyn - vx * omega,
            omega_dot_dyn,
        )

        # Kinematic branch: relax vx to wheel speed, vy to 0, omega to kinematic yaw rate
        tau = p.get('tau_kin', 0.05)
        speed_mag = ca.sqrt(vx**2 + vy**2 + 1e-6)
        speed = ca.sign(vx) * speed_mag
        omega_kin = speed * ca.tan(steer) / p['wheelbase']

        ax_kin = (s_w * v_w - vx) / tau
        omega_dot_kin = (omega_kin - omega) / tau
        ay_kin = vx * omega

        f_kin = ca.vertcat(
            vx * ca.cos(theta) - vy * ca.sin(theta),
            vx * ca.sin(theta) + vy * ca.cos(theta),
            omega,
            ax_kin,
            -vy / tau,
            omega_dot_kin,
        )

        # Mixing weights (as in single_track_model)
        v_b = p.get('v_blend', 2.05)
        v_s = p.get('v_blend_width', 1.0)
        w = 0.5 * (1 + ca.tanh((speed_mag - v_b) / v_s))
        w = ca.if_else(speed_mag < (v_b - 2 * v_s), 0.0, w)

        f_mixed = w * f_dyn + (1 - w) * f_kin
        ax = w * ax_dyn + (1 - w) * ax_kin
        ay = w * ay_dyn + (1 - w) * ay_kin
        omega_dot = w * omega_dot_dyn + (1 - w) * omega_dot_kin

        # Slow states (s_w, biases, disturbances) are random walks: zero drift, driven by Q
        xdot = ca.vertcat(f_mixed, ca.SX.zeros(self.dim_x - 6))
        return xdot, ax, ay, omega_dot

    def predict(self, ekf, dt, u):
        x_flat = np.array(ekf.x).flatten()
        x_new = np.array(self.f(x_flat, u, dt)).flatten()
        F = np.array(self.F(x_flat, u, dt)).reshape(self.dim_x, self.dim_x)
        ekf.x = x_new
        ekf.P = F @ ekf.P @ F.T + self.Q

    def get_odom_state(self, x):
        # Published odometry is at base_link: state = [x, y, psi, vx, vy, vpsi]
        return self.Hx_vio(np.array(x).flatten())
