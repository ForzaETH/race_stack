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
        v_safe = vx + ca.if_else(ca.fabs(vx) >= 0, eps, -eps)  # prevent division by 0 later

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
        speed = ca.sqrt(vx**2 + vy**2 + 1e-6)
        fxu_kin = ca.vertcat(
            x + vx * ca.cos(theta) * dt,
            y + vx * ca.sin(theta) * dt,
            theta + (speed * ca.tan(steer) / self.p['wheelbase']) * dt,
            vx + accel * dt,
            ca.MX(0.0),
            speed * ca.tan(steer) / self.p['wheelbase']
        )

        # mixing weights
        v_b = 2.05  # empiricaly tuned variable
        v_s = 1
        w_std = 0.5 * (1 + ca.tanh((speed - v_b) / v_s))
        w_std = ca.if_else(speed < (v_b - 2 * v_s), 0.0, w_std)

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
