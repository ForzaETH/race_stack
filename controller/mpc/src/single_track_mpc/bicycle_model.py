#
# Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
# Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
# Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
# Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#

import types
# author: Daniel Kloeser

import casadi as cs
import numpy as np
from casadi import MX, Function, cos, fmod, interpolant, sin, vertcat
from pbl_config import STMPCConfig, CarConfig, PacejkaTireConfig


def bicycle_model(s0: list, kapparef: list, d_left: list, d_right: list,
                  stmpc_config: STMPCConfig, car_config: CarConfig, tire_config: PacejkaTireConfig):
    # define structs
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "Spatialbicycle_model"

    length = len(s0)
    pathlength = s0[-1]
    # copy loop to beginning and end

    s0 = np.append(s0, [s0[length - 1] + s0[1:length]])
    s0 = np.append([s0[:length - 1] - s0[length - 1]], s0)
    kapparef = np.append(kapparef, kapparef[1:length])
    kapparef = np.append([kapparef[:length - 1] - kapparef[length - 1]], kapparef)

    d_left = np.append(d_left, d_left[1:length])
    d_left = np.append([d_left[:length - 1] - d_left[length - 1]], d_left)
    d_right = np.append(d_right, d_right[1:length])
    d_right = np.append([d_right[:length - 1] - d_right[length - 1]], d_right)

    # compute spline interpolations
    kapparef_s = interpolant("kapparef_s", "bspline", [s0], kapparef)
    left_bound_s = interpolant("left_bound_s", "bspline", [s0], d_left)
    right_bound_s = interpolant("right_bound_s", "bspline", [s0], d_right)

    # CasADi Model
    # set up states & controls
    s = MX.sym("s")
    n = MX.sym("n")
    theta = MX.sym("theta")
    v_x = MX.sym("v_x")
    v_y = MX.sym("v_y")
    delta = MX.sym("delta")
    yaw_rate = MX.sym("yaw_rate")
    accel = MX.sym("accel")
    x = vertcat(s, n, theta, v_x, v_y, delta, yaw_rate, accel)
    n_x = x.size()[0]

    # controls
    jerk = MX.sym("jerk")
    derDelta = MX.sym("derDelta")
    u = vertcat(jerk, derDelta)
    n_u = u.size()[0]

    # xdot
    s_dot = MX.sym("sdot")
    n_dot = MX.sym("ndot")
    theta_dot = MX.sym("thetadot")
    v_x_dot = MX.sym("v_x_dot")
    v_y_dot = MX.sym("v_y_dot")
    delta_dot = MX.sym("deltadot")
    yaw_rate_dot = MX.sym("yaw_ratedot")
    accel_dot = MX.sym("accel_dot")

    # algebraic variables
    z = vertcat([])

    # parameters
    freq = stmpc_config.MPC_freq

    lr = car_config.lr
    lf = car_config.lf
    m = car_config.m
    Iz = car_config.Iz
    h_cg = car_config.h_cg

    friction_coeff = tire_config.friction_coeff
    Bf = tire_config.Bf
    Cf = tire_config.Cf
    Df = tire_config.Df
    Ef = tire_config.Ef
    Br = tire_config.Br
    Cr = tire_config.Cr
    Dr = tire_config.Dr
    Er = tire_config.Er
    g = 9.81

    # # online parameters
    V_target = MX.sym('V_target')
    weight_adv = MX.sym('weight_adv')
    weight_qn = MX.sym('weight_qn')
    weight_qv = MX.sym('weight_qv')
    weight_qalpha = MX.sym('weight_qalpha')
    weight_qjerk = MX.sym('weight_qjerk')
    weight_ddelta = MX.sym('weight_ddelta')
    a_lat_max = MX.sym('a_lat_max')
    a_long_min = MX.sym('a_long_min')
    a_long_max = MX.sym('a_long_max')
    safety_margin = MX.sym('safety_margin')
    overtake_d = MX.sym('overtake_d')
    p = vertcat(V_target, weight_adv, weight_qv, weight_qn, weight_qalpha, weight_qjerk, weight_ddelta,  # weights
                a_lat_max, a_long_min, a_long_max, safety_margin, overtake_d)  # lateral acceleration constraints

    # calculate the index
    s_mod = fmod(s, pathlength)

    # single track model definition with pacejka tire model
    # tire model
    # side slip angle
    v_0 = 0.5
    # func_v = v_x * (1 / (1 + cs.exp(-v_x / v_0))) - v_x * \
    #     (1 / (1 + cs.exp(v_x / v_0))) + v_0 * (1 / (cs.cosh(v_x / v_0)))
    alpha_f = (cs.atan2((-v_y - (lf * yaw_rate)), v_x + v_0) + delta)
    alpha_r = (cs.atan2((-v_y + (lr * yaw_rate)), v_x + v_0))

    # load transfer
    if stmpc_config.load_transfer:
        F_zf = m * ((-accel*h_cg)+(g*lr))/(lf+lr)
        F_zr = m * ((accel*h_cg)+(g*lf))/(lf+lr)
    else:
        F_zf = m * g * lr / (lf + lr)
        F_zr = m * g * lf / (lf + lr)

    # lateral tire forces
    F_yf = friction_coeff * Df * F_zf * sin(Cf * cs.atan(Bf * alpha_f - Ef * (Bf * alpha_f - cs.atan(Bf * alpha_f))))
    F_yr = friction_coeff * Dr * F_zr * sin(Cr * cs.atan(Br * alpha_r - Er * (Br * alpha_r - cs.atan(Br * alpha_r))))

    # motor force
    F_motor = accel * m
    motor_split_front = 0.5
    motor_split_rear = 1 - motor_split_front
    F_xf = motor_split_front * F_motor
    F_xr = motor_split_rear * F_motor

    # dynamics
    s_dot = ((v_x * cos(theta)) - (v_y * sin(theta))) / (1 - kapparef_s(s_mod) * n)
    n_dot = v_x * sin(theta) + v_y * cos(theta)
    theta_dot = yaw_rate - (kapparef_s(s_mod) * (((v_x * cos(theta)) -
                            (v_y * sin(theta))) / (1 - kapparef_s(s_mod) * n)))
    v_x_dot = (1 / m) * (
        F_xr
        + F_xf * cos(delta)
        - F_yf * sin(delta)
        + m * v_y * yaw_rate
    )
    v_y_dot = (1 / m) * (
        F_yr
        + F_xf * sin(delta)
        + F_yf * cos(delta)
        - m * v_x * yaw_rate
    )
    yaw_rate_dot = (1 / Iz) * ((-F_yr * lr) + (F_yf * lf) * cos(delta))

    # delta inputs for smoother control
    delta_dot = derDelta
    accel_dot = jerk

    # continuous dynamics
    f_expl = vertcat(
        s_dot,
        n_dot,
        theta_dot,
        v_x_dot,
        v_y_dot,
        delta_dot,
        yaw_rate_dot,
        accel_dot
    )
    f_expl_func = Function('f_expl_func', [s, n, theta, v_x, v_y, delta, yaw_rate, accel, jerk, derDelta, p], [f_expl])

    # Define initial conditions
    model.x0 = np.zeros(n_x)

    terminal_multiplier = 10
    model.cost_expr_ext_cost_0 = weight_qn * (n - overtake_d)**2 + weight_qalpha * theta**2 + weight_qv * \
        (v_x - V_target)**2 + weight_qjerk * jerk**2 + weight_ddelta * derDelta**2
    model.cost_expr_ext_cost = weight_qn * (n - overtake_d)**2 + weight_qalpha * theta**2 + weight_qv * \
        (v_x - V_target)**2 + weight_qjerk * jerk**2 + weight_ddelta * derDelta**2
    model.cost_expr_ext_cost_e = terminal_multiplier * \
        (weight_qn * (n - overtake_d)**2 + weight_qalpha * theta**2 + weight_qv * (v_x - V_target)**2)
    if stmpc_config.vy_minimization:
        model.cost_expr_ext_cost_0 += 100 * v_y**2
        model.cost_expr_ext_cost += 100 * v_y**2
        model.cost_expr_ext_cost_e += terminal_multiplier * (100 * v_y**2)
    if stmpc_config.adv_maximization:
        model.cost_expr_ext_cost -= weight_adv * s_dot / freq
        model.cost_expr_ext_cost_e -= terminal_multiplier * weight_adv * s_dot / freq

    # constraint on lateral errors
    n_right_bound = n + right_bound_s(s_mod) - safety_margin
    n_left_bound = n - left_bound_s(s_mod) + safety_margin

    # nonlinear constraints
    if stmpc_config.correct_v_y_dot:
        a_lat = v_y_dot
    else:
        a_lat = v_x * v_x * cs.tan(delta) / (lr + lf)
    constraint.pathlength = pathlength
    # TODO: introduce asymmetric constraints (now you can just over constrain breaking) with a_long_min
    if stmpc_config.combined_constraints == "ellipse":
        combined_constraints = (a_lat / a_lat_max)**2 + (v_x_dot / a_long_max)**2
    elif stmpc_config.combined_constraints == "diamond":
        combined_constraints = cs.fabs(a_lat / a_lat_max) + cs.fabs(v_x_dot / a_long_max)
    else:
        combined_constraints = 0
    # TODO fix the n constraint with a single constraint
    # NOTE: easiest when non-terminal constraints have the extra constraints
    # only at the end, otherwise need to change acados_settings.py
    constraint.expr = vertcat(
        n_right_bound,
        n_left_bound,
        combined_constraints
    )
    constraint.expr_e = vertcat(
        n_right_bound,
        n_left_bound
    )

    # read the parameters range from the yaml file

    # state bounds
    constraint.delta_min = stmpc_config.delta_min  # minimum steering angle [rad]
    constraint.delta_max = stmpc_config.delta_max  # maximum steering angle [rad]
    constraint.v_x_min = stmpc_config.v_min  # minimum velocity [m/s]
    constraint.v_x_max = stmpc_config.v_max  # maximum velocity [m/s]
    constraint.a_min = stmpc_config.a_min  # minimum velocity change rate
    constraint.a_max = stmpc_config.a_max  # maximum velocity change rate

    # input bounds
    constraint.ddelta_min = stmpc_config.ddelta_min  # minimum change rate of stering angle [rad/s]
    constraint.ddelta_max = stmpc_config.ddelta_max  # maximum change rate of steering angle [rad/s]
    constraint.jerk_min = stmpc_config.jerk_min  # minimum jerk [m/s^3]
    constraint.jerk_max = stmpc_config.jerk_max  # maximum jerk [m/s^3]

    # Define model struct
    params = types.SimpleNamespace()
    # model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.n_x = n_x
    # model.xdot = xdot
    model.u = u
    model.n_u = n_u
    model.z = z
    model.p = p
    model.name = model_name
    model.params = params
    model.kappa = kapparef_s
    model.f_expl_func = f_expl_func
    model.left_bound_s = left_bound_s
    model.right_bound_s = right_bound_s
    model.track_max = stmpc_config.track_max_width
    # model.safety_width = stmpc_config.track_safety_margin
    return model, constraint, params
