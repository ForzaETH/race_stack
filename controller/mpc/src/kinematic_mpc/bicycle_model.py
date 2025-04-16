
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

# author: Daniel Kloeser

import types

import casadi as cs
import numpy as np
from casadi import MX, Function, cos, interpolant, fmod, sin, vertcat
from pbl_config import KMPCConfig, CarConfig


def bicycle_model(
        s0: list,
        kapparef: list,
        vx_ref: list,
        d_left: list,
        d_right: list,
        kmpc_config: KMPCConfig,
        car_config: CarConfig):
    # define structs
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "Spatialbicycle_model"

    length = len(s0)
    pathlength = s0[-1]
    # copy loop to beginning and end
    # Something wrong：s0[1:length]
    s0 = np.append(s0, [s0[length - 1] + s0[1:length]])
    s0 = np.append([s0[:length - 1] - s0[length - 1]], s0)

    kapparef = np.append(kapparef, kapparef[1:length])
    kapparef = np.append(kapparef[:length - 1], kapparef)


    vx_ref = np.append(vx_ref, vx_ref[1:length])
    vx_ref = np.append(vx_ref[:length - 1], vx_ref)

    d_left = np.append(d_left, d_left[1:length])
    d_left = np.append(d_left[:length - 1], d_left)

    d_right = np.append(d_right, d_right[1:length])
    d_right = np.append(d_right[:length - 1], d_right)

    # compute spline interpolations
    kapparef_s = interpolant("kapparef_s", "bspline", [s0], kapparef)
    vx_ref_s = interpolant("vx_ref_s", "bspline", [s0], vx_ref)
    left_bound_s = interpolant("left_bound_s", "bspline", [s0], d_left)
    right_bound_s = interpolant("right_bound_s", "bspline", [s0], d_right)

    # CasADi Model
    # set up states & controls
    s = MX.sym("s")
    n = MX.sym("n")
    alpha = MX.sym("alpha")
    v = MX.sym("v")
    delta = MX.sym("delta")

    x = vertcat(s, n, alpha, v, delta)
    n_x = x.size()[0]
    # controls
    der_v = MX.sym("der_v")
    derDelta = MX.sym("derDelta")
    u = vertcat(der_v, derDelta)
    n_u = u.size()[0]

    # xdot
    sdot = MX.sym("sdot")
    ndot = MX.sym("ndot")
    alphadot = MX.sym("alphadot")
    vdot = MX.sym("vdot")
    deltadot = MX.sym("deltadot")
    xdot = vertcat(sdot, ndot, alphadot, vdot, deltadot)

    # algebraic variables
    z = vertcat([])

    # parameters
    lr = car_config.lr
    lf = car_config.lf
    freq = kmpc_config.MPC_freq

    # parameters
    V_target = MX.sym('V_target')
    weight_adv = MX.sym('weight_adv')
    weight_qn = MX.sym('weight_qn')
    weight_qv = MX.sym('weight_qv')
    weight_qalpha = MX.sym('weight_qalpha')
    weight_qac = MX.sym('weight_qac')
    weight_ddelta = MX.sym('weight_ddelta')
    a_lat_max = MX.sym('a_lat_max')
    # dynamic boundary inflation
    bound_inflation = MX.sym('boundary_inflation')
    overtake_d = MX.sym('overtake_d')

    p = vertcat(
        V_target,
        weight_adv,
        weight_qv,
        weight_qn,
        weight_qalpha,
        weight_qac,
        weight_ddelta,
        a_lat_max,
        bound_inflation, # boundary inflation
        overtake_d)

    # calculate the index
    s_mod = fmod(s, pathlength)

    # kinematic model
    sdota = (v * cos(alpha)) / (1 - kapparef_s(s_mod) * n)
    ndot = v * sin(alpha)  # (v * cos(alpha + beta))**2 * (0.5 * kapparef_s(s_mod))
    alphadot = v / (lr + lf) * cs.tan(delta) - kapparef_s(s_mod) * sdota
    # continuous dynamics
    f_expl = vertcat(
        sdota,
        ndot,
        alphadot,
        der_v,
        derDelta,
    )
    f_expl_func = Function('f_expl_func', [s, n, alpha, v, delta, der_v, derDelta, p], [f_expl])

    # external discretized dynamics for the reverse behavior
    # runge kutta simulation
    control_period = 1 / freq
    f_c_runge = vertcat(
        sdota,
        ndot,
        alphadot,
    )
    x_runge = vertcat(s, n, alpha)
    f_c_runge_func = Function('f_c_runge_func', [s, n, alpha, v, delta, p], [f_c_runge])
    # arg input
    dis_v_k = v + der_v
    dis_delta_k = delta + derDelta

    k1 = control_period * f_c_runge_func(s, n, alpha, dis_v_k, dis_delta_k, p)
    x_next_runge = x_runge + 0.5 * k1
    k2 = control_period * f_c_runge_func(x_next_runge[0], x_next_runge[1], x_next_runge[2], dis_v_k, dis_delta_k, p)
    x_next_runge = x_runge + 0.5 * k2
    k3 = control_period * f_c_runge_func(x_next_runge[0], x_next_runge[1], x_next_runge[2], dis_v_k, dis_delta_k, p)
    x_next_runge = x_runge + k3
    k4 = control_period * f_c_runge_func(x_next_runge[0], x_next_runge[1], x_next_runge[2], dis_v_k, dis_delta_k, p)
    x_next_runge = x_runge + (k1 + 2 * k2 + 2 * k3 + k4) / 6.

    f_disc = vertcat(x_next_runge, dis_v_k, dis_delta_k)

    # Define initial conditions
    model.x0 = np.array([-2, 0, 0, 0, 0])

    # Definition of external cost function
    model.cost_expr_ext_cost = -weight_adv * sdota / freq + weight_qn * (n - overtake_d)**2 + weight_qalpha * \
        alpha**2 + weight_qv * (v - V_target)**2 + weight_qac * der_v**2 + weight_ddelta * derDelta**2
    model.cost_expr_ext_cost_e = -weight_adv * sdota / freq + weight_qn * \
        (n - overtake_d)**2 + weight_qalpha * alpha**2 + weight_qv * (v - V_target)**2

    # constraint on forces
    a_lat = v * v * cs.tan(delta) / (lr + lf)

    # constraint on lateral erros
    # distance to border
    n_right_bound = n + right_bound_s(s_mod) - bound_inflation
    n_left_bound = n - left_bound_s(s_mod) + bound_inflation

    # nonlinear constraints
    constraint.alat = Function("a_lat", [x, u], [a_lat])
    constraint.pathlength = pathlength
    # construct online parameter turning constraints
    constraint.expr = vertcat(
        a_lat - (-a_lat_max),
        a_lat - a_lat_max,
        n_right_bound,
        n_left_bound)
    constraint.expr_e = constraint.expr
    # read the parameters range from the yaml file
    # Model bounds
    model.n_min = 0  # width of the track [m]
    model.n_max = kmpc_config.track_max_width  # width of the track [m]
    # state bounds

    model.delta_min = kmpc_config.delta_min  # minimum steering angle [rad]
    model.delta_max = kmpc_config.delta_max  # maximum steering angle [rad]

    # input bounds
    model.ddelta_min = kmpc_config.ddelta_min  # minimum change rate of stering angle [rad/s]
    model.ddelta_max = kmpc_config.ddelta_max  # maximum change rate of steering angle [rad/s]
    model.dv_min = kmpc_config.a_min  # minimum velocity change rate
    model.dv_max = kmpc_config.a_max  # maximum velocity change rate

    # nonlinear constraint
    constraint.alat_max = kmpc_config.alat_max  # maximum lateral force [m/s^1]

    # Velocity constraint
    constraint.v_min = kmpc_config.v_min  # minimum velocity [m/s]
    constraint.v_max = kmpc_config.v_max  # maximum velocity [m/s]

    # Define model struct
    params = types.SimpleNamespace()
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.dyn_disc_fun = f_disc
    model.x = x
    model.n_x = n_x
    model.xdot = xdot
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
    return model, constraint, params
