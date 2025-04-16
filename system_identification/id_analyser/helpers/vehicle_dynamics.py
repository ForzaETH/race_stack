import math
from pbl_config import CarConfig, PacejkaTireConfig

g_ = 9.81

def vehicle_dynamics(x, uInit, car_config: CarConfig, pacejka_config: PacejkaTireConfig):
    """
    vehicleDynamics_st - single-track vehicle dynamics
    reference point: center of mass

    Syntax:
        f = vehicleDynamics_st(x,u,p)

    Inputs:
        :param x: vehicle state vector
        :param uInit: vehicle input vector
        :param p: vehicle parameter vector

    Outputs:
        :return f: right-hand side of differential equations

    Author: Matthias Althoff
    Written: 12-January-2017
    Last update: 16-December-2017
                    03-September-2019
    Last revision: 17-November-2020
    """
    
    # set gravity constant
    g = 9.81  #[m/s^2]

    #create equivalent bicycle parameters
    mu = pacejka_config.friction_coeff

    B_f = pacejka_config.Bf
    C_f = pacejka_config.Cf
    D_f = pacejka_config.Df
    E_f = pacejka_config.Ef
    B_r = pacejka_config.Br
    C_r = pacejka_config.Cr
    D_r = pacejka_config.Dr
    E_r = pacejka_config.Er

    lf = car_config.lf
    lr = car_config.lr
    h = car_config.h_cg
    m = car_config.m
    I = car_config.Iz

    #states
    #x0 = x-position in a global coordinate system
    #x1 = y-position in a global coordinate system
    #x2 = yaw angle
    #x3 = velocity in x-direction
    #x4 = velocity in y direction
    #x5 = yaw rate

    #u1 = steering angle
    #u2 = longitudinal acceleration

    u = uInit

    # system dynamics

    # compute lateral tire slip angles
    alpha_f = -math.atan((x[4] + x[5] * lf) / x[3]) + u[0]
    alpha_r = -math.atan((x[4] - x[5] * lr) / x[3])


    # compute vertical tire forces
    F_zf = m * (-u[1] * h + g * lr) / (lr + lf)
    F_zr = m * (u[1] * h + g * lf) / (lr + lf)

    F_yf = F_yr = 0

    # combined slip lateral forces
    F_yf = mu * F_zf * D_f * math.sin(C_f * math.atan(B_f * alpha_f - E_f*(B_f * alpha_f - math.atan(B_f * alpha_f))))
    F_yr = mu * F_zr * D_r * math.sin(C_r * math.atan(B_r * alpha_r - E_r*(B_r * alpha_r - math.atan(B_r * alpha_r))))


    f = [x[3]*math.cos(x[2]) - x[4]*math.sin(x[2]),
        x[3]*math.sin(x[2]) + x[4]*math.cos(x[2]),
        x[5],
        u[1],
        1/m * (F_yr + F_yf) - x[3] * x[5],
        1/I * (-lr * F_yr + lf * F_yf)]

    return f
