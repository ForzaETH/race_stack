import math

g_ = 9.81

def vehicle_dynamics(x, u, p, type) -> list:
    """
    single-track vehicle dynamics with linear and pacejka tire models
    Inputs:
        :param x: vehicle state vector (x, y, yaw, v_x, v_y, omega)
        :param u: vehicle input vector (steering angle, longitudinal acceleration)
        :param p: vehicle parameter vector 
        :param type: tire model type (linear or pacejka)
    Outputs:
        :return f: derivative of vehicle state vector
    """
    mu = p.mu
    if type == "pacejka":
        B_f = p.C_Pf[0]
        C_f = p.C_Pf[1]
        D_f = p.C_Pf[2]
        E_f = p.C_Pf[3]
        B_r = p.C_Pr[0]
        C_r = p.C_Pr[1]
        D_r = p.C_Pr[2]
        E_r = p.C_Pr[3]
    elif type == "linear":
        C_Sf = p.C_Sf
        C_Sr = p.C_Sr
    lf = p.l_f
    lr = p.l_r
    h = p.h_cg 
    m = p.m 
    I_z = p.I_z

    # compute lateral tire slip angles
    alpha_f = -math.atan((x[4] + x[5] * lf) / x[3]) + u[0] 
    alpha_r = -math.atan((x[4] - x[5] * lr) / x[3])

    # compute vertical tire forces
    F_zf = m * (-u[1] * h + g_ * lr) / (lr + lf)
    F_zr = m * (u[1] * h + g_ * lf) / (lr + lf)

    F_yf = F_yr = 0

    # calculate combined slip lateral forces
    if type == "pacejka":
        F_yf = mu * F_zf * D_f * math.sin(C_f * math.atan(B_f * alpha_f - E_f*(B_f * alpha_f - math.atan(B_f * alpha_f))))
        F_yr = mu * F_zr * D_r * math.sin(C_r * math.atan(B_r * alpha_r - E_r*(B_r * alpha_r - math.atan(B_r * alpha_r))))
    elif type == "linear":
        F_yf = mu * F_zf * C_Sf * alpha_f
        F_yr = mu * F_zr * C_Sr * alpha_r

    f = [x[3]*math.cos(x[2]) - x[4]*math.sin(x[2]), 
         x[3]*math.sin(x[2]) + x[4]*math.cos(x[2]), 
         x[5], 
         u[1], 
         1/m * (F_yr + F_yf) - x[3] * x[5],
         1/I_z * (-lr * F_yr + lf * F_yf)] 

    return f
