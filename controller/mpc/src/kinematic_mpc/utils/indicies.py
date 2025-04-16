from enum import IntEnum

class StateIndex(IntEnum):
    POS_ON_CENTER_LINE_S = 0
    MIN_DIST_TO_CENTER_LINE_N = 1
    ORIENTATION_THETA = 2
    VELOCITY_V_X = 3
    VELOCITY_V_Y = 4
    STEERING_ANGLE_DELTA = 5
    YAW_RATE = 6
    ACCEL = 7

class Input(IntEnum):
    JERK = 0
    D_STEERING_ANGLE = 1


class Parameter(IntEnum):
    m = 0
    C1 = 1
    Cm2 = 2
    Cr0 = 3
    Cr2 = 4
    Cr3 = 5
    Iz = 6
    lr = 7
    lf = 8
    Bf = 9
    Cf = 10
    Df = 11
    Br = 12
    Cr = 13
    Dr = 14
    Ef = 15
    Er = 16
    friction_coeff = 17
    Imax_c = 18
    Caccel = 19
    Cdecel = 20
    qadv = 21
    qn = 22
    qalpha = 23
    qvy = 24
    freq = 25
    C_S_front = 26
    C_S_rear = 27
    h_cg = 28
    mu = 29
