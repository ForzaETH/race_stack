#pragma once

#include <string>
namespace racecar_simulator {

struct CarParams {
    double wheelbase;
    double friction_coeff;
    double h_cg; // height of car's CG
    double l_f; // length from CG to front axle
    double l_r; // length from CG to rear axle
    double cs_f; // cornering stiffness coeff for front wheels
    double cs_r; // cornering stiffness coeff for rear wheels
    double mass;
    double I_z; // moment of inertia about z axis from CG
    double B_f; // stiffness coeff for front wheels
    double B_r; // stiffness coeff for rear wheels
    double C_f; // shape factor for front wheels
    double C_r; // shape factor for rear wheels
    double D_f; // peak value of lateral force over F_zf
    double D_r; // peak value of lateral force over F_zr
    double E_f; // curvature factor for front wheels
    double E_r; // curvature factor for rear wheels
    std::string tire_model; // model name
};

}
