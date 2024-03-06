#pragma once

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/car_params.hpp"

namespace racecar_simulator {

class STDKinematics {

public:

    static CarState update_pacejka(
        const CarState start,
        double accel,
        double steer_angle_vel,
        CarParams p,
        double dt);

    static CarState update_linear(
        const CarState start,
        double accel,
        double steer_angle_vel,
        CarParams p,
        double dt);


    static CarState update_k(
        const CarState start,
        double accel,
        double steer_angle_vel,
        CarParams p,
        double dt);
};

}
