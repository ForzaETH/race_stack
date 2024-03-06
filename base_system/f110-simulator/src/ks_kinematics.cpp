#include <cmath>

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/ks_kinematics.hpp"

using namespace racecar_simulator;

// Implementation based off of Kinematic Single Track Dynamics defined in CommonRoad: Vehicle Models
// https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/blob/master/vehicleModels_commonRoad.pdf

CarState KSKinematics::update(
        const CarState start,
        double accel,
        double steer_angle_vel,
        CarParams p,
        double dt) {

    CarState end;

    // compute first derivatives of state
    double x_dot = start.velocity * std::cos(start.theta);
    double y_dot = start.velocity * std::sin(start.theta);
    double v_dot = accel;
    double steer_ang_dot = steer_angle_vel;
    double theta_dot = start.velocity / p.wheelbase * std::tan(start.steer_angle);

    // update state
    end.x = start.x + x_dot * dt;
    end.y = start.y + y_dot * dt;
    end.theta = start.theta + theta_dot * dt;
    end.velocity = start.velocity + v_dot * dt;
    end.steer_angle = start.steer_angle + steer_ang_dot * dt;
    end.angular_velocity = start.angular_velocity;
    end.slip_angle = start.slip_angle;

    return end;
}
