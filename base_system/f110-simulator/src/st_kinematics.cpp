#include <cmath>

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/st_kinematics.hpp"
#include <iostream>

using namespace racecar_simulator;

// Implementation based off of Single Track Dynamics defined in CommonRoad: Vehicle Models
// https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/blob/master/vehicleModels_commonRoad.pdf

CarState STKinematics::update(
        const CarState start,
        double accel,
        double steer_angle_vel,
        CarParams p,
        double dt) {


    double thresh = 2; // cut off to avoid singular behavior
    double err = .03; // deadband to avoid flip flop
    if (!start.st_dyn)
        thresh += err;

    // if velocity is low or negative, use normal Kinematic Single Track dynamics
    if (start.velocity < thresh) {
        return update_k(
                    start,
                    accel,
                    steer_angle_vel,
                    p,
                    dt);
    }


    CarState end;

    double g = 9.81; // m/s^2

    // compute first derivatives of state
    double x_dot = start.velocity * std::cos(start.theta + start.slip_angle);
    double y_dot = start.velocity * std::sin(start.theta + start.slip_angle);
    double v_dot = accel;
    double steer_angle_dot = steer_angle_vel;
    double theta_dot = start.angular_velocity;

    // for eases of next two calculations
    double rear_val = g * p.l_r - accel * p.h_cg;
    double front_val = g * p.l_f + accel * p.h_cg;

    // in case velocity is 0
    double vel_ratio, first_term;
    if (start.velocity == 0) {
        vel_ratio = 0;
        first_term = 0;
    }
    else {
        vel_ratio = start.angular_velocity / start.velocity;
        first_term = p.friction_coeff / (start.velocity * (p.l_r + p.l_f));
    }

    double theta_double_dot = (p.friction_coeff * p.mass / (p.I_z * p.wheelbase)) *
            (p.l_f * p.cs_f * start.steer_angle * (rear_val) +
             start.slip_angle * (p.l_r * p.cs_r * (front_val) - p.l_f * p.cs_f * (rear_val)) -
             vel_ratio * (std::pow(p.l_f, 2) * p.cs_f * (rear_val) + std::pow(p.l_r, 2) * p.cs_r * (front_val)));\

    double slip_angle_dot = (first_term) *
            (p.cs_f * start.steer_angle * (rear_val) -
             start.slip_angle * (p.cs_r * (front_val) + p.cs_f * (rear_val)) +
             vel_ratio * (p.cs_r * p.l_r * (front_val) - p.cs_f * p.l_f * (rear_val))) -
            start.angular_velocity;


    // update state
    end.x = start.x + x_dot * dt;
    end.y = start.y + y_dot * dt;
    end.theta = start.theta + theta_dot * dt;
    end.velocity = start.velocity + v_dot * dt;
    end.steer_angle = start.steer_angle + steer_angle_dot * dt;
    end.angular_velocity = start.angular_velocity + theta_double_dot * dt;
    end.slip_angle = start.slip_angle + slip_angle_dot * dt;
    end.st_dyn = true;

    return end;
}

CarState STKinematics::update_k(
        const CarState start,
        double accel,
        double steer_angle_vel,
        CarParams p,
        double dt) {

    CarState end;

    // compute first derivatives of state
    double x_dot = start.velocity * std::cos(start.theta + start.slip_angle);
    double y_dot = start.velocity * std::sin(start.theta + start.slip_angle);
    double v_dot = accel;
    double steer_angle_dot = steer_angle_vel;
    double theta_dot = start.velocity * std::tan(start.steer_angle) * std::cos(start.slip_angle)/ p.wheelbase ;
    double slip_angle_dot = (1 / (1 + std::pow(((p.l_r/p.wheelbase) * std::tan(start.steer_angle)), 2))) *
            (p.l_r / (p.wheelbase * std::pow(std::cos(start.steer_angle), 2))) * steer_angle_vel;
    double theta_double_dot = accel * std::tan(start.steer_angle) * std::cos(start.slip_angle)/ p.wheelbase  +
            start.velocity * steer_angle_vel * std::cos(start.slip_angle) / (p.wheelbase * std::pow(std::cos(start.steer_angle), 2)) -
            start.velocity * std::sin(start.slip_angle) * std::tan(start.steer_angle) * slip_angle_dot / p.wheelbase;

    // update state
    end.x = start.x + x_dot * dt;
    end.y = start.y + y_dot * dt;
    end.theta = start.theta + theta_dot * dt;
    end.velocity = start.velocity + v_dot * dt;
    end.steer_angle = start.steer_angle + steer_angle_dot * dt;
    end.angular_velocity = start.angular_velocity + theta_double_dot * dt;
    end.slip_angle = start.slip_angle + slip_angle_dot * dt;
    end.st_dyn = false;


    return end;

}
