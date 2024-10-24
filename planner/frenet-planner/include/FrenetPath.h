#ifndef FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H
#define FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H

#include "py_cpp_struct.h"
#include "CubicSpline2D.h"
#include "CubicSpline1D.h"

#include <eigen3/Eigen/Dense>
#include <vector>
#include <tuple>

#include <ros/ros.h>
#include <f110_msgs/Obstacle.h> 

namespace frenet_planner {

class FrenetPath {
public:
    // Frenet attributes
    std::vector<double> t;          // time
    std::vector<double> d;          // lateral offset
    std::vector<double> d_d;        // lateral speed
    std::vector<double> d_dd;       // lateral acceleration
    std::vector<double> d_ddd;      // lateral jerk
    std::vector<double> s;          // s position along spline
    std::vector<double> s_d;        // s speed
    std::vector<double> s_dd;       // s acceleration
    std::vector<double> s_ddd;      // s jerk
    CubicSpline1D d_spline_;         // d spline
    std::vector<double> d_left;
    std::vector<double> d_right;

    // Euclidean attributes
    std::vector<double> x;          // x position
    std::vector<double> y;          // y position
    std::vector<double> yaw;        // yaw in rad
    std::vector<double> ds;         // change of position
    std::vector<double> c;          // curvature

    // Debug
    std::vector<double> ix;
    std::vector<double> iy;
    std::vector<double> iyaw;

    // Cost attributes
    // lateral costs
    double c_lateral_deviation = 0.0;
    double c_lateral_velocity = 0.0;
    double c_lateral_acceleration = 0.0;
    double c_lateral_jerk = 0.0;
    double c_lateral = 0.0;

    // longitudinal costs
    double c_longitudinal_acceleration = 0.0;
    double c_longitudinal_jerk = 0.0;
    double c_time_taken = 0.0;
    double c_end_speed_deviation = 0.0;
    double c_speed_deviation = 0.0;
    double c_longitudinal = 0.0;

    // obstacle costs
    double c_inv_dist_to_obstacles = 0.0;
    double c_inv_dist_to_track_bound = 0.0;

    // temporal cost
    double c_consistency = 0.0;

    // final cost
    double cf = 0.0;

    FrenetPath(FrenetHyperparameters *fot_hp_);
    FrenetPath();
    double calc_difference(FrenetPath* other);
    void SetHyperParameters(FrenetHyperparameters *fot_hp_);
    void CalcDSpline();
    bool to_global_path(CubicSpline2D* csp);
    bool is_within_constraints();
    bool is_within_track(CubicSpline2D* csp);
    bool is_collision(const std::vector<f110_msgs::Obstacle> *obstacles);
    // double inverse_distance_to_obstacles(
    //     const std::vector<Obstacle *> obstacles);

private:
    // Hyperparameters
    FrenetHyperparameters *fot_hp;
};
} // end of namespace

#endif //FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H
