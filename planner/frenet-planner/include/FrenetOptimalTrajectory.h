// Author: Edward Fang
// Email: edward.fang@berkeley.edu
//
// This code is adapted from
// https://github.com/AtsushiSakai/PythonRobotics/tree/
// master/PathPlanning/FrenetOptimalTrajectory.
// Its author is Atsushi Sakai.
//
// Reference Papers:
// - [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet
// Frame]
// (https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)
// - [Optimal trajectory generation for dynamic street scenarios in a Frenet
// Frame] (https://www.youtube.com/watch?v=Cj6tAQe7UCY)

#ifndef FRENET_OPTIMAL_TRAJECTORY_FRENET_OPTIMAL_TRAJECTORY_H
#define FRENET_OPTIMAL_TRAJECTORY_FRENET_OPTIMAL_TRAJECTORY_H

#include "CubicSpline2D.h"
#include "FrenetPath.h"
#include "py_cpp_struct.h"
#include <ros/ros.h>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <vector>

namespace frenet_planner {

class FrenetOptimalTrajectory {
public:
    FrenetOptimalTrajectory(ros::NodeHandle& nh, bool debug_mode = false);
    ~FrenetOptimalTrajectory();
    // FrenetPath *getBestPath();
    void SetHyperParameters(FrenetHyperparameters *fot_hp_);
    void UpdateCurrentPosition(FrenetInitialConditions &fot_ic_);
    void UpdateTrackWidth(double track_advancement);
    void SetGlobalTrajectory(std::vector<double> *wx, 
                             std::vector<double> *wy, 
                             std::vector<double> *d_right, 
                             std::vector<double> *d_left, 
                             std::vector<double> *v,
                             bool is_closed_contour);
    // void setObstacles();
    // void addObstacle(Eigen::Vector2f first_point, Eigen::Vector2f second_point);
    void CleanUp();
    bool UpdateOptimalTrajectory(FrenetPath *best_path);
    bool UpdateObstacles(std::vector<f110_msgs::Obstacle> obstacles);

private:
    // debug:
    ros::NodeHandle nh_;
    ros::Publisher good_path_pub_;
    ros::Publisher bad_path_pub_;
    bool debug_mode_;
    std::vector<FrenetPath *> deleted_paths;

    FrenetInitialConditions fot_ic;
    FrenetHyperparameters *fot_hp;
    std::mutex *mu;
    FrenetPath *best_frenet_path_ptr;
    CubicSpline2D *csp{nullptr};
    std::vector<f110_msgs::Obstacle> obstacle_array_;
    std::vector<FrenetPath *> frenet_paths;
    void calc_frenet_paths(int start_di_index, int end_di_index,
                           bool multithreaded, FrenetPath *prev_best_path);
    void threaded_calc_all_frenet_paths(FrenetPath *prev_best_path);
};
} // end of namespace frenet_planner
#endif // FRENET_OPTIMAL_TRAJECTORY_FRENET_OPTIMAL_TRAJECTORY_H
