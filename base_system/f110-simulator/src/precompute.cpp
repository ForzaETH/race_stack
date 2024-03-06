#include <math.h>
#include <cmath>
#include <vector>

#include "f1tenth_simulator/precompute.hpp"

using namespace racecar_simulator;

std::vector<double> Precompute::get_car_distances(int scan_beams, double wheelbase, double width, 
    double scan_distance_to_base_link, double angle_min, double scan_ang_incr) {
    // Precompute distance from lidar to edge of car for each beam

    std::vector<double> car_distances = std::vector<double>();
    car_distances.reserve(scan_beams);
    double dist_to_sides = width / 2.0;
    double dist_to_front = wheelbase - scan_distance_to_base_link;
    double dist_to_back = scan_distance_to_base_link;
    // loop through each angle
    for (int i = 0; i < scan_beams; i++) {
        double angle = angle_min + i * scan_ang_incr;

        if (angle > 0) {
            if (angle < PI / 2.0) {
                // between 0 and pi/2
                double to_side = dist_to_sides / std::sin(angle);
                double to_front = dist_to_front / std::cos(angle);
                car_distances[i] = std::min(to_side, to_front);
            } else {
                // between pi/2 and pi
                double to_side = dist_to_sides / std::cos(angle - PI / 2.0);
                double to_back = dist_to_back / std::sin(angle - PI / 2.0);
                car_distances[i] = std::min(to_side, to_back);
            } 
        } else {
            if (angle > -PI / 2.0) {
                // between 0 and -pi/2
                double to_side = dist_to_sides / std::sin(-angle);
                double to_front = dist_to_front / std::cos(-angle);
                car_distances[i] = std::min(to_side, to_front);
            } else {
                // between -pi/2 and -pi
                double to_side = dist_to_sides / std::cos(-angle - PI / 2.0);
                double to_back = dist_to_back / std::sin(-angle - PI / 2.0);
                car_distances[i] = std::min(to_side, to_back);
            }
          }
        }

    return car_distances;
}

std::vector<double> Precompute::get_cosines(int scan_beams, double angle_min, double scan_ang_incr) {
    // Precompute distance from lidar to edge of car for each beam
    std::vector<double> cosines = std::vector<double>();
    cosines.reserve(scan_beams);

    for (int i = 0; i < scan_beams; i++) {
        double angle = angle_min + i * scan_ang_incr;
        cosines[i] = std::cos(angle);
    }

    return cosines;
}