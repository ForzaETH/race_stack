#include "FrenetPath.h"
#include "utils.h"
#include <ros/ros.h>

#include <algorithm>

const double pi = 3.14159265358979323846;
const float COLLISION_CHECK_THRESHOLD = 2; // don't check unless within 6m

namespace frenet_planner{
FrenetPath::FrenetPath(FrenetHyperparameters *fot_hp_) {
    fot_hp = fot_hp_;
}

FrenetPath::FrenetPath() {}

void FrenetPath::SetHyperParameters(FrenetHyperparameters *fot_hp_) {
    fot_hp = fot_hp_;
}

void FrenetPath::CalcDSpline(){
    d_spline_ = CubicSpline1D(this->s, this->d, false);
}

// assume other was already splineified
double FrenetPath::calc_difference(FrenetPath* other) {
    // find range of s contained in both paths
    double diff = 0.0;
    for (std::size_t i = 0; i <= this->s.size(); i++) {
        if (this->s[i] > other->s[0] && this->s[i] < other->s.back()) {
            diff  += std::pow(this->d[i] - other->d_spline_.calc_der0(this->s[i]), 2);
        }
    }
    return diff;
}

// Convert the frenet path to global path in terms of x, y, yaw, velocity
bool FrenetPath::to_global_path(CubicSpline2D* csp) {
    double ix_, iy_, iyaw_, di, fx, fy, dx, dy;
    // calc global positions
    for (std::size_t i = 0; i < s.size(); i++) {
        ix_ = csp->calc_x(s[i]);
        iy_ = csp->calc_y(s[i]);
        if (isnan(ix_) || isnan(iy_)) break;

        iyaw_ = csp->calc_yaw(s[i]);
        ix.push_back(ix_);
        iy.push_back(iy_);
        iyaw.push_back(iyaw_);
        di = d[i];
        fx = ix_ + di * cos(iyaw_ + pi/2.0);
        fy = iy_ + di * sin(iyaw_ + pi/2.0);
        x.push_back(fx);
        y.push_back(fy);
    }

    // not enough points to construct a valid path
    if (x.size() <= 1) {
        return false;
    }

    // calc yaw and ds
    for (std::size_t i = 0; i < x.size() - 1; i++) {
        dx = x[i+1] - x[i];
        dy = y[i+1] - y[i];
        yaw.push_back(std::atan2(dy, dx));
        // ds.push_back(std::hypot(dx, dy));
        ds.push_back(norm(dx, dy));
    }
    yaw.push_back(yaw.back());
    ds.push_back(ds.back());

    // calc curvature
    for (std::size_t i = 0; i < yaw.size() - 1; i++) {
        // don't consider points for which ds is too small, causes large numeric error
        // and rejection of paths that are viable
        double dyaw = yaw[i+1] - yaw[i];
        if (dyaw > pi/2.0) {
            dyaw -= 2*pi;
        } else if (dyaw < -pi/2.0) {
            dyaw += 2*pi;
        }
        double curvature = dyaw / ds[i];
        if (ds[i] < 0.1) {
            curvature = std::min(fot_hp->max_curvature,std::max(-fot_hp->max_curvature, curvature));
        }
        c.push_back(curvature);
        //c.push_back(dyaw / fot_hp->dt);
    }
    // max curvature check
    if (any_of(c.begin(), c.end(),
            [this](int i){return std::abs(i) > fot_hp->max_curvature;})) {
        return false;
    }

    return true;
}

// Validate the calculated frenet paths against threshold speed, acceleration,
// curvature and collision checks
bool FrenetPath::is_within_constraints() {
    if (any_of(s_d.begin(), s_d.end(),
            [this](int i){return std::abs(i) > fot_hp->max_speed;})) {
        return false;
    }
    // max accel check
    else if (any_of(s_dd.begin(), s_dd.end(),
            [this](int i){return std::abs(i) > fot_hp->max_accel;})) {
        return false;
    }
    else {
        return true;
    }
}

bool FrenetPath::is_within_track(CubicSpline2D* csp) {
    double min_d = INFINITY;
    for (std::size_t i = 0; i < s.size(); i++) {
        double d_r = d[i] + csp->calc_d_right(s[i]);
        double d_l = csp->calc_d_left(s[i]) - d[i];
    
        if (d_r <= 0.0 ||d_l <= 0.0) {
            return false;
        } else {
            d_right.push_back(d_r);
            d_left.push_back(d_l);
            min_d = std::min(min_d, std::min(d_l, d_r));
        }
    }
    // initialize c_inv_dist with dist to track bounds, 
    // should be zero when in middle of track 
    c_inv_dist_to_track_bound = (fot_hp->max_road_width_r / min_d) - 1;
    return true;
}

// check path for collision with obstacles
bool FrenetPath::is_collision(const std::vector<f110_msgs::Obstacle> *obstacles) {
    // no obstacles
    if (obstacles->empty()) {
        return false;
    }
    c_inv_dist_to_obstacles = 0.0;
    // filter relevant obstacles
    std::vector<f110_msgs::Obstacle> relevant_obstacles;
    for (auto obstacle : *obstacles) {
        // ROS_WARN_STREAM(obstacle.s_start)
        // if we start in second half of the track, 
        // add track length to obstacles s in the other half
        if (s[0] > fot_hp->global_path_length / 2){
            if (obstacle.s_start < fot_hp->global_path_length / 2) {
                obstacle.s_start += fot_hp->global_path_length;
            }
            if (obstacle.s_end < fot_hp->global_path_length / 2) {
                obstacle.s_end += fot_hp->global_path_length;
            }
        } else {
            // if we start in first half of the track, 
            // substract track length from obstacles that wrap, otherwise not relevant
            if (obstacle.s_start > obstacle.s_end) {
                obstacle.s_start -= fot_hp->global_path_length;
            }
        }
        
        double k_obstacle_lookahead = 5.0;
        
        if ((obstacle.s_start >= s[0] &&
            obstacle.s_start <= s.back() + k_obstacle_lookahead)) {
            // obstacle starts during or shortly after path
            relevant_obstacles.push_back(obstacle);
        } else if (obstacle.s_start < s[0] && obstacle.s_end > s[0]) {
            // obstacle starts before path and covers some section of path
            relevant_obstacles.push_back(obstacle);
        }
    }

    // consider all obstacles
    // for (auto &obstacle : *obstacles) {
    //         relevant_obstacles.push_back(obstacle);
    //         if (relevant_obstacles.back().s_start > obstacle.s_end) {
    //             // obstacle wraps, unwrap for later checks
    //             relevant_obstacles.back().s_end += fot_hp->global_path_length;
    //         }
    // }
    for (auto &obstacle : relevant_obstacles) {
        // ROS_WARN_STREAM(obstacle);
        // ensure that at least one point is checked
        bool first_point = true;
        double d_check;
        double min_d = INFINITY;
        for (std::size_t i = 1; i < s.size(); i++) {
            if (s[i] >= obstacle.s_start) {
                // handle leading edge of obstacle
                if(first_point) {
                    // check in between points to prevent clipping
                    d_check = (d[i] - d[i-1]) / (s[i] - s[i-1]) 
                            * (obstacle.s_start - s[i-1]) + d[i-1];

                    if (d_check >= obstacle.d_right &&
                        d_check <= obstacle.d_left) {
                        return true;
                    } else if (d_check >= obstacle.d_left) {
                        min_d = std::min(min_d, d_check - obstacle.d_left);
                    } else if (d[i] <= obstacle.d_right) {
                        min_d = std::min(min_d, obstacle.d_right - d_check);
                    }
                    first_point = false;
                }
                
                // handle trailing edge of obstacle
                if (s[i] > obstacle.s_end) {
                    // check in between points to prevent clipping
                    d_check = (d[i] - d[i-1]) / (s[i] - s[i-1]) 
                            * (obstacle.s_end - s[i-1]) + d[i-1];

                    if (d_check >= obstacle.d_right &&
                        d_check <= obstacle.d_left) {
                        return true;
                    } else if (d_check >= obstacle.d_left) {
                        min_d = std::min(min_d, d_check - obstacle.d_left);
                    } else if (d[i] <= obstacle.d_right) {
                        min_d = std::min(min_d, obstacle.d_right - d_check);
                    }
                    break;
                } 
                
                // handle normal point
                d_check = d[i];
                if (d_check >= obstacle.d_right &&
                    d_check <= obstacle.d_left) {
                    return true;
                } else if (d_check >= obstacle.d_left) {
                    min_d = std::min(min_d, d_check - obstacle.d_left);
                } else if (d[i] <= obstacle.d_right) {
                    min_d = std::min(min_d, obstacle.d_right - d_check);
                }
            }
        }
        // if obstacle starts after path, first_point was never set to false
        if (first_point) {
            double dd_sq = 0.0;
            if (d.back() >= obstacle.d_left) {
                dd_sq = std::pow(d.back() - obstacle.d_left, 2);
            } else if (d.back() <= obstacle.d_right) {
                dd_sq = std::pow(obstacle.d_right - d.back(), 2);
            }
            double ds_sq = std::pow(obstacle.s_start - s.back(), 2);
            min_d = std::sqrt(dd_sq + ds_sq);
        }
        if (min_d < INFINITY) {
            c_inv_dist_to_obstacles += 1.0 / min_d;
        }
    }
    // no collisition
    return false;
}

} // end of namespace
