#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <ros/ros.h>

#include "FrenetOptimalTrajectory.h"
#include "QuarticPolynomial.h"
#include "QuinticPolynomial.h"
#include "utils.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

namespace frenet_planner {

// Compute the frenet optimal trajectory
FrenetOptimalTrajectory::FrenetOptimalTrajectory(ros::NodeHandle& nh, 
                                                 bool debug_mode):
    nh_(nh),
    debug_mode_(debug_mode){
    if (debug_mode_) {
        good_path_pub_ = nh_.advertise<nav_msgs::Path>
        ("planner/good_paths", 1000);
        bad_path_pub_ = nh_.advertise<nav_msgs::Path>
        ("planner/deleted_paths", 1000);
    }
}

FrenetOptimalTrajectory::~FrenetOptimalTrajectory() {
    if (csp) {
         delete csp;
    }
   
    // for (Obstacle *ob : obstacles) {
    //     delete ob;
    // }
}

void FrenetOptimalTrajectory::SetHyperParameters(FrenetHyperparameters *fot_hp_) {
   fot_hp = fot_hp_;
}

void FrenetOptimalTrajectory::SetGlobalTrajectory(std::vector<double> *wx, 
                                                  std::vector<double> *wy, 
                                                  std::vector<double> *d_right, 
                                                  std::vector<double> *d_left, 
                                                  std::vector<double> *v,
                                                  bool is_closed_contour) {
    // clean up if not nullptr
    if (csp) {
        delete csp;
        csp = nullptr;
    }
    
    // exit if not enough waypoints
    if (wx->size() < 2 || wx->size() != wy->size()) {
        throw std::runtime_error(std::string("[FrenetOptimalTrajectory] Not enough waypoints for construction")); 
        return;
    }
    csp = new CubicSpline2D(*wx, *wy, *d_right, *d_left, *v, is_closed_contour);
}

bool FrenetOptimalTrajectory::UpdateObstacles(std::vector<f110_msgs::Obstacle> obstacles) {
    obstacle_array_.clear();
    for (auto &ob : obstacles) {
        if (ob.is_actually_a_gap) {
            ob.is_actually_a_gap = false;
            // a gap adds two obstacles, to the left and right
            f110_msgs::Obstacle gap_obstacle = ob;
            
            // left obstacle of gap
            gap_obstacle.id = ob.id + 100;
            gap_obstacle.d_left = csp->calc_d_left(ob.s_start) + fot_hp->track_clearance;
            gap_obstacle.d_right = ob.d_left - fot_hp->obstacle_clearance;
            // check if obstacle has nonzero width
            if (gap_obstacle.d_right <  gap_obstacle.d_left) {
                obstacle_array_.push_back(gap_obstacle);
            }

            // right obstacle of gap
            ob.d_left = ob.d_right + fot_hp->obstacle_clearance;
            ob.d_right = -csp->calc_d_right(ob.s_start) - fot_hp->track_clearance;
            if (ob.d_right <  ob.d_left) {
                obstacle_array_.push_back(ob);
            }
            continue;
        }
        // check if obstacle is within track bounds (do already consider clearance)
        if ( !(ob.d_left < -csp->calc_d_right(ob.s_start) ||
               ob.d_right > csp->calc_d_left(ob.s_start) ||
               (ob.s_start == ob.s_end) )) {
            // might get messy in corner cases where s wraps? TODO check this
            ob.s_start -= fot_hp->obstacle_clearance;
            ob.s_end += fot_hp->obstacle_clearance;
            ob.d_left += fot_hp->obstacle_clearance;
            ob.d_right -= fot_hp->obstacle_clearance;
            obstacle_array_.push_back(ob);
        } 
    }

    // true if we have obstacle that we need to consider
    return (obstacle_array_.size() > 0);
}

void FrenetOptimalTrajectory::UpdateCurrentPosition(
        FrenetInitialConditions &fot_ic_) {
    // constrain the lateral offset to be within track bounds and threshold
    // add 0.05m to avoid infinite cost for distance to track bounds
    fot_ic_.c_d = std::min(fot_ic_.c_d, csp->calc_d_left(fot_ic_.s0) - 0.05);
    fot_ic_.c_d = std::max(fot_ic_.c_d, -csp->calc_d_right(fot_ic_.s0) + 0.05);

    // check if we are close to any obstacle
    if (fot_hp->ic_avoid_obstacles){
        for (auto &ob : obstacle_array_){
            bool consider = false;
            if (ob.s_end < ob.s_start && // handles wrapping
                    (fot_ic_.s0 < ob.s_end || fot_ic_.s0 > ob.s_start)) {
                consider = true;
            } else if (fot_ic_.s0 > ob.s_start && fot_ic_.s0 < ob.s_end) {
                consider = true;
            } 
            if (consider) {
                // move initial condition outside of obstacle to whichever side is closer
                double d_center = 0.5 * (ob.d_left + ob.d_right); 
                if (fot_ic_.c_d > d_center) {
                    fot_ic_.c_d = std::max(fot_ic_.c_d, ob.d_left + 0.05);
                } else {
                    fot_ic_.c_d = std::min(fot_ic_.c_d, ob.d_right - 0.05);
                }
            }
        }
    }
    fot_ic = fot_ic_;
}

void FrenetOptimalTrajectory::UpdateTrackWidth(double track_advancement) {
    // get the track width at s = track advancement + some margin
    fot_hp->max_road_width_r = csp->calc_d_right(track_advancement) + fot_hp->track_clearance + 0.2;
    fot_hp->max_road_width_l = csp->calc_d_left(track_advancement) + fot_hp->track_clearance + 0.2;
}

void FrenetOptimalTrajectory::CleanUp() {
    delete mu;

    if (debug_mode_) {
        int counter = 0;
        for (FrenetPath *fp : frenet_paths) {
            if (counter > 10) {
                std::vector<geometry_msgs::PoseStamped> path_poses;
                for (std::size_t i = 0; i < fp->x.size(); i++) {
                    geometry_msgs::PoseStamped pose_msg;
                    pose_msg.pose.position.x = fp->x[i];
                    pose_msg.pose.position.y = fp->y[i];
                    path_poses.push_back(pose_msg);
                } 
                nav_msgs::Path path_msg;
                path_msg.poses = path_poses;
                path_msg.header.frame_id = "map";
                path_msg.header.stamp = ros::Time::now();
                good_path_pub_.publish(path_msg);
                counter = 0;
            } 
            counter++;
            delete fp;
            fp = nullptr;
        }
        frenet_paths.clear();
        for (FrenetPath *fp : deleted_paths) {
            if (counter > 10) {
                std::vector<geometry_msgs::PoseStamped> path_poses;
                for (std::size_t i = 0; i < fp->x.size(); i++) {
                    geometry_msgs::PoseStamped pose_msg;
                    pose_msg.pose.position.x = fp->x[i];
                    pose_msg.pose.position.y = fp->y[i];
                    path_poses.push_back(pose_msg);
                } 
                nav_msgs::Path path_msg;
                path_msg.poses = path_poses;
                path_msg.header.frame_id = "map";
                path_msg.header.stamp = ros::Time::now();
                bad_path_pub_.publish(path_msg);
                counter = 0;
            } 
            counter++;
            delete fp;
            fp = nullptr;
        }
        deleted_paths.clear();
    } else {
        for (FrenetPath *fp : frenet_paths) {
            delete fp;
            fp = nullptr;
        }
        frenet_paths.clear();
    }
}

// Input previous best path -> is ovewritten by function
bool FrenetOptimalTrajectory::UpdateOptimalTrajectory(FrenetPath *best_path) {
    auto start = chrono::high_resolution_clock::now();
    mu = new mutex();

    // init pointer
    best_frenet_path_ptr = nullptr;

    // calculate the trajectories
    if (fot_hp->num_threads == 0) {
        // calculate how to split computation across threads

        int total_di_iter = static_cast<int>((fot_hp->max_road_width_l +
                                              fot_hp->max_road_width_r) /
                                             fot_hp->d_road_w) +
                            1; // account for the last index

        calc_frenet_paths(0, total_di_iter, false, best_path);

    } else { // if threading
        threaded_calc_all_frenet_paths(best_path);
    }

    // select the best path
    double mincost = std::numeric_limits<double>::infinity();
    for (FrenetPath *fp : frenet_paths) {
        if (fp->cf <= mincost) {
            mincost = fp->cf;
            best_frenet_path_ptr = fp;
        }
    }
    
    auto end = chrono::high_resolution_clock::now();
    if (debug_mode_) {
        double run_time =
        chrono::duration_cast<chrono::nanoseconds>(end - start).count();
        run_time *= 1e-6;
        ROS_INFO_STREAM("Planning runtime " << run_time << "\n");
    }

    // create deep copy of the best path if not nullptr
    if (best_frenet_path_ptr && !best_frenet_path_ptr->x.empty()) {
        *best_path = *best_frenet_path_ptr;
        best_path->CalcDSpline(); // calculate spline only for best path
        CleanUp();
        return true;
    } 
    CleanUp();
    return false;
}

/*
 * Spawn threads to calculate frenet paths
 * Called when multithreading.
 */
void FrenetOptimalTrajectory::threaded_calc_all_frenet_paths(FrenetPath *prev_best_path) {
    std::vector<thread> threads;

    // calculate how to split computation across threads
    int num_di_iter =
        static_cast<int>((fot_hp->max_road_width_l + fot_hp->max_road_width_r) /
                         fot_hp->d_road_w);
    num_di_iter = num_di_iter + 1; // account for the last index

    int iter_di_index_range =
        static_cast<int>(num_di_iter / fot_hp->num_threads);

    for (int i = 0; i < fot_hp->num_threads; i++) {
        if (i != fot_hp->num_threads - 1) {
            threads.push_back(thread(
                &FrenetOptimalTrajectory::calc_frenet_paths, this,
                i * iter_di_index_range, (i + 1) * iter_di_index_range, true,
                prev_best_path));
        } else { // account for last thread edge case
            threads.push_back(
                thread(&FrenetOptimalTrajectory::calc_frenet_paths, this,
                       i * iter_di_index_range, num_di_iter, true, 
                       prev_best_path));
        }
    }

    // wait for all threads to finish computation
    for (auto &t : threads) {
        t.join();
    }
}

/*
 * Calculate frenet paths
 * If running we are multithreading,
 * We parallelize on the outer loop, in terms of di
 * Iterates over possible values of di, from start index to end index
 * (exclusive). Then, computes the actual di value for path planning.
 * Mutex is only enabled when we are multithreading.
 */
void FrenetOptimalTrajectory::calc_frenet_paths(int start_di_index,
                                                int end_di_index,
                                                bool multithreaded,
                                                FrenetPath *prev_best_path) {
    double t, ti, tv;
    double lateral_deviation, lateral_velocity, lateral_acceleration,
        lateral_jerk;
    double longitudinal_deviation, longitudinal_acceleration, longitudinal_jerk;
    FrenetPath *fp, *tfp;
    int num_paths = 0;
    int num_viable_paths = 0;
    // double valid_path_time = 0;

    // initialize di, with start_di_index
    double di = -fot_hp->max_road_width_l + start_di_index * fot_hp->d_road_w;

    // generate path to each offset goal
    // note di goes up to but not including end_di_index*fot_hp->d_road_w
    while ((di < -fot_hp->max_road_width_l + end_di_index * fot_hp->d_road_w) &&
           (di <= fot_hp->max_road_width_r)) {
        ti = fot_hp->mint;
        // lateral motion planning
        while (ti <= fot_hp->maxt) {
            lateral_deviation = 0;
            lateral_velocity = 0;
            lateral_acceleration = 0;
            lateral_jerk = 0;

            fp = new FrenetPath(fot_hp);
            QuinticPolynomial lat_qp = QuinticPolynomial(
                fot_ic.c_d, fot_ic.c_d_d, fot_ic.c_d_dd, di, 0.0, 0.0, ti);

            // construct frenet path
            t = 0;
            while (t <= ti) {
                fp->t.push_back(t);
                fp->d.push_back(lat_qp.calc_point(t));
                fp->d_d.push_back(lat_qp.calc_first_derivative(t));
                fp->d_dd.push_back(lat_qp.calc_second_derivative(t));
                fp->d_ddd.push_back(lat_qp.calc_third_derivative(t));
                lateral_deviation += abs(lat_qp.calc_point(t));
                lateral_velocity += abs(lat_qp.calc_first_derivative(t));
                lateral_acceleration += abs(lat_qp.calc_second_derivative(t));
                lateral_jerk += abs(lat_qp.calc_third_derivative(t));
                t += fot_hp->dt;
            }

            // velocity keeping, only sample valid speeds
            double max_speed = std::min(fot_hp->max_speed, 
                                        fot_ic.c_speed - fot_hp->min_speed + ti * fot_hp->max_accel);
            tv = std::max(fot_hp->min_speed, fot_ic.target_speed - fot_hp->d_t_s * fot_hp->n_s_sample);
            tv = std::min(max_speed, tv);
            double tv_max = fot_ic.target_speed;// + fot_hp->d_t_s * fot_hp->n_s_sample;
            while (tv <= std::min(tv_max, max_speed)) {
                longitudinal_deviation = 0;
                longitudinal_acceleration = 0;
                longitudinal_jerk = 0;

                // copy frenet path
                tfp = new FrenetPath(fot_hp);
                tfp->t.assign(fp->t.begin(), fp->t.end());
                tfp->d.assign(fp->d.begin(), fp->d.end());
                tfp->d_d.assign(fp->d_d.begin(), fp->d_d.end());
                tfp->d_dd.assign(fp->d_dd.begin(), fp->d_dd.end());
                tfp->d_ddd.assign(fp->d_ddd.begin(), fp->d_ddd.end());
                QuarticPolynomial lon_qp = QuarticPolynomial(
                    fot_ic.s0, fot_ic.c_speed, fot_ic.c_s_dd, tv, 0.0, ti);

                // longitudinal motion
                for (double tp : tfp->t) {
                    tfp->s.push_back(lon_qp.calc_point(tp));
                    tfp->s_d.push_back(lon_qp.calc_first_derivative(tp));
                    tfp->s_dd.push_back(lon_qp.calc_second_derivative(tp));
                    tfp->s_ddd.push_back(lon_qp.calc_third_derivative(tp));
                    longitudinal_deviation += abs(tfp->s_d.back() - csp->calc_v(tfp->s.back()));
                    longitudinal_acceleration += abs(tfp->s_dd.back());
                    longitudinal_jerk += abs(tfp->s_ddd.back());
                }


                num_paths++;
                // delete if failure or invalid path
                if (tfp->s.size() < 2 
                        || !tfp->is_within_constraints()
                        || !tfp->is_within_track(csp) 
                        || tfp->is_collision(&obstacle_array_)
                        || !tfp->to_global_path(csp)) {
                    if (debug_mode_) {
                        tfp->to_global_path(csp);
                        if (multithreaded) {
                            // added mutex lock to prevent threads competing to write to
                            // frenet_path
                            mu->lock();
                            deleted_paths.push_back(tfp);
                            mu->unlock();
                        } else {
                            deleted_paths.push_back(tfp);
                        }
                    } else {
                        // deallocate memory and continue   
                        delete tfp;
                    }
                    tv += fot_hp->d_t_s;
                    continue;
                }
                num_viable_paths++;
                
                // auto start = chrono::high_resolution_clock::now();
                // bool valid_path = (tfp->is_within_constraints() &&
                //                    tfp->is_within_track(csp));
                // auto end = chrono::high_resolution_clock::now();
                // valid_path_time +=
                // chrono::duration_cast<chrono::nanoseconds>(end -
                // start).count();
                // if (!valid_path) {
                //     // deallocate memory and continue
                //     delete tfp;
                //     tv += fot_hp->d_t_s;
                //     continue;
                // }

                // temporal cost
                if (prev_best_path->s.size() != 0) {
                    tfp->c_consistency = tfp->calc_difference(prev_best_path);
                }

                // lateral costs
                double path_length_scale = 1.0 / (tfp->t.size());
                tfp->c_lateral_deviation = lateral_deviation;
                tfp->c_lateral_velocity = lateral_velocity;
                tfp->c_lateral_acceleration = lateral_acceleration;
                tfp->c_lateral_jerk = lateral_jerk;
                tfp->c_lateral = path_length_scale * (
                                 fot_hp->k_lat_dev * tfp->c_lateral_deviation +
                                 fot_hp->k_lat_v * tfp->c_lateral_velocity +
                                 fot_hp->k_lat_a * tfp->c_lateral_acceleration +
                                 fot_hp->k_lat_j * tfp->c_lateral_jerk);

                // longitudinal costs
                tfp->c_longitudinal = path_length_scale * (
                                      fot_hp->k_lon_dev * longitudinal_deviation);

                

                // final cost
                tfp->cf = fot_hp->k_lat * tfp->c_lateral +
                          fot_hp->k_lon * tfp->c_longitudinal +
                          fot_hp->k_obstacles * tfp->c_inv_dist_to_obstacles +
                          fot_hp->k_track_bounds * tfp->c_inv_dist_to_track_bound +
                          fot_hp->k_temporal * tfp->c_consistency +
                          fot_hp->k_time * (fot_hp->maxt - ti);
                        //   - 100 * (tfp->s.back() - tfp->s.front()); // track advancement


                if (multithreaded) {
                    // added mutex lock to prevent threads competing to write to
                    // frenet_path
                    mu->lock();
                    frenet_paths.push_back(tfp);
                    mu->unlock();
                } else {
                    frenet_paths.push_back(tfp);
                }

                tv += fot_hp->d_t_s;
            }
            ti += fot_hp->d_sample_t;
            // make sure to deallocate
            delete fp;
        }
        di += fot_hp->d_road_w;
    }
    // valid_path_time *= 1e-6;
    // cout << "NUM THREADS = " << fot_hp->num_threads << "\n"; // check if
    // Thread argument is passed down cout << "Found " << frenet_paths.size() <<
    // " valid paths out of " << num_paths << " paths; Valid path time " <<
    // valid_path_time << "\n";
}

// void FrenetOptimalTrajectory::setObstacles() {
//     // Construct obstacles
//     std::vector<double> llx(fot_ic.o_llx, fot_ic.o_llx + fot_ic.no);
//     std::vector<double> lly(fot_ic.o_lly, fot_ic.o_lly + fot_ic.no);
//     std::vector<double> urx(fot_ic.o_urx, fot_ic.o_urx + fot_ic.no);
//     std::vector<double> ury(fot_ic.o_ury, fot_ic.o_ury + fot_ic.no);

//     for (int i = 0; i < fot_ic.no; i++) {
//         addObstacle(Eigen::Vector2f(llx[i], lly[i]), Eigen::Vector2f(urx[i], ury[i]));
//     }
// }

// void FrenetOptimalTrajectory::addObstacle(Eigen::Vector2f first_point,
//                                           Eigen::Vector2f second_point) {
//     obstacles.push_back(new Obstacle(std::move(first_point),
//                                      std::move(second_point),
//                                      fot_hp->obstacle_clearance));
// }
} // end of namespace frenet_planner