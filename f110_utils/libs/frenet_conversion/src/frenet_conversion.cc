#include "frenet_conversion.h"
#include <math.h>
#include <ros/ros.h>

namespace frenet_conversion{

  FrenetConverter::FrenetConverter() {}

  FrenetConverter::~FrenetConverter() {}

  /*** -------------- PUBLIC ----------------- ***/

  void FrenetConverter::SetGlobalTrajectory(
                                    const std::vector<f110_msgs::Wpnt> *wptns,
                                    const bool is_closed_contour) {
    // deep copy wptns
    wpt_array_ = *wptns;
    global_trajectory_length_ = wpt_array_.back().s_m;
    is_closed_contour_ = is_closed_contour;
    // remove the first element -> no negative s
    if (is_closed_contour_) {
      wpt_array_.erase(wpt_array_.begin());
    }
    has_global_trajectory_ = true;
  }

  void FrenetConverter::GetFrenetPoint(const double x, const double y, 
                                       double* s, double* d, int* idx, bool full_search) {

    if (!has_global_trajectory_) {
      ROS_ERROR("[FrenetConverter] No global trajectory set!");
      return;
    }

    UpdateClosestIndex(x, y, idx, full_search);
  

    // calculate frenet point
    CalcFrenetPoint(x, y, s, d);
  }

  void FrenetConverter::GetFrenetOdometry(const double x, const double y, 
                                          const double theta, const double v_x, 
                                          const double v_y, double* s, 
                                          double* d, double* v_s, double* v_d,
                                          int* idx) {
    
    if (!has_global_trajectory_) {
      ROS_ERROR("[FrenetConverter] No global trajectory set!");
      return;
    }
    std::unique_lock<std::mutex> lock(mutexGlobalTrajectory_);   

    UpdateClosestIndex(x, y, idx, false);

    CalcFrenetPoint(x, y, s, d);

    CalcFrenetVelocity(v_x, v_y, theta, v_s, v_d);
    lock.unlock();
  }

  void FrenetConverter::GetGlobalPoint(const double s, const double d, 
                                       double* x, double* y) {

    if (!has_global_trajectory_) {
      ROS_ERROR("[FrenetConverter] No global trajectory set!");
      return;
    }
    std::unique_lock<std::mutex> lock(mutexGlobalTrajectory_);

    UpdateClosestIndex(s);
    
    // calculate frenet point
    CalcGlobalPoint(s, d, x, y);
    lock.unlock();
  }

  void FrenetConverter::GetClosestIndex(const double s, int* idx) {
    if (!has_global_trajectory_) {
      ROS_ERROR("[FrenetConverter] No global trajectory set!");
      return;
    }
    UpdateClosestIndex(s);
    *idx = (closest_idx_ + 1); // account for removing the first element
  }

  void FrenetConverter::GetClosestIndex(const double x, const double y, 
                                         int* idx) {
    if (!has_global_trajectory_) {
      ROS_ERROR("[FrenetConverter] No global trajectory set!");
      return;
    }
    UpdateClosestIndex(x, y, idx, false);
  }

  /*** -------------- PRIVATE ----------------- ***/

  void FrenetConverter::CalcFrenetPoint(const double x, const double y,
                                             double* s, double* d) {
    // project current position onto trajectory:
    // s = s_wp + cos(phi)*dx + sin(phi)*dy
    double d_x = x - wpt_array_.at(closest_idx_).x_m;
    double d_y = y - wpt_array_.at(closest_idx_).y_m;

    *s = d_x * std::cos(wpt_array_.at(closest_idx_).psi_rad) +
        d_y * std::sin(wpt_array_.at(closest_idx_).psi_rad) + 
        wpt_array_.at(closest_idx_).s_m;
    if (is_closed_contour_) {
       // limit to length of global trajectory
      *s = std::fmod(*s, global_trajectory_length_);
    }
    *d = - d_x * std::sin(wpt_array_.at(closest_idx_).psi_rad) +
        d_y * std::cos(wpt_array_.at(closest_idx_).psi_rad);
  }

  void FrenetConverter::CalcGlobalPoint(const double s, const double d,
                                        double* x, double* y) {
    double d_s = s - wpt_array_.at(closest_idx_).s_m;
    *x = wpt_array_.at(closest_idx_).x_m 
        + d_s * std::cos(wpt_array_.at(closest_idx_).psi_rad) 
        - d * std::sin(wpt_array_.at(closest_idx_).psi_rad);
    *y = wpt_array_.at(closest_idx_).y_m 
        + d * std::cos(wpt_array_.at(closest_idx_).psi_rad) 
        + d_s * std::sin(wpt_array_.at(closest_idx_).psi_rad);
  }

  void FrenetConverter::CalcFrenetVelocity(const double v_x, const double v_y, 
                                          const double theta, double* v_s, 
                                          double* v_d) {
    double delta_psi = theta - wpt_array_.at(closest_idx_).psi_rad;
    *v_s = v_x * std::cos(delta_psi) - v_y * std::sin(delta_psi);
    *v_d = v_x * std::sin(delta_psi) + v_y * std::cos(delta_psi);
  }

  // TODO speed up by using intelligent search (binary search)
  void FrenetConverter::UpdateClosestIndex(const double x, const double y,  
                                           int* idx, bool full_search) {
    // get the closest waypoint
    double min_dist = INFINITY;
    int search_ahead_radius = 20; // TODO don't hardcode this
    int start = (wpt_array_.size() + *idx - 1) % wpt_array_.size();
    int end_idx = (start + search_ahead_radius) % wpt_array_.size();

    // -- proximity search: -- 
    if (!full_search) {
      if (end_idx < start) {
        // search end of array
        for (int i = start; i < wpt_array_.size(); i++) {
          double d_squared = std::pow(x - wpt_array_[i].x_m, 2) +
                            std::pow(y - wpt_array_[i].y_m, 2);
          if (d_squared < min_dist) {
            min_dist = d_squared;
            closest_idx_ = i;
          }
        } 
        for (int i = 0; i < end_idx; i++) {
          double d_squared = std::pow(x - wpt_array_[i].x_m, 2) +
                            std::pow(y - wpt_array_[i].y_m, 2);
          if (d_squared < min_dist) {
            min_dist = d_squared;
            closest_idx_ = i;
          }
        }
      } else {
        for (int i = start; i < end_idx; i++) {
          double d_squared = std::pow(x - wpt_array_[i].x_m, 2) +
                            std::pow(y - wpt_array_[i].y_m, 2);
          if (d_squared < min_dist) {
            min_dist = d_squared;
            closest_idx_ = i;
          }
        }
      }
    }
    // if we didn't find anything good in proximity, search the whole array
    if (min_dist > 4 || full_search) { // TODO don't hardcode this
      //ROS_WARN_STREAM("[Converter] Searching Entire Array");
      for (int i = 0; i < wpt_array_.size(); i++) {
        double d_squared = std::pow(x - wpt_array_[i].x_m, 2) +
                          std::pow(y - wpt_array_[i].y_m, 2);
        if (d_squared < min_dist) {
          min_dist = d_squared;
          closest_idx_ = i;
        }
      }
    }
    *idx = (closest_idx_ + 1); // account for removing the first element
  }

  void FrenetConverter::UpdateClosestIndex(const double s) {
    closest_idx_ = wpt_array_.size() - 1; // lazy don't handle wrapping
    for (int i = 1; i < wpt_array_.size(); i++) {
      // find first waypoint with s > s
      if (wpt_array_[i].s_m > s) {
        if (wpt_array_[i].s_m - s > s - wpt_array_[i-1].s_m) {
          closest_idx_ = i - 1;
          break;
        } else {
          closest_idx_ = i;
          break;
        }
      }
    }
  }

} // end of namespace frenet_conversion
