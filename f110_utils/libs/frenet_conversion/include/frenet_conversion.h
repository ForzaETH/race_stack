#ifndef FRENET_CONVERSION_H_
#define FRENET_CONVERSION_H_

#include <ros/ros.h>
#include <f110_msgs/Wpnt.h>
#include <mutex>
#include <vector>

namespace frenet_conversion{
    
class FrenetConverter {
 public:
  FrenetConverter();
  ~FrenetConverter();

  /**
   * @brief Initializes the frenet converter with the global trajectory
   * 
   * @param wptns pointer to vector containing the waypoints describing the 
   * global trajectory, the last and first waypoints should overlap if the 
   * trajectory is wrapping around
   * 
   */
  void SetGlobalTrajectory(const std::vector<f110_msgs::Wpnt> *wptns,
                           const bool is_closed_contour);

  /**
   * @brief Returns the frenet point corresponding to the given position
   * 
   * @param x input x position
   * @param y input y position
   * @param s returns the frenet s coordinate
   * @param d returns the frenet d coordinate
   * @param idx returns the index of the closest waypoint
   * 
   */
  void GetFrenetPoint(const double x, const double y, double* s, double* d, 
                      int* idx, bool full_search);
  
  /**
   * @brief Returns the global point corresponding to the frenet position
   * 
   * @param s input frenet s coordinate
   * @param d input frenet d coordinate
   * @param x output x position
   * @param y output y position
   * 
   */
  void GetGlobalPoint(const double s, const double d, double* x, double* y);

  /**
   * @brief Get the Closest Index on the global trajectory to the given position
   * 
   * @param x input
   * @param y input
   * @param idx returns the index of the closest waypoint
   */
  void GetClosestIndex(const double x, const double y, int* idx);

  /**
   * @brief Get the Closest Index on the global trajectory to track advancement
   * 
   * @param s input track advancement
   * @param idx returns the index of the closest waypoint
   */
  void GetClosestIndex(const double s, int* idx);

  /**
   * @brief Returns the frenet point and locally projected velocity
   *  corresponding to the given position and velocity
   * 
   * @param x input x position
   * @param y input y position
   * @param theta car heading angle
   * @param v_x input x velocity, in body frame
   * @param v_y input y velocity, in body frame
   * @param s returns the frenet s coordinate
   * @param d returns the frenet d coordinate
   * @param v_s returns the frenet s velocity
   * @param v_d returns the frenet d velocity
   * @param idx returns the index of the closest waypoint
   */
  void GetFrenetOdometry(const double x, const double y, const double theta,
                         const double v_x, const double v_y, double* s,
                         double* d, double* v_s, double* v_d, int* idx);

 private:
  /**
   * @brief Calculates the frenet coordinates of the given position
   * 
   * @param x input x position
   * @param y input y position
   * @param s returns the frenet s coordinate
   * @param d returns the frenet d coordinate
   */
  void CalcFrenetPoint(const double x, const double y, double* s, double* d);

  /**
   * @brief Calculates the global position based on the frenet position
   * 
   * @param s input track advancement
   * @param d input track offset
   * @param x output x position
   * @param y output y position
   */
  void CalcGlobalPoint(const double s, const double d, double* x, double* y);

  /**
   * @brief Projects the velocities into local frenet frame
   * 
   * @param v_x input x velocity, in body frame
   * @param v_y input y velocity, in body frame
   * @param theta car heading angle
   * @param v_s returns the frenet s velocity
   * @param v_d returns the frenet d velocity
   */
  void CalcFrenetVelocity(const double v_x, const double v_y, const double theta,
                          double* v_s, double* v_d);

  /**
   * @brief Updates the closest index of waypoint array to the given position
   * 
   * @param x input x position
   * @param y input y position
   */
  void UpdateClosestIndex(const double x, const double y,  int* idx, bool full_search);

  /**
   * @brief Updates the closest index of waypoint array to the given 
   * track advancement
   * 
   * @param s 
   */
  void UpdateClosestIndex(const double s);

  int closest_idx_;
  std::vector<f110_msgs::Wpnt> wpt_array_;
  bool has_global_trajectory_{false};
  double global_trajectory_length_;
  bool is_closed_contour_;
  std::mutex mutexGlobalTrajectory_;

};
   
}// end namespace frenet_conversion

#endif /* FRENET_CONVERSION_H_ */