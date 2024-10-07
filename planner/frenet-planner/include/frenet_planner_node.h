#ifndef FRENET_PLANNER_H_
#define FRENET_PLANNER_H_

#include <ros/ros.h>
#include "FrenetOptimalTrajectory.h"
#include "FrenetPath.h"
#include "py_cpp_struct.h"
#include <mutex>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/Config.h>

#include <f110_msgs/WpntArray.h>
#include <f110_msgs/Wpnt.h>
#include <f110_msgs/Obstacle.h> 
#include <f110_msgs/ObstacleArray.h> 

#include "frenet_conversion.h"

namespace frenet_planner{
    
class FrenetPlanner {
 public:
  FrenetPlanner(ros::NodeHandle& nh,
      ros::NodeHandle& nh_private, bool debug, bool measure);
  ~FrenetPlanner();
 private:
  void InitSubscribersPublishers();
  void LoadHyperParameters();
  void GlobalTrajectoryCallback(const f110_msgs::WpntArrayConstPtr &wpt_array);
  void OdomCallback(const nav_msgs::OdometryConstPtr &msg);
  void ObstacleCallback(const f110_msgs::ObstacleArrayConstPtr &msg);
  void InitConditions();
  void MainLoop();
  bool FindLocalTrajectory();
  bool UpdateInitialConditions();
  void PublishLocalTrajectory(const FrenetPath* path);
  void PublishEmptyTrajectory();
  void PublishObstacles(const std::vector<f110_msgs::Obstacle> &obstacles);
  void PublishInitialPosition();
  bool UpdatePath();
  void PrintInitialConditions();
  void UpdateHyperParamsOT();
  void DynamicOTHandle();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber global_trajectory_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber obstacle_sub_;
  ros::Publisher local_trajectory_pub_;
  ros::Publisher local_trajectory_marker_pub_;
  ros::Publisher obstacle_marker_pub_;
  ros::Publisher initial_position_marker_pub_;
  ros::Publisher planner_dyn_params_pub_;
  ros::Publisher latency_pub_;

  std::mutex mutexGlobalTrajectory_;
  std::mutex mutexBestTrajectory_;

  FrenetHyperparameters fot_hp_;
  FrenetInitialConditions fot_ic_;
  FrenetOptimalTrajectory frenet_solver_;
  FrenetPath best_frenet_path_;

  frenet_conversion::FrenetConverter frenet_converter_;

  // ros params
  bool debug_;
  bool measure_;
  bool glob_wpnt_initialised_{false};
  bool use_car_state_;
  bool pub_only_if_obstacles_;
  bool pub_global_track_advancement_;
  double max_rate_;
  double speed_scale_;
  double replanning_distance_;

  std::vector<f110_msgs::Wpnt> wpt_array_;
  bool has_global_trajectory_{false};
  bool has_obstacles_{false};
  double global_path_length_;
  bool on_path_{false};
  int on_path_count_ = 0;
  int id_start_;

  double odom_x_;
  double odom_y_;
  double odom_yaw_;
  double v_x_;
  double v_y_;
  //For scaled wpnt counter to subsample 
  int wpnt_cnt_ = 0;
  double max_gb_speed_ = 10;

  //For dynamic hyperparam adjustment in OT sectors
  struct OTParameters {double start, end; bool val;};
  std::vector<OTParameters> ot_sectors_params_;
};
   
}// end namespace frenet_planner

#endif /* FRENET_PLANNER_H_ */