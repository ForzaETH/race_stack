#ifndef FRENET_ODOM_REPUBLISHER_H_
#define FRENET_ODOM_REPUBLISHER_H_

#include <ros/ros.h>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <f110_msgs/WpntArray.h>
#include <f110_msgs/Wpnt.h>

#include "frenet_conversion.h"

namespace frenet_odom_republisher{
    
class FrenetRepublisher {
 public:
  FrenetRepublisher(ros::NodeHandle& nh);
  ~FrenetRepublisher();
 private:
  void InitSubscribersPublishers();
  void GlobalTrajectoryCallback(const f110_msgs::WpntArrayConstPtr &wpt_array);
  void OdomCallback(const nav_msgs::OdometryConstPtr &msg);

  ros::NodeHandle nh_;
  ros::Subscriber global_trajectory_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher frenet_odom_pub_;

  std::vector<f110_msgs::Wpnt> wpt_array_;
  int closest_wpt_index_{0};

  frenet_conversion::FrenetConverter frenet_converter_;

  bool has_global_trajectory_{false};
};
   
}// end namespace frenet_odom_republisher

#endif /* FRENET_ODOM_REPUBLISHER_H_ */