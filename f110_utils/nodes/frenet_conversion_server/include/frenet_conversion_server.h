#ifndef FRENET_CONVERSION_SERVER_H_
#define FRENET_CONVERSION_SERVER_H_

#include <ros/ros.h>
#include <f110_msgs/WpntArray.h>
#include "frenet_conversion.h"
#include "frenet_conversion/Frenet2Glob.h"
#include "frenet_conversion/Frenet2GlobArr.h"
#include "frenet_conversion/Glob2Frenet.h"
#include "frenet_conversion/Glob2FrenetArr.h"


namespace frenet_conversion_server
{

  class FrenetConversionServer
  {
  public:
    FrenetConversionServer(ros::NodeHandle &nh, bool PerceptionOnly);
    ~FrenetConversionServer();
    bool PerceptionOnly_;

  private:
    void InitSubscribersPublishers();
    void GlobalTrajectoryCallback(const f110_msgs::WpntArrayConstPtr &wpt_array);
    bool Glob2FrenetConversionCallback(
        frenet_conversion::Glob2Frenet::Request &req,
        frenet_conversion::Glob2Frenet::Response &res);
    bool Frenet2GlobConversionCallback(
      frenet_conversion::Frenet2Glob::Request &req,
      frenet_conversion::Frenet2Glob::Response &res);
    bool Glob2FrenetArrConversionCallback(
        frenet_conversion::Glob2FrenetArr::Request &req,
        frenet_conversion::Glob2FrenetArr::Response &res);
    bool Frenet2GlobArrConversionCallback(
      frenet_conversion::Frenet2GlobArr::Request &req,
      frenet_conversion::Frenet2GlobArr::Response &res);

    ros::NodeHandle nh_;
    ros::Subscriber global_trajectory_sub_;
    ros::ServiceServer convert_glob2frenet_server_;
    ros::ServiceServer convert_glob2frenetarr_server_;
    ros::ServiceServer convert_frenet2glob_server_;
    ros::ServiceServer convert_frenet2globarr_server_;

    std::vector<f110_msgs::Wpnt> wpt_array_;

    frenet_conversion::FrenetConverter frenet_converter_;

    bool has_global_trajectory_{false};
  };

} // end namespace frenet_conversion_server

#endif /* FRENET_CONVERSION_SERVER_H_ */