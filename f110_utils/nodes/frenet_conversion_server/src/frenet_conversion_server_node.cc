#include "frenet_conversion_server.h"

namespace frenet_conversion_server
{

  FrenetConversionServer::FrenetConversionServer(ros::NodeHandle &nh, bool PerceptionOnly) : nh_(nh), PerceptionOnly_(PerceptionOnly) {
    InitSubscribersPublishers();
    ROS_INFO("[Frenet Conversion] Waiting for global waypoints...");
    ros::topic::waitForMessage<f110_msgs::WpntArray>("/global_waypoints");
    ROS_INFO("[Frenet Conversion] Frenet Conversion Server ready.");
    ros::spin();
  }

  FrenetConversionServer::~FrenetConversionServer() {}

  void FrenetConversionServer::InitSubscribersPublishers()
  {
    global_trajectory_sub_ = nh_.subscribe<f110_msgs::WpntArray>("/global_waypoints", 10, &FrenetConversionServer::GlobalTrajectoryCallback, this);
    ROS_INFO("[Frenet Conversion] PERCEPTION ONLY: %d", PerceptionOnly_);
    if(!PerceptionOnly_){
    convert_glob2frenet_server_ = nh_.advertiseService(
        "convert_glob2frenet_service",
        &FrenetConversionServer::Glob2FrenetConversionCallback, this);
    convert_glob2frenetarr_server_ = nh_.advertiseService(
        "convert_glob2frenetarr_service",
        &FrenetConversionServer::Glob2FrenetArrConversionCallback, this);
    convert_frenet2glob_server_ = nh_.advertiseService(
        "convert_frenet2glob_service",
        &FrenetConversionServer::Frenet2GlobConversionCallback, this);
    convert_frenet2globarr_server_ = nh_.advertiseService(
        "convert_frenet2globarr_service",
        &FrenetConversionServer::Frenet2GlobArrConversionCallback, this);
    }
    else{
      convert_glob2frenet_server_ = nh_.advertiseService(
        "convert_glob2frenet_perception_service",
        &FrenetConversionServer::Glob2FrenetConversionCallback, this);
      convert_glob2frenetarr_server_ = nh_.advertiseService(
          "convert_glob2frenetarr_perception_service",
          &FrenetConversionServer::Glob2FrenetArrConversionCallback, this);
      convert_frenet2glob_server_ = nh_.advertiseService(
          "convert_frenet2glob_perception_service",
          &FrenetConversionServer::Frenet2GlobConversionCallback, this);
      convert_frenet2globarr_server_ = nh_.advertiseService(
          "convert_frenet2globarr_perception_service",
          &FrenetConversionServer::Frenet2GlobArrConversionCallback, this);
    }
  }

  bool FrenetConversionServer::Glob2FrenetConversionCallback(
      frenet_conversion::Glob2Frenet::Request &req,
      frenet_conversion::Glob2Frenet::Response &res)
  {
    frenet_converter_.GetFrenetPoint(
        req.x, req.y, &res.s, &res.d, &res.idx, true);
    return true;
  }

  bool FrenetConversionServer::Glob2FrenetArrConversionCallback(
      frenet_conversion::Glob2FrenetArr::Request &req,
      frenet_conversion::Glob2FrenetArr::Response &res)
  {
    std::vector<double> s,d;
    std::vector<int> idx;
    for (int i = 0; i < req.x.size(); i++)
    {
        double s_i,d_i;
        int idx_i;
        frenet_converter_.GetFrenetPoint(req.x[i], req.y[i], &s_i, &d_i, &idx_i, true);
        s.push_back(s_i);
        d.push_back(d_i);
        idx.push_back(idx_i);
    }
    res.s = s;
    res.d = d;
    res.idx = idx;
    return true;
  }


  bool FrenetConversionServer::Frenet2GlobConversionCallback(
      frenet_conversion::Frenet2Glob::Request &req,
      frenet_conversion::Frenet2Glob::Response &res)
  {
    frenet_converter_.GetGlobalPoint(
        req.s, req.d, &res.x, &res.y);
    return true;
  }

  bool FrenetConversionServer::Frenet2GlobArrConversionCallback(
    frenet_conversion::Frenet2GlobArr::Request &req,
    frenet_conversion::Frenet2GlobArr::Response &res)
  {
    std::vector<double> x,y;
    for (int i = 0; i < req.s.size(); i++)
    {
        double x_i,y_i;
        frenet_converter_.GetGlobalPoint(req.s[i], req.d[i], &x_i, &y_i);
        x.push_back(x_i);
        y.push_back(y_i);
    }
    res.x = x;
    res.y = y;
    return true;
  }

  void FrenetConversionServer::GlobalTrajectoryCallback(
      const f110_msgs::WpntArrayConstPtr &wpt_array)
  {
    wpt_array_ = wpt_array->wpnts;
    bool enable_wrapping = true;
    frenet_converter_.SetGlobalTrajectory(&wpt_array_, enable_wrapping);
    if (!has_global_trajectory_)
    {
      ROS_INFO("Global waypoints received.");
    }
    has_global_trajectory_ = true;
  }
} // end namespace frenet_conversion_server

// launch node
int main(int argc, char **argv)
{
  ros::init(argc, argv, "frenet_conversion_server");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  bool PerceptionOnly;
  if (!private_nh.getParam("PerceptionOnly", PerceptionOnly))
  {
    PerceptionOnly = false;
  }
  else
  {
    private_nh.getParam("PerceptionOnly", PerceptionOnly);
  }

  frenet_conversion_server::FrenetConversionServer node(nh, PerceptionOnly);
  return 0;
}
