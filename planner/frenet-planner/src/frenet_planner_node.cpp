#include "FrenetOptimalTrajectory.h"
#include "FrenetPath.h"
#include "py_cpp_struct.h"
#include <frenet_planner_node.h>
#include <math.h> 
#include <tf/tf.h>

#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


namespace frenet_planner{

FrenetPlanner::FrenetPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                             bool debug, bool measure):
  nh_(nh),
  nh_private_(nh_private), 
  frenet_solver_(nh, debug){
  
  debug_ = debug;
  measure_ = measure;
  glob_wpnt_initialised_ = false;

  InitConditions();
  LoadHyperParameters();
  InitSubscribersPublishers();

  ROS_WARN_STREAM("[Planner] Finished initializing Frenet Planner node.");

  MainLoop();
}

FrenetPlanner::~FrenetPlanner() {}

void FrenetPlanner::MainLoop() {
  ros::Rate node_rate(max_rate_); // maximum rate this node will run at, does not guarantee anything
  auto start_time = std::chrono::high_resolution_clock::now();
  auto end_time = std::chrono::high_resolution_clock::now();
  while(ros::ok()) {
    // start time measurement
    if (measure_) {
      start_time = std::chrono::high_resolution_clock::now();
    }
    ros::spinOnce();

    if (!has_global_trajectory_ || (pub_only_if_obstacles_ && !has_obstacles_)) {
      node_rate.sleep();
      continue;
    }
    if (UpdateInitialConditions()) {
      on_path_ = FindLocalTrajectory();
      if (!on_path_) {
        PublishEmptyTrajectory();
        on_path_count_ = 0;
      } else if (on_path_count_ < 3) {
        // start publishing if on path three times consecutively
        on_path_count_++;
      } else {
        PublishLocalTrajectory(&best_frenet_path_);
      }
    }

    //OT dynamic params adjustment only if gb wpnts initialised already
    /*
    if(glob_wpnt_initialised_){
      //Read dynamic OT params
      UpdateHyperParamsOT();
      //Check if we are in OT sector and have to change params
      DynamicOTHandle();
      //Publish dyn param values for debug
      std_msgs::Float64 dts_msg;
      dts_msg.data = fot_hp_.d_t_s;
      planner_dyn_params_pub_.publish(dts_msg);
    }
    */
    // publish the latency
    if (measure_) {
      end_time = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed_time = end_time - start_time;
      std_msgs::Float64 latency_msg;
      latency_msg.data = elapsed_time.count();
      latency_pub_.publish(latency_msg);
    }

    node_rate.sleep();
  }
  return;
}

bool FrenetPlanner::UpdateInitialConditions() {
  if (!has_global_trajectory_) {
    ROS_WARN_THROTTLE(1, "[Planner] No global trajectory available.");
    return false;
  }

  if (!on_path_) {
    frenet_converter_.GetFrenetOdometry(odom_x_, odom_y_, odom_yaw_, v_x_, v_y_,
                                     &fot_ic_.s0, &fot_ic_.c_d, &fot_ic_.c_speed, 
                                     &fot_ic_.c_d_d, &id_start_);
    fot_ic_.c_d_dd = 0.0;
    fot_ic_.c_s_dd = 0.0;
  } else {
    // get the closest waypoint
    double min_dist = INFINITY;
    int closest_idx = 0;
    for (size_t i = 0; i < best_frenet_path_.x.size(); i++) {
      double d_squared = std::pow(odom_x_ - best_frenet_path_.x.at(i), 2) +
                        std::pow(odom_y_ - best_frenet_path_.y.at(i), 2);
      if (d_squared < min_dist) {
        min_dist = d_squared;
        closest_idx = i;
      }
    }
    if (min_dist > replanning_distance_*replanning_distance_) {
      frenet_converter_.GetFrenetOdometry(odom_x_, odom_y_, odom_yaw_, v_x_, v_y_,
                                     &fot_ic_.s0, &fot_ic_.c_d, &fot_ic_.c_speed, 
                                     &fot_ic_.c_d_d, &id_start_);
      fot_ic_.c_d_dd = 0.0;
      fot_ic_.c_s_dd = 0.0;
    } else {
      fot_ic_.s0 = std::fmod(best_frenet_path_.s.at(closest_idx), global_path_length_);
      fot_ic_.c_d = best_frenet_path_.d.at(closest_idx);
      fot_ic_.c_speed = best_frenet_path_.s_d.at(closest_idx);
      fot_ic_.c_d_d = best_frenet_path_.d_d.at(closest_idx);
      fot_ic_.c_d_dd = best_frenet_path_.d_dd.at(closest_idx);
      fot_ic_.c_s_dd = best_frenet_path_.s_dd.at(closest_idx);
    }
  }
  return true;
}

bool FrenetPlanner::FindLocalTrajectory() {
  if (!has_global_trajectory_) {
    ROS_WARN_THROTTLE(1, "[Planner] No global trajectory available.");
    return false;
  }

  // limit track velocity to be positive
  fot_ic_.c_speed = std::max(fot_hp_.min_speed, fot_ic_.c_speed);
  // uncomment to publish global trajectory ahead:
  // fot_ic_.c_d = 0.0;
  // fot_ic_.c_d_d = 0.0;

  if (abs(fot_ic_.c_d) > 3.0) {
    ROS_WARN_STREAM ("[Planner] Lateral distance more than 3m, not planning.");
    return false;
  }
  // get approximate target velocity at end of trajectory

  double s_dist = std::max(0.5, fot_ic_.c_speed * 
                                (fot_hp_.maxt));
  s_dist = std::fmod(fot_ic_.s0 + s_dist, global_path_length_);
  
  int id_end;
  frenet_converter_.GetClosestIndex(s_dist, &id_end);

  // take last waypoint as target_speed
  // TODO ensure target speed is actually reachable
  fot_ic_.target_speed = wpt_array_.at(id_end).vx_mps;
  
  frenet_solver_.UpdateCurrentPosition(fot_ic_);
  // frenet_solver_.UpdateTrackWidth(wpt_array_.at(id_end).s_m);
  // publish marker for initial position
  if (debug_) {
    PublishInitialPosition();
  }
  // PrintInitialConditions();
  // fot_ic_.no = 2; // leave obstacles uninitialized for now
  // double o_llx[2] = {-11.5, -11.5};
  // double o_lly[2] = {11, -20.5};
  // double o_urx[2] = {-12, -12};
  // double o_ury[2] = {11.5, -21.0};
  // fot_ic_.o_llx = o_llx;
  // fot_ic_.o_lly = o_lly;
  // fot_ic_.o_urx = o_urx;
  // fot_ic_.o_ury = o_ury;

  // PrintInitialConditions();

  if(UpdatePath()) {
    return true;
  } else {
    return false;
  }
}

bool FrenetPlanner::UpdatePath() {  
  // find frenet solution
  // PrintInitialConditions();
  
  if (frenet_solver_.UpdateOptimalTrajectory(&best_frenet_path_)) {
    if (debug_) {
      ROS_INFO_STREAM("[Planner] Success");
    }
  } else {
    ROS_WARN_STREAM("[Planner] Failure");
    return false;
  }

  if (debug_){
    ROS_INFO_STREAM("unweighted... lateral deviation " << best_frenet_path_.c_lateral_deviation);
    ROS_INFO_STREAM("lateral velocity " << best_frenet_path_.c_lateral_velocity);
    ROS_INFO_STREAM("lateral acceleration " << best_frenet_path_.c_lateral_acceleration);
    ROS_INFO_STREAM("lateral jerk " << best_frenet_path_.c_lateral_jerk);
    ROS_INFO_STREAM("weighted lateral total " << best_frenet_path_.c_lateral);
    ROS_INFO_STREAM("weighted longitudinal dev " << best_frenet_path_.c_longitudinal);
    ROS_INFO_STREAM("distance to track bounds " << best_frenet_path_.c_inv_dist_to_track_bound);
    ROS_INFO_STREAM("distance to obstacles " << best_frenet_path_.c_inv_dist_to_obstacles);
    ROS_INFO_STREAM("weighted total " << best_frenet_path_.cf);
  }

  return true;
}

void FrenetPlanner::PublishEmptyTrajectory() {
  visualization_msgs::MarkerArray local_markers; 
  f110_msgs::WpntArray local_wpts; 
  visualization_msgs::Marker clearing_marker;
  clearing_marker.action = visualization_msgs::Marker::DELETEALL;
  local_markers.markers.push_back(clearing_marker);
  local_wpts.header.frame_id = "map";
  local_wpts.header.stamp = ros::Time::now();
  local_trajectory_pub_.publish(local_wpts);
  local_trajectory_marker_pub_.publish(local_markers);
}

void FrenetPlanner::PublishLocalTrajectory(const FrenetPath* path) {
  visualization_msgs::MarkerArray local_markers; 
  f110_msgs::WpntArray local_wpts; 
  std::vector<double> wxs(path->x);
  // there are only size-1 points with curvature 
  double local_trajectory_advancement = 0.0;
  // create a clearing marker 
  visualization_msgs::Marker clearing_marker;
  clearing_marker.action = visualization_msgs::Marker::DELETEALL;
  local_markers.markers.push_back(clearing_marker);
  
  double clamp_speed_ = max_gb_speed_;

  
  for (unsigned int i = 0; i < wxs.size() - 1; i++) {
    // construct marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.type = marker.SPHERE;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 0.5;
    marker.color.g = 1.0;
    marker.pose.orientation.w = 1;
    marker.id = i;
    try {
      marker.pose.position.x = path->x.at(i);
      marker.pose.position.y = path->y.at(i);
      //publish height as normed velocity of scaled wpnts
      marker.pose.position.z = std::min(path->ds.at(i) / fot_hp_.dt, clamp_speed_)/max_gb_speed_;
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM(e.what());
      ROS_ERROR_STREAM(path->x[i]);
    }
    local_markers.markers.push_back(marker);

    // construct waypoint
    f110_msgs::Wpnt wpt;
    wpt.id = i;
    try {
      if (pub_global_track_advancement_) {
        wpt.s_m = std::fmod(path->s.at(i), global_path_length_);
      } else {
        wpt.s_m = local_trajectory_advancement;
      }
      wpt.d_m = path->d.at(i);
      wpt.d_right = path->d_right.at(i);
      wpt.d_left = path->d_left.at(i);
      wpt.x_m = path->x.at(i);
      wpt.y_m = path->y.at(i);
      wpt.psi_rad = path->yaw.at(i);
      wpt.kappa_radpm = path->c.at(i);
      wpt.vx_mps = std::min(path->ds.at(i) / fot_hp_.dt, clamp_speed_);
      // !!!! --- APPROXIMATE acceleration with longitudinal acceleration along global path
      wpt.ax_mps2 = path->s_dd.at(i);
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM(e.what());
      ROS_ERROR_STREAM("At index: " << i);
    }
    local_wpts.wpnts.push_back(wpt);
    local_trajectory_advancement += path->ds.at(i);
  }
  local_wpts.header.frame_id = "map";
  local_wpts.header.stamp = ros::Time::now();
  local_trajectory_pub_.publish(local_wpts);
  local_trajectory_marker_pub_.publish(local_markers);
}

void FrenetPlanner::PublishObstacles(
    const std::vector<f110_msgs::Obstacle> &obstacles) {
  visualization_msgs::MarkerArray obstacle_markers;
  visualization_msgs::Marker clearing_marker;
  clearing_marker.action = visualization_msgs::Marker::DELETEALL;
  obstacle_markers.markers.push_back(clearing_marker);
  for (auto const & ob : obstacles) {
    // construct marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.type = marker.SPHERE;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 0.7;
    marker.color.r = (ob.is_actually_a_gap) ? 0.1 : 0.9;
    marker.color.g = (ob.is_actually_a_gap) ? 0.9 : 0.1;
    marker.pose.orientation.w = 1;

    marker.id = ob.id*4;
    frenet_converter_.GetGlobalPoint(ob.s_start, ob.d_left, &marker.pose.position.x, &marker.pose.position.y);
    obstacle_markers.markers.push_back(marker);

    marker.id = ob.id*4 + 1;
    frenet_converter_.GetGlobalPoint(ob.s_end, ob.d_left, &marker.pose.position.x, &marker.pose.position.y);
    obstacle_markers.markers.push_back(marker);

    marker.id = ob.id*4 + 2;
    frenet_converter_.GetGlobalPoint(ob.s_start, ob.d_right, &marker.pose.position.x, &marker.pose.position.y);
    obstacle_markers.markers.push_back(marker);

    marker.id = ob.id*4 + 3;
    frenet_converter_.GetGlobalPoint(ob.s_end, ob.d_right, &marker.pose.position.x, &marker.pose.position.y);
    obstacle_markers.markers.push_back(marker);
  }
  obstacle_marker_pub_.publish(obstacle_markers);
}

void FrenetPlanner::PublishInitialPosition() {
    // construct marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.type = marker.SPHERE;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;
    marker.color.b = 1.0;
    marker.pose.orientation.w = 1;

    marker.id = 101;
    frenet_converter_.GetGlobalPoint(fot_ic_.s0, fot_ic_.c_d, &marker.pose.position.x, &marker.pose.position.y);
    initial_position_marker_pub_.publish(marker);
}

void FrenetPlanner::InitSubscribersPublishers() {

  global_trajectory_sub_ = nh_.subscribe<f110_msgs::WpntArray>
      ("/global_waypoints_scaled", 10, &FrenetPlanner::GlobalTrajectoryCallback, this);

  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>
      ("/odom", 10, &FrenetPlanner::OdomCallback, this);

  obstacle_sub_ = nh_.subscribe<f110_msgs::ObstacleArray>
      ("/perception/obstacles", 10, &FrenetPlanner::ObstacleCallback, this);

  local_trajectory_pub_ = nh_.advertise<f110_msgs::WpntArray>
      ("planner/waypoints", 1);

  local_trajectory_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>
      ("planner/markers", 1);

  obstacle_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>
      ("planner/obstacle_markers", 1);

  planner_dyn_params_pub_ = nh_.advertise<std_msgs::Float64>
      ("planner/dyn_params/d_t_s", 1);
      
  if (debug_) {
    initial_position_marker_pub_ = nh_.advertise<visualization_msgs::Marker>
      ("planner/initial_condition_marker", 1);
  }

  if (measure_) {
    // publish the latency
    latency_pub_ = nh_.advertise<std_msgs::Float64>
      ("planner/frenet/latency", 1);
  }
}

void FrenetPlanner::InitConditions() {
  odom_x_ = 0.0;
  odom_y_ = 0.0;
  v_x_ = 0;
  has_global_trajectory_ = false;
  on_path_ = false;
  has_obstacles_ = false;
}

void FrenetPlanner::OdomCallback(const nav_msgs::OdometryConstPtr &msg){
  odom_x_ = msg->pose.pose.position.x;
  odom_y_ = msg->pose.pose.position.y;
  tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, odom_yaw_);
  v_y_ = msg->twist.twist.linear.y;
  v_x_ = msg->twist.twist.linear.x;
}

void FrenetPlanner::GlobalTrajectoryCallback(
    const f110_msgs::WpntArrayConstPtr &wpt_array){
  std::unique_lock<std::mutex> lock(mutexGlobalTrajectory_);
  if(wpnt_cnt_ % 8 == 0){
    wpt_array_ = wpt_array->wpnts;
    global_path_length_ = wpt_array_.back().s_m;
    max_gb_speed_ = std::max_element(wpt_array_.begin(),
                                     wpt_array_.end(),
                                     [](const auto& a,const auto& b) { 
                                         return a.vx_mps < b.vx_mps; 
                                     })->vx_mps;
    //ROS_INFO("[Planner] Max speed of scaled gb: %f", max_gb_speed_);
    glob_wpnt_initialised_ = true;

    // TODO check if it has changed before updating it
    std::vector<double> wx, wy, d_right, d_left, v_s;
    for (auto & wpt : wpt_array_) {
      wpt.vx_mps *= speed_scale_;
      wx.push_back(wpt.x_m);
      wy.push_back(wpt.y_m);
      d_right.push_back(wpt.d_right - fot_hp_.track_clearance);
      d_left.push_back(wpt.d_left - fot_hp_.track_clearance);
      v_s.push_back(wpt.vx_mps);
    }
    // assume all trajectories are closed contours for now
    bool enable_wrapping = true;
    frenet_solver_.SetGlobalTrajectory(&wx, &wy, &d_right, &d_left, &v_s,
        enable_wrapping);
    frenet_converter_.SetGlobalTrajectory(&wpt_array_, enable_wrapping);
    has_global_trajectory_ = true;

    // Set hyperparameters
    fot_hp_.global_path_length = global_path_length_;
    frenet_solver_.SetHyperParameters(&fot_hp_);
    best_frenet_path_.SetHyperParameters(&fot_hp_);
  }
  wpnt_cnt_++;
}

void FrenetPlanner::ObstacleCallback(
    const f110_msgs::ObstacleArrayConstPtr &obstacle_array){
      // TODO add mutex, don't update during solving
  if (!has_global_trajectory_) {
    ROS_WARN("[Planner] Trying to set Obstacle, but no trajectoy received yet");
    return;
  }
  has_obstacles_ = frenet_solver_.UpdateObstacles(obstacle_array->obstacles);
  PublishObstacles(obstacle_array->obstacles);
}

void FrenetPlanner::DynamicOTHandle(){
  int nb_ot_sector = 1;
  nh_.getParam("/ot_map_params/n_sectors", nb_ot_sector);
  bool ot_sector_val = false;
  int ot_sector_start = 0;
  int ot_sector_end = 1;

  OTParameters ot_param = {};
  ot_sectors_params_.clear();
  ot_sectors_params_ = vector<OTParameters>(nb_ot_sector);

  for(int i = 0; i < nb_ot_sector; i++){
    nh_.getParam("/ot_map_params/Overtaking_sector" + std::to_string(i) + "/start", ot_sector_start);
    nh_.getParam("/ot_map_params/Overtaking_sector" + std::to_string(i) + "/end", ot_sector_end);
    nh_.getParam("/ot_dyn_sector_server/Overtaking_sector" + std::to_string(i), ot_sector_val);
    
    ot_param.start = wpt_array_[ot_sector_start].s_m;
    //Check if wrapped
    if(wpt_array_[ot_sector_start].s_m > wpt_array_[ot_sector_end].s_m){
        ot_param.end = global_path_length_;
    }
    else{
      ot_param.end = wpt_array_[ot_sector_end].s_m;
    }
    ot_param.val = ot_sector_val;

    //Push into global OT Sector params
    ot_sectors_params_[i] = ot_param;
    //ROS_INFO("[Planner] OT vals %d", ot_sectors_params_[i].val);
    //ROS_INFO("[Planner] OT start %f", ot_sectors_params_[i].start);
    //ROS_INFO("[Planner] OT end %f", ot_sectors_params_[i].end);
  }
}

void FrenetPlanner::UpdateHyperParamsOT(){
  int nb_ot_sectors = ot_sectors_params_.size();
  for(int i = 0; i < nb_ot_sectors; i++){
    //Check if s is between the sectors and if the OT is on, then use different params
    if((fot_ic_.s0 <= ot_sectors_params_[i].end && fot_ic_.s0 >= ot_sectors_params_[i].start) && ot_sectors_params_[i].val){      
      fot_hp_.d_t_s = fot_hp_.d_t_s_ot;
      ROS_INFO("[Planner] Inside active OT Sector using OT HyperParams: %f", fot_hp_.d_t_s);
      ROS_INFO("[Planner] start: %f", ot_sectors_params_[i].start);
      ROS_INFO("[Planner] end: %f", ot_sectors_params_[i].end);
      break;
    }
    //Not in OT sector so use init params again
    else{
      fot_hp_.d_t_s = fot_hp_.init_d_t_s;
    }
  }
}

void FrenetPlanner::LoadHyperParameters() {
  if (!nh_private_.getParam("max_speed", fot_hp_.max_speed) ||
      !nh_private_.getParam("min_speed", fot_hp_.min_speed) ||
      !nh_private_.getParam("max_accel", fot_hp_.max_accel) ||
      !nh_private_.getParam("max_curvature", fot_hp_.max_curvature) ||
      !nh_private_.getParam("max_road_width_l", fot_hp_.max_road_width_l) ||
      !nh_private_.getParam("max_road_width_r", fot_hp_.max_road_width_r) ||
      !nh_private_.getParam("d_road_w", fot_hp_.d_road_w) ||
      !nh_private_.getParam("dt", fot_hp_.dt) ||
      !nh_private_.getParam("maxt", fot_hp_.maxt) ||
      !nh_private_.getParam("mint", fot_hp_.mint) ||
      !nh_private_.getParam("d_sample_t", fot_hp_.d_sample_t) ||
      !nh_private_.getParam("d_t_s", fot_hp_.d_t_s) ||
      !nh_private_.getParam("d_t_s", fot_hp_.init_d_t_s) ||
      !nh_private_.getParam("d_t_s_ot", fot_hp_.d_t_s_ot) ||
      !nh_private_.getParam("n_s_sample", fot_hp_.n_s_sample) ||
      !nh_private_.getParam("obstacle_clearance", fot_hp_.obstacle_clearance) ||
      !nh_private_.getParam("track_clearance", fot_hp_.track_clearance) ||
      !nh_private_.getParam("k_lat_dev", fot_hp_.k_lat_dev) ||
      !nh_private_.getParam("k_lat_v", fot_hp_.k_lat_v) ||
      !nh_private_.getParam("k_lat_a", fot_hp_.k_lat_a) ||
      !nh_private_.getParam("k_lat_j", fot_hp_.k_lat_j) ||
      !nh_private_.getParam("k_lon_dev", fot_hp_.k_lon_dev) ||
      !nh_private_.getParam("k_track_bounds", fot_hp_.k_track_bounds) ||
      !nh_private_.getParam("k_obstacles", fot_hp_.k_obstacles) ||
      !nh_private_.getParam("k_temporal", fot_hp_.k_temporal) ||
      !nh_private_.getParam("k_lat", fot_hp_.k_lat) ||
      !nh_private_.getParam("k_lon", fot_hp_.k_lon) ||
      !nh_private_.getParam("k_time", fot_hp_.k_time) ||
      !nh_private_.getParam("num_threads", fot_hp_.num_threads) ||
      !nh_private_.getParam("ic_avoid_obstacles", fot_hp_.ic_avoid_obstacles) ||
      !nh_private_.getParam("max_frequency", max_rate_) ||
      !nh_private_.getParam("pub_only_if_obstacles", pub_only_if_obstacles_) ||
      !nh_private_.getParam("pub_global_track_advancement", pub_global_track_advancement_) ||
      !nh_private_.getParam("replanning_distance", replanning_distance_) ||
      !nh_private_.getParam("frenet_speed_scaling", speed_scale_)) {
        ROS_ERROR("[Planner] Missing hyperparameters");
        ros::shutdown();
        return;
      }
}

void FrenetPlanner::PrintInitialConditions() {
  ROS_WARN_STREAM("input s0: " << fot_ic_.s0);
  ROS_WARN_STREAM("input cspeed: " << fot_ic_.c_speed);
  ROS_WARN_STREAM("input cd: " << fot_ic_.c_d);
  ROS_WARN_STREAM("input cdd: " << fot_ic_.c_d_d);
  ROS_WARN_STREAM("input cddd: " << fot_ic_.c_d_dd);
  ROS_WARN_STREAM("input target_speed: " << fot_ic_.target_speed);
}

} // end of namespace frenet_planner

// launch node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "frenet_planner_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  bool debug, measure;
  nh_private.getParam("debug", debug);
  nh_private.getParam("measure", measure);

  frenet_planner::FrenetPlanner node(nh, nh_private, debug, measure);

  // start spinner
  ros::spin();
  return 0;
}
