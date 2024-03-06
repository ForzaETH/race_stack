#ifndef LASER_MAP_OVERLAP_CHECKER_H
#define LASER_MAP_OVERLAP_CHECKER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/LaserScan.h>
#include <f110_msgs/WpntArray.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>


class LaserMapOverlapChecker {
public:
    LaserMapOverlapChecker();
    ~LaserMapOverlapChecker() = default;

    void mapCallback(const nav_msgs::OccupancyGrid& data);
    void laserCallback(const sensor_msgs::LaserScan& data);
    void trackboundsCallback(const visualization_msgs::MarkerArray& data);
    void glbWpntsCallback(const f110_msgs::WpntArray& data);

    void loop();

private:
    Eigen::MatrixXd laser2mapConversion(const sensor_msgs::LaserScan& laser);
    double computeScanAlignment(const Eigen::MatrixXd& scan_in_map, const visualization_msgs::MarkerArray& trackbounds);

    // ROS NodeHandle
    ros::NodeHandle nh_;

    // ROS Subscribers
    ros::Subscriber map_sub_;
    ros::Subscriber laser_sub_;
    ros::Subscriber trackbounds_sub_;
    ros::Subscriber gb_sub_;

    // ROS Publishers
    ros::Publisher debug_lidar_pub_;
    ros::Publisher alignment_score_pub_;
    ros::Publisher alignment_textmrk_pub_;

    // ROS tf listener
    tf::TransformListener tf_listener_;

    // Parameters
    bool debug_flag_;
    int downsample_factor_;
    double threshold_;
    int frequency_;

    // Data
    nav_msgs::OccupancyGrid map_;
    sensor_msgs::LaserScan scan_;
    visualization_msgs::MarkerArray trackbounds_;
    f110_msgs::WpntArray glb_wpnts_;

    // Misc
    ros::Rate rate_;
    bool first_visualization_;
    double x_viz_;
    double y_viz_;
    bool scan_initialized_;
    bool trackbounds_initialized_;
};

#endif  // LASER_MAP_OVERLAP_CHECKER_H
