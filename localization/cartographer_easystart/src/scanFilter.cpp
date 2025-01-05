#include "ros/ros.h"
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
// #include "LowPassFilter.h"

ros::Publisher scan_pub;
ros::Publisher test_pub;

double roll_, pitch_, yaw_;

// double initial_roll_ = 0.0;
// double initial_pitch_ = 0.0;

double initial_roll_ = 1.05;
double initial_pitch_ = -0.27;

double rad2deg(double rad) {
  return rad * 180.0 / M_PI;
}

double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}
    
void Imucallback(sensor_msgs::Imu::ConstPtr imu_msg){
    tf::Quaternion q(imu_msg->orientation.x, imu_msg->orientation.y,
                     imu_msg->orientation.z, imu_msg->orientation.w);
    tf::Matrix3x3 m(q);

    m.getRPY(roll_, pitch_, yaw_);

    if (rad2deg(roll_) > 0){
        roll_ = 180 - rad2deg(roll_) - initial_roll_;
        roll_ = deg2rad(roll_);
    }
    else {
        roll_ = -180 - rad2deg(roll_) - initial_roll_;
        roll_ = deg2rad(roll_);

    }

    pitch_ = rad2deg(pitch_) - initial_pitch_;
    pitch_ = deg2rad(pitch_);

    nav_msgs::Odometry test_;
    test_.twist.twist.linear.x = rad2deg(roll_); //차량기준 위로 들리면 +
    test_.twist.twist.linear.y = rad2deg(pitch_); //차량기준 오른쪽이 들리면 +
    test_.twist.twist.linear.z = rad2deg(yaw_);

    test_pub.publish(test_);
}

// Roll & Pitch
void Scancallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    sensor_msgs::LaserScan scan_filter;
    scan_filter = *scan_msg;
    // scan_filter.header.frame_id = "laser_2";

    // Define constants
    double angle_min = scan_msg->angle_min;
    double angle_increment = scan_msg->angle_increment;
    double range_min = scan_msg->range_min;

    // Loop through each range value in the scan
    for (int i = 0; i < scan_msg->ranges.size(); ++i) {
    // for (int i = 0; i < 10; ++i) {
        double range = scan_msg->ranges[i];

        // Ignore ranges that are too small
        if (range < range_min) {
            continue;
        }

        // Calculate the angle of the ray
        // double angle = angle_min + i * angle_increment;
        double angle = deg2rad(-45) + i * angle_increment;

        // Convert polar coordinates to Cartesian coordinates
        double x = range * cos(angle);
        double y = range * sin(angle);
        
        // double height_x = y * sin(roll_);
        // double height_y = x * sin(pitch_);

        double height_x = y * sin(pitch_);
        double height_y = x * sin(roll_);

        // nav_msgs::Odometry test_;
        // test_.twist.twist.linear.x = height; //차량기준 위로 들리면 +
        // test_.twist.twist.linear.y = rad2deg(roll_); //차량기준 오른쪽이 들리면 +
        // test_.twist.twist.linear.z = rad2deg(yaw_);
        // test_pub.publish(test_);

        // if (-0.2 < height_x &&  height_x < 0.1){
        //     if (-0.2 < height_y &&  height_y < 0.1){

        if (-0.15 < height_x ){
            // if (-0.0 < height_y ){
            //     scan_filter.ranges[i] = scan_msg->ranges[i];
            // }
            // else{
            //     scan_filter.ranges[i] =  std::numeric_limits<float>::quiet_NaN();
            // }
            
            scan_filter.ranges[i] = scan_msg->ranges[i];
        }
        else{
            scan_filter.ranges[i] =  std::numeric_limits<float>::quiet_NaN();
        }
    }
    scan_pub.publish(scan_filter);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scanFilter");
    ros::NodeHandle nh;
    scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_filter",1);
    test_pub =  nh.advertise<nav_msgs::Odometry>("/test",1);

    ros::Subscriber imu_sub=nh.subscribe("/imu/data",10,Imucallback);
    ros::Subscriber scan_sub=nh.subscribe("/scan",10,Scancallback);
    ros::spin();
    return 0;
}