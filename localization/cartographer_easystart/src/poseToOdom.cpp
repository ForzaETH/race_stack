#include "ros/ros.h"
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::PoseStamped last_pose;
bool is_pose_init = false;
bool is_odom_init = false;
ros::Time last_pose_stamp;
ros::Publisher odom_pub;
double vesc_vel;

void odomCallBack(nav_msgs::Odometry::ConstPtr in_odom){
    vesc_vel = in_odom->twist.twist.linear.x;
    is_odom_init = true;
}

void poseCallBack(geometry_msgs::PoseStamped::ConstPtr in_pose){
    nav_msgs::Odometry tmp_odom_msg;
    tmp_odom_msg.header = in_pose->header;
    tmp_odom_msg.pose.pose = in_pose->pose;
    if (is_odom_init){
        tmp_odom_msg.twist.twist.linear.x = vesc_vel;
    }
    // if(is_pose_init){
    //     double dt = in_pose->header.stamp.toSec() - last_pose_stamp.toSec();
    //     double disp_x = last_pose.pose.position.x - in_pose->pose.position.x;
    //     double disp_y = last_pose.pose.position.y - in_pose->pose.position.y;
    //     std::cout << "dt  : " << dt << std::endl;
    //     std::cout << "dx  : " << disp_x << std::endl;
    //     std::cout << "dy  : " << disp_y << std::endl;
    //     tmp_odom_msg.twist.twist.linear.x  = sqrt(pow(disp_x,2) + pow(disp_y,2))/dt;
    // }
    // else{
    //     is_pose_init = true;
    // }
    // last_pose_stamp = in_pose->header.stamp;
    // last_pose = *in_pose;
    odom_pub.publish(tmp_odom_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "poseToOdom");
    ros::NodeHandle nh;
    odom_pub =  nh.advertise<nav_msgs::Odometry>("/pf/pose/odom",1);
    ros::Subscriber vesc_odom_sub = nh.subscribe("/vesc/odom", 1, odomCallBack);
    ros::Subscriber pose_sub = nh.subscribe("/tracked_pose",1, poseCallBack);
    ros::spin();
    return 0;
}