#include "ros/ros.h"
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
// #include "LowPassFilter.h"


geometry_msgs::PoseStamped last_pose;
bool is_pose_init = false;
bool is_odom_init = false;
ros::Time last_pose_stamp;
ros::Publisher odom_pub;
// ros::Publisher og_pub;
int n = 0;
static geometry_msgs::PoseStamped prev_pose;
// static geometry_msgs::PoseStamped prev_pose_og;
// static ros::Time prev_time;
double prev_time;
double angluar_velocity_x;
double angluar_velocity_y;
double angluar_velocity_z;

double sum_x = 0.0;
double sum_y = 0.0;

double prev_velocity_x = 0.0;
double prev_velocity_y = 0.0;

// double prev_time_og;
void Imucallback(sensor_msgs::Imu::ConstPtr msg){
    angluar_velocity_x = msg->angular_velocity.x;
    angluar_velocity_y = msg->angular_velocity.y;
    angluar_velocity_z = msg->angular_velocity.z;
}

void velocityCallBack(geometry_msgs::PoseStamped::ConstPtr in_pose){
    double vx, vy, dt;
    double d = 0.15;
    double alpha = 0.3;
    nav_msgs::Odometry vel_pose;
    // geometry_msgs::PoseStamped vel_pose_og;
    geometry_msgs::PoseStamped cur_pose;
    // geometry_msgs::PoseStamped cur_pose_og;

 // ros::Time cur_time = ros::Time::now();
    double cur_time = in_pose->header.stamp.nsec;
    cur_pose.header = in_pose->header;
    cur_pose.pose.orientation = in_pose->pose.orientation;
    // 자세값에서 yaw 각도 추출
    tf::Quaternion q(
        cur_pose.pose.orientation.x,
        cur_pose.pose.orientation.y,
        cur_pose.pose.orientation.z,
        cur_pose.pose.orientation.w);
    double yaw = tf::getYaw(q);
    
    // cur_pose_og.header = in_pose->header;
    // cur_pose_og.pose = in_pose->pose;
    // 자세값에서 yaw 각도 추출

    // 차량 기준에서 x, y 속도 계산
    double cx = in_pose->pose.position.x;
    double cy = in_pose->pose.position.y;
    cur_pose.pose.position.x = cx - d*cos(yaw);
    cur_pose.pose.position.y = cy - d*sin(yaw);
    cur_pose.pose.position.z = in_pose->pose.position.z;

    // if (n != 3){
    //     n +=1;
    //     return ;
    // }
    // n = 0 ;
    if (prev_time != 0.0)
    {
        // 두 포즈값 사이의 시간 간격 계산
        // dt = (cur_time - prev_time).toSec();
        dt = (cur_time - prev_time)/1e9;
        vx = (cur_pose.pose.position.x - prev_pose.pose.position.x) / dt;
        vy = (cur_pose.pose.position.y - prev_pose.pose.position.y) / dt;
        // vx_og = (cur_pose_og.pose.position.x - prev_pose_og.pose.position.x) / dt;
        // vy_og = (cur_pose_og.pose.position.y - prev_pose_og.pose.position.y) / dt;    
        // TODO: 속도 값을 사용하는 코드 작성
    
        // 결과 출력
        // ROS_INFO("vx: %f, vy: %f", vx, vy);
    }
    
    // 현재 포즈값을 이전 포즈값으로 저장
    prev_time = cur_time;
    prev_pose = cur_pose;
    
    // 차량 기준에서 x, y 속도 계산
    double vx_car = vx * cos(yaw) + vy * sin(yaw);
    double vy_car = -vx * sin(yaw) + vy * cos(yaw);
    
    // median filter
    if (n != 3){
        n +=1;
        sum_x = sum_x + vx_car;
        sum_y = sum_y + vy_car;
        return ;
    }
    if (n == 3){
        sum_x = sum_x + vx_car;
        sum_y = sum_y + vy_car;
        vx_car = sum_x / 4;
        vy_car = sum_y / 4;
    }
    sum_x = 0;
    sum_y = 0;
    n = 0 ;

    // Low-pass-filter
    double filtered_velocity_x = alpha * vx_car + (1 - alpha) * prev_velocity_x;
    double filtered_velocity_y = alpha * vy_car + (1 - alpha) * prev_velocity_y;
    prev_velocity_x = filtered_velocity_x;
    prev_velocity_y = filtered_velocity_y;

    // 결과 출력
    vel_pose.header = in_pose->header;
    vel_pose.pose.pose = in_pose->pose;
    // vel_pose.twist.twist.linear.x = vx_car;
    // vel_pose.twist.twist.linear.y = vy_car;
    vel_pose.twist.twist.linear.x = filtered_velocity_x;
    vel_pose.twist.twist.linear.y = filtered_velocity_y;
    vel_pose.twist.twist.linear.z = 0.0;
    vel_pose.twist.twist.angular.x = angluar_velocity_x;
    vel_pose.twist.twist.angular.y = angluar_velocity_y;
    vel_pose.twist.twist.angular.z = angluar_velocity_z;

    odom_pub.publish(vel_pose);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "cartoVelocity");
    ros::NodeHandle nh;
    odom_pub =  nh.advertise<nav_msgs::Odometry>("/tracked_odom",1);
    // og_pub =  nh.advertise<geometry_msgs::PoseStamped>("/carto_og_velocity",1);
    ros::Subscriber track_pose_sub = nh.subscribe("/tracked_pose", 1, velocityCallBack);
    ros::Subscriber imu_sub=nh.subscribe("/imu",10,Imucallback);
    ros::spin();
    return 0;
}