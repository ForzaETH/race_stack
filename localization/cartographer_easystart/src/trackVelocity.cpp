#include "ros/ros.h"
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "vrpn_client_ros/vrpn_client_ros.h"

geometry_msgs::PoseStamped last_pose;
bool is_pose_init = false;
bool is_odom_init = false;
ros::Time last_pose_stamp;
ros::Publisher odom_pub;
double vesc_vel;

static geometry_msgs::PoseStamped prev_pose;
// static ros::Time prev_time;
double prev_time;

void velocityCallBack(geometry_msgs::PoseStamped::ConstPtr in_pose){
    double vx, vy, dt;
    geometry_msgs::PoseStamped vel_pose;
    geometry_msgs::PoseStamped cur_pose;
    // ros::Time cur_time = ros::Time::now();
    double cur_time = in_pose->header.stamp.nsec;
    cur_pose.header = in_pose->header;
    cur_pose.pose = in_pose->pose;

    // if (prev_time != ros::Time(0))
    if (prev_time != 0.0)
    {
        // 두 포즈값 사이의 시간 간격 계산
        // dt = (cur_time - prev_time).toSec();
        dt = (cur_time - prev_time)/1e9;

        // x, y 속도 계산
        vx = (cur_pose.pose.position.x - prev_pose.pose.position.x) / dt;
        vy = (cur_pose.pose.position.y - prev_pose.pose.position.y) / dt;
    
        // TODO: 속도 값을 사용하는 코드 작성
    
        // 결과 출력
        ROS_INFO("vx: %f, vy: %f", vx, vy);
    }
    
    // 현재 포즈값을 이전 포즈값으로 저장
    prev_time = cur_time;
    prev_pose = cur_pose;

    // 자세값에서 yaw 각도 추출
    tf::Quaternion q(
        cur_pose.pose.orientation.x,
        cur_pose.pose.orientation.y,
        cur_pose.pose.orientation.z,
        cur_pose.pose.orientation.w);
    double yaw = tf::getYaw(q);
    
    // 차량 기준에서 x, y 속도 계산
    double vx_car = vx * cos(yaw) + vy * sin(yaw);
    double vy_car = -vx * sin(yaw) + vy * cos(yaw);
    
    // TODO: 차량 기준에서 x, y 속도 값을 사용하는 코드 작성
    
    // 결과 출력
    ROS_INFO("vx_car: %f, vy_car: %f", vx_car, vy_car);
    if ( vx_car > 5.0 || vx_car < -5.0){
        vx_car = 0.0;
    }
    if ( vy_car > 5.0 || vy_car < -5.0){
        vy_car = 0.0;
    }
    vel_pose.header = in_pose->header;
    vel_pose.pose.position.x = vx_car;
    vel_pose.pose.position.y = vy_car;
    

    odom_pub.publish(vel_pose);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trackVelocity");
    ros::NodeHandle nh;
    odom_pub =  nh.advertise<geometry_msgs::PoseStamped>("/track_velocity",1);
    ros::Subscriber track_pose_sub = nh.subscribe("/vrpn_client_node/CAR/pose", 1, velocityCallBack);
    ros::spin();
    return 0;
}