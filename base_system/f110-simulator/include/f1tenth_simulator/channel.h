#ifndef CHANNEL_H
#define CHANNEL_H

#include <ros/ros.h>

#include <ackermann_msgs/AckermannDriveStamped.h>

class Mux;

class Channel {
private:
    // Publish drive data to simulator/car
    ros::Publisher drive_pub;

    // Listen to drive data from a specific topic
    ros::Subscriber channel_sub;

    // Mux index for this channel
    int mux_idx;

    // Pointer to mux object (to access mux controller and nodeHandle)
    Mux* mp_mux;


public:
    Channel();

    Channel(std::string channel_name, std::string drive_topic, int mux_idx_, Mux* mux);

    void drive_callback(const ackermann_msgs::AckermannDriveStamped & msg);
};


#endif // CHANNEL_H