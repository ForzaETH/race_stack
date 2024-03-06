#include <ros/ros.h>

#include <std_msgs/String.h>

#include <termios.h>

#include <stdio.h>
#include <signal.h>

// for printing
#include <iostream>

static volatile sig_atomic_t keep_running = 1;


void sigHandler(int not_used) {
    keep_running = 0;
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "keyboard");
    // Initialize Node Handle
    ros::NodeHandle n = ros::NodeHandle("~");

    // Initialze publisher
    std::string keyboard_topic;
    n.getParam("keyboard_topic", keyboard_topic);

    ros::Publisher key_pub = n.advertise<std_msgs::String>(keyboard_topic, 10);


    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    tcsetattr( STDIN_FILENO, 0, &newt);

    struct sigaction act;
    act.sa_handler = sigHandler;
    sigaction(SIGINT, &act, NULL);
    

    std_msgs::String msg;
    int c;
    while ((ros::ok()) && (keep_running)) {
        // get the character pressed
        c = getchar();

        // Publish the character 
        msg.data = c;
        key_pub.publish(msg);
    }

    tcsetattr( STDIN_FILENO, 0, &oldt);
    
    return 0;
}