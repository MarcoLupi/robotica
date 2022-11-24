#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
    // initialize ros comunication
    ros::init(argc, argv, "cpp_talker_node");

    // create the basic ros function provider
    ros::NodeHandle node_;

    // create the publisher
    ros::Publisher pub = node_.advertise<std_msgs::String>("FILIPPO", 1);

    // create the message
    std_msgs::String msg;
    msg.data = "ciao";

    // choose our rate
    ros::Rate loop(12);

    // run until ros is ok
    while(ros::ok())
    { 
        // publish the message
        pub.publish(msg);
        // sleep to respect the rate
        loop.sleep();
    }

    return 0;
}