#include<iostream>
#include<ros/ros.h>
#include<std_msgs/String.h>

void callback (const std_msgs::String::Ptr& msg)
{
    ROS_INFO_STREAM("received: " <<msg->data);
}

int main(int argc, char** argv)
{
    // initialize comunication
    ros::init(argc, argv, "cpp_listener_node");
    
    // create basic function provider
    ros::NodeHandle node_;

    // create subscriber 
    ros::Subscriber sub = node_.subscribe("FILIPPO",1,callback);
    
    // just waits for messages
    ros::spin();

    return 0;
}