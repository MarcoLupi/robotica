#include<iostream>
#include<ros/ros.h>
#include<std_msgs/String.h>

ros::Time last_time;
bool first_time = true;

void callback(const std_msgs::String::Ptr& msg)
{
    if (first_time)
    {
        last_time = ros::Time::now();
        first_time = false;
    }
    else
    {

        // get the current time
        ros::Time now = ros::Time::now();
    
        // compute elapsed time between two consecutive messages
        ros::Duration elapsed_time = now - last_time;

        // compute frequency as the opposite of elapsed second
        double frequency = 1.0/ (elapsed_time.toSec());

        // print
        ROS_INFO_STREAM("Rate: " <<frequency);
    
        // update last message time
        last_time = now;
    }
}

int main (int argc, char** argv)
{
    // initialize comunication
    ros::init(argc,argv,"cpp_rate_computing_node");

    // create basic function provider
    ros::NodeHandle node_;

    // create subscriber
    ros::Subscriber sub = node_.subscribe("/chat",1,callback);

    // wait for messages
    ros::spin();

    return 0;
}