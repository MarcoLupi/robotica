#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include "robotica/turtleExtendedState.h"

class extendedStatePublisher
{   
public:
    extendedStatePublisher();
    ~extendedStatePublisher();
    void run();

private:
    ros::NodeHandle nh;

    ros::Subscriber pose_sub;
    void pose_callback(const turtlesim::Pose::Ptr& pose);

    ros::Subscriber twist_sub;
    void twist_callback(const geometry_msgs::Twist::Ptr& twist);

    ros::Publisher ext_state_pub;
    robotica::turtleExtendedState ext_state_msg;
};