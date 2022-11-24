#include "robotica/turtle_ext_state_pub/turtle_ext_state_pub.h"

extendedStatePublisher::extendedStatePublisher()
{

    pose_sub = nh.subscribe("/turtle1/pose",1,&extendedStatePublisher::pose_callback,this);
    twist_sub = nh.subscribe("/turtle1/cmd_vel",1,&extendedStatePublisher::twist_callback,this);
    ext_state_pub = nh.advertise<robotica::turtleExtendedState>("/turtle1/ext_state",1);

}



void extendedStatePublisher::pose_callback(const turtlesim::Pose::Ptr& pose)
{
    ext_state_msg.pose = *pose;
}

void extendedStatePublisher::twist_callback(const geometry_msgs::Twist::Ptr& twist)
{
    ext_state_msg.twist = *twist;
}

void extendedStatePublisher::run()
{
    ext_state_pub.publish(ext_state_msg);
}

extendedStatePublisher::~extendedStatePublisher()
{
    
}