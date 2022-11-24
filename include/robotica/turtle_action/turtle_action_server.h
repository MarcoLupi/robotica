#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "robotica/waypointAction.h"
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

class turtleActionServer
{
public:
    turtleActionServer();
    ~turtleActionServer();

private:
    ros::NodeHandle nh;
    
    // action server  
    actionlib::SimpleActionServer<robotica::waypointAction> action_server;
    void action_callback(const robotica::waypointGoalConstPtr& goal);
    robotica::waypointFeedback feedback_;
    robotica::waypointResult result_;
    
    // listen to turtle pose 
    ros::Subscriber pose_sub;
    void pose_callback(const turtlesim::PoseConstPtr& pose);
    turtlesim::Pose current_pose;

    // publish desired speed
    ros::Publisher twist_pub;
    geometry_msgs::Twist twist;

    // control functions
    double euclidean_error(const turtlesim::Pose& turtle_pose, const geometry_msgs::Pose2D& target_pose);
    double angular_error(const double& start_angle, const double& end_angle);
};