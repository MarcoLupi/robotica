#include "robotica/turtle_action/turtle_action_server.h"
#include <angles/angles.h>

turtleActionServer::turtleActionServer():
action_server(nh, "waypoint_action", boost::bind(&turtleActionServer::action_callback, this, _1), false)
{
    action_server.start();
    pose_sub = nh.subscribe("/turtle1/pose", 1, &turtleActionServer::pose_callback, this);
    twist_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);

}

double turtleActionServer::euclidean_error(const turtlesim::Pose& turtle_pose, const geometry_msgs::Pose2D& target_pose)
{
    return sqrt(pow(turtle_pose.x-target_pose.x,2)+pow(turtle_pose.y-target_pose.y,2));
}

double turtleActionServer::angular_error(const double& start_angle, const double& end_angle)
{
    return angles::shortest_angular_distance(start_angle,end_angle);
}

void turtleActionServer::pose_callback(const turtlesim::PoseConstPtr& pose)
{
    current_pose = *pose;
}

void turtleActionServer::action_callback(const robotica::waypointGoalConstPtr& goal)
{
    ROS_INFO_STREAM("New waypoint to reack: ["<<goal->pose.x<<", "<<goal->pose.y<<" , "<<goal->pose.theta<<" ]");
    
    bool reached = false;
    ros::Rate loop(10);
    int control_phase = 0;
    double angle_to_target;
    double distance_tolerance = 0.1;
    double angle_tolerance = M_PI/180;

    while(!reached)
    {
        // check on preemption
        if (action_server.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO_STREAM(" - PREEMPTED!");
            action_server.setPreempted();
            break;
        }

        angle_to_target = atan2(goal->pose.y-current_pose.y,goal->pose.x-current_pose.x);

        switch(control_phase)
        {
            case 0: //align toward goal
                if(abs(angular_error(current_pose.theta,angle_to_target) < angle_tolerance)) //phase is completed
                {
                    ROS_INFO_STREAM ("ALIGNED TOWARD GOAL");
                    twist.linear.x=0.0;
                    twist.angular.z=0.0;
                    control_phase = 1;

                }
                else // execute control
                {
                    twist.linear.x=0.0;
                    twist.angular.z=0.5 * angular_error(current_pose.theta, angle_to_target);
                }
            break;

            case 1: //get close
                if(euclidean_error(current_pose,goal->pose) < distance_tolerance) //phase is completed
                {
                    ROS_INFO_STREAM ("CLOSE TO GOAL");
                    twist.linear.x=0.0;
                    twist.angular.z=0.0;
                    control_phase = 2;
                }
                else // execute control
                {
                    twist.linear.x = 0.1 + euclidean_error(current_pose,goal->pose) / 10;
                    twist.angular.z = 0.1 +angular_error(current_pose.theta, angle_to_target);

                }
            break;

            case 2: //align to final angle 
                if(abs(angular_error(current_pose.theta, goal->pose.theta)) < angle_tolerance) //phase is completed
                {
                    ROS_INFO_STREAM ("ALIGNED TO FINAL ANGLE");
                    twist.linear.x=0.0;
                    twist.angular.z=0.0;
                    control_phase = 3;
                }
                else // execute control
                {
                    twist.linear.x = 0.0;
                    twist.angular.z = 0.5 * angular_error(current_pose.theta, goal->pose.theta);                
                }
            break;

            default: //stop
                twist.linear.x=0.0;
                twist.angular.z=0.0;
            break;
        }

        // feedbackk update
        feedback_.control_phase = control_phase;
        feedback_.distance_error = euclidean_error(current_pose, goal->pose);
        feedback_.angle_error = angular_error(current_pose.theta, goal->pose.theta);
        action_server.publishFeedback(feedback_);
        
        //check for task completion
        reached = (feedback_.distance_error < distance_tolerance) && (feedback_.angle_error < angle_tolerance);
        
        if(reached)
        {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;

        }
        
        //control publish
        twist_pub.publish(twist);
        loop.sleep();
    }

    if (reached)
    {
        ROS_INFO_STREAM(" - WAYPOINT REACHED!");
        /// notify action server
        result_.success = true;
        action_server.setSucceeded(result_);

    }
}

turtleActionServer::~turtleActionServer()
{

}