#include "robotica/turtle_action/turtle_action_client.h"

turtleActionClient::turtleActionClient():
pnh("~"), action_client("waypoint_action", true)
{
    action_client.waitForServer();

    pnh.param("goal_x", goal.pose.x, 7.0);
    pnh.param("goal_y", goal.pose.y, 7.0);
    pnh.param("goal_theta", goal.pose.theta, 0.0);
}

void turtleActionClient::sendGoal()
{
    action_client.sendGoal(goal, boost::bind(&turtleActionClient::result_callback,this,_1,_2), 
                                 boost::bind(&turtleActionClient::active_callback,this),
                                 boost::bind(&turtleActionClient::feedback_callback,this,_1));
                                
}

void turtleActionClient::result_callback(const actionlib::SimpleClientGoalState& state, const robotica::waypointResultConstPtr& result)
{
    ROS_INFO_STREAM("Finished in state"<<state.toString());
    ROS_INFO_STREAM("Success: "<<int(result->success));
    ros::shutdown();
}

void turtleActionClient::active_callback()
{
    ROS_INFO_STREAM("Goal active!");
}

void turtleActionClient::feedback_callback(const robotica::waypointFeedbackConstPtr& feedback)
{
    ROS_INFO_STREAM("Control Phase [" <<int(feedback->control_phase)<<"] errors: ("<<feedback->distance_error<<" "<<feedback->angle_error<<")");
}

turtleActionClient::~turtleActionClient()
{

}
