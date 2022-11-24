#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "robotica/waypointAction.h"

class turtleActionClient
{
public:

    turtleActionClient();
    ~turtleActionClient();
    void sendGoal();

private:
    ros::NodeHandle nh, pnh;

    // action client 
    actionlib::SimpleActionClient<robotica::waypointAction> action_client;
    robotica::waypointGoal goal;

    // action callback
    void result_callback(const actionlib::SimpleClientGoalState& state, const robotica::waypointResultConstPtr& result);
    void active_callback();
    void feedback_callback(const robotica::waypointFeedbackConstPtr& feedback);
    

};