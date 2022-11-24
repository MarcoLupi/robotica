#include "robotica/turtle_action/turtle_action_client.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_action_client");

    turtleActionClient my_action_client;
    my_action_client.sendGoal();
    ros::spin();
    
    return 0;
}