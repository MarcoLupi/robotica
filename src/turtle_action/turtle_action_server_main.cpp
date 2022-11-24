#include "robotica/turtle_action/turtle_action_server.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_action_server");

    turtleActionServer my_action_server;
    ros::spin();

    return 0;
}