#include "robotica/turtle_service/turtle_service_client.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_service_client");

    turtleServiceClient my_client;

    my_client.run();
    
    return 0;
}