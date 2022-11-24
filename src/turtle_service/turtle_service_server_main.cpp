#include "robotica/turtle_service/turtle_service_server.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"turtle_service_server");

    turtleServiceServer my_server;

    ros::Rate loop(10);

    while (ros::ok())
    {
        my_server.run();
        ros::spinOnce();
        loop.sleep();
    }
    
    return 0;
}