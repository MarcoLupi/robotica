#include <ros/ros.h>
#include <ros/service.h>
#include "robotica/spiral.h"

class turtleServiceClient
{
public:
    turtleServiceClient();
    ~turtleServiceClient();
    void run();

private:
    ros::NodeHandle nh, pnh;
   
    double duration = 0.0;
    double angle_increment = 0.0;

    robotica::spiral spiral_srv;
    ros::ServiceClient spiral_client;
};