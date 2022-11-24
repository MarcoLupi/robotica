#include <ros/ros.h>
#include <ros/service.h>
#include "robotica/spiral.h"
#include <geometry_msgs/Twist.h>


class turtleServiceServer
{
public:
    turtleServiceServer();
    ~turtleServiceServer();
    void run();

private:
    ros::NodeHandle nh;

    geometry_msgs::Twist twist;
    ros::Publisher twist_pub;

    bool spiral_requested = false;
    ros::Time start_time;
    double duration = 0;
    double angle_increment = 0;

    ros::ServiceServer spiral_server;
    bool spiral_callback(robotica::spiralRequest& req, robotica::spiralResponse& res);

};
