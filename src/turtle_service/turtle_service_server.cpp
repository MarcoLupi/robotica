#include "robotica/turtle_service/turtle_service_server.h"

turtleServiceServer::turtleServiceServer()
{
    twist_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);

    spiral_server = nh.advertiseService("/spiral", &turtleServiceServer::spiral_callback, this);
}

bool turtleServiceServer::spiral_callback(robotica::spiralRequest& req, robotica::spiralResponse& res)
{
    // per scrivere qualcosa quando la funzione è chiamata
    ROS_INFO_STREAM("Received spiral request - duration: "<<req.duration<< "angle_increment: "<<req.angle_increment);

    duration = req.duration;
    angle_increment = req.angle_increment;
    start_time = ros::Time::now();
    spiral_requested = true;

    res.success = true;
    return true;
}

void turtleServiceServer::run()
{
    // a new spiral_request arrive 
    if (spiral_requested)
    {
        // spiral has been executed for at least the provided duration
        if(ros::Time::now()-start_time >= ros::Duration(duration))
        {
            spiral_requested = false;
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
        }
        else // perform the spiral
        {
            twist.linear.x = 1.0;
            twist.angular.z += angle_increment;
        }
     
     
    }
    else //wait
    {
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
    }

    //publishing of the twist
    twist_pub.publish(twist);

}

turtleServiceServer::~turtleServiceServer()
{

}