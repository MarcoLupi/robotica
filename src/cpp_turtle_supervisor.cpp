#include<ros/ros.h>
#include<ros/service_client.h>
#include<turtlesim/Pose.h>
#include<turtlesim/TeleportAbsolute.h>

// create service client
ros::ServiceClient teleport_client;
//create service message
turtlesim::TeleportAbsolute teleport_srv;

void callback(const turtlesim::Pose::Ptr& pose)
{
    // if the turtle is outside the circle centered in (5.5,5.5) and radii 3, teleport it in the center
    if(pow(pose->x-5.5,2) + pow(pose->y-5.5,2) > pow(3.0,2))
    {
        // set the turtle orientetion
        teleport_srv.request.theta = pose->theta;
        // call the service
        teleport_client.call(teleport_srv);
    }
}

int main (int argc, char** argv)
{ 
    //initialize comunication 
    ros::init(argc,argv,"cpp_turtle_supervisor");

    //create functions provider
    ros::NodeHandle nh;

    //create the subscriber to get the turtle pose
    ros::Subscriber pose_sub = nh.subscribe("/turtle1/pose",1,callback);

    //initialize service client
    teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");

    // prepare the service message
    teleport_srv.request.x = 5.5;
    teleport_srv.request.y = 5.5;

    // wait the message

    ros::spin();
}