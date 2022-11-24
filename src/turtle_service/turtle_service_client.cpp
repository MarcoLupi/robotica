#include "robotica/turtle_service/turtle_service_client.h"

turtleServiceClient::turtleServiceClient(): pnh("~")
{
    spiral_client = nh.serviceClient<robotica::spiral>("/spiral");

    pnh.param("duration", spiral_srv.request.duration,1.0);
    pnh.param("angle_increment", spiral_srv.request.angle_increment,0.1);
}

void turtleServiceClient::run()
{
    ROS_INFO_STREAM("Requesting a spiral of a duration: "<<spiral_srv.request.duration<<" and angle increment: "<< spiral_srv.request.angle_increment);

    spiral_client.call(spiral_srv);

    if(spiral_srv.response.success)
    {
        ROS_INFO_STREAM(" - OK !");
    }
    else
    {
        ROS_ERROR(" - something wrong!");
    }
}

turtleServiceClient::~turtleServiceClient()
{

}