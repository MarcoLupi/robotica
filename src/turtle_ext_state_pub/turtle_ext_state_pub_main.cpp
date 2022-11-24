#include "robotica/turtle_ext_state_pub/turtle_ext_state_pub.h"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"turtle_ext_state_pub");

    extendedStatePublisher my_pub;

    ros::Rate loop(10);

    while(ros::ok())
    {
        my_pub.run();
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}