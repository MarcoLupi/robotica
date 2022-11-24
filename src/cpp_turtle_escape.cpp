#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<turtlesim/Pose.h>
#include<turtlesim/TeleportAbsolute.h>

int action = 0;
double current_angle = 0.0;
double desired_angle = 0.0;

void pose_callback(const turtlesim::Pose::Ptr& pose)
{
    current_angle = pose->theta;
    if (current_angle < 0)
    {
        current_angle += M_PI*2.0;
    }
    
    // pow mi fa la potenza di un numero: pow (NUMERO,ESPONENTE)
    if (pow(pose->x-5.5,2)+pow(pose->y-5.5,2) < pow(1,2))
    {
        if (action == 0)
        {
            desired_angle += M_PI/4.0;
            ROS_INFO_STREAM("New desired angle: "<<desired_angle);
            action = 1;
        }
        
    }
    else
    {
        action = 0;
    }
}



int main(int argc, char** argv)
{
    ros::init(argc,argv,"cpp_turtle_escape");
    ros::NodeHandle nh;
    ros::Subscriber pose_sub = nh.subscribe("/turtle1/pose",1,pose_callback);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);
    geometry_msgs::Twist twist;
    ros::Rate loop(10);

    while(ros::ok())
    {
        switch(action)
        {
            case 0:
                twist.linear.x = 1.0;
                twist.angular.z = 0.0;
            break;
            case 1:
                if (fabs(desired_angle-current_angle) < 0.05)
                {
                    action = 2;
                }
                else
                {
                    twist.linear.x = 0.0;
                    twist.angular.z = 1.5 * (desired_angle-current_angle);
                }
            break;
            case 2:
                twist.linear.x = 1.0;
                twist.angular.z = 0.0;
            break;
            default:
            break;
        }

        twist_pub.publish(twist);
        loop.sleep();
        ros::spinOnce();

    }
    
    return 0;
}