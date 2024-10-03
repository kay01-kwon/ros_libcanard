#include "ros_libcanard/ros_wrapper_libcanard.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_wrapper_libcanard");

    ros::NodeHandle nh;

    RosWrapperLibcanard ros_wrapper_libcanard(nh);

    ros::Rate loop_rate(200);

    while(ros::ok())
    {
        ros_wrapper_libcanard.publish_actual_rpm();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}