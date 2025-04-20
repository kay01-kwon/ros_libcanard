#include "ros_libcanard/ros_libcanard.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_libcanard_v2");
    
    ros::NodeHandle nh;

    RosLibcanard ros_libcanard(nh);

    ros_libcanard.run();

}