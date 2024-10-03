#ifndef ROS_WRAPPER_LIBCANARD_HPP
#define ROS_WRAPPER_LIBCANARD_HPP
#include "canard_interface/canard_interface.hpp"
#include "canard_interface/drone_can_node.hpp"

#include <iostream>
#include <ros/ros.h>
#include "ros_libcanard/actual_rpm.h"
#include "ros_libcanard/cmd_raw.h"



class RosWrapperLibcanard
{
    public:

    RosWrapperLibcanard(ros::NodeHandle &nh);

    RosWrapperLibcanard(ros::NodeHandle &nh, const char *interface_name);

    void publish_actual_rpm();

    private:

    void callback_cmd_raw(const ros_libcanard::cmd_raw::ConstPtr& msg);

    ros::Publisher actual_rpm_pub_;
    ros::Subscriber raw_value_sub_;

    DroneCanNode drone_can_node_;

};



#endif