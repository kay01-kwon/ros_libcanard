#ifndef ROS_WRAPPER_LIBCANARD_HPP
#define ROS_WRAPPER_LIBCANARD_HPP
#include "canard_interface/canard_interface.hpp"
#include "canard_interface/drone_can_node.hpp"

#include <iostream>
#include <ros/ros.h>

#include "ros_libcanard/actual_rpm.h"
#include "ros_libcanard/cmd_raw.h"
#include <std_msgs/Float32.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

class RosWrapperLibcanard
{
    public:

    RosWrapperLibcanard(ros::NodeHandle &nh);

    RosWrapperLibcanard(ros::NodeHandle &nh, const char *interface_name);

    void publish_actual_rpm();

    ~RosWrapperLibcanard();

    private:

    void callback_cmd_raw(const ros_libcanard::cmd_raw::ConstPtr& msg);

    void ros_run();

    void process_drone_can_process();

    ros::Publisher actual_rpm_pub_;
    ros::Publisher voltage_pub_;
    ros::Subscriber raw_value_sub_;

    DroneCanNode drone_can_node_;

    boost::thread drone_can_process_thread_;
    boost::thread ros_run_thread_;

    boost::mutex mtx_;
    boost::condition_variable cv_;

    ros::Rate loop_rate_{200};

};



#endif
