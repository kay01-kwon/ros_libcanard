#include "ros_wrapper_libcanard.hpp"

RosWrapperLibcanard::RosWrapperLibcanard(ros::NodeHandle &nh) : drone_can_node_{}
{
    actual_rpm_pub_ = nh.advertise<ros_libcanard::actual_rpm>("actual_rpm", 1);
    raw_value_sub_ = nh.subscribe<ros_libcanard::cmd_raw>("cmd_raw", 
    1, 
    &RosWrapperLibcanard::callback_cmd_raw, 
    this);


    drone_can_node_.initiate_and_switch_to_op_mode("can0");

    drone_can_process_thread_ = boost::thread(&RosWrapperLibcanard::process_drone_can_process, this);
    ros_run_thread_ = boost::thread(&RosWrapperLibcanard::ros_run, this);
}

RosWrapperLibcanard::RosWrapperLibcanard(ros::NodeHandle &nh, const char *interface_name) : drone_can_node_{}
{
    actual_rpm_pub_ = nh.advertise<ros_libcanard::actual_rpm>("actual_rpm", 1);
    raw_value_sub_ = nh.subscribe<ros_libcanard::cmd_raw>("cmd_raw", 
    1, 
    &RosWrapperLibcanard::callback_cmd_raw, 
    this);

    drone_can_node_.initiate_and_switch_to_op_mode(interface_name);

    drone_can_process_thread_ =
    boost::thread(&RosWrapperLibcanard::process_drone_can_process,this);

    ros_run_thread_ = boost::thread(&RosWrapperLibcanard::ros_run, this);

}


RosWrapperLibcanard::~RosWrapperLibcanard()
{
    drone_can_process_thread_.join();
    ros_run_thread_.join();
}

void RosWrapperLibcanard::publish_actual_rpm()
{

    int32_t rpm[NUM_ESCS];
    drone_can_node_.get_esc_rpm(rpm);

    ros_libcanard::actual_rpm rpm_msg;

    rpm_msg.stamp = ros::Time::now();

    for(size_t i = 0; i < NUM_ESCS; i++)
    {
        rpm_msg.rpm[i] = rpm[i];
    }

    actual_rpm_pub_.publish(rpm_msg);
}

void RosWrapperLibcanard::callback_cmd_raw(const ros_libcanard::cmd_raw::ConstPtr &cmd_msg)
{
    int16_t raw_value[NUM_ESCS];
    for(size_t i = 0; i < NUM_ESCS; i++)
    {
        raw_value[i] = cmd_msg->raw[i];
    }
    drone_can_node_.set_esc_raw(raw_value);

}

void RosWrapperLibcanard::ros_run()
{
    
    while(ros::ok())
    {
        boost::unique_lock<boost::mutex> lock(mtx_);
    
        cv_.wait(
            lock,
            [this]{
            return drone_can_node_.is_broadcasted();
                }
            );
        
        publish_actual_rpm();
        ros::spinOnce();
        loop_rate_.sleep();
    }

}


void RosWrapperLibcanard::process_drone_can_process()
{
    boost::lock_guard<boost::mutex> lock(mtx_);
    while(ros::ok())
    {
        drone_can_node_.process_node();
        cv_.notify_all();
    }
}
