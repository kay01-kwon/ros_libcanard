#include "ros_libcanard.hpp"

RosLibcanard::RosLibcanard(ros::NodeHandle &nh)
{
    int NUM_ESC;
    std::string if_name;

    // Get number of esc and interface name from launch file
    nh.getParam("NUM_ESC", NUM_ESC);
    nh.getParam("if_name", if_name);

    // Set number of ESC
    NUM_ESC_ = static_cast<uint8_t>(NUM_ESC);

    // Set interface name and get node id
    canard_iface_.init(if_name.c_str());
    uint8_t node_id = canard_iface_.get_node_id();

    // Display the number of ESC and Drone can info
    std::cout<<"Number of ESC from launch file: "<<NUM_ESC<<std::endl;
    printf("Drone Can started on %s, node ID: %d\n",
    if_name.c_str(),
    canard_iface_.get_node_id());

    send_NodeStatus();
    // Pop Tx queue and socketcanReceive for 10 ms
    canard_iface_.process(10);

    uavcan_protocol_GetNodeInfoRequest req;

    // Request node info for nodes
    for(size_t i = 0; i < NUM_ESC_; i++)
    {
        printf("Requesting node info for node %ld\n",i+1);
        req = {};
        while(!get_node_info_client_.request(i+1, req))
        {
            printf("Pending response\n");
            // Pop Tx queue and socketcanReceive for 10 ms
            canard_iface_.process(10);
        }
    }

    // Switch to Operational Mode

    printf("Switch to Operational mode\n");

    uavcan_cmd_msg_.cmd.len = NUM_ESC_;

    for(size_t i = 0; i < NUM_ESC_; i++)
    {
        uavcan_cmd_msg_.cmd.data[i] = 10;
    }
    // By broadcasting cmd data to esc
    // switch to operational mode
    uavcan_esc_cmd_publisher_.broadcast(uavcan_cmd_msg_);

    
    // Set Publisher and Subscriber for ROS
    ros_rpm_publisher_ = nh.advertise<ros_libcanard::actual_rpm>("/uav/actual_rpm",1);
    ros_voltage_publisher_ = nh.advertise<std_msgs::Float32>("/uav/voltage",1);
    ros_cmd_raw_subscriber_ = nh.subscribe<ros_libcanard::cmd_raw>(
        "/uav/cmd_raw",
        1,
        &RosLibcanard::callback_cmd_raw,
        this
    );

}

void RosLibcanard::run()
{
    uint64_t next_1hz_service_at = 0;
    uint64_t ts;

    last_pub_time_ = ros::Time::now();
    while(ros::ok())
    {
        ts = micros64();
        
        if(ts >= next_1hz_service_at)
        {
            next_1hz_service_at += 1000000ULL;
            send_NodeStatus();
        }
        canard_iface_.process(1);
        
        // printf("ros %f \n",ros::Time::now().toSec());

        ros::spinOnce();
    }
}

RosLibcanard::~RosLibcanard()
{
}

void RosLibcanard::handle_escStatus(const CanardRxTransfer &transfer, 
const uavcan_equipment_esc_Status &msg)
{
    // Get rpm msg


    ros_rpm_msg_.rpm[msg.esc_index] = msg.rpm;
    esc_count_++;

    if(esc_count_ == NUM_ESC_)
    {
        if ((ros::Time::now() - last_pub_time_).toSec() >= 0.005)
        {
            // printf("Handle esc status:  %f \n",ros::Time::now().toSec());
            ros_voltage_msg_.data = msg.voltage;
            ros_rpm_msg_.stamp = ros::Time::now();
            ros_voltage_publisher_.publish(ros_voltage_msg_);
            ros_rpm_publisher_.publish(ros_rpm_msg_);
            esc_count_ = 0;

            last_pub_time_ = ros::Time::now();
        }
    }
    // loop_rate.sleep();

}

void RosLibcanard::handle_getNodeInfo(const CanardRxTransfer &transfer, 
const uavcan_protocol_GetNodeInfoResponse &rsp)
{
    printf("Got GetNodeInfo response\n");
    printf("ESC Name: ");
    for(int i = 0; i < rsp.name.len; i++)
    {
        printf("%c", rsp.name.data[i]);
    }
    printf("\n");
}

void RosLibcanard::send_NodeStatus()
{
    uavcan_node_status_msg_.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    uavcan_node_status_msg_.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    uavcan_node_status_msg_.sub_mode = 0;
    uavcan_node_status_msg_.uptime_sec = millis32() / 1000UL;
    uavcan_node_status_publisher_.broadcast(uavcan_node_status_msg_);
}

void RosLibcanard::callback_cmd_raw(const ros_libcanard::cmd_raw::ConstPtr &msg)
{
    for(size_t i = 0; i < NUM_ESC_; i++)
    {
        uavcan_cmd_msg_.cmd.data[i] = msg->raw[i];
    }
    uavcan_esc_cmd_publisher_.broadcast(uavcan_cmd_msg_);
}
