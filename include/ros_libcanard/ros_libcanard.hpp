#ifndef ROS_LIBCANARD_HPP
#define ROS_LIBCANARD_HPP

#include "canard_interface/canard_interface.hpp"
#include "dsdl_generated/dronecan_msgs.h"

#include <iostream>
#include <ros/ros.h>

// #include "ros_libcanard/actual_rpm.h"
// #include "ros_libcanard/cmd_raw.h"

#include "ros_libcanard/hexa_actual_rpm.h"
#include "ros_libcanard/hexa_cmd_raw.h"

#include <std_msgs/Float32.h>

class RosLibcanard{

    public:

        RosLibcanard(ros::NodeHandle &nh);

        void run();
        
        ~RosLibcanard();


    private:

        CanardInterface canard_iface_{0};
        uint8_t NUM_ESC_;


        // 1. Declare Canard publishers
        Canard::Publisher<uavcan_protocol_NodeStatus> 
        uavcan_node_status_publisher_{canard_iface_};
        Canard::Publisher<uavcan_equipment_esc_RawCommand> 
        uavcan_esc_cmd_publisher_{canard_iface_};


        // 2. ESC Status

        // 2.1 Handler for esc Status
        void handle_escStatus(const CanardRxTransfer& transfer, 
        const uavcan_equipment_esc_Status &msg);

        // 2.2 Object callback for esc status
        Canard::ObjCallback<RosLibcanard, uavcan_equipment_esc_Status>
        esc_status_cb_{this, &RosLibcanard::handle_escStatus};

        // 2.3 Declare ESC status subscriber
        Canard::Subscriber<uavcan_equipment_esc_Status>
        esc_status_sub_{esc_status_cb_, 0};


        // 3. Node handler
        
        // 3.1 Handler for getNodeInfo
        void handle_getNodeInfo(const CanardRxTransfer& transfer,
        const uavcan_protocol_GetNodeInfoResponse &rsp);

        // 3.2 Object callback for getNodeInfo
        Canard::ObjCallback<RosLibcanard, uavcan_protocol_GetNodeInfoResponse>
        get_node_info_cb_{this, &RosLibcanard::handle_getNodeInfo};

        // 3.3 Declare Client
        Canard::Client<uavcan_protocol_GetNodeInfoResponse>
        get_node_info_client_{canard_iface_, get_node_info_cb_};

        // 4. Member function for Libcanard
        
        void send_NodeStatus();

        uavcan_equipment_esc_RawCommand uavcan_cmd_msg_;
        uavcan_protocol_NodeStatus uavcan_node_status_msg_;


        // 4. ROS publisher and subscriber
        ros::Publisher ros_rpm_publisher_;
        ros::Publisher ros_voltage_publisher_;
        ros::Subscriber ros_cmd_raw_subscriber_;

        std_msgs::Float32 ros_voltage_msg_;
        // ros_libcanard::actual_rpm ros_rpm_msg_;
        ros_libcanard::hexa_actual_rpm ros_rpm_msg_;

        uint8_t esc_count_{0};

        ros::Rate loop_rate{200};

        ros::Time last_pub_time_;

        void callback_cmd_raw(const ros_libcanard::hexa_cmd_raw::ConstPtr &msg);


};



#endif