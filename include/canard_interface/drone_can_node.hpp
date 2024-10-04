#ifndef DRONE_CAN_NODE_HPP
#define DRONE_CAN_NODE_HPP
#include "dsdl_generated/dronecan_msgs.h"
#include "canard_interface/canard_interface.hpp"

#define NUM_ESCS 4

class CanardInterface;

class DroneCanNode
{
    public:

        void initiate_and_switch_to_op_mode(const char *interface_name);

        void process_node();

        void set_esc_raw(int16_t raw_value[NUM_ESCS]);

        void get_esc_rpm(int32_t rpm[NUM_ESCS]);


    private:

        CanardInterface canard_iface_{0};
        
        Canard::Publisher<uavcan_protocol_NodeStatus> node_status_pub_{canard_iface_};
        Canard::Publisher<uavcan_equipment_esc_RPMCommand> esc_rpm_pub_{canard_iface_};
        Canard::Publisher<uavcan_equipment_esc_RawCommand> esc_raw_pub_{canard_iface_};

        void handle_EscStatus(const CanardRxTransfer& transfer, const uavcan_equipment_esc_Status& msg);
        Canard::ObjCallback<DroneCanNode, uavcan_equipment_esc_Status> esc_status_cb_{this, &DroneCanNode::handle_EscStatus};
        Canard::Subscriber<uavcan_equipment_esc_Status> esc_status_sub_{esc_status_cb_, 0};

        void handle_GetNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoResponse& rsp);
        Canard::ObjCallback<DroneCanNode, uavcan_protocol_GetNodeInfoResponse> get_node_info_cb_{this, &DroneCanNode::handle_GetNodeInfo};
        Canard::Client<uavcan_protocol_GetNodeInfoResponse> get_node_info_client_{canard_iface_, get_node_info_cb_};

        void send_NodeStatus();

        void broadcast_RPMCommand(int32_t rpm[NUM_ESCS]);

        void broadcast_RawCommand(int16_t throttle[NUM_ESCS]);
        
        uavcan_protocol_NodeStatus node_status_msg_;
        uavcan_equipment_esc_RPMCommand rpm_cmd_;
        uavcan_equipment_esc_RawCommand raw_cmd_;

        int32_t actual_rpm_[NUM_ESCS] = {0, 0, 0, 0};
        float actual_current_[NUM_ESCS] = {0, 0, 0, 0};

        int16_t raw_value_[NUM_ESCS] = {0, 0, 0, 0};

        uint8_t prev_ecs_index_{0};
        uint8_t esc_count_{0};

        uint64_t next_1hz_service_at_{0};

};

#endif // DRONECAN_NODE_HPP