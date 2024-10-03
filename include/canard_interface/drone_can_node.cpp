#include "drone_can_node.hpp"

void DroneCanNode::initiate_and_switch_to_op_mode(const char *interface_name)
{
    canard_iface_.init(interface_name);

    uint8_t node_id = canard_iface_.get_node_id();

    // int32_t rpm_op[4] = {10, 10, 10, 10};
    int16_t raw_op[4] = {10, 10, 10, 10};

    printf("DroneCanNode started on %s, node ID %d\n", 
    interface_name, canard_iface_.get_node_id());

    /*
    Run the main loop.
    */

    send_NodeStatus();
    canard_iface_.process(10);

    uavcan_protocol_GetNodeInfoRequest req;
    
    for(size_t i = 1; i < NUM_ESCS + 1; i++) {
        
        printf("Requesting node info for node %ld\n", i);
        req = {};
        while(!get_node_info_client_.request(i,req))
        {
            canard_iface_.process(10);
        };
    }

    // broadcast_RPMCommand(operation);

    broadcast_RawCommand(raw_op);

}

void DroneCanNode::start_node(const char *interface_name)
{
   
        uint64_t ts = micros64();

        if (ts >= next_1hz_service_at_) {
            next_1hz_service_at_ += 1000000ULL;
            send_NodeStatus();
        }

        canard_iface_.process(10);
}

void DroneCanNode::set_esc_raw(int16_t raw_value[NUM_ESCS])
{
    for(size_t i = 0; i < NUM_ESCS ; i++)
        raw_value_[i] = raw_value[i];
}

void DroneCanNode::get_esc_rpm(int32_t rpm[NUM_ESCS])
{
    for(size_t i = 0; i < NUM_ESCS ; i++)
        rpm[i] = actual_rpm_[i];
}

void DroneCanNode::handle_EscStatus(const CanardRxTransfer &transfer, 
const uavcan_equipment_esc_Status &msg)
{

    switch(msg.esc_index) {
        case 0:
            actual_rpm_[0] = msg.rpm;
            actual_current_[0] = msg.current;
            esc_count_++;
            break;
        case 1:
            actual_rpm_[1] = msg.rpm;
            actual_current_[1] = msg.current;
            esc_count_++;
            break;
        case 2:
            actual_rpm_[2] = msg.rpm;
            actual_current_[2] = msg.current;
            esc_count_++;
            break;
        case 3:
            actual_rpm_[3] = msg.rpm;
            actual_current_[3] = msg.current;
            esc_count_++;
            break;
        default:
            break;
    }

    if(esc_count_ == 4) {
        esc_count_ = 0;

        printf("Voltage: %lf\n", msg.voltage);

        printf("Current: %lf %lf %lf %lf\n",
        actual_current_[0],
        actual_current_[1],
        actual_current_[2],
        actual_current_[3]);

        printf("Actual RPM: %d %d %d %d\n", 
        actual_rpm_[0],
        actual_rpm_[1], 
        actual_rpm_[2], 
        actual_rpm_[3]);

        printf("Command Raw: %d %d %d %d\n",
        raw_value_[0],
        raw_value_[1],
        raw_value_[2],
        raw_value_[3]);

        broadcast_RawCommand(raw_value_);

        printf("****************************************\n");
    }
    

}

void DroneCanNode::handle_GetNodeInfo(const CanardRxTransfer &transfer, 
const uavcan_protocol_GetNodeInfoResponse &rsp)
{
    printf("Got GetNodeInfo response\n");
    printf("ESC name: ");
    for(int i = 0; i < rsp.name.len; i++) {
        printf("%c", rsp.name.data[i]);
    }
    printf("\n");

}
void DroneCanNode::send_NodeStatus()
{

    node_status_msg_.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status_msg_.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status_msg_.sub_mode = 0;
    node_status_msg_.uptime_sec = millis32() / 1000UL;

    node_status_pub_.broadcast(node_status_msg_);

}

void DroneCanNode::broadcast_RPMCommand(int32_t rpm[NUM_ESCS])
{
    rpm_cmd_.rpm.len = NUM_ESCS;
    for (size_t i = 0; i < NUM_ESCS; i++) {
        rpm_cmd_.rpm.data[i] = rpm[i];
    }
    esc_rpm_pub_.broadcast(rpm_cmd_);
}

void DroneCanNode::broadcast_RawCommand(int16_t raw[NUM_ESCS])
{
    raw_cmd_.cmd.len = NUM_ESCS;
    for (size_t i = 0; i < NUM_ESCS; i++) {
        raw_cmd_.cmd.data[i] = raw[i];
    }
    esc_raw_pub_.broadcast(raw_cmd_);
}