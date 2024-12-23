#include "drone_can_node.hpp"

void DroneCanNode::initiate_and_switch_to_op_mode(const char *interface_name)
{
    canard_iface_.init(interface_name);

    uint8_t node_id = canard_iface_.get_node_id();

    int16_t raw_op[NUM_ESCS];

    for(size_t i = 0; i < NUM_ESCS; i++)
    {
        actual_rpm_[i] = 0;
        actual_current_[i] = 0;
        raw_value_[i] = 0;
        raw_op[i] = 10;
    }
    

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

void DroneCanNode::process_node()
{
   
    uint64_t ts = micros64();

    if (ts >= next_1hz_service_at_) {
        next_1hz_service_at_ += 1000000ULL;
        send_NodeStatus();
    }

    canard_iface_.process(1);
}

void DroneCanNode::set_esc_raw(const int16_t raw_value[NUM_ESCS])
{
    for(size_t i = 0; i < NUM_ESCS ; i++)
        raw_value_[i] = raw_value[i];
}

void DroneCanNode::get_esc_rpm(int32_t rpm[NUM_ESCS]) const
{
    for(size_t i = 0; i < NUM_ESCS ; i++)
        rpm[i] = actual_rpm_[i];
}

void DroneCanNode::get_voltage(float &voltage) const
{
    voltage = voltage_;
}

void DroneCanNode::handle_EscStatus(const CanardRxTransfer &transfer, 
const uavcan_equipment_esc_Status &msg)
{
    if(msg.esc_index != prev_ecs_index_) {
        esc_count_++;
    }
    else if(NUM_ESCS == 1)
    {
        esc_count_ = 1;
    }

    printf("%d \n", esc_count_);

    actual_rpm_[msg.esc_index] = msg.rpm;
    actual_current_[msg.esc_index] = msg.current;

    if(esc_count_ == NUM_ESCS) {
        esc_count_ = 0;

        printf("\n");

        printf("Voltage: %lf\n", msg.voltage);

        printf("Current:");

        for(size_t i = 0; i < NUM_ESCS; i++)
            printf(" %lf ", actual_current_[i]);
        printf("\n");

        printf("Actual RPM:");

        for(size_t i = 0; i < NUM_ESCS; i++)
            printf(" %d ", actual_rpm_[i]);
        printf("\n");

        printf("Command Raw: ");

        for(size_t i = 0; i < NUM_ESCS; i++)
            printf(" %d ", raw_value_[i]);
        printf("\n");

        voltage_ = msg.voltage;

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