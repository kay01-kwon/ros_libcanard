# ROS2 LibCanard Package - AI Coding Instructions

## Project Overview
This is a ROS2 package (`ros_libcanard`) that provides message definitions for CAN-based communication, likely using the libcanard library for UAVCAN/DroneCAN protocol communication. The package defines standardized message formats for both quadcopter and hexacopter motor control and feedback.

## Architecture & Message Structure

### Message Definitions
- **Command Messages**: `*_cmd_raw.msg` contain `uint16` arrays for raw motor commands
- **Feedback Messages**: `*_actual_rpm.msg` contain `uint32` arrays for actual RPM values
- **Vehicle Types**: Support for both quad (4 motors) and hexa (6 motors) configurations
- **Timestamp Pattern**: All messages include `time stamp` field for synchronization

### Key Message Files:
- `msg/quad_cmd_raw.msg` - 4-motor command array (`uint16[4]`)
- `msg/quad_actual_rpm.msg` - 4-motor RPM feedback (`uint32[4]`)
- `msg/hexa_cmd_raw.msg` - 6-motor command array (`uint16[6]`)
- `msg/hexa_actual_rpm.msg` - 6-motor RPM feedback (`uint32[6]`)

## Development Patterns

### Package Structure
- Pure message-only package with no source code implementation
- Uses standard ROS2 `ament_cmake` build system
- Minimal dependencies: only `std_msgs`
- Empty `src/` and `include/` directories indicate this is purely a message interface package

### Build & Development Workflow
```bash
# Build from workspace root
cd /home/kay/ros2_ws
colcon build --packages-select ros_libcanard

# Source the setup
source install/setup.bash
```

### Code Conventions
- Message naming follows pattern: `{vehicle_type}_{data_type}.msg`
- Array sizes match motor count: quad=4, hexa=6
- Raw commands use `uint16` (0-65535 range)
- RPM feedback uses `uint32` for higher precision
- All messages timestamped for synchronization

## Integration Points
- Designed to interface with CAN bus communication systems
- Likely integrates with libcanard library for UAVCAN protocol
- Message types suggest ESC (Electronic Speed Controller) communication
- Part of larger UAV/drone control system in `/home/kay/ros2_ws/src/`

## Key Considerations for AI Agents
1. **No Source Code**: This package only defines message interfaces - implementation happens in consuming packages
2. **Motor Count Awareness**: Always respect quad vs hexa distinctions in message usage
3. **Data Range Limits**: `uint16` cmd_raw values (0-65535), `uint32` rpm values (0-4294967295)
4. **CAN Protocol Context**: Changes should consider CAN bus bandwidth and timing constraints
5. **Timestamping Critical**: All messages must maintain timestamp consistency for control loops

## Testing & Validation
- Package uses standard ROS2 linting with copyright/cpplint checks disabled
- Message validation happens at build time via ROS2 message compiler
- Test new messages with `ros2 interface show ros_libcanard/msg/<MsgName>`