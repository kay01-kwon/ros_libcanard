cmake_minimum_required(VERSION 3.0.2)
project(ros_libcanard)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CANARD_INCLUDE include)


# CANARD sources
set(CANARD_SRC
    ${CANARD_INCLUDE}/canard_internals/canard.c
    ${CANARD_INCLUDE}/driver/socketcan.c
)

# DSDL generated sources
set(DSDL_GENERATED_SRC
    ${CANARD_INCLUDE}/dsdl_generated/uavcan.protocol.NodeStatus.c
    ${CANARD_INCLUDE}/dsdl_generated/uavcan.protocol.GetNodeInfo_res.c
    ${CANARD_INCLUDE}/dsdl_generated/uavcan.protocol.GetNodeInfo_req.c
    ${CANARD_INCLUDE}/dsdl_generated/uavcan.equipment.esc.RawCommand.c
    ${CANARD_INCLUDE}/dsdl_generated/uavcan.equipment.esc.RPMCommand.c
    ${CANARD_INCLUDE}/dsdl_generated/uavcan.equipment.esc.Status.c
    ${CANARD_INCLUDE}/dsdl_generated/uavcan.protocol.dynamic_node_id.Allocation.c
    ${CANARD_INCLUDE}/dsdl_generated/uavcan.protocol.param.GetSet_req.c
    ${CANARD_INCLUDE}/dsdl_generated/uavcan.protocol.param.GetSet_res.c
    ${CANARD_INCLUDE}/dsdl_generated/uavcan.protocol.param.ExecuteOpcode_req.c
    ${CANARD_INCLUDE}/dsdl_generated/uavcan.protocol.param.ExecuteOpcode_res.c
)


# Compiler flags
add_definitions(-DDRONECAN_CXX_WRAPPERS)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  cmd_raw.msg
  actual_rpm.msg
)



## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ros_libcanard
 CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${CANARD_INCLUDE}
  ${CANARD_INCLUDE}/canard_internals
  ${CANARD_INCLUDE}/driver
  ${CANARD_INCLUDE}/dsdl_generated
)

## Declare a C++ library
add_library(canard STATIC ${CANARD_SRC})
add_library(dsdl_generated STATIC ${DSDL_GENERATED_SRC} ${CANARD_INCLUDE})


add_executable(ros_libcanard_node 
src/ros_libcanard_node.cpp
include/ros_libcanard/ros_wrapper_libcanard.cpp
include/canard_interface/canard_interface.cpp
include/canard_interface/drone_can_node.cpp)

target_link_libraries(ros_libcanard_node ${catkin_LIBRARIES} canard dsdl_generated)