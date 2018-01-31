cmake_minimum_required(VERSION 2.8.3)
project(object_avoidance_cpp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  flight_code
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
   FILES
   RingsFlowMsg.msg
   YawRateCmdMsg.msg
   FRFlowMsg.msg
   FRHarmonicsMsg.msg
   FRDTMsg.msg
   FRAllDataMsg.msg
   FOFFlowMsg.msg
   FOFDTMsg.msg
   FOFAllDataMsg.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )


catkin_package(CATKIN_DEPENDS roscpp message_generation message_runtime std_msgs)

###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

add_executable(FOF_node src/FOF.cpp)
add_dependencies(FOF_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(FOF_node ${catkin_LIBRARIES})

add_executable(FR_node src/FR.cpp)
add_dependencies(FR_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(FR_node ${catkin_LIBRARIES})

add_executable(record_FOF_node src/record_FOF.cpp)
add_dependencies(record_FOF_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_FOF_node ${catkin_LIBRARIES})

add_executable(record_FR_node src/record_FR.cpp)
add_dependencies(record_FR_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_FR_node ${catkin_LIBRARIES})
