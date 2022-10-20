# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_CXX_FLAGS "-std=c++11")
project(utra_controller)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg actionlib control_msgs utra_api utra_msg)

## Declare ROS messages and services
# add_message_files(
#     FILES 
#     RobotMsg.msg
#     )

# add_service_files(
#     FILES 
#     Api.srv
#     Connect.srv
#     Disconnect.srv
#     Checkconnect.srv
#     )

## Generate added messages and services


catkin_package(
  CATKIN_DEPENDS roscpp  utra_msg utra_api
)

## Declare a catkin package
# catkin_package(INCLUDE_DIRS src/utapi)

## Build joint_publisher and adra_listener
include_directories(include ${catkin_INCLUDE_DIRS})

# include_directories("utapi")
# add_subdirectory(utapi utapi_arm)

add_executable(joint_publisher_450 src/joint_publisher_450.cpp)
target_link_libraries(joint_publisher_450 ${catkin_LIBRARIES})
add_dependencies(joint_publisher_450 utra_controller_gencpp utra_msg_generate_messages_cpp)

add_executable(joint_trajectory_450 src/joint_trajectory_450.cpp)
target_link_libraries(joint_trajectory_450 ${catkin_LIBRARIES})
add_dependencies(joint_trajectory_450 utra_controller_gencpp utra_msg_generate_messages_cpp)


add_executable(joint_publisher_6 src/joint_publisher_6.cpp)
target_link_libraries(joint_publisher_6 ${catkin_LIBRARIES})
add_dependencies(joint_publisher_6 utra_controller_gencpp utra_msg_generate_messages_cpp)

add_executable(joint_trajectory_6 src/joint_trajectory_6.cpp)
target_link_libraries(joint_trajectory_6 ${catkin_LIBRARIES})
add_dependencies(joint_trajectory_6 utra_controller_gencpp utra_msg_generate_messages_cpp)

add_executable(utra_server src/utra_server.cpp)
target_link_libraries(utra_server ${catkin_LIBRARIES})
add_dependencies(utra_server utra_server_gencpp utra_msg_generate_messages_cpp)

add_executable(utra_publish src/utra_publish.cpp)
target_link_libraries(utra_publish ${catkin_LIBRARIES})
add_dependencies(utra_publish utra_publish_gencpp utra_msg_generate_messages_cpp)


