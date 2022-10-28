# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_CXX_FLAGS "-std=c++11")
project(arm_controller)

# # Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg actionlib control_msgs ut_api ut_msg)

# # Declare ROS messages and services
# add_message_files(
# FILES
# RobotMsg.msg
# )

# add_service_files(
# FILES
# Api.srv
# Connect.srv
# Disconnect.srv
# CheckConnect.srv
# )

# # Generate added messages and services
catkin_package(
  CATKIN_DEPENDS roscpp ut_msg ut_api
)

# # Declare a catkin package
# catkin_package(INCLUDE_DIRS src/utapi)

# # Build utarm_status_transmit and adra_listener
include_directories(include ${catkin_INCLUDE_DIRS})

# include_directories("utapi")
# add_subdirectory(utapi utapi_arm)
add_executable(utarm_status_transmit src/utarm_status_transmit.cpp)
target_link_libraries(utarm_status_transmit ${catkin_LIBRARIES})
add_dependencies(utarm_status_transmit ut_msg_generate_messages_cpp)

add_executable(utarm_mvservo_trajectory src/utarm_mvservo_trajectory.cpp)
target_link_libraries(utarm_mvservo_trajectory ${catkin_LIBRARIES})
add_dependencies(utarm_mvservo_trajectory ut_msg_generate_messages_cpp)

add_executable(utarm_api_server src/utarm_api_server.cpp)
target_link_libraries(utarm_api_server ${catkin_LIBRARIES})
add_dependencies(utarm_api_server ut_msg_generate_messages_cpp)

add_executable(utarm_report_status10hz src/utarm_report_status10hz.cpp)
target_link_libraries(utarm_report_status10hz ${catkin_LIBRARIES})
add_dependencies(utarm_report_status10hz ut_msg_generate_messages_cpp)
