# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_CXX_FLAGS "-std=c++11")
project(utra_api)
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
#  DEPENDS other non-ROS libs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} STATIC
    src/common/network.cpp    
    src/common/preempt_rt.cpp    
    src/common/socket_serial.cpp
    src/common/socket_tcp.cpp
    src/base/servo_api_base.cpp


    src/base/arm_api_base.cpp
    src/base/arm_report_status.cpp
    src/base/arm_report_config.cpp
    src/utra/utra_api_tcp.cpp
    src/utra/utra_report_status.cpp
    src/utra/utra_report_config.cpp
    )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} rt pthread)
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)