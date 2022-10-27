cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_CXX_FLAGS "-std=c++11")
project(utra_gripper)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs actionlib control_msgs)

# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(

    # INCLUDE_DIRS include
    # LIBRARIES arm_description
    # CATKIN_DEPENDS other_catkin_pkg
    # DEPENDS system_lib
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

# # Declare a C++ executable
# # With catkin_make all packages are built within a single CMake context
# # The recommended prefix ensures that target names across packages don't collide
add_executable(gripper_trajectory src/gripper_trajectory.cpp)
target_link_libraries(gripper_trajectory ${catkin_LIBRARIES})
