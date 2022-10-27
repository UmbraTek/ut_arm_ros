cmake_minimum_required(VERSION 3.0.2)
project(utra_rviz)

# # Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  std_msgs
  utra_msg
)

catkin_package(
  CATKIN_DEPENDS roscpp rviz std_msgs utra_msg
)

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(

  # include
  ${catkin_INCLUDE_DIRS}
)

# # This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
include_directories(${catkin_INCLUDE_DIRS})

# # I prefer the Qt signals and slots to avoid defining "emit", "slots",
# # etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

# # Here we specify which header files need to be run through "moc",
# # Qt's meta-object compiler.
qt5_wrap_cpp(MOC_FILES
  src/utra_rviz.h
)

# # Here we specify the list of source files, including the output of
# # the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/utra_rviz.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} utra_msg_generate_messages_cpp)