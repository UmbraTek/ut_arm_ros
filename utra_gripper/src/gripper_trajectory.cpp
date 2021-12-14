


/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: johnson <johnson@umbratek.com>
 ============================================================================*/
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <sstream>

using namespace std;

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;


void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as) {
  as->setSucceeded();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_server");
  ros::NodeHandle nh;
  std::string utra_ns_gripper;
  if (nh.getParam("utra_ns_gripper", utra_ns_gripper)) {
    ROS_INFO("Got param: %s", utra_ns_gripper.c_str());
  } else {
    ROS_ERROR("Failed to get param 'utra_ns_gripper'");
    return -1;
  }

  char* cstr = new char[utra_ns_gripper.length() + 1];
  std::strcpy(cstr, utra_ns_gripper.c_str());
  Server server(nh, cstr, boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
}
