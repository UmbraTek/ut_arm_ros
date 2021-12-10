

/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: johnson <johnson@umbratek.com>
 ============================================================================*/
#include <actionlib/server/simple_action_server.h>
#include <arpa/inet.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <errno.h>
#include <netinet/in.h>
#include <pthread.h>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Float32MultiArray.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include "utra_msg/Mservojoint.h"

ros::ServiceClient Mservojoint_client;

using namespace std;

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

std::basic_string<char> Link1;
std::basic_string<char> Link2;
std::basic_string<char> Link3;
std::basic_string<char> Link4;
std::basic_string<char> Link5;
std::basic_string<char> Link6;

float cp1;
float cp2;
float cp3;
float cp4;
float cp5;
float cp6;


void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as) {
  utra_msg::Mservojoint srv;
  srv.request.axiz = 6;
  srv.request.num = goal->trajectory.points.size();
  srv.request.time = 0.1;
  for (int i = 0; i < goal->trajectory.points.size(); i++) {
    for (size_t j = 0; j < 6; j++)
    {
      srv.request.frames.push_back(goal->trajectory.points[i].positions[j]);
    }
  }

  if (Mservojoint_client.call(srv))
  {
    ROS_INFO("call ret: %d", srv.response.ret);
  }
  else
  {
    ROS_ERROR("Failed to call service ");
  }
  ROS_INFO("Recieve action successful!");
  as->setSucceeded();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_server");
  ros::NodeHandle nh;
  std::string utra_ns;
  if (nh.getParam("utra_ns", utra_ns)) {
    ROS_INFO("Got param: %s", utra_ns.c_str());
  } else {
    ROS_ERROR("Failed to get param 'utra_ns'");
    return -1;
  }
 
  Mservojoint_client = nh.serviceClient<utra_msg::Mservojoint>("utra/mv_servo_joint");

  char* cstr = new char[utra_ns.length() + 1];
  std::strcpy(cstr, utra_ns.c_str());
  Server server(nh, cstr, boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
}
