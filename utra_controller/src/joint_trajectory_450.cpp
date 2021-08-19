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
#include "utra/utra_api_tcp.h"

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

UtraApiTcp* utra = NULL;

void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as) {
  // ros::Rate rate(1);

  Link1 = goal->trajectory.joint_names[0];
  Link2 = goal->trajectory.joint_names[1];
  Link3 = goal->trajectory.joint_names[2];
  Link4 = goal->trajectory.joint_names[3];
  float speed = 0.4;
  float acc = 60;
  int ret = -1;
  for (int i = 0; i < goal->trajectory.points.size(); i++) {
    cp1 = goal->trajectory.points[i].positions[0];
    cp2 = goal->trajectory.points[i].positions[1];
    cp3 = (goal->trajectory.points[i].positions[2] - 0.007) * 943.4;
    cp4 = goal->trajectory.points[i].positions[3];
    cp5 = 0;
    cp6 = 0;
    ROS_INFO("goal=[%f,%f,%f,%f,%f,%f]\n", cp1, cp2, cp3, cp4, cp5, cp6);
    float joint[6] = {cp1, cp2, cp3, cp4, cp5, cp6};
    ret = utra->moveto_joint_p2p(joint, speed, acc, 0);
    printf("moveto_joint_p2p  : %d\n", ret);
  }

  // control_msgs::FollowJointTrajectoryFeedback feedback;
  // feedback = NULL;
  // as->publishFeedback(feedback);

  // printf("goal=[%f,%f,%f,%f,%f,%f]\n",tp1,tp2,tp3,tp4,tp5,tp6);
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
  std::string utra_ip;
  if (nh.getParam("utra_ip", utra_ip)) {
    ROS_INFO("Got param: %s", utra_ip.c_str());
  } else {
    ROS_ERROR("Failed to get param 'utra_ip'");
    return -1;
  }
  char* ip = new char[utra_ip.length() + 1];
  std::strcpy(ip, utra_ip.c_str());
  utra = new UtraApiTcp(ip);
  int ret = utra->set_motion_mode(1);
  ROS_INFO("set_motion_mode   : %d\n", ret);
  ret = utra->set_motion_enable(8, 1);
  ROS_INFO("set_motion_enable : %d\n", ret);
  ret = utra->set_motion_status(0);
  printf("set_motion_status : %d\n", ret);

  char* cstr = new char[utra_ns.length() + 1];
  std::strcpy(cstr, utra_ns.c_str());
  Server server(nh, cstr, boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
}
