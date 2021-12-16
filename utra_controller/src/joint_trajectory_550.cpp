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
#include "utra_msg/Mservojoint.h"
#include "utra_msg/GetInt16.h"

ros::ServiceClient Mservojoint_client;
ros::ServiceClient status_get_client;
using namespace std;

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

float sample_duration = 0.03;



void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as) {
  utra_msg::Mservojoint srv;
  srv.request.axiz = 6;
  srv.request.num = goal->trajectory.points.size();
  srv.request.time = sample_duration;
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

  ros::Duration(0.2).sleep();
  utra_msg::GetInt16 srv1;
  while (1)
  {
    
    if (status_get_client.call(srv1))
    {
      if(srv1.response.ret == -3)
      {
        as->setAborted();
        return;
      }else if(srv1.response.data != 1){
        as->setSucceeded();
        ROS_INFO("moveing finish!!");
        return;
      }
    }else{
      as->setAborted();
      return;
    }
    ros::Duration(0.1).sleep();
  }
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
  status_get_client = nh.serviceClient<utra_msg::GetInt16>("utra/status_get");

  char* cstr = new char[utra_ns.length() + 1];
  std::strcpy(cstr, utra_ns.c_str());
  Server server(nh, cstr, boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
}