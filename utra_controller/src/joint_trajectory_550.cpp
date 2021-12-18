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

  ros::NodeHandle nh;
  bool ut_states_update;
  // publish from utra_publish.cpp
  if (nh.getParam("ut_states_update", ut_states_update)) {
    if(ut_states_update == false){
      ROS_ERROR("utra states no update !!!!!");
      as->setAborted();
      return;
    }
  }else{
    as->setAborted();
    return;
  }

  utra_msg::Mservojoint srv;
  srv.request.axiz = 6;
  float sample_duration,plan_delay;
  int CON;

  nh.param<float>("ut_sample_duration", sample_duration, 0.1);
  srv.request.time = sample_duration;

  nh.param<float>("ut_plan_delay", plan_delay, 0.5);
  srv.request.plan_delay = plan_delay; //第一次发数据，延迟0.2s

  nh.param<int>("ut_send_once", CON, 30);
  ROS_INFO("ut_sample_duration : %f  ut_plan_delay :%f ,ut_send_once : %d",sample_duration,plan_delay,CON);

  int p_size = goal->trajectory.points.size();
  int have_CON_s = p_size/CON;
  int have_CON_m = p_size%CON;
  // 分一个循环 每次传30个点， 最后传余数的点
  for (size_t s = 0; s < have_CON_s; s++)
  {
    srv.request.num = CON;
    for (int i = s*CON; i < s*CON + CON; i++) {
      for (size_t j = 0; j < 6; j++)
      {
        srv.request.frames.push_back(goal->trajectory.points[i].positions[j]);
      }
    }
    if (Mservojoint_client.call(srv))
    {
      ROS_INFO("call ret: %d", srv.response.ret);
      if(srv.response.ret == -3){
        as->setAborted();
        return;
      }
    }
    else
    {
      ROS_ERROR("Failed to call service ");
      as->setAborted();
      return;
    }
    srv.request.plan_delay = 0;
    srv.request.frames.clear();
  }
  //  最后传余数的点
  srv.request.frames.clear();
  srv.request.num = have_CON_m;
  for (int i = have_CON_s*CON; i < have_CON_s*CON + have_CON_m; i++) {
    for (size_t j = 0; j < 6; j++)
    {
      srv.request.frames.push_back(goal->trajectory.points[i].positions[j]);
    }
  }
  if (Mservojoint_client.call(srv))
  {
    ROS_INFO("call ret: %d", srv.response.ret);
    if(srv.response.ret == -3){
      as->setAborted();
      return;
    }
  }
  else
  {
    ROS_ERROR("Failed to call service ");
    as->setAborted();
    return;
  }
  ROS_INFO("Recieve action successful!");


  // check utra moveing state, and update the return 

  ros::Duration(0.1).sleep();
  utra_msg::GetInt16 srv1;
  int error_count = 0;
  bool isStart = false;
  int sleep_time=0;
  while (1)
  {
    sleep_time++;
    if (status_get_client.call(srv1))
    {
      if(srv1.response.ret == -3)
      {
        error_count++;
        if(error_count>3){
          as->setAborted();
          return;
        }
      }else if(srv1.response.data == 1 ){
        isStart = true;  // set flog that utra is moveing
      }
      else if(srv1.response.data == 0 || srv1.response.data == 2 ){
        if(isStart){ // if state from moving to narmal or sleep , it mean that utra is run finish
          as->setSucceeded();
          ROS_INFO("moveing finish!!");
          return;
        }else{
          if(sleep_time>10){ // if isStart alway false, and more than 10 times , return error.
            as->setAborted();
            ROS_INFO("moveing finish!!");
            return;
          }
        }
      }else if(srv1.response.data == 3 || srv1.response.data == 4 ){
        as->setAborted();
        ROS_ERROR("moveing pause or stop!!");
        return;
      }
      else{
        if(error_count>0){
          error_count--;
        }
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