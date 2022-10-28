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
#include <math.h>
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
#include "ut_msg/GetFloat32A.h"
#include "ut_msg/GetInt16.h"
#include "ut_msg/MovetoServoJoint.h"

int axis;
ros::ServiceClient MovetoServoJoint_client;
ros::ServiceClient status_get_client;
ros::ServiceClient joint_actual_pos_client;
using namespace std;

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

float sample_duration = 0.03;

void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as) {
  // Detecting the gap between the first point and the start position, too large, will not allow the planning to run
  ut_msg::GetFloat32A srv2;
  if (!joint_actual_pos_client.call(srv2)) {
    ROS_ERROR("[ArmMvSeT] Failed to call service : get_joint_actual_pos");
    as->setAborted();
    return;
  }

  ROS_INFO("[ArmMvSeT] call get_joint_actual_pos ret: %d", srv2.response.ret);
  if (srv2.response.ret == -3) {
    as->setAborted();
    return;
  }

  // ROS_INFO("joint1: %f, joint2: %f, joint3: %f, joint4: %f, joint5: %f, joint6: %f",srv2.response.data[0],
  // srv2.response.data[1],srv2.response.data[2],srv2.response.data[3],srv2.response.data[4],srv2.response.data[5]);
  for (int i = 0; i < axis; i++) {
    float diff = fabs(goal->trajectory.points[0].positions[i] - srv2.response.data[i]);
    if (diff > 0.01) {
      ROS_ERROR("[ArmMvSeT] current joint%d diff more than %f", i, diff);
      as->setAborted();
      return;
    }
  }

  ros::NodeHandle nh;
  bool is_utarm_states_update;
  // publish from utarm_report_status10hz.cpp
  // If the data broadcast is not updated, the path planning is not run
  if (nh.getParam("is_utarm_states_update", is_utarm_states_update)) {
    if (is_utarm_states_update == false) {
      ROS_ERROR("[ArmMvSeT] arm states no update !!!!!");
      as->setAborted();
      return;
    }
  } else {
    as->setAborted();
    return;
  }

  // Start sending data
  ut_msg::MovetoServoJoint srv;
  srv.request.axiz = axis;
  float sample_duration, plan_delay;
  int CON;

  nh.param<float>("ut_sample_duration", sample_duration, 0.1);
  srv.request.time = sample_duration;

  nh.param<float>("ut_plan_delay", plan_delay, 0.5);
  srv.request.plan_delay = plan_delay;  // The first time the data is sent, the delay is 0.2s

  nh.param<int>("ut_send_once", CON, 30);
  ROS_INFO("[ArmMvSeT] ut_sample_duration:%f, ut_plan_delay:%f, ut_send_once:%d", sample_duration, plan_delay, CON);

  int p_size = goal->trajectory.points.size();
  int have_CON_s = p_size / CON;
  int have_CON_m = p_size % CON;
  // Divide a cycle to pass 30 points each time, and finally pass the remainder of the points
  for (size_t s = 0; s < have_CON_s; s++) {
    srv.request.num = CON;
    for (int i = s * CON; i < s * CON + CON; i++) {
      for (size_t j = 0; j < axis; j++) srv.request.frames.push_back(goal->trajectory.points[i].positions[j]);
    }
    if (MovetoServoJoint_client.call(srv)) {
      ROS_INFO("[ArmMvSeT] MovetoServoJoint_client.call ret: %d", srv.response.ret);
      if (srv.response.ret == -3) {
        ROS_ERROR("[ArmMvSeT] Failed in call MovetoServoJoint_client.call");
        as->setAborted();
        return;
      }
    } else {
      ROS_ERROR("[ArmMvSeT] Failed to MovetoServoJoint_client.call ");
      as->setAborted();
      return;
    }
    srv.request.plan_delay = 0;
    srv.request.frames.clear();
  }

  //  The last point to pass the remainder
  srv.request.frames.clear();
  srv.request.num = have_CON_m;
  for (int i = have_CON_s * CON; i < have_CON_s * CON + have_CON_m; i++) {
    for (size_t j = 0; j < axis; j++) srv.request.frames.push_back(goal->trajectory.points[i].positions[j]);
  }
  if (MovetoServoJoint_client.call(srv)) {
    ROS_INFO("[ArmMvSeT] MovetoServoJoint_client.call ret: %d", srv.response.ret);
    if (srv.response.ret == -3) {
      ROS_ERROR("[ArmMvSeT] Failed in call MovetoServoJoint_client.call");
      as->setAborted();
      return;
    }
  } else {
    ROS_ERROR("[ArmMvSeT] Failed to call MovetoServoJoint_client.call ");
    as->setAborted();
    return;
  }
  ROS_INFO("[ArmMvSeT] Recieve action successful!");

  // check arm moveing state, and update the return
  ros::Duration(0.1).sleep();
  ut_msg::GetInt16 srv1;
  int error_count = 0;
  bool isStart = false;
  int sleep_time = 0;
  while (1) {
    sleep_time++;
    if (status_get_client.call(srv1)) {
      if (srv1.response.ret == -3) {
        error_count++;
        if (error_count > 3) {
          as->setAborted();
          return;
        }
      } else if (srv1.response.data == 1) {
        isStart = true;  // set flog that arm is moveing
      } else if (srv1.response.data == 0 || srv1.response.data == 2) {
        if (isStart) {  // if state from moving to narmal or sleep , it mean that arm is run finish
          // ros::Duration(0.05).sleep();
          as->setSucceeded();
          ROS_INFO("[ArmMvSeT] moveing finish!!");
          return;
        } else {
          if (sleep_time > 10) {  // if isStart alway false, and more than 10 times , return error.
            as->setAborted();
            ROS_ERROR("[ArmMvSeT] arm not move!! ");
            return;
          }
        }
      } else if (srv1.response.data == 3 || srv1.response.data == 4) {
        as->setAborted();
        ROS_ERROR("[ArmMvSeT] arm pause or stop!!");
        return;
      } else {
        if (error_count > 0) error_count--;
      }
    } else {
      as->setAborted();
      return;
    }
    ros::Duration(0.1).sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "utarm_mvservo_trajectory");
  ros::NodeHandle nh;
  ros::NodeHandle pn("~");

  std::string utra_ns;
  if (pn.getParam("utra_ns", utra_ns)) {
    ROS_INFO("[ArmMvSeT] Got param: %s", utra_ns.c_str());
  } else {
    ROS_ERROR("[ArmMvSeT] Failed to get param 'utra_ns'");
    return -1;
  }

  std::string axis_c;
  if (pn.getParam("axis", axis_c)) {
    axis = std::stoi(axis_c);
    ROS_INFO("[ArmMvSeT] get axis: %s, %d", axis_c.c_str(), axis);
  } else {
    ROS_ERROR("[ArmMvSeT] Failed to get param 'axis'");
    return -1;
  }

  MovetoServoJoint_client = nh.serviceClient<ut_msg::MovetoServoJoint>("utsrv/mv_servo_joint");
  status_get_client = nh.serviceClient<ut_msg::GetInt16>("utsrv/status_get");
  joint_actual_pos_client = nh.serviceClient<ut_msg::GetFloat32A>("utsrv/get_joint_actual_pos");

  char* cstr = new char[utra_ns.length() + 1];
  std::strcpy(cstr, utra_ns.c_str());
  Server server(nh, cstr, boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
}