

/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: johnson <johnson@umbratek.com>
 ============================================================================*/
#include <ut_msg/RobotMsg.h>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Header.h"
#include "utra/utra_report_status.h"

ros::Publisher joint_msg_pub;
sensor_msgs::JointState jointState;

bool gripper;
int axis;

void statesCallback(const ut_msg::RobotMsg& msg) {
  jointState.header.stamp = ros::Time::now();
  jointState.position.clear();

  if (6 == axis) {
    jointState.position.push_back(msg.joint[0]);
    jointState.position.push_back(msg.joint[1]);
    jointState.position.push_back(msg.joint[2]);
    jointState.position.push_back(msg.joint[3]);
    jointState.position.push_back(msg.joint[4]);
    jointState.position.push_back(msg.joint[5]);
  } else if (7 == axis) {
    jointState.position.push_back(msg.joint[0]);
    jointState.position.push_back(msg.joint[1]);
    jointState.position.push_back(msg.joint[2]);
    jointState.position.push_back(msg.joint[3]);
    jointState.position.push_back(msg.joint[4]);
    jointState.position.push_back(msg.joint[5]);
    jointState.position.push_back(msg.joint[6]);
  }

  if (gripper) {
    float utra_gripper_pos;
    ros::NodeHandle n;
    if (n.getParam("utra_gripper_pos", utra_gripper_pos)) {
      jointState.position.push_back(utra_gripper_pos);
    } else {
      jointState.position.push_back(0);
    }
  }
  joint_msg_pub.publish(jointState);
}

/**
 * Forward messages[ut_arm/states] to custom utra_ns messages.
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "utarm_status_transmit");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  std::string utra_ns;
  if (pn.getParam("utra_ns", utra_ns)) {
    ROS_INFO("[ArmSteTr] get utra_ns: %s", utra_ns.c_str());
  } else {
    ROS_ERROR("[ArmSteTr] Failed to get param 'utra_ns'");
    return -1;
  }

  std::string axis_c;
  if (pn.getParam("axis", axis_c)) {
    axis = std::stoi(axis_c);
    ROS_INFO("[ArmSteTr] get axis: %s, %d", axis_c.c_str(), axis);
  } else {
    ROS_ERROR("[ArmSteTr] Failed to get param 'axis'");
    return -1;
  }

  char* cstr = new char[utra_ns.length() + 1];
  std::strcpy(cstr, utra_ns.c_str());
  joint_msg_pub = n.advertise<sensor_msgs::JointState>(cstr, 1000);
  ros::Subscriber sub = n.subscribe("ut_arm/states", 1000, statesCallback);

  if (6 == axis) {
    jointState.name.push_back("joint1");
    jointState.name.push_back("joint2");
    jointState.name.push_back("joint3");
    jointState.name.push_back("joint4");
    jointState.name.push_back("joint5");
    jointState.name.push_back("joint6");
  } else if (7 == axis) {
    jointState.name.push_back("joint1");
    jointState.name.push_back("joint2");
    jointState.name.push_back("joint3");
    jointState.name.push_back("joint4");
    jointState.name.push_back("joint5");
    jointState.name.push_back("joint6");
    jointState.name.push_back("joint7");
  }

  if (n.getParam("gripper", gripper)) {
    ROS_INFO("Get gripper param: %d", gripper);
    if (gripper) jointState.name.push_back("left_knuckle_joint");
  }

  ros::spin();
  return 0;
}