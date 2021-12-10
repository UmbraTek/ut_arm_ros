/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: johnson <johnson@umbratek.com>
 ============================================================================*/
#include "utra/utra_report_status.h"

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Header.h"
#include <utra_msg/RobotMsg.h>

ros::Publisher joint_msg_pub ;
sensor_msgs::JointState jointState ;
void statesCallback(const utra_msg::RobotMsg& msg)
{
  jointState.header.stamp = ros::Time::now();
  jointState.position.clear();
  jointState.position.push_back(msg.joint[0]);
  jointState.position.push_back(msg.joint[1]);
  // float shift = 0.007 + (msg.joint[2] / 1.5) * 0.5 / 3.1415 * 0.01;
  float shift = 0.007 + msg.joint[2] * 0.00106;
  jointState.position.push_back(shift);
  jointState.position.push_back(msg.joint[3]);
  joint_msg_pub.publish(jointState);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "joint_publisher");

  ros::NodeHandle n;

  joint_msg_pub = n.advertise<sensor_msgs::JointState>("utra/joint_states_450", 1000);
  ros::Subscriber sub = n.subscribe("utra/states", 1000, statesCallback);
  
  jointState.name.push_back("joint1");
  jointState.name.push_back("joint2");
  jointState.name.push_back("joint3");
  jointState.name.push_back("joint4");

  ros::spin();
  return 0;
}