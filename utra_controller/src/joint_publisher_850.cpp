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

bool gripper;

void statesCallback(const utra_msg::RobotMsg& msg)
{
  jointState.header.stamp = ros::Time::now();
  jointState.position.clear();
  jointState.position.push_back(msg.joint[0]);
  jointState.position.push_back(msg.joint[1]);
  jointState.position.push_back(msg.joint[2]);
  jointState.position.push_back(msg.joint[3]);
  jointState.position.push_back(msg.joint[4]);
  jointState.position.push_back(msg.joint[5]);
  if(gripper){
      float utra_gripper_pos;
      ros::NodeHandle n;
      if (n.getParam("utra_gripper_pos", utra_gripper_pos)) {
        jointState.position.push_back(utra_gripper_pos);
      }else{
        jointState.position.push_back(0);
      }
    }
  joint_msg_pub.publish(jointState);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "joint_publisher");

  ros::NodeHandle n;

  joint_msg_pub = n.advertise<sensor_msgs::JointState>("utra/joint_states_850", 1000);
  ros::Subscriber sub = n.subscribe("utra/states", 1000, statesCallback);
 
  jointState.name.push_back("joint1");
  jointState.name.push_back("joint2");
  jointState.name.push_back("joint3");
  jointState.name.push_back("joint4");
  jointState.name.push_back("joint5");
  jointState.name.push_back("joint6");

  if (n.getParam("gripper", gripper)) {
    ROS_INFO("Get gripper param: %d", gripper);
    if(gripper){
      jointState.name.push_back("left_knuckle_joint");
    }
  }
  // ros::MultiThreadedSpinner s(2);  
  // ros::spin(s); 
  ros::spin(); 
  return 0;
}