

/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: johnson <johnson@umbratek.com>
 ============================================================================*/
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Header.h"
#include "utra/utra_report_status.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "joint_publisher");

  ros::NodeHandle n;

  ros::Publisher joint_msg_pub = n.advertise<sensor_msgs::JointState>("utra/joint_states_550", 1000);
  std::string utra_ip;
  if (n.getParam("utra_ip", utra_ip)) {
    ROS_INFO("Got param: %s", utra_ip.c_str());
  } else {
    ROS_ERROR("Failed to get param 'utra_ip'");
    return -1;
  }

  ros::Rate loop_rate(10);
  sensor_msgs::JointState jointState;
  jointState.name.push_back("joint1");
  jointState.name.push_back("joint2");
  jointState.name.push_back("joint3");
  jointState.name.push_back("joint4");
  jointState.name.push_back("joint5");
  jointState.name.push_back("joint6");

  arm_report_status_t rx_data;
  UtraReportStatus10Hz *utra_report;
  char *cstr = new char[utra_ip.length() + 1];
  std::strcpy(cstr, utra_ip.c_str());
  utra_report = new UtraReportStatus10Hz(cstr, 6);

  while (ros::ok()) {
    jointState.header.stamp = ros::Time::now();

    if (utra_report->is_update()) {
      utra_report->get_data(&rx_data);
      // utra_report->print_data(&rx_data);
      jointState.position.clear();
      jointState.position.push_back(rx_data.joint[0]);
      jointState.position.push_back(rx_data.joint[1]);
      jointState.position.push_back(rx_data.joint[2]);
      jointState.position.push_back(rx_data.joint[3]);
      jointState.position.push_back(rx_data.joint[4]);
      jointState.position.push_back(rx_data.joint[5]);
    }

    joint_msg_pub.publish(jointState);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}