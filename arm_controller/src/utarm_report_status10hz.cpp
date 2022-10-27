#include <utra_msg/RobotMsg.h>
#include "ros/ros.h"
#include "utra/utra_api_tcp.h"
#include "utra/utra_report_status.h"

/**
 * The automatic reporting data of the Arm NTRO controller, published to the message[ut_arm/states]
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "utarm_report_status10hz");
  ros::NodeHandle nh;
  ros::Publisher robotStates = nh.advertise<utra_msg::RobotMsg>("ut_arm/states", 1000, true);
  std::string arm_ip;
  if (nh.getParam("arm_ip", arm_ip)) {
    ROS_INFO("Got param: %s", arm_ip.c_str());
  } else {
    ROS_ERROR("Failed to get param 'arm_ip'");
    return -1;
  }
  char *cstr = new char[arm_ip.length() + 1];
  std::strcpy(cstr, arm_ip.c_str());

  uint8_t axis = 6;
  int no_update = 0;
  utra_msg::RobotMsg robotMsg;
  arm_report_status_t rx_data;
  UtraReportStatus10Hz *utra_report;
  UtraApiTcp *utra = new UtraApiTcp(cstr);

  int ret = -3;
  for (int i = 0; i < 3; i++) {
    ret = utra->get_axis(&axis);
    if (ret != -3) break;
  }

  utra_report = new UtraReportStatus10Hz(cstr, axis);
  nh.setParam("is_utarm_states_update", true);

  ros::Rate loop_rate(9);
  while (ros::ok()) {
    if (utra_report->is_update()) {
      // ROS_INFO("utra_report->is_update");
      utra_report->get_data(&rx_data);
      // utra_report->print_data(&rx_data);
      robotMsg.len = rx_data.len;
      robotMsg.axis = rx_data.axis;
      robotMsg.motion_status = rx_data.motion_status;
      robotMsg.motion_mode = rx_data.motion_mode;
      robotMsg.mt_brake = rx_data.mt_brake;
      robotMsg.mt_able = rx_data.mt_able;
      robotMsg.err_code = rx_data.err_code;
      robotMsg.war_code = rx_data.war_code;
      robotMsg.cmd_num = rx_data.cmd_num;
      for (size_t i = 0; i < 32; i++) robotMsg.joint[i] = rx_data.joint[i];
      for (size_t i = 0; i < 6; i++) robotMsg.pose[i] = rx_data.pose[i];
      for (size_t i = 0; i < 32; i++) robotMsg.tau[i] = rx_data.tau[i];
      if (no_update > 0) no_update--;

      nh.setParam("is_utarm_states_update", true);
      robotStates.publish(robotMsg);
    } else {
      if (no_update < 10) no_update++;
    }

    if (no_update >= 10) nh.setParam("is_utarm_states_update", false);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}