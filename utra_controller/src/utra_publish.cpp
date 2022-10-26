#include <utra_msg/RobotMsg.h>
#include "ros/ros.h"
#include "utra/utra_api_tcp.h"
#include "utra/utra_report_status.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "utra_publish");
  ros::NodeHandle nh;
  ros::Publisher robotStates = nh.advertise<utra_msg::RobotMsg>("utra/states", 1000, true);
  std::string utra_ip;
  if (nh.getParam("utra_ip", utra_ip)) {
    ROS_INFO("Got param: %s", utra_ip.c_str());
  } else {
    ROS_ERROR("Failed to get param 'utra_ip'");
    return -1;
  }

  ros::Rate loop_rate(10);
  utra_msg::RobotMsg robotMsg;

  arm_report_status_t rx_data;
  UtraReportStatus10Hz *utra_report;
  char *cstr = new char[utra_ip.length() + 1];
  std::strcpy(cstr, utra_ip.c_str());

  uint8_t axis = 6;
  UtraApiTcp *utra = new UtraApiTcp(cstr);
  int ret = -3;
  for (int i = 0; i < 3; i++) {
    ret = utra->get_axis(&axis);
    if (ret != -3) break;
  }
  utra_report = new UtraReportStatus10Hz(cstr, axis);

  int no_update = 0;
  nh.setParam("ut_states_update", true);
  while (ros::ok()) {
    if (utra_report->is_error()) {
      // do something
    }
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
      for (size_t i = 0; i < 32; i++) {
        robotMsg.joint[i] = rx_data.joint[i];
      }
      for (size_t i = 0; i < 6; i++) {
        robotMsg.pose[i] = rx_data.pose[i];
      }
      for (size_t i = 0; i < 32; i++) {
        robotMsg.tau[i] = rx_data.tau[i];
      }
      if (no_update > 0) {
        no_update--;
      }
      nh.setParam("ut_states_update", true);
      robotStates.publish(robotMsg);
    } else {
      if (no_update < 10) {
        no_update++;
      }
    }
    if (no_update >= 10) {
      nh.setParam("ut_states_update", false);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}