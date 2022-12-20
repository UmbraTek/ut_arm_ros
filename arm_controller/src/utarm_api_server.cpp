/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: johnson <johnson@umbratek.com>
 ============================================================================*/
#include "ros/ros.h"
#include "utra/utra_api_tcp.h"
#include "utra/utra_flxie_api.h"

#include "ut_msg/CheckConnect.h"
#include "ut_msg/Connect.h"
#include "ut_msg/Disconnect.h"
#include "ut_msg/GetFloat32.h"
#include "ut_msg/GetFloat32A.h"
#include "ut_msg/GetGripperState.h"
#include "ut_msg/GetInt16.h"
#include "ut_msg/GetUInt16A.h"
#include "ut_msg/Grippermv.h"
#include "ut_msg/MovetoServoJoint.h"
#include "ut_msg/SetEnable.h"
#include "ut_msg/SetFloat32.h"
#include "ut_msg/SetGripperState.h"
#include "ut_msg/SetInt16.h"

#include "ut_msg/MovetoCartesianLine.h"
#include "ut_msg/MovetoCartesianLineB.h"
#include "ut_msg/MovetoJointP2p.h"

UtraApiTcp *armapi = NULL;
UtraFlxiE2Api *fixie = NULL;
std::string arm_ip = "";

constexpr unsigned int hash(const char *s, int off = 0) { return !s[off] ? 5381 : (hash(s, off + 1) * 33) ^ s[off]; }

//------------------------------------------------------------------------------------
//                                      ARM
//------------------------------------------------------------------------------------
#define CHECK_ARM_CONNECT()                                \
  if (armapi == NULL) {                                    \
    res.ret = -3;                                          \
    ROS_ERROR("[ArmContr] server have not connected arm"); \
    return true;                                           \
  }

bool connect_api(ut_msg::Connect::Request &req, ut_msg::Connect::Response &res) {
  if (armapi != NULL) {
    res.ret = 0;
    res.message = "server have connected arm";
    return true;
  }

  uint8_t axis;
  arm_ip = req.ip_address;
  char *ip = new char[req.ip_address.length() + 1];
  std::strcpy(ip, req.ip_address.c_str());
  armapi = new UtraApiTcp(ip);
  int ret = armapi->get_axis(&axis);
  res.ret = ret;
  if (ret == -3) {
    res.message = "[ArmContr] can not connect the arm";
  } else {
    res.message = "connect seccess";
  }
  return true;
}

bool disconnect_api(ut_msg::Disconnect::Request &req, ut_msg::Disconnect::Response &res) {
  CHECK_ARM_CONNECT()

  delete armapi;
  armapi = NULL;
  res.ret = 0;
  res.message = "arm have disconnected";
  return true;
}

bool check_c_api(ut_msg::CheckConnect::Request &req, ut_msg::CheckConnect::Response &res) {
  res.ip_address = "";
  CHECK_ARM_CONNECT()

  uint8_t axis;
  int ret = armapi->get_axis(&axis);
  res.ip_address = arm_ip;
  res.ret = ret;
  if (ret == -3) {
    res.message = "can not connect the arm";
  } else {
    res.message = "connect seccess";
  }
  return true;
}

bool mv_servo_joint(ut_msg::MovetoServoJoint::Request &req, ut_msg::MovetoServoJoint::Response &res) {
  CHECK_ARM_CONNECT()

  int ret = 0;
  int frames_index = 0;
  int frames_per_cmd = 3;  // how many points with one time to send

  int axiz = req.axiz;
  int frames_num = req.num;
  float time_per_cmd = req.time;
  int have_cmd_s = frames_num / frames_per_cmd;
  int have_cmd_m = frames_num % frames_per_cmd;

  ROS_INFO("[ArmContr] mv_servo_joint %d %f", frames_num, time_per_cmd);

  float frames[frames_per_cmd * 12] = {0};
  float mvtime[frames_per_cmd] = {0};
  for (int i = 0; i < frames_per_cmd; i++) mvtime[i] = time_per_cmd;

  // set arm sleep to wait for command
  if (req.plan_delay > 0) {
    ret = armapi->plan_sleep(req.plan_delay);
    ROS_INFO("[ArmContr] plan_sleep %d, delya:%f", ret, req.plan_delay);
  }

  for (int i = 0; i < have_cmd_s; i++) {
    for (int j = 0; j < frames_per_cmd * axiz; j++) frames[j] = req.frames[frames_index + j];
    frames_index = frames_index + frames_per_cmd * axiz;
    ret = armapi->moveto_servo_joint(frames_per_cmd, frames, mvtime);
    ROS_INFO("[ArmContr] moveto_servo_joint %d, i: %d", ret, i);
    if (ret != 0) {
      res.ret = ret;
      res.message = "moveto_servo_joint failed";
      ROS_ERROR("[ArmContr] moveto_servo_joint failed %d", ret);
      return true;
    }
  }

  // last time send
  for (size_t i = 0; i < have_cmd_m * axiz; i++) frames[i] = req.frames[frames_index + i];
  ret = armapi->moveto_servo_joint(have_cmd_m, frames, mvtime);
  ROS_INFO("[ArmContr] moveto_servo_joint %d, i: %d", ret, have_cmd_s);
  if (ret != 0) {
    res.ret = ret;
    res.message = "moveto_servo_joint failed";
    ROS_ERROR("[ArmContr] moveto_servo_joint failed %d", ret);
    return true;
  }

  res.ret = ret;
  res.message = "moveto_servo_joint success";
  return true;
}

bool status_set(ut_msg::SetInt16::Request &req, ut_msg::SetInt16::Response &res) {
  CHECK_ARM_CONNECT()

  int ret = armapi->set_motion_status(req.data);
  if (ret != 0) ROS_ERROR("[ArmContr] set_motion_status failed %d", ret);
  res.ret = ret;
  return true;
}

bool status_get(ut_msg::GetInt16::Request &req, ut_msg::GetInt16::Response &res) {
  CHECK_ARM_CONNECT()

  uint8_t status;
  int ret = armapi->get_motion_status(&status);
  if (ret != 0) ROS_ERROR("[ArmContr] get_motion_status failed %d", ret);
  res.ret = ret;
  res.data = status;
  ROS_INFO("[ArmContr] status_get %d, status %d", ret, status);
  return true;
}

bool mode_set(ut_msg::SetInt16::Request &req, ut_msg::SetInt16::Response &res) {
  CHECK_ARM_CONNECT()

  int ret = armapi->set_motion_mode(req.data);
  if (ret != 0) ROS_ERROR("[ArmContr] set_motion_mode failed %d", ret);
  res.ret = ret;
  return true;
}

bool mode_get(ut_msg::GetInt16::Request &req, ut_msg::GetInt16::Response &res) {
  CHECK_ARM_CONNECT()

  uint8_t mode;
  int ret = armapi->get_motion_mode(&mode);
  if (ret != 0) ROS_ERROR("[ArmContr] get_motion_mode failed %d", ret);
  res.ret = ret;
  res.data = mode;
  return true;
}

bool enable_set(ut_msg::SetEnable::Request &req, ut_msg::SetEnable::Response &res) {
  CHECK_ARM_CONNECT()

  int ret = armapi->set_motion_enable(req.axis, req.enable);
  if (ret != 0) ROS_ERROR("[ArmContr] set_motion_enable failed %d", ret);
  res.ret = ret;
  return true;
}

bool enable_get(ut_msg::GetInt16::Request &req, ut_msg::GetInt16::Response &res) {
  CHECK_ARM_CONNECT()

  int enable;
  int ret = armapi->get_motion_enable(&enable);
  if (ret != 0) ROS_ERROR("[ArmContr] get_motion_enable failed %d", ret);
  res.ret = ret;
  res.data = enable;
  return true;
}

bool get_error_code(ut_msg::GetUInt16A::Request &req, ut_msg::GetUInt16A::Response &res) {
  CHECK_ARM_CONNECT()

  uint8_t array[24] = {0};
  int ret = armapi->get_error_code(array);
  if (ret != 0) ROS_ERROR("[ArmContr] get_error_code failed %d", ret);
  res.ret = ret;
  for (size_t j = 0; j < 24; j++) res.data.push_back(array[j]);
  return true;
}

bool get_servo_msg(ut_msg::GetUInt16A::Request &req, ut_msg::GetUInt16A::Response &res) {
  CHECK_ARM_CONNECT()

  uint8_t array[24] = {0};
  int ret = armapi->get_servo_msg(array);
  if (ret != 0) ROS_ERROR("[ArmContr] get_servo_msg failed %d", ret);
  res.ret = ret;
  for (size_t j = 0; j < 24; j++) res.data.push_back(array[j]);
  return true;
}

bool moveto_joint_p2p(ut_msg::MovetoJointP2p::Request &req, ut_msg::MovetoJointP2p::Response &res) {
  CHECK_ARM_CONNECT()

  int axis = req.joints.size();
  if (axis == 6) {
    float joints[6] = {req.joints[0], req.joints[1], req.joints[2], req.joints[3], req.joints[4], req.joints[5]};
    int ret = armapi->moveto_joint_p2p(joints, req.speed, req.acc, 0);
    if (ret != 0) ROS_ERROR("[ArmContr] moveto_joint_p2p1 failed %d", ret);
    res.ret = ret;
    return true;
  } else if (axis == 7) {
    float joints[7] = {req.joints[0], req.joints[1], req.joints[2], req.joints[3],
                       req.joints[4], req.joints[5], req.joints[6]};
    int ret = armapi->moveto_joint_p2p(joints, req.speed, req.acc, 0);
    if (ret != 0) ROS_ERROR("[ArmContr] moveto_joint_p2p2 failed %d", ret);
    res.ret = ret;
    return true;
  } else {
    res.ret = -3;
    return true;
  }
}

bool moveto_cartesian_line(ut_msg::MovetoCartesianLine::Request &req, ut_msg::MovetoCartesianLine::Response &res) {
  CHECK_ARM_CONNECT()

  int axis = req.pose.size();
  if (axis == 6 || axis == 7) {
    float pose[6] = {req.pose[0], req.pose[1], req.pose[2], req.pose[3], req.pose[4], req.pose[5]};
    int ret = armapi->moveto_cartesian_line(pose, req.speed, req.acc, 0);
    if (ret != 0) ROS_ERROR("[ArmContr] moveto_cartesian_line failed %d", ret);
    res.ret = ret;
    return true;
  } else {
    res.ret = -3;
    return true;
  }
}

bool plan_sleep(ut_msg::SetFloat32::Request &req, ut_msg::SetFloat32::Response &res) {
  CHECK_ARM_CONNECT()

  int ret = armapi->plan_sleep(req.data);
  if (ret != 0) ROS_ERROR("[ArmContr] plan_sleep failed %d", ret);
  res.ret = ret;
  return true;
}

bool moveto_cartesian_lineb(ut_msg::MovetoCartesianLineB::Request &req, ut_msg::MovetoCartesianLineB::Response &res) {
  CHECK_ARM_CONNECT()

  int axis = req.pose.size();
  if (axis == 6 || axis == 7) {
    float pose[6] = {req.pose[0], req.pose[1], req.pose[2], req.pose[3], req.pose[4], req.pose[5]};
    int ret = armapi->moveto_cartesian_lineb(pose, req.speed, req.acc, 0, req.radii);
    if (ret != 0) ROS_ERROR("[ArmContr] moveto_cartesian_lineb failed %d", ret);
    res.ret = ret;
    return true;
  } else {
    res.ret = -3;
    return true;
  }
}

bool get_joint_actual_pos(ut_msg::GetFloat32A::Request &req, ut_msg::GetFloat32A::Response &res) {
  CHECK_ARM_CONNECT()

  float array[10] = {0};
  int ret = armapi->get_joint_target_pos(array);
  if (ret != 0) ROS_ERROR("[ArmContr] get_joint_target_pos failed %d", ret);
  res.ret = ret;
  for (size_t j = 0; j < 10; j++) res.data.push_back(array[j]);
  return true;
}


bool set_limit_angle_enable(ut_msg::SetInt16::Request &req, ut_msg::SetInt16::Response &res) {
  CHECK_ARM_CONNECT()

  int ret = armapi->set_limit_angle_enable(req.data);
  if (ret != 0) ROS_ERROR("[ArmContr] set_limit_angle_enable failed %d", ret);
  res.ret = ret;
  return true;
}


bool set_limit_geometry_enable(ut_msg::SetInt16::Request &req, ut_msg::SetInt16::Response &res) {
  CHECK_ARM_CONNECT()

  int ret = armapi->set_limit_geometry_enable(req.data);
  if (ret != 0) ROS_ERROR("[ArmContr] set_limit_geometry_enable failed %d", ret);
  res.ret = ret;
  return true;
}


//------------------------------------------------------------------------------------
//                                      FLXIE
//------------------------------------------------------------------------------------
#define CHECK_FLXIE_CONNECT()                                         \
  if (armapi == NULL || fixie == NULL) {                              \
    res.ret = -3;                                                     \
    ROS_ERROR("[ArmContr] server have not connected arm or gripper"); \
    return true;                                                      \
  }

bool gripper_state_set(ut_msg::SetGripperState::Request &req, ut_msg::SetGripperState::Response &res) {
  if (armapi == NULL) {
    res.ret = -3;
    res.message = "server have not connected arm";
    return true;
  }

  if (fixie == NULL) fixie = new UtraFlxiE2Api(armapi, 101);

  if (req.state == 1) {
    float value;
    int ret = fixie->set_motion_mode(1);
    ROS_INFO("[ArmContr] gripper_state_set1: %d\n", ret);
    ret = fixie->set_motion_enable(1);
    ROS_INFO("[ArmContr] gripper_state_set2: %d\n", ret);
    ret = fixie->get_pos_target(&value);
    if (ret != 0) ROS_ERROR("[ArmContr] flixe get_pos_target failed %d", ret);

    res.pos = value;
    res.ret = ret;
    res.message = "OK";
    return true;
  } else {
    int ret = fixie->set_motion_mode(0);
    ROS_INFO("[ArmContr] gripper_state_set3: %d\n", ret);
    ret = fixie->set_motion_enable(0);
    ROS_INFO("[ArmContr] gripper_state_set4: %d\n", ret);
    if (ret != 0) ROS_ERROR("[ArmContr] flixe set_motion_enable failed %d", ret);

    res.ret = ret;
    res.message = "OK";
    return true;
  }
  return true;
}

bool gripper_state_get(ut_msg::GetGripperState::Request &req, ut_msg::GetGripperState::Response &res) {
  res.enable = 0;
  CHECK_FLXIE_CONNECT()

  float value;
  int ret = fixie->get_pos_target(&value);
  if (ret != -3) {
    ros::NodeHandle n;
    float pos = (value / 100.0) - 0.4;
    n.setParam("utra_gripper_pos", pos);
  }

  uint8_t enable;
  ret = fixie->get_motion_enable(&enable);
  if (ret != 0) ROS_ERROR("[ArmContr] flixe get_motion_enable failed %d", ret);
  res.ret = ret;
  res.enable = enable;
  res.pos = value;
  res.message = "OK";
  return true;

  return true;
}

bool gripper_mv(ut_msg::Grippermv::Request &req, ut_msg::Grippermv::Response &res) {
  CHECK_FLXIE_CONNECT()

  int ret = fixie->set_pos_target(req.pos);
  if (ret != 0) ROS_ERROR("[ArmContr] flixe set_pos_target failed %d", ret);
  if (ret != -3) {
    ros::NodeHandle n;
    float pos = (req.pos / 100) - 0.4;
    n.setParam("utra_gripper_pos", pos);
  }
  res.ret = ret;
  res.message = "OK";
  return true;
}

bool gripper_vel_set(ut_msg::SetFloat32::Request &req, ut_msg::SetFloat32::Response &res) {
  CHECK_FLXIE_CONNECT()

  int ret1 = fixie->set_vel_limit_min(-req.data, true);
  int ret2 = fixie->set_vel_limit_max(req.data, true);
  res.ret = ret1 + ret2;
  if (res.ret != 0) ROS_ERROR("[ArmContr] flixe set_vel_limit_max failed %d %d", ret1, ret2);
  return true;
}

bool gripper_vel_get(ut_msg::GetFloat32::Request &req, ut_msg::GetFloat32::Response &res) {
  CHECK_FLXIE_CONNECT()

  float value;
  int ret1 = fixie->get_vel_limit_min(&value);
  int ret2 = fixie->get_vel_limit_max(&value);
  res.ret = ret1 + ret2;
  if (res.ret != 0) ROS_ERROR("[ArmContr] flixe get_vel_limit_max failed %d %d", ret1, ret2);
  res.data = value;
  return true;
}

bool gripper_acc_set(ut_msg::SetFloat32::Request &req, ut_msg::SetFloat32::Response &res) {
  CHECK_FLXIE_CONNECT()

  float data = req.data;
  if (req.data < 0) {
    res.ret = -3;
    return true;
  }
  if (req.data > 300) data = 300;

  int ret = fixie->set_pos_adrc_param(3, data);
  if (ret != 0) ROS_ERROR("[ArmContr] flixe set_pos_adrc_param failed %d", ret);
  res.ret = ret;
  return true;
}

bool gripper_acc_get(ut_msg::GetFloat32::Request &req, ut_msg::GetFloat32::Response &res) {
  CHECK_FLXIE_CONNECT()

  float value;
  int ret = fixie->get_pos_adrc_param(3, &value);
  if (ret != 0) ROS_ERROR("[ArmContr] flixe get_pos_adrc_param failed %d", ret);
  res.ret = ret;
  res.data = value;
  return true;
}

bool gripper_error_code(ut_msg::GetInt16::Request &req, ut_msg::GetInt16::Response &res) {
  CHECK_FLXIE_CONNECT()

  uint8_t value;
  int ret = fixie->get_error_code(&value);
  if (ret != 0) ROS_ERROR("[ArmContr] flixe get_error_code failed %d", ret);
  res.ret = ret;
  res.data = value;
  return true;
}

bool gripper_reset_err(ut_msg::GetInt16::Request &req, ut_msg::GetInt16::Response &res) {
  CHECK_FLXIE_CONNECT()

  int ret = fixie->reset_err();
  if (ret != 0) ROS_ERROR("[ArmContr] flixe reset_err failed %d", ret);
  res.ret = ret;
  res.data = 0;
  return true;
}

bool gripper_unlock(ut_msg::SetInt16::Request &req, ut_msg::SetInt16::Response &res) {
  CHECK_FLXIE_CONNECT()

  int data = req.data;
  int ret = fixie->set_unlock_function(data);
  if (ret != 0) ROS_ERROR("[ArmContr] flixe set_unlock_function failed %d", ret);
  res.ret = ret;
  return true;
}

//------------------------------------------------------------------------------------
//                                      MAIN
//------------------------------------------------------------------------------------
/**
 * API service for the Arm NTRO controller
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "utarm_api_server");
  ros::NodeHandle nh;

  if (nh.getParam("arm_ip", arm_ip)) {
    ROS_INFO("[ArmContr] Got param: %s", arm_ip.c_str());
  } else {
    ROS_ERROR("[ArmContr] Failed to get param 'arm_ip'");
    arm_ip = "";
  }
  if (arm_ip != "") {
    char *ip = new char[arm_ip.length() + 1];
    std::strcpy(ip, arm_ip.c_str());
    armapi = new UtraApiTcp(ip);
    uint8_t axis;
    int ret = armapi->get_axis(&axis);
    ROS_INFO("[ArmContr] get_axis %d, axis:%d", ret, axis);
    if (ret == -3) {
      ROS_ERROR("[ArmContr] can not connect the arm");
    }
  }

  ros::ServiceServer connect = nh.advertiseService("utsrv/connect", connect_api);
  ros::ServiceServer disconnect = nh.advertiseService("utsrv/disconnect", disconnect_api);
  ros::ServiceServer check_c = nh.advertiseService("utsrv/check_connect", check_c_api);
  ros::ServiceServer mservojoint = nh.advertiseService("utsrv/mv_servo_joint", mv_servo_joint);

  ros::ServiceServer statusset = nh.advertiseService("utsrv/status_set", status_set);
  ros::ServiceServer statusget = nh.advertiseService("utsrv/status_get", status_get);
  ros::ServiceServer modeset = nh.advertiseService("utsrv/mode_set", mode_set);
  ros::ServiceServer modeget = nh.advertiseService("utsrv/mode_get", mode_get);
  ros::ServiceServer enableset = nh.advertiseService("utsrv/enable_set", enable_set);
  ros::ServiceServer enableget = nh.advertiseService("utsrv/enable_get", enable_get);

  ros::ServiceServer setlimitangleenable = nh.advertiseService("utsrv/set_limit_angle_enable", set_limit_angle_enable);
  ros::ServiceServer setlimitgeometryenable = nh.advertiseService("utsrv/set_limit_geometry_enable", set_limit_geometry_enable);

  ros::ServiceServer grippermv = nh.advertiseService("utsrv/gripper_mv", gripper_mv);
  ros::ServiceServer gripperstateset = nh.advertiseService("utsrv/gripper_state_set", gripper_state_set);
  ros::ServiceServer gripperstateget = nh.advertiseService("utsrv/gripper_state_get", gripper_state_get);
  ros::ServiceServer grippervelset = nh.advertiseService("utsrv/gripper_vel_set", gripper_vel_set);
  ros::ServiceServer grippervelget = nh.advertiseService("utsrv/gripper_vel_get", gripper_vel_get);
  ros::ServiceServer gripperaccset = nh.advertiseService("utsrv/gripper_acc_set", gripper_acc_set);
  ros::ServiceServer gripperaccget = nh.advertiseService("utsrv/gripper_acc_get", gripper_acc_get);
  ros::ServiceServer gripperreseterr = nh.advertiseService("utsrv/gripper_reset_err", gripper_reset_err);
  ros::ServiceServer grippererrorcode = nh.advertiseService("utsrv/gripper_error_code", gripper_error_code);
  ros::ServiceServer gripperunlock = nh.advertiseService("utsrv/gripper_unlock", gripper_unlock);

  ros::ServiceServer geterrorcode = nh.advertiseService("utsrv/get_error_code", get_error_code);
  ros::ServiceServer getservomsg = nh.advertiseService("utsrv/get_servo_msg", get_servo_msg);

  ros::ServiceServer movetojointp2p = nh.advertiseService("utsrv/moveto_joint_p2p", moveto_joint_p2p);
  ros::ServiceServer movetocartesianline = nh.advertiseService("utsrv/moveto_cartesian_line", moveto_cartesian_line);
  ros::ServiceServer plansleep = nh.advertiseService("utsrv/plan_sleep", plan_sleep);
  ros::ServiceServer movetocartesianlineb = nh.advertiseService("utsrv/moveto_cartesian_lineb", moveto_cartesian_lineb);
  // ros::ServiceServer moveto_cartesian_circle = nh.advertiseService("utsrv/moveto_cartesian_circle",
  // moveto_cartesian_circle);
  ros::ServiceServer getjointactualpos = nh.advertiseService("utsrv/get_joint_actual_pos", get_joint_actual_pos);

  ROS_INFO("[ArmContr] ready for arm controller server.");
  ros::spin();
}
