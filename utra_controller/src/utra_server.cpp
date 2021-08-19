/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: johnson <johnson@umbratek.com>
 ============================================================================*/
#include "ros/ros.h"
#include "utra/utra_api_tcp.h"
#include "utra_controller/Api.h"

UtraApiTcp *utra = NULL;

constexpr unsigned int hash(const char *s, int off = 0) { return !s[off] ? 5381 : (hash(s, off + 1) * 33) ^ s[off]; }
bool check_connect(utra_controller::Api::Response &res) {
  if (utra == NULL) {
    res.rets.push_back("-1");
    res.rets.push_back("controller have not connect utra");
    return false;
  } else {
    return true;
  }
}
bool check_arg_count(utra_controller::Api::Request &req, utra_controller::Api::Response &res, int count) {
  if (req.args.size() < count) {
    res.rets.push_back("-1");
    char str[25];
    sprintf(str, "args's count must, %d", count);
    res.rets.push_back(str);
    return false;
  } else {
    return true;
  }
}
void connect(utra_controller::Api::Request &req, utra_controller::Api::Response &res) {
  if (utra != NULL) {
    res.rets.push_back("0");
    res.rets.push_back("server have connected adra");
    return;
  }
  if (check_arg_count(req, res, 1) == false) return;

  char *ip = new char[req.args[0].length() + 1];
  std::strcpy(ip, req.args[0].c_str());
  utra = new UtraApiTcp(ip);
  uint8_t axis;
  int ret = utra->get_axis(&axis);
  if (ret == -3) {
    res.rets.push_back("-3");
    res.rets.push_back("can not connect the utra");
    return;
  } else {
    res.rets.push_back("0");
    res.rets.push_back("connect seccess");
  }
}

void disconnect(utra_controller::Api::Response &res) {
  if (utra != NULL) {
    delete utra;
    utra = NULL;
    res.rets.push_back("0");
    res.rets.push_back("disconnect seccess");
    return;
  }
  res.rets.push_back("-1");
  res.rets.push_back("server have not connect utra");
}

void move_joints(utra_controller::Api::Request &req, utra_controller::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 8) == false) return;

  float j1 = std::stof(req.args[0].c_str());
  float j2 = std::stof(req.args[1].c_str());
  float j3 = std::stof(req.args[2].c_str());
  float j4 = std::stof(req.args[3].c_str());
  float j5 = std::stof(req.args[4].c_str());
  float j6 = std::stof(req.args[5].c_str());
  float speed = std::stof(req.args[6].c_str());
  float acc = std::stof(req.args[7].c_str());
  float joint[6] = {j1, j2, j3, j4, j5, j6};
  ROS_INFO("move_joints");
  int ret = utra->moveto_joint_p2p(joint, speed, acc, 0);
  res.rets.push_back(std::to_string(ret));
}
void move_line(utra_controller::Api::Request &req, utra_controller::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 8) == false) return;

  float x = std::stof(req.args[0].c_str());
  float y = std::stof(req.args[1].c_str());
  float z = std::stof(req.args[2].c_str());
  float roll = std::stof(req.args[3].c_str());
  float pitch = std::stof(req.args[4].c_str());
  float yaw = std::stof(req.args[5].c_str());
  float speed = std::stof(req.args[6].c_str());
  float acc = std::stof(req.args[7].c_str());
  float pos[6] = {x, y, z, roll, pitch, yaw};
  int ret = utra->moveto_cartesian_line(pos, speed, acc, 0);
  res.rets.push_back(std::to_string(ret));
}

void get_motion_enable(utra_controller::Api::Request &req, utra_controller::Api::Response &res) {
  if (check_connect(res) == false) return;
  int able = -1;
  int ret = utra->get_motion_enable(&able);
  if (ret == -3) {
    res.rets.push_back("-3");
    res.rets.push_back("can not connect the utra");
    return;
  }
  if (able == 63) {
    res.rets.push_back(std::to_string(ret));
    res.rets.push_back("motion is enable");
  } else {
    res.rets.push_back(std::to_string(ret));
    res.rets.push_back("motion is disable");
  }
  return;
}
void set_motion_enable(utra_controller::Api::Request &req, utra_controller::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 1) == false) return;
  int en = std::stoi(req.args[0].c_str());
  int ret = utra->set_motion_enable(100, en);
  res.rets.push_back(std::to_string(ret));
}

void get_motion_status(utra_controller::Api::Request &req, utra_controller::Api::Response &res) {
  if (check_connect(res) == false) return;
  uint8_t status = -1;
  int ret = utra->get_motion_status(&status);
  if (ret == -3) {
    res.rets.push_back("-3");
    res.rets.push_back("can not connect the utra");
    return;
  }
  // #set 4 to stop motion ,it will clear all move cammand
  // #set 3 to pause motion,it can play again
  // # 2 is sleep
  // # 1 is move
  // # 0 is NORMAL
  res.rets.push_back(std::to_string(ret));
  switch (status) {
    case 0:
      res.rets.push_back("current status is normal");
      break;
    case 1:
      res.rets.push_back("current status is moving");
      break;
    case 2:
      res.rets.push_back("current status is sleeping");
      break;
    case 3:
      res.rets.push_back("current status is pause");
      break;
    case 4:
      res.rets.push_back("current status is stop");
      break;
  }
  return;
}

void set_motion_status(utra_controller::Api::Request &req, utra_controller::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 1) == false) return;
  int status = std::stoi(req.args[0].c_str());
  int ret = utra->set_motion_status(status);
  res.rets.push_back(std::to_string(ret));
}

bool api(utra_controller::Api::Request &req, utra_controller::Api::Response &res) {
  ROS_INFO("request: api_name=%s", req.api_name.c_str());
  switch (hash(req.api_name.c_str())) {
    case hash("connect"):
      connect(req, res);
      break;
    case hash("disconnect"):
      disconnect(res);
      break;
    case hash("get_motion_enable"):
      get_motion_enable(req, res);
      break;
    case hash("set_motion_enable"):
      set_motion_enable(req, res);
      break;
    case hash("get_motion_status"):
      get_motion_status(req, res);
      break;
    case hash("set_motion_status"):
      set_motion_status(req, res);
      break;
    case hash("move_joints"):
      move_joints(req, res);
      break;
    case hash("move_line"):
      move_line(req, res);
      break;
    default:
      break;
  }
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "utra_server");
  ros::NodeHandle nh;

  std::string utra_ip;
  if (nh.getParam("utra_ip", utra_ip)) {
    ROS_INFO("Got param: %s", utra_ip.c_str());
  } else {
    ROS_ERROR("Failed to get param 'utra_ip'");
    return -1;
  }
  char *ip = new char[utra_ip.length() + 1];
  std::strcpy(ip, utra_ip.c_str());
  utra = new UtraApiTcp(ip);
  uint8_t axis;
  int ret = utra->get_axis(&axis);
  ROS_INFO("ret %d", ret);
  if (ret == -3) {
    ROS_ERROR("can not connect the utra");
    return -1;
  }

  ros::ServiceServer service = nh.advertiseService("utra_server", api);

  ROS_INFO("Ready for adra server.");
  ros::spin();
}