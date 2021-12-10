/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: johnson <johnson@umbratek.com>
 ============================================================================*/
#include "ros/ros.h"
#include "utra/utra_api_tcp.h"

#include "utra_msg/Api.h"
#include "utra_msg/Connect.h"
#include "utra_msg/Disconnect.h"
#include "utra_msg/Checkconnect.h"
#include "utra_msg/Mservojoint.h"

UtraApiTcp *utra = NULL;
std::string utra_ip = "";

constexpr unsigned int hash(const char *s, int off = 0) { return !s[off] ? 5381 : (hash(s, off + 1) * 33) ^ s[off]; }
bool check_connect(utra_msg::Api::Response &res) {
  if (utra == NULL) {
    res.rets.push_back("-1");
    res.rets.push_back("controller have not connect utra");
    return false;
  } else {
    return true;
  }
}
bool check_arg_count(utra_msg::Api::Request &req, utra_msg::Api::Response &res, int count) {
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
bool connect_api(utra_msg::Connect::Request &req, utra_msg::Connect::Response &res) {
  if (utra != NULL) {
    res.ret=0;
    res.message="server have connected utra";
    return true;
  }
  utra_ip = req.ip_address;
  char *ip = new char[req.ip_address.length() + 1];
  std::strcpy(ip, req.ip_address.c_str());
  utra = new UtraApiTcp(ip);
  uint8_t axis;
  int ret = utra->get_axis(&axis);
  res.ret=ret;
  if (ret == -3) {
    res.message="can not connect the utra";
  } else {
    res.message="connect seccess";
  }
  return true;
}

bool disconnect_api(utra_msg::Disconnect::Request &req, utra_msg::Disconnect::Response &res) {
  if(utra == NULL){
    res.ret=0;
    res.message="server have not connected utra";
    return true;
  }
  res.ret=0;
  res.message="utra have disconnected";
  delete utra;
  utra = NULL;
  return true;
}

bool check_c_api(utra_msg::Checkconnect::Request &req, utra_msg::Checkconnect::Response &res) {
  if(utra == NULL){
    res.ret=-3;
    res.ip_address = "";
    res.message="server have not connected utra";
    return true;
  }
  uint8_t axis;
  int ret = utra->get_axis(&axis);
  res.ip_address = utra_ip;
  res.ret=ret;
  if (ret == -3) {
    res.message="can not connect the utra";
  } else {
    res.message="connect seccess";
  }
  return true;
}

void move_joints(utra_msg::Api::Request &req, utra_msg::Api::Response &res) {
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
void move_line(utra_msg::Api::Request &req, utra_msg::Api::Response &res) {
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

void get_motion_enable(utra_msg::Api::Request &req, utra_msg::Api::Response &res) {
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
void set_motion_enable(utra_msg::Api::Request &req, utra_msg::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 1) == false) return;
  int en = std::stoi(req.args[0].c_str());
  int ret = utra->set_motion_enable(100, en);
  res.rets.push_back(std::to_string(ret));
}

void get_motion_status(utra_msg::Api::Request &req, utra_msg::Api::Response &res) {
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

void print_joints(float* frames,int con)
{
  for (size_t j = 0; j < con; j++)
  { 
    ROS_INFO("%f %f %f %f %f %f",frames[j*6],frames[j*6+1],frames[j*6+2],frames[j*6+3],frames[j*6+4],frames[j*6+5]);
  }
}

bool mv_servo_joint(utra_msg::Mservojoint::Request &req, utra_msg::Mservojoint::Response &res) {
  if(utra == NULL){
    res.ret=-3;
    res.message="server have not connected utra";
    return true;
  }
  int axiz = req.axiz;
  int CON = 3;
  if(axiz == 6){
    int frames_num = req.num;
    ROS_INFO("mv_servo_joint %d %f",frames_num,req.time);
    
    float frames[CON*6] = {0};
    float mvtime[CON] = {0};
    for (size_t i = 0; i < CON; i++)
    {
      mvtime[i] = req.time;
    }
    

    int have_CON_s = frames_num/CON;
    int have_CON_m = frames_num%CON;

    int req_frames_index = 0;
    int ret = 0;
  //    for (size_t j = 0; j < frames_num; j++)
  // { 
  //   ROS_INFO("%f %f %f %f %f %f",req.frames[j*6],req.frames[j*6+1],req.frames[j*6+2],req.frames[j*6+3],frames[j*6+4],req.frames[j*6+5]);
  // }
    //for run have_CON_s time send
    for (size_t i = 0; i < have_CON_s; i++)
    {
      for (size_t j = 0; j < CON*6; j++)
      { 
        frames[j] = req.frames[req_frames_index+j];
      }
      print_joints(frames,CON);
      req_frames_index = req_frames_index + CON*6;
      ret = utra->moveto_servo_joint(CON, frames, mvtime);
      ROS_INFO("req_frames_index %d ret %d",req_frames_index,ret);
    }
    
    //last time send
    for (size_t i = 0; i < have_CON_m*6; i++)
    { 
      frames[i] = req.frames[req_frames_index+i];
    }
    print_joints(frames,have_CON_m);
    ret = utra->moveto_servo_joint(have_CON_m, frames, mvtime);

    ROS_INFO("ret %d",ret);
    res.ret=ret;
    res.message="send cmd success";
    return true;
  }
  return true;
}

void set_motion_status(utra_msg::Api::Request &req, utra_msg::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 1) == false) return;
  int status = std::stoi(req.args[0].c_str());
  int ret = utra->set_motion_status(status);
  res.rets.push_back(std::to_string(ret));
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "utra_server");
  ros::NodeHandle nh;

  
  if (nh.getParam("utra_ip", utra_ip)) {
    ROS_INFO("Got param: %s", utra_ip.c_str());
  } else {
    ROS_ERROR("Failed to get param 'utra_ip'");
    utra_ip="";
    // return -1;
  }
  if(utra_ip != ""){
    char *ip = new char[utra_ip.length() + 1];
    std::strcpy(ip, utra_ip.c_str());
    utra = new UtraApiTcp(ip);
    uint8_t axis;
    int ret = utra->get_axis(&axis);
    ROS_INFO("ret %d", ret);
    if (ret == -3) {
      ROS_ERROR("can not connect the utra");
      // return -1;
    }
  }

  ros::ServiceServer connect = nh.advertiseService("utra/connect", connect_api);
  ros::ServiceServer disconnect = nh.advertiseService("utra/disconnect", disconnect_api);
  ros::ServiceServer check_c = nh.advertiseService("utra/check_connect", check_c_api);
  ros::ServiceServer mservojoint = nh.advertiseService("utra/mv_servo_joint", mv_servo_joint);

  ROS_INFO("Ready for utra server.");
  ros::spin();
}