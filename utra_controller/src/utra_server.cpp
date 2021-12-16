/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: johnson <johnson@umbratek.com>
 ============================================================================*/
#include "ros/ros.h"
#include "utra/utra_api_tcp.h"
#include "utra/utra_flxie_api.h"

#include "utra_msg/Connect.h"
#include "utra_msg/Disconnect.h"
#include "utra_msg/Checkconnect.h"
#include "utra_msg/Mservojoint.h"
#include "utra_msg/Grippermv.h"
#include "utra_msg/EnableSet.h"
#include "utra_msg/GripperStateGet.h"
#include "utra_msg/GripperStateSet.h"
#include "utra_msg/GetInt16.h"
#include "utra_msg/SetInt16.h"
#include "utra_msg/GetFloat32.h"
#include "utra_msg/SetFloat32.h"
#include "utra_msg/GetUInt16A.h"

UtraApiTcp *utra = NULL;
UtraFlxiE2Api *fixi = NULL;


std::string utra_ip = "";

constexpr unsigned int hash(const char *s, int off = 0) { return !s[off] ? 5381 : (hash(s, off + 1) * 33) ^ s[off]; }


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

// void move_joints(utra_msg::Api::Request &req, utra_msg::Api::Response &res) {
//   if (check_connect(res) == false) return;
//   if (check_arg_count(req, res, 8) == false) return;

//   float j1 = std::stof(req.args[0].c_str());
//   float j2 = std::stof(req.args[1].c_str());
//   float j3 = std::stof(req.args[2].c_str());
//   float j4 = std::stof(req.args[3].c_str());
//   float j5 = std::stof(req.args[4].c_str());
//   float j6 = std::stof(req.args[5].c_str());
//   float speed = std::stof(req.args[6].c_str());
//   float acc = std::stof(req.args[7].c_str());
//   float joint[6] = {j1, j2, j3, j4, j5, j6};
//   ROS_INFO("move_joints");
//   int ret = utra->moveto_joint_p2p(joint, speed, acc, 0);
//   res.rets.push_back(std::to_string(ret));
// }
// void move_line(utra_msg::Api::Request &req, utra_msg::Api::Response &res) {
//   if (check_connect(res) == false) return;
//   if (check_arg_count(req, res, 8) == false) return;

//   float x = std::stof(req.args[0].c_str());
//   float y = std::stof(req.args[1].c_str());
//   float z = std::stof(req.args[2].c_str());
//   float roll = std::stof(req.args[3].c_str());
//   float pitch = std::stof(req.args[4].c_str());
//   float yaw = std::stof(req.args[5].c_str());
//   float speed = std::stof(req.args[6].c_str());
//   float acc = std::stof(req.args[7].c_str());
//   float pos[6] = {x, y, z, roll, pitch, yaw};
//   int ret = utra->moveto_cartesian_line(pos, speed, acc, 0);
//   res.rets.push_back(std::to_string(ret));
// }


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
  int CON = 3; //how many points with one time to send
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
    utra->plan_sleep(0.2); // set utra sleep 0.1s to wait for command 
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


bool gripper_state_set(utra_msg::GripperStateSet::Request &req, utra_msg::GripperStateSet::Response &res) {
  ROS_INFO("state value: %d",req.state);
  if(utra == NULL){
    res.ret=-3;
    res.message="server have not connected utra";
    return true;
  }
  if(req.state == 1){
    if(fixi == NULL){
      fixi = new UtraFlxiE2Api(utra, 101);
    }
    int ret = fixi->set_motion_mode(1);
    ROS_INFO("set_motion_mode: %d\n", ret);
    ret = fixi->set_motion_enable(1);
    printf("set_motion_enable: %d\n", ret);
    float value;
    ret = fixi->get_pos_target(&value);
    res.pos = value;
    res.message="OK";
    return true;
  }
  if(req.state == 0){
    if(fixi == NULL){
      res.ret=0;
      res.message="server have not connected gripper";
      return true;
    }else{
      int ret = fixi->set_motion_mode(0);
      ROS_INFO("set_motion_mode: %d\n", ret);
      ret = fixi->set_motion_enable(0);
      printf("set_motion_enable: %d\n", ret);
      res.ret=ret;
      res.message="OK";
      return true;
    }
  }
  return true;
}
bool gripper_state_get(utra_msg::GripperStateGet::Request &req, utra_msg::GripperStateGet::Response &res) {
    if(fixi == NULL){
      res.ret=-3;
      res.enable=0;
      res.message="server have not connected gripper";
      return true;
    }else{
      float value;
      int ret = fixi->get_pos_target(&value);
      if(ret != -3){
        ros::NodeHandle n;
        float pos = (value/100)-0.4;
        n.setParam("utra_gripper_pos",pos);
      }
      uint8_t enable;
      ret = fixi->get_motion_enable(&enable);
      res.ret=ret;
      res.enable=enable;
      res.pos=value;
      res.message="OK";
      return true;
    }
  
  return true;
}
bool gripper_mv(utra_msg::Grippermv::Request &req, utra_msg::Grippermv::Response &res) {
  if(utra == NULL || fixi == NULL){
    res.ret=-3;
    res.message="server have not connected utra or gripper";
    return true;
  }
  int ret = fixi->set_pos_target(req.pos);
  if(ret != -3){
    ros::NodeHandle n;
    float pos = (req.pos/100)-0.4;
    n.setParam("utra_gripper_pos",pos);
  }
  res.ret=ret;
  res.message="OK";
  return true;
}

bool status_set(utra_msg::SetInt16::Request &req, utra_msg::SetInt16::Response &res) {
  if(utra == NULL)
  {
    res.ret=-3;
  }
  int ret = utra->set_motion_status(req.data);
  res.ret=ret;
  return true;
}
bool status_get(utra_msg::GetInt16::Request &req, utra_msg::GetInt16::Response &res) {
  if(utra == NULL)
  {
    res.ret=-3;
  }
  uint8_t status;
  int ret = utra->get_motion_status(&status);
  res.ret=ret;
  res.data = status;
  return true;
}
bool mode_set(utra_msg::SetInt16::Request &req, utra_msg::SetInt16::Response &res) {
  if(utra == NULL)
  {
    res.ret=-3;
  }
  int ret = utra->set_motion_mode(req.data);
  res.ret=ret;
  return true;
}
bool mode_get(utra_msg::GetInt16::Request &req, utra_msg::GetInt16::Response &res) {
  if(utra == NULL)
  {
    res.ret=-3;
  }
  uint8_t mode;
  int ret = utra->get_motion_mode(&mode);
  res.ret=ret;
  res.data = mode;
  return true;
}
bool enable_set(utra_msg::EnableSet::Request &req, utra_msg::EnableSet::Response &res) {
  if(utra == NULL)
  {
    res.ret=-3;
  }
  int ret = utra->set_motion_enable(req.axis,req.enable);
  res.ret=ret;
  return true;
}
bool enable_get(utra_msg::GetInt16::Request &req, utra_msg::GetInt16::Response &res) {
  if(utra == NULL)
  {
    res.ret=-3;
  }
  int enable;
  int ret = utra->get_motion_enable(&enable);
  res.ret=ret;
  res.data = enable;
  return true;
}
bool gripper_vel_set(utra_msg::SetFloat32::Request &req, utra_msg::SetFloat32::Response &res) {
  if(utra == NULL || fixi == NULL){
    res.ret=-3;
    return true;
  }
  int ret1 = fixi->set_vel_limit_min(-req.data,true);
  int ret2 = fixi->set_vel_limit_max(req.data,true);
  res.ret=ret1+ret2;
  return true;
}
bool gripper_vel_get(utra_msg::GetFloat32::Request &req, utra_msg::GetFloat32::Response &res) {
  if(utra == NULL || fixi == NULL){
    res.ret=-3;
    return true;
  }
  float value;
  int ret1 = fixi->get_vel_limit_min(&value);
  int ret2 = fixi->get_vel_limit_max(&value);
  res.ret=ret1+ret2;
  res.data = value;
  return true;
}

bool get_error_code(utra_msg::GetUInt16A::Request &req, utra_msg::GetUInt16A::Response &res) {
  if(utra == NULL){
    res.ret=-3;
    return true;
  }
  uint8_t array[24] = {0};
  int ret = utra->get_error_code(array);
  res.ret=ret;
  for (size_t j = 0; j < 24; j++)
    {
      res.data.push_back(array[j]);
    }
  return true;
}
bool get_servo_msg(utra_msg::GetUInt16A::Request &req, utra_msg::GetUInt16A::Response &res) {
  if(utra == NULL){
    res.ret=-3;
    return true;
  }
  uint8_t array[24] = {0};
  int ret = utra->get_servo_msg(array);
  res.ret=ret;
  for (size_t j = 0; j < 24; j++)
    {
      res.data.push_back(array[j]);
    }
  return true;
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
  
  ros::ServiceServer statusset = nh.advertiseService("utra/status_set", status_set);
  ros::ServiceServer statusget = nh.advertiseService("utra/status_get", status_get);
  ros::ServiceServer modeset = nh.advertiseService("utra/mode_set", mode_set);
  ros::ServiceServer modeget = nh.advertiseService("utra/mode_get", mode_get);
  ros::ServiceServer enableset = nh.advertiseService("utra/enable_set", enable_set);
  ros::ServiceServer enableget = nh.advertiseService("utra/enable_get", enable_get);

  ros::ServiceServer grippermv = nh.advertiseService("utra/gripper_mv", gripper_mv);
  ros::ServiceServer gripperstateset = nh.advertiseService("utra/gripper_state_set", gripper_state_set);
  ros::ServiceServer gripperstateget = nh.advertiseService("utra/gripper_state_get", gripper_state_get);
  ros::ServiceServer grippervelset = nh.advertiseService("utra/gripper_vel_set", gripper_vel_set);
  ros::ServiceServer grippervelget = nh.advertiseService("utra/gripper_vel_get", gripper_vel_get);

  ros::ServiceServer geterrorcode = nh.advertiseService("utra/get_error_code", get_error_code);
  ros::ServiceServer getservomsg = nh.advertiseService("utra/get_servo_msg", get_servo_msg);

  ROS_INFO("Ready for utra server.");
  ros::spin();
}