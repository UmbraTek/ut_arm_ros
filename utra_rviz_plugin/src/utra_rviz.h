#ifndef UTRA_RVIZ_H
#define UTRA_RVIZ_H

#include <stdio.h>

//所需要包含的头文件
#include <ros/console.h>
#include <ros/ros.h>
#include <rviz/panel.h>  //plugin基类的头文件

#include <geometry_msgs/Twist.h>
#include <utra_msg/RobotMsg.h>
#include <QCheckBox>
#include <QDebug>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
namespace utra_rviz_space {
class utra_rviz : public rviz::Panel {
  Q_OBJECT
 public:
  // Constructor, in the class will use the QWidget instance to implement the GUI interface, here initialized to 0
  utra_rviz(QWidget* parent = 0);
  // Override the rviz::Panel accumulation function to save and load the data from the configuration file.In our case,
  // the data is the name of the topic
  //    virtual void load( const rviz::Config& config );
  virtual void save(rviz::Config config) const;
  void move_base(char k, float speed_linear, float speed_trun);
 public Q_SLOTS:
  void set_gripper_pos();
  void enable_gripper();
  void resumeState();
  void enable();
  void e_stop();
  void check_connect();
  void set_gripper_vel();

 protected:
  QLabel* ip_lable;
  QLabel* ip_addrees;
  QPushButton* checkConnect;

  QLabel* utraStatus;
  QCheckBox* utraEnable;
  QPushButton* eStop;
  QPushButton* resume;

  QLabel* gripperLabel;
  QCheckBox* gripperEnable;
  QLineEdit* gripperEdit;
  QPushButton* gripperButton;

  QLineEdit* gripper_vel;
  QPushButton* gripper_velButton;

  QSlider* yaw_slider;
  QSlider* linera_slider;
  // ROS的publisher，用来发布速度topic
  // ros::Publisher velocity_publisher_;
  // QString output_topic_="cmd_vel";

  // The ROS node handle.
  ros::NodeHandle nh_;
  ros::ServiceClient Grippermv_client;
  ros::ServiceClient Gripperstate_get;
  ros::ServiceClient Gripperstate_set;
  ros::ServiceClient Grippervel_get;
  ros::ServiceClient Grippervel_set;
  ros::Subscriber utra_states_sub;

  ros::ServiceClient status_set_client;
  ros::ServiceClient mode_set_client;
  ros::ServiceClient enable_set_client;
  ros::ServiceClient checkconnect_client;

  // Currently saved linear and angular velocity values
  float linear_velocity_;
  float angular_velocity_;

  void refreshUI();
  void statesCallback(const utra_msg::RobotMsg& msg);
};
}  // namespace utra_rviz_space

#endif  // UTRA_RVIZ_H