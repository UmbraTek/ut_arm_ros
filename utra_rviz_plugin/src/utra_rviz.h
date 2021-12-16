#ifndef UTRA_RVIZ_H
#define UTRA_RVIZ_H

#include <stdio.h>

//所需要包含的头文件
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin基类的头文件


#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>
#include <geometry_msgs/Twist.h>
#include <QDebug>
#include <QCheckBox>
#include <QSlider>
#include <utra_msg/RobotMsg.h>
namespace utra_rviz_space {
class utra_rviz:public rviz::Panel
{
    Q_OBJECT
public:
    // 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
    utra_rviz(QWidget* parent=0);
    // 重载rviz::Panel积累中的函数，用于保存、加载配置文件中的数据，在我们这个plugin中，数据就是topic的名称
//    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;
    void move_base(char k,float speed_linear,float speed_trun);
public Q_SLOTS:
    void set_gripper_pos();
    void enable_gripper();
    void resumeState();
    void enable();
    void e_stop();
    void check_connect();
    void set_gripper_vel();
    // 内部变量.
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

    QLineEdit *gripper_vel;
    QPushButton *gripper_velButton;

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

    // 当前保存的线速度和角速度值
    float linear_velocity_;
    float angular_velocity_;

    void refreshUI();
    void statesCallback(const utra_msg::RobotMsg &msg);
    
};
}

#endif // UTRA_RVIZ_H