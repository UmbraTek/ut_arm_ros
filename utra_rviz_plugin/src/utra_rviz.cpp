#include "utra_rviz.h"
#include <stdio.h>
#include "ros/ros.h"
#include <QPainter>
#include <QLineEdit>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QDoubleValidator>

#include <geometry_msgs/Twist.h>
#include <QDebug>
#include "utra_msg/GripperStateGet.h"
#include "utra_msg/GripperStateSet.h"
#include "utra_msg/Grippermv.h"
#include "utra_msg/EnableSet.h"
#include "utra_msg/Checkconnect.h"
#include <utra_msg/RobotMsg.h>
#include "utra_msg/SetInt16.h"
#include "utra_msg/GetInt16.h"



namespace utra_rviz_space {
utra_rviz::utra_rviz(QWidget* parent)
    :rviz::Panel (parent)
{


    //初始化ui
    QVBoxLayout *mainlayout=new QVBoxLayout;
    ip_lable=new QLabel;
    ip_lable->setText("IP: ");
    ip_addrees=new QLabel;
    ip_addrees->setText("disconnect!!!");

    checkConnect=new QPushButton;
    checkConnect->setText("check");
    checkConnect->setStyleSheet("QPushButton#connectButton:pressed{background-color:rgb(239, 41, 41)}");

    QHBoxLayout *first=new QHBoxLayout;
    first->addWidget(ip_lable);
    first->addWidget(ip_addrees);
    first->addWidget(checkConnect);
    mainlayout->addLayout(first);

    utraStatus=new QLabel;
    utraStatus->setText("nomal");

    resume=new QPushButton;
    resume->setText("resume");
    resume->setStyleSheet("QPushButton#resume:pressed{background-color:rgb(239, 41, 41)}");
    resume->setVisible(false);

    utraEnable=new QCheckBox;
    utraEnable->setText("Enable");

    eStop=new QPushButton;
    eStop->setText("E-STOP");
    eStop->setStyleSheet("QPushButton#eStop:pressed{background-color:rgb(239, 41, 41)}");

    QGroupBox *utra_status_label=new QGroupBox("UTRA STATUS:");
    QHBoxLayout *second=new QHBoxLayout;
    second->addWidget(utraEnable);
    second->addWidget(resume);
    second->addWidget(utraStatus);
    second->addWidget(eStop);
    utra_status_label->setLayout(second);
    mainlayout->addWidget(utra_status_label);

    
    QGroupBox *gripperLabel=new QGroupBox("Gripper:");
    gripperEnable=new QCheckBox;
    gripperEnable->setText("Enable");
    gripperEdit=new QLineEdit;
    QDoubleValidator* vali = new QDoubleValidator;
    vali->setRange(0, 80,2);
    gripperEdit->setValidator(vali);
    gripperEdit->setPlaceholderText("0-80");
    gripperButton=new QPushButton;
    gripperButton->setText("set");
    gripperButton->setStyleSheet("QPushButton#gripperButton:pressed{background-color:rgb(239, 41, 41)}");

    QHBoxLayout *third=new QHBoxLayout;
    third->addWidget(gripperEnable);
    third->addWidget(gripperEdit);
    third->addWidget(gripperButton);
    gripperLabel->setLayout(third);
    mainlayout->addWidget(gripperLabel);


    setLayout(mainlayout);

    //绑定信号
    connect(gripperButton,SIGNAL(clicked()),this,SLOT(set_gripper_pos()));
    connect(gripperEnable,SIGNAL(clicked()),this,SLOT(enable_gripper()));
    connect(resume,SIGNAL(clicked()),this,SLOT(resumeState()));
    connect(utraEnable,SIGNAL(clicked()),this,SLOT(enable()));
    connect(eStop,SIGNAL(clicked()),this,SLOT(e_stop()));
    connect(checkConnect,SIGNAL(clicked()),this,SLOT(check_connect()));
    


    Grippermv_client = nh_.serviceClient<utra_msg::Grippermv>("utra/gripper_mv");
    Gripperstate_get = nh_.serviceClient<utra_msg::GripperStateGet>("utra/gripper_state_get");
    Gripperstate_set = nh_.serviceClient<utra_msg::GripperStateSet>("utra/gripper_state_set");
    status_set_client = nh_.serviceClient<utra_msg::StatusSet>("utra/status_set");
    mode_set_client = nh_.serviceClient<utra_msg::ModeSet>("utra/mode_set");
    enable_set_client = nh_.serviceClient<utra_msg::EnableSet>("utra/enable_set");
    checkconnect_client = nh_.serviceClient<utra_msg::Checkconnect>("utra/check_connect");

    utra_states_sub = nh_.subscribe("utra/states", 1000, &utra_rviz::statesCallback,this);
    refreshUI();
}
void utra_rviz::check_connect(){
    refreshUI();
}

void utra_rviz::refreshUI()
{
    utra_msg::Checkconnect srv1;
    if(checkconnect_client.call(srv1))
    {
        if(srv1.response.ret != -3){
            ip_addrees->setText(QString::fromStdString(srv1.response.ip_address));
        }
        else{
            ROS_INFO("can not connect the device");
        }
    }

    utra_msg::GripperStateGet srv;
    if(Gripperstate_get.call(srv))
    {
        ROS_INFO("ret %d, pos %f",srv.response.ret,srv.response.pos);
        if(srv.response.enable == 1){
            gripperEnable->setCheckState(Qt::Checked);
            gripperEdit->setText(QString().setNum(srv.response.pos));
        }else{
            gripperEnable->setCheckState(Qt::Unchecked);
            gripperEdit->setPlaceholderText("0-80");
        }
    }
}

void utra_rviz::statesCallback(const utra_msg::RobotMsg& msg)
{
    // ROS_INFO("motion_status %d",msg.motion_status);
    switch (msg.motion_status)
    {
    case 0:
        utraStatus->setText("nomal");
        resume->setVisible(false);
        break;
    case 1:
        utraStatus->setText("moveing");
        resume->setVisible(false);
        break;
    case 2:
        utraStatus->setText("sleep");
        resume->setVisible(false);
        break;
    case 3:
        utraStatus->setText("pause");
        resume->setVisible(false);
        break;
    case 4:
        utraStatus->setText("stop");
        resume->setVisible(true);
        break;
    
    default:
        break;
    }
    // ROS_INFO("mt_able %d",msg.mt_able);
    int enable = 0;
    if(msg.axis == 4){
        enable = 15;
    }
    else if (msg.axis == 6)
    {
        enable = 63;
    }
    if(msg.mt_able == enable){
        utraEnable->setCheckState(Qt::Checked);
    }else{
        utraEnable->setCheckState(Qt::Unchecked);
    }
    
}

void utra_rviz::set_gripper_pos()
{
    float value=gripperEdit->text().toFloat();
    ROS_INFO("value %f",value);
    utra_msg::Grippermv srv;
    srv.request.pos = value;
    Grippermv_client.call(srv);
}

void utra_rviz::enable_gripper(){
    Qt::CheckState state= gripperEnable->checkState();
    utra_msg::GripperStateSet srv;
    switch (state)
    {
    case Qt::Unchecked:
        srv.request.state = 0;
        Gripperstate_set.call(srv);
        break;
    case Qt::Checked:
        srv.request.state = 1;
        Gripperstate_set.call(srv);
        break;
    }
}
void utra_rviz::resumeState(){
    utra_msg::SetInt16 srv1;
    srv1.request.data = 0;
    if(mode_set_client.call(srv1))
    {
        ROS_INFO("mode_set ret %d,",srv1.response.ret);
    }

    utra_msg::SetInt16 srv;
    srv.request.data = 0;
    if(status_set_client.call(srv))
    {
        ROS_INFO("status_set ret %d,",srv.response.ret);
    }
}
void utra_rviz::enable(){
    Qt::CheckState state= utraEnable->checkState();
    utra_msg::EnableSet srv;
    switch (state)
    {
    case Qt::Unchecked:
        srv.request.axis = 100;
        srv.request.enable = 0;
        enable_set_client.call(srv);
        break;
    case Qt::Checked:
        srv.request.axis = 100;
        srv.request.enable = 1;
        enable_set_client.call(srv);
        break;
    }
}
void utra_rviz::e_stop(){
    utra_msg::SetInt16 srv1;
    srv1.request.data = 0;
    if(mode_set_client.call(srv1))
    {
        ROS_INFO("mode_set ret %d,",srv1.response.ret);
    }

    utra_msg::SetInt16 srv;
    srv.request.data = 4;
    if(status_set_client.call(srv))
    {
        ROS_INFO("status_set ret %d,",srv.response.ret);
    }
}
// 重载父类的功能
void utra_rviz::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
//   config.mapSetValue( "Topic", output_topic_ );
}

//// 重载父类的功能，加载配置数据
//void utra_rviz::load( const rviz::Config& config )
//{
//  rviz::Panel::load( config );
//  QString topic;
//  if( config.mapGetString( "Topic", &topic ))
//  {
//    output_topic_editor_->setText( topic );
//    updateTopic();
//  }
//}


}

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(utra_rviz_space::utra_rviz,rviz::Panel )
// END_TUTORIAL