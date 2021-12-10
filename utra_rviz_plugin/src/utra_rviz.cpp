#include "utra_rviz.h"
#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <geometry_msgs/Twist.h>
#include <QDebug>

namespace utra_rviz_space {
utra_rviz::utra_rviz(QWidget* parent)
    :rviz::Panel (parent)
{


    //初始化ui
QVBoxLayout *mainlayout=new QVBoxLayout;
ip_addrees=new QLineEdit;
ip_addrees->setPlaceholderText("please input ip address");

connectButton=new QPushButton;
connectButton->setText("connect");
connectButton->setStyleSheet("QPushButton#connectButton:pressed{background-color:rgb(239, 41, 41)}");

QHBoxLayout *first=new QHBoxLayout;
first->addWidget(ip_addrees);
first->addWidget(connectButton);
mainlayout->addLayout(first);

utraStatus=new QLabel;
utraStatus->setText("nomal");

resume=new QPushButton;
resume->setText("resume");
resume->setStyleSheet("QPushButton#resume:pressed{background-color:rgb(239, 41, 41)}");

utraEnable=new QCheckBox;
utraEnable->setText("Enable");

eStop=new QPushButton;
eStop->setText("E-STOP");
eStop->setStyleSheet("QPushButton#eStop:pressed{background-color:rgb(239, 41, 41)}");

QHBoxLayout *second=new QHBoxLayout;
second->addWidget(resume);
second->addWidget(utraStatus);
second->addWidget(utraEnable);
second->addWidget(eStop);
mainlayout->addLayout(second);

gripperLabel=new QLabel;
gripperLabel->setText("Gripper");
gripperEdit=new QLineEdit;
gripperEdit->setPlaceholderText("please gripper value");
gripperButton=new QPushButton;
gripperButton->setText("set");
gripperButton->setStyleSheet("QPushButton#gripperButton:pressed{background-color:rgb(239, 41, 41)}");

QHBoxLayout *third=new QHBoxLayout;
third->addWidget(gripperLabel);
third->addWidget(gripperEdit);
third->addWidget(gripperButton);
mainlayout->addLayout(third);


 setLayout(mainlayout);

 //绑定信号

 //绑定速度控制按钮
//  connect(pushButton_i,SIGNAL(clicked()),this,SLOT(slot_utra_rviz()));
//  connect(pushButton_u,SIGNAL(clicked()),this,SLOT(slot_utra_rviz()));
//  connect(pushButton_o,SIGNAL(clicked()),this,SLOT(slot_utra_rviz()));
//  connect(pushButton_j,SIGNAL(clicked()),this,SLOT(slot_utra_rviz()));
//  connect(pushButton_l,SIGNAL(clicked()),this,SLOT(slot_utra_rviz()));
//  connect(pushButton_m,SIGNAL(clicked()),this,SLOT(slot_utra_rviz()));
//  connect(pushButton_back,SIGNAL(clicked()),this,SLOT(slot_utra_rviz()));
//  connect(pushButton_backr,SIGNAL(clicked()),this,SLOT(slot_utra_rviz()));


}

void utra_rviz::slot_utra_rviz()
{

    QPushButton* btn=qobject_cast<QPushButton*>(sender());
    char key=btn->text().toStdString()[0];

    switch (key) {
        case 'u':
        
        break;
    }

}
void utra_rviz::move_base(char k,float speed_linear,float speed_trun)
{
   
}


// 重载父类的功能
void utra_rviz::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
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