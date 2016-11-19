#include "rqt_example_cpp/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>
#include <pthread.h>
#include <QTimer>

namespace rqt_example_cpp {

MyPlugin::MyPlugin()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
{
    // Constructor is called first before initPlugin function, needless to say.

    // give QObjects reasonable names
    setObjectName("MyPlugin");
}
int q = 0;
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}
int i = 0;
ros::Time begin;
Ui::MyPluginWidget uii;
pthread_t ROS_Thread;
ros::Publisher chatter_pub;
ros::Subscriber sub;
bool threadShutdown = false;
bool RosStatus;
int cam = 0;
ros::NodeHandle n;
void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
    int argc;
    char **argv1;
    ros::init(argc, argv1, "talker");
    ROS_INFO("Ros started");
    chatter_pub = n.advertise<std_msgs::Int8>("camera_select", 1000);
    sub = n.subscribe("chatter", 1000, chatterCallback);
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui_.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);
   
    connect(ui_.One, SIGNAL(clicked()), this,SLOT(Cam1()));
    connect(ui_.Two, SIGNAL(clicked()), this,SLOT(Cam2()));
    connect(ui_.Three, SIGNAL(clicked()), this,SLOT(Cam3()));
    // QTimer *timer = new QTimer(this);
    timer = new QTimer(this);
    begin = ros::Time::now();
    connect(timer, SIGNAL(timeout()), this, SLOT(updateROS()));
   
    timer->start(250);
}


void MyPlugin::updateROS(){
    ROS_INFO("PING ");

    std_msgs::Int8 msg;

    
    msg.data = cam;

    ROS_INFO("%d", msg.data);
    if(chatter_pub){
    chatter_pub.publish(msg);
    }
 std::stringstream ss;
    ss << (cam+1);
    ui_.rosStatus->setText(  QString::fromStdString(ss.str()));
    //  ros::spinOnce();

}

void MyPlugin::shutdownPlugin()
{
    ROS_INFO("Shutting Down");
    chatter_pub.shutdown();
    ROS_INFO("ShutDown complete");
    // TODO unregister all publishers here
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
    // TODO save intrinsic configuration, usually using:
    // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    // TODO restore intrinsic configuration, usually using:
    // v = instance_settings.value(k)
}



void MyPlugin::Cam1(){
   cam = 0;
}
void MyPlugin::Cam2(){
   cam = 1;
}
void MyPlugin::Cam3(){
   cam = 2;
}
/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

} // namespace
PLUGINLIB_DECLARE_CLASS(rqt_example_cpp, MyPlugin, rqt_example_cpp::MyPlugin, rqt_gui_cpp::Plugin)
