#include <ros/ros.h>
#include <stdlib.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <rqt_gui_cpp/plugin.h>
//#include <rqt_gui_cpp/plugin.h>
//#include <my_namespace/ui_my_plugin.h>
#include <QWidget>
#include <rqt_gui_cpp/plugin.h>

std_msgs::Float64 msg;


std_msgs::Float64 conv1;
ros::Publisher temperatureC;
ros::Publisher temperatureF;
ros::Publisher vol_pub;
double delay = 0.2;


std_msgs::Float64 toF(std_msgs::Float64 data){
    conv1.data = ((data.data*(9.0/5.0))+32.0);
    return conv1;
}

void chatterCallback(const std_msgs::Float64 msg1)
{
    //ROS_INFO("I heard: [%s]", msg);
    //ros::Duration(delay).sleep();
    msg.data = msg1.data;
    //	ROS_INFO("LOOP TEMP: %f ",msg.data);
    //	TC.data = msg1.data;
    //	TF.data = toF(msg1).data;

}



int main( int argc, char** argv )
{


    ros::init(argc, argv, "sample");
    ROS_INFO("Starting ROS");
    ros::NodeHandle n;
    ros::Rate r(100);
    ROS_INFO("Starting Publishers");
    vol_pub = n.advertise<std_msgs::Float64>("datavol_1", 2);


    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    temperatureC = n.advertise<std_msgs::Float64>("Temperature/C", 2);
    temperatureF = n.advertise<std_msgs::Float64>("Temperature/F", 2);
    ROS_INFO("Publishers Started");
    ROS_INFO("Starting Subscribers");
    ros::Subscriber sub = n.subscribe<std_msgs::Float64>("/imu/temperature", 1000, chatterCallback);
    ROS_INFO("Subscribers Started");
    //ROS_INFO("LOOP2 %f ",msg.data);
    //vol_pub.publish(msg);
    //ros::spin();
    ROS_INFO("Starting Loop");
    while(ros::ok()){



        std_msgs::Float64 TC;
        std_msgs::Float64 TF;
        TC.data = msg.data;
        TF.data = toF(msg).data;
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << 1;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        //	ROS_INFO("LOOP C: %f ",TC.data);
        //	ROS_INFO("LOOP F: %f ",TF.data);

        //vol_pub.publish(msg);
        while ( temperatureC.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the markers!!!! v3");
            //	sleep(0.1);
        }
        temperatureC.publish(TC);
        temperatureF.publish(TF);
        chatter_pub.publish(msg);
        ros::spinOnce();
        r.sleep();
    }
}
