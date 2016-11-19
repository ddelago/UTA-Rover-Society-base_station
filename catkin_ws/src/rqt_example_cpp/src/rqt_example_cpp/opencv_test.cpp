#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"

cv::VideoCapture cap(0);
int cam = 0;
void chatterCallback(const std_msgs::Int8::ConstPtr& msg)
{
ROS_INFO("IN");
  ROS_INFO("I heard: [%d]", msg->data);
  if(cam != msg->data){
cam = msg->data;
  cap.open(cam);
	}
}




int main(int argc, char** argv)
{
 ROS_INFO("subbing");
  // Check if video source has been passed as a parameter
  

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

 ROS_INFO("subbing to");
  ros::Subscriber sub = nh.subscribe("camera_select", 1000, chatterCallback);
  // Convert the passed as command line parameter index for the video device to an integer
 ROS_INFO("subbingxxx");
  // Check if video device can be opened with the given index
  if(!cap.isOpened()) return 1;
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    cap >> frame;
    // Check if grabbed frame is actually full with some content
  if(!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
     pub.publish(msg);
      cv::waitKey(1);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
