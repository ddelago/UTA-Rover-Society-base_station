#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_listener.h"
#define M_PI 3.14159265358979323846
#define RAD_TO_DEG 57.29578
#define magXmax 1180
#define magYmax 910
#define magZmax 1181
#define magXmin -1118
#define magYmin -1119
#define magZmin -889

#define DT 0.02         // [s/loop] loop period. 20ms
#define AA 0.97         // complementary filter constant

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
float kalmanFilterX(float accAngle, float gyroRate);
float kalmanFilterY(float accAngle, float gyroRate);

geometry_msgs::Quaternion orent;
geometry_msgs::Vector3 mag;
geometry_msgs::Vector3 acc;

float Q_angle  =  0.01;
float Q_gyro   =  0.0003;
float R_angle  =  0.01;
float x_bias = 0;
float z_bias = 0;
float y_bias = 0;
float XP_00 = 0, XP_01 = 0, XP_10 = 0, XP_11 = 0;
float YP_00 = 0, YP_01 = 0, YP_10 = 0, YP_11 = 0;
float ZP_00 = 0, ZP_01 = 0, ZP_10 = 0, ZP_11 = 0;


float KFangleX = 0.0;
float KFangleY = 0.0;

float gyroXangle = 0.0;
float gyroYangle = 0.0;
float gyroZangle = 0.0;
float AccYangle = 0.0;
float AccXangle = 0.0;
float CFangleX = 0.0;
float CFangleY = 0.0;

void get_oren_Values(const sensor_msgs::Imu::ConstPtr& msg)
{
   orent = msg->orientation;
}

void get_acc_raw_Values(const sensor_msgs::Imu::ConstPtr& msg)
{
   acc = msg->linear_acceleration;
}

void get_Mag_Values(const sensor_msgs::MagneticField::ConstPtr& msg)
{
    mag = msg->magnetic_field;
}
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub_mag = n.subscribe("imu/mag", 1000, get_Mag_Values);
  ros::Subscriber sub_acc_raw = n.subscribe("imu/data_raw", 1000, get_acc_raw_Values);
  ros::Subscriber sub_acc = n.subscribe("imu/data", 1000, get_oren_Values);
  ros::Publisher pub_cmd = n.advertise<std_msgs::String>("drive_cmd",1000);
  ros::Rate loop_rate(10);
  float accXnorm,accYnorm,pitch,roll,magXcomp,magYcomp,heading;
  while(ros::ok()){
	
	float accXnorm,accYnorm,pitch,roll,heading;
  	float accData[3];
 	float magData[3];

  	accData[0] = acc.x;
 	accData[1] = acc.y;
  	accData[2] = acc.z;


	magData[0] = mag.x*10000;
 	magData[1] = mag.y*10000;
  	magData[2] = mag.z*10000;

  	accXnorm= accData[0]/sqrt(accData[0]*accData[0]+ accData[1]*accData[1]+accData[2]*accData[2]);
  	accYnorm= accData[1]/sqrt(accData[0]*accData[0]+ accData[1]*accData[1]+accData[2]*accData[2]);
	pitch = asin(accXnorm);
	roll = -asin(accYnorm/cos(pitch));
  	ROS_INFO("X: %f Y: %f",pitch*(180/M_PI),roll*(180/M_PI));
	
	//ROS_INFO("MAG X: %f", mag.x*10000);
	//heading=180*atan2(magData[1],magData[0])/M_PI;
	magXcomp=magData[0]*cos(pitch)+magData[2]*sin(pitch);
        magYcomp= magData[0]*sin(roll)*sin(pitch)+magData[1]*cos(roll)-magData[2]*sin(roll)*cos(pitch);

	heading=180*atan2(magYcomp,magXcomp)/M_PI;

        if(heading < 0){
		heading +=360;
	}
	//ROS_INFO("HEADING: %f", heading);
	
	//sleep(1);
	ros::spinOnce();

  }	

  return 0;
}
