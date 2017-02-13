/* 
 * Daniel Delago   1/27/2016 
 * Description: Creates node "joystick" that takes values from the joystick 
 * and publishes the edited values to multiple topics.
 * 
 * Node: joystick
 * Subscriptions: joystick_raw
 * Publishing Topics: joystick_topic, IK TOPIC  
*/
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>

class Controls
{
    public: Controls();
    private:void CallBack(const sensor_msgs::Joy::ConstPtr& joy); 
	    void joystick_output(const sensor_msgs::Joy::ConstPtr& joy);		//Add more functions as needed 
    ros::NodeHandle n;									
    ros::Subscriber joystick_sub;
    ros::Publisher joystick_pub; 					  		//Add more publishers as needed
};

Controls::Controls()
{      
    joystick_sub = n.subscribe("joystick_raw",100, &Controls::CallBack, this); 
    joystick_pub = n.advertise<std_msgs::Float32MultiArray>("joystick_topic",100);	//Publish to topic 
    //pubs....
}

//CallBack Function
void Controls::CallBack(const sensor_msgs::Joy::ConstPtr& joy)				//Add all function calls in here (for controller)
{
    Controls::joystick_output(joy);
}

//Joystick Functions_______________________________________________________
void Controls::joystick_output(const sensor_msgs::Joy::ConstPtr& joy)			//Function Description
{

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "joystick");  						//Create node named "joystick"
    Controls controls;   

    ros::spin();
}



