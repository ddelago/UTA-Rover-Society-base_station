/* 
 * Daniel Delago   11/25/2015 
 * Description: Creates node "controller" that takes values from the controller 
 * and publishes the edited values to multiple topics.
 * 
 * Node: controller
 * Subscriptions: controller_raw
 * Publishing Topics: turtle1/cmd_vel, cmd_vel
*/
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>

class Controls
{
    public: Controls();
    private:void CallBack(const sensor_msgs::Joy::ConstPtr& joy);		
	    void Drive(const sensor_msgs::Joy::ConstPtr& joy);				//Add more functions as needed 
    ros::NodeHandle n;
    ros::Subscriber controller_sub;
    ros::Publisher Joy2Turtlesim_pub, Drive_pub; 					//Add more publishers as needed
};

Controls::Controls()
{      
    controller_sub 	= n.subscribe("controller_raw",100, &Controls::CallBack, this); 
    //Joy2Turtlesim_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",100);		//Drive Testing
    Drive_pub 	= n.advertise<geometry_msgs::Twist>("cmd_vel",100);				//New Drive Testing
    //Val_pub  	= n.advertise<std_msgs::Float32MultiArray>("cmd_vel",100);		
    //pubs..
}

//CallBack Function
void Controls::CallBack(const sensor_msgs::Joy::ConstPtr& joy)				//Add all function calls in here
{
    Controls::Drive(joy);
}

//Controller Functions
void Controls::Drive(const sensor_msgs::Joy::ConstPtr& joy)				
{
    float x_axis = -1.0 * (joy->axes[0]);
    int x = joy->buttons[1];
    float forward = (joy->axes[4]);
    float reverse = (joy->axes[3]);
    float Drive; 
    forward = -1.0*((forward-1.0)/2.0);  // 2.0;
    reverse = (reverse-1.0)/2.0; // 2.0;
    Drive = forward + reverse;
    
    geometry_msgs::Twist msg;								
    msg.linear.x=(Drive);
    msg.angular.z=(x_axis);
    Drive_pub.publish(msg);
    
    //Publish Needed Controller Values to Apropriate Topics

    /*std_msgs::Float32MultiArray msg;  
    msg.data.push_back(Drive);
    msg.data.push_back(x_axis);
    msg.data.push_back(x);
    Val_pub.publish(msg);
    */

    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");  						//Create node named "controller"
    Controls controls;   

    ros::spin();
}



