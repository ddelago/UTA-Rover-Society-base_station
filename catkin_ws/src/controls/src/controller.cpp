/* 
 * Daniel Delago   11/25/2015 
 * Description: Creates node "controller" that takes values from the controller 
 * and publishes the edited values to multiple topics.
 * 
 * Node: controller
 * Subscriptions: controller_raw
 * Publishing Topics: cmd_vel
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
    ros::Publisher Joy2Turtlesim_pub, Drive_pub, arm_pub; 				//Add more publishers as needed
    
    
};

Controls::Controls()
{      
    controller_sub 	= n.subscribe("controller_raw",1, &Controls::CallBack, this); 	//Queue size of 1
    //Joy2Turtlesim_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",100);	//Drive Testing
    Drive_pub 	= n.advertise<geometry_msgs::Twist>("cmd_vel",1);			//Queue size of 1
    arm_pub  	= n.advertise<std_msgs::Float32MultiArray>("arm_cmd",1);		//Publish Values into Array
}

//CallBack Function
void Controls::CallBack(const sensor_msgs::Joy::ConstPtr& joy)				//Add all function calls in here
{
    Controls::Drive(joy);
}

//Controller Functions
void Controls::Drive(const sensor_msgs::Joy::ConstPtr& joy)				//Inputs are -1.0 to 1.0	
{
    float x_axis = -1.0 * (joy->axes[0]);	//Left Stick
    int x = joy->buttons[1];			//X Button
    float forward = (joy->axes[4]);		//Right Trigger R2
    float reverse = (joy->axes[3]);		//Left Trigger L2
    float Drive; 
    forward = -1.0*((forward-1.0)/2.0);		//Converts values  
    reverse = (reverse-1.0)/2.0; 		//Converts values
    Drive = forward + reverse;			//Combines Forward and Revers Values for net Drive Speed and Direction
    
    geometry_msgs::Twist msg;			//Publish Values as Twist Message: Linear and Angular with xyz Values				
    msg.linear.x=(Drive);
    msg.angular.z=(x_axis);
    Drive_pub.publish(msg);
    

    //Arm Controls
    float shoulderExt = joy->axes[1];	//Left Stick up down
    float elbowExt = 	joy->axes[5];	//Right Stick up down
    int wristExtU = 	joy->buttons[4];//L1
    int wristExtD = 	joy->buttons[5];//R1
    float wristRotateL = joy->axes[3];	//L2
    float wristRotateR = joy->axes[4];	//R2
    wristRotateL = -1.0*((wristRotateL-1.0)/2.0);
    wristRotateR = -1.0*((wristRotateR-1.0)/2.0);
    int startArm = joy->buttons[3];	//Triangle

    //Sample Return
    int tray;
    if(joy->axes[6]>0){      //Right on D-Pad
        tray = -1;            //Rotate Right
    } 
    else if(joy->axes[6]<0){ //Left on D-Pad
        tray = 1;           //Rotate Left
    }
    else{                    //Dont Move
        tray = 0;
    }

    int probe;
    if(joy->axes[7]<0){      //Up on D-Pad
        probe = -1;           //Move Up
    } 
    else if(joy->axes[7]>0){ //Down on D-Pad
        probe = 1;          //Move Down
    }
    else{                    //Don't Move
        probe = 0;
    }



    std_msgs::Float32MultiArray msg1;  	//Publish Values to an Array
    msg1.data.push_back(shoulderExt);	//0 Shoulder Extension
    msg1.data.push_back(elbowExt);	//1	
    msg1.data.push_back(wristExtU);	//2
    msg1.data.push_back(wristExtD);	//3
    msg1.data.push_back(wristRotateL);	//4
    msg1.data.push_back(wristRotateR);	//5
    msg1.data.push_back(startArm);	//6
    msg1.data.push_back(Drive);		//7
    msg1.data.push_back(x_axis);	//8
    msg1.data.push_back(tray);      //9
    msg1.data.push_back(probe);     //10

    arm_pub.publish(msg1);
    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");  	//Create node named "controller"
    Controls controls;   

    ros::Rate loop_rate(2);			//Publishes x times per second
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}