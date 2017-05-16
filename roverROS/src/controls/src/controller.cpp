/* 
 * Daniel Delago   11/25/2015 
 * Description: Creates node "controller" that takes values from the controller 
 * and publishes the edited values to multiple topics.
 * 
 * Node: controller
 * Subscriptions: controller_raw
 * Publishing Topics: cmd_vel AND arm_cmd
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
    ros::Publisher arm_pub; 				//Add more publishers as needed
    
    
};

Controls::Controls()
{      
    controller_sub 	= n.subscribe("controller_raw",1, &Controls::CallBack, this); 	//Queue size of 1
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
    float forward = (joy->axes[4]);         //Right Trigger R2
    float reverse = (joy->axes[3]);         //Left Trigger L2
    float Drive; 
    forward = -1.0*((forward-1.0)/2.0);		//Converts values  
    reverse = (reverse-1.0)/2.0;            //Converts values
    Drive = forward + reverse;              //Combines Forward and Revers Values for net Drive Speed and Direction
    
    //Arm Controls
    float shoulderExt = joy->axes[1];			//Left Stick up down
    float elbowExt = 	joy->axes[5];			//Right Stick up down
    float gripperOpen = -1.0*((joy->axes[3]-1.0)/2.0);  //L2
    float gripperClose = -1.0*((joy->axes[4]-1.0)/2.0); //R2
    int mode = joy->buttons[3];					//Triangle, switches between drive/arm/sample return
    int driveFullSpeed = joy->buttons[1];       //X

    int wristExt;
    if(joy->axes[7]<0){      //Up on D-Pad
        wristExt = -1;       //Move Up
    } 
    else if(joy->axes[7]>0){ //Down on D-Pad
        wristExt = 1;        //Move Down
    }
    else{                    //Don't Move
        wristExt = 0;
    }

/*DRV:+100,-100
ARM:1,(+-)100
ARM: 6 joints
shoulder rotation:1
shouder extension: 2
elbow:3
wrist extension: 4
wrist rptation: 5
Gripper: 6
SAR:?
*/
    int wristRot;
    if(joy->axes[6]<0){      //Up on D-Pad
        wristRot = -1;       //Move Up
    }
    else if(joy->axes[6]>0){ //Down on D-Pad
        wristRot = 1;        //Move Down
    }
    else{                    //Don't Move
        wristRot = 0;
    }


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
    msg1.data.push_back(elbowExt);		//1	
    msg1.data.push_back(wristExt);		//2
    msg1.data.push_back(wristRot);		//3
    msg1.data.push_back(gripperOpen);   //4
    msg1.data.push_back(gripperClose);  //5
    msg1.data.push_back(mode);			//6
    msg1.data.push_back(Drive);			//7
    msg1.data.push_back(x_axis);		//8
    msg1.data.push_back(tray);      	//9
    msg1.data.push_back(probe);    	 	//10
    msg1.data.push_back(driveFullSpeed);//11

    arm_pub.publish(msg1);
    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");  	//Create node named "controller"
    Controls controls;   

    ros::Rate loop_rate(3);			//Publishes x times per second
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}
