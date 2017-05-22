/* 
 * Daniel Delago   11/25/2015 
 * Description: Creates node "controller" that takes values from the controller 
 * and publishes the edited values to multiple topics. USES A PS4 CONTROLLER!
 * 
 * Node: controls
 * Subscriptions: controller_raw
 * Publishing Topics: controls_input
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
    //Add more functions as needed
    private:
		void CallBack(const sensor_msgs::Joy::ConstPtr& joy);		
	    void controls_input(const sensor_msgs::Joy::ConstPtr& joy);				 
    ros::NodeHandle n;
    ros::Subscriber controller_sub;
    //Add more publishers as needed
    ros::Publisher controls_pub; 				
    
    
};

Controls::Controls()
{      
    //Queue size of 1	
    controller_sub 	= n.subscribe("controller_raw",1, &Controls::CallBack, this); 	
    //Publish Values into Array
    controls_pub  		= n.advertise<std_msgs::Float32MultiArray>("controls_input",1);		
}

//CallBack Function
void Controls::CallBack(const sensor_msgs::Joy::ConstPtr& joy)				
{
    //Add all function calls in here
    Controls::controls_input(joy);
}

//Controller Functions
//All inputs are -1.0 to 1.0
void Controls::controls_input(const sensor_msgs::Joy::ConstPtr& joy)					
{
	/*-*-*-*-* Drive *-*-*-*-*/

    //Left Stick
    float angular = -1.0 * (joy->axes[0]);   
    //Right Trigger R2
    float forward = (joy->axes[4]);         
	//Left Trigger L2
    float reverse = (joy->axes[3]);         
	
	//Value to be published (-1.0 to 1.0)
    float drive; 
	//Converts values for R2 and L2 to correct velocity values
    forward = -1.0*((forward-1.0)/2.0);	    
    reverse = (reverse-1.0)/2.0;            	
	//Combines Forward and Reverse values for net drive speed and direction
    drive = forward + reverse;              
    

    /*-*-*-*-* Arm *-*-*-*-*/

	//Right stick left & right
    float shoulder_rotation  = -1.0 * joy->axes[2];;
	//Left stick up & down
    float shoulder_extension = joy->axes[1];			
	//Right stick up & down
    float elbow_extension = 	joy->axes[5];			
	
    int wrist_extension;
	//Up on D-Pad, move up
    if(joy->axes[7]<0)      
        wrist_extension = -1;      
	//Down on D-Pad, move down 
    else if(joy->axes[7]>0) 
        wrist_extension = 1;        
    else                    
        wrist_extension = 0;

    int wrist_rotation;
	//Left on D-Pad, rotate left
    if(joy->axes[6]<0)      
        wrist_rotation = -1;      
	//Right on D-Pad, rotate right 
    else if(joy->axes[6]>0) 
        wrist_rotation = 1;
    else                   
        wrist_rotation = 0;
    
	//L2
    float gripper_open = -1.0*((joy->axes[3]-1.0)/2.0);  

	//R2
    float gripper_close = -1.0*((joy->axes[4]-1.0)/2.0);


    /*-*-*-*-* Sample Return *-*-*-*-*/

	int tray;
	//Right on D-Pad, rotate right
    if(joy->axes[6]>0)      
        tray = -1;         
	//Left on D-Pad, rotate left    
    else if(joy->axes[6]<0)
        tray = 1;           
    else                 
        tray = 0;
    
    int probe;
	//Up on D-Pad, raise probe
    if(joy->axes[7]<0)      
        probe = -1;         
	//Down on D-Pad, lower probe      
    else if(joy->axes[7]>0) 
        probe = 1;              
    else                   
        probe = 0;
    
	/*-*-*-*-* Extra *-*-*-*-*/

	//X, allows max drive speed
    int driveFullSpeed = joy->buttons[1];       	
	//Triangle, switches between drive/arm/sample return 
    int mode = joy->buttons[3];					

	//Publish to an Array
    std_msgs::Float32MultiArray msg1;  	
    msg1.data.push_back(shoulder_extension);//0 
    msg1.data.push_back(elbow_extension);	//1	
    msg1.data.push_back(wrist_extension);	//2
    msg1.data.push_back(wrist_rotation);	//3
    msg1.data.push_back(gripper_open);   	//4
    msg1.data.push_back(gripper_close);  	//5
    msg1.data.push_back(mode);				//6
    msg1.data.push_back(drive);				//7
    msg1.data.push_back(angular);			//8
    msg1.data.push_back(tray);      		//9
    msg1.data.push_back(probe);    	 		//10
    msg1.data.push_back(driveFullSpeed);	//11
    msg1.data.push_back(shoulder_rotation);	//12

    controls_pub.publish(msg1);
    
}

int main(int argc, char **argv)
{	
	//Create node named "controller"
    ros::init(argc, argv, "controls");  	
    Controls controls;   
	
	//Publishes x times per second
    ros::Rate loop_rate(6);			
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}
