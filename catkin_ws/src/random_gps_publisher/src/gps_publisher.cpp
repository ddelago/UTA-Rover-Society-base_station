/*
 * random_gps_publisher.cpp
 * This node publishes GPS coordinates that are increment by 0.0001 every 2 seconds from a given starting position
 *
 * Copyright (C) 2015 Robert Hamilton rghamilton3@gmail.com
 *
 * Distributed under terms of the BSD license.
 *
 */
 
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

class Gps
{   
    public: Gps();
    private:void CallBack(const sensor_msgs::NavSatFix::ConstPtr& gps);     
    ros::NodeHandle n;
    ros::Subscriber gps_sub; 
    ros::Publisher gps_pub;                 
    
};

Gps::Gps()
{      
    gps_sub = n.subscribe("/fix",5, &Gps::CallBack, this);  //Queue size of 1   
    gps_pub = n.advertise<sensor_msgs::NavSatFix>("gps_test", 5);              
}

//CallBack Function
void Gps::CallBack(const sensor_msgs::NavSatFix::ConstPtr& gps)         
{
    
    // Create NavSatFix message object
    sensor_msgs::NavSatFix coords;
    sensor_msgs::NavSatStatus coordStatus;

    /*
     * Navigation Satellite fix status for any Global Navigation Satellite System
     * This is the first part of NavSatFix
     */
    coordStatus.status = coordStatus.STATUS_NO_FIX;
    coordStatus.service = coordStatus.SERVICE_GPS;
    coords.status = coordStatus;

    /*
     * Start at Hanksville, Utah
     */
    coords.latitude = gps->latitude;
    coords.longitude = gps->longitude;
    coords.altitude = gps->altitude;

    // Output for comparison
    ROS_INFO("LAT: %f LONG: %f", coords.latitude, coords.longitude);

    gps_pub.publish(coords);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps");   //Create node named gps
    Gps gps;   

    ros::Rate loop_rate(3);     //Publishes x times per second
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}
