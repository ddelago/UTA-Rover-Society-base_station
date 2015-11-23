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

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("gps_test", 1000);

    ros::Rate loop_rate(.5);

    float count = 0;
    while (ros::ok()) {
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
        coords.latitude = 38.3714 + count;
        coords.longitude = -110.7131;
        coords.altitude = 0.0;
        
        // Output for comparison
        ROS_INFO("LAT: %f LONG: %f", coords.latitude, coords.longitude);

        gps_pub.publish(coords);

        ros::spinOnce();

        loop_rate.sleep();
        count += 0.0001;
    }

    return 0;
}
