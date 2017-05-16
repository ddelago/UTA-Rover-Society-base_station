# README #

* Repository for the Rover Society Base Station

### Dependencies ###

* Ubuntu 14.04 (http://www.ubuntu.com)
* ROS Indigo Igloo (http://wiki.ros.org/indigo/Installation/Ubuntu)
+ Marble and Marble development packages (http://marble.kde.org) 
> sudo apt-get install marble libmarble-dev


### Contribution guidelines ###

* This repo follows the ROS Developer's Guide (http://wiki.ros.org/DevelopersGuide)
* All working code should be commited to base-devel
* When code is ready for testing submit a pull request to the testing branch

### How to start up ROS ###
Base Station Bash:

Enable Static IP: 
> cd /etc/network/interfaces
Set ROS_IP to that of the Base Station (192.168.10.10)
> export ROS_IP=192.168.10.10				      
Set ROS_HOSTBAME to that of the Base Station (192.168.10.10)
> export ROS_HOSTNAME=192.168.10.10			      
Set IP to that of the master(odroid): 192.168.1.101
> export ROS_MASTER_URI="http://192.168.1.101:11311"	

Odroid Bash:
* IP of Odroid
> export ROS_IP=192.168.1.101				
* IP of Ordoid
> export ROS_HOSTNAME=192.168.1.101			
* IP of master(Odroid)
> export ROS_MASTER_URI="http://192.168.1.101:11311"

On Base Station:

 - Connect to rocket
 - SSH into Odroid: 
 > ssh -X odroid@192.168.1.101 
 > roslaunch controls start_controls.launch

On Odroid: 
 
 > roscore
 - Start GPS: 
 > rosrun nmea_navsat_driver nmea_serial_driver _port:=/dev/ttyACM# _baud:=38400
   ON BASESTATION: 
 > rosrun random_gps_publisher gps_publisher
 - Start IMU:
 > rosrun myahrs_driver myahrs_driver _port:=/dev/ttyACM#    OR
 > roslaunch myahrs_driver myahrs_driver.launch
 - Start Cameras:
 > rosrun rqt_example_cpp opencv_test 0
 > rosrun rosToArduino rosToArduino.py

On Base Station:

 - Start rqt and run gps and Rviz for IMU 
 - NOTE: When you try to zoom in on the gps on OpenStreetMap View, It zooms out if you zoom in too far,
   Just keep zooming back in after it zooms out and it will work after a few trys


