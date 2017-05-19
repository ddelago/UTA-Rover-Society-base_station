#!/usr/bin/env python
import rospy
import serial
import time
ser = serial.Serial('/dev/ttyACM0')

ser.baudrate = 9600
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

def Drive_auto(msg):
	rospy.loginfo(rospy.get_caller_id() + "Autonomy is sending: %s",msg.data)
	ser.write(msg.data.encode())

#Global Variables: 
system = 0
shoulder_rotation_prev = 0.0							
shoulder_extension_prev = 0.0
elbow_extension_prev = 0.0
wrist_extension_prev = 0.0
gripperState_prev = 0.0
wrist_rotation_prev = 0.0
sar_position_prev = 0.0
drill_position_prev = 0.0
spin_drill_prev = 0.0
move_tray_prev = 0.0
move_probe_prev = 0.0
def Drive(msg):
	"""
	Control System:
	The value of system (0, 1, or 2) will determine what control system is currently activated.
	Each time the triangle button is pressed, it increments the system variable by one. 
	This also changes the control system.
	0: Drive
	1: Arm
	2: Sample Return 	
	"""
	global system
	if(msg.data[6]==1):
		system += 1
	system = system%3

	#Drive	
	#Format: 
	#DRV:Linear,Angular
	#DRV:(+-)(0-100),(+-)(0-100)
	if(system==0):
		#soft_max is the maximum speed that the rover will move using only the triggers. (Without X pressed)
		soft_max = 50 							
		linear = round(soft_max*(msg.data[7]))
		angular = round(soft_max*(msg.data[8]))
		int_linear = int(float(linear))
		int_angular = int(float(angular))
		
		"""
		Maximum Speed:
		Full speed forward is set only when the trigger is fully pushed and when the X button is also pressed. 
		This will allow a maximum linear speed of 80 (80% of the true maximum speed).   
  		If the X button is not also pressed, the maximum linear speed will be 50.
		"""
		if(msg.data[11]==1 and int_linear==50):			 
			int_linear=80
		if(msg.data[11]==1 and int_linear==-50):					
			int_linear=-80
		
		#Formatting serial output string
		output = "DRV" +":" + str(int_linear) + "," + str(int_angular)
		ser.write(output.encode())
		
		#Print to Base Station terminal
		print(output);

	#Arm
	#Format:
	#ARM:Joint,Speed
	#ARM:(1-6),(+-)(0-100)
	elif(system==1):
		"""
		Joints:
		1: Shoulder Rotation
		2: Shoulder Extension
		3: Elbow Extension
		4: Wrist Extension
		5: Wrist Rotation
		6: Gripper Open/Close
		"""
		#If current = previous value, do not update value. Prevents flooding of system
		global shoulder_rotation_prev, shoulder_extension_prev, elbow_extension_prev, wrist_extension_prev, gripperState_prev, wrist_rotation_prev
		shoulder_rotation = int(round(100*msg.data[12]))		#-1 to 1, Right Stick left/right									
		shoulder_extension = -1*int(round(100*msg.data[0]))   	#-1 to 1, Left Stick up/down
		elbow_extension = -1*int(round(100*msg.data[1]))		#-1 to 1, Right Stick
		wrist_extension = msg.data[2]							#-1, 0, or 1, D-pad up/down
		gripperOpen = int(round(-100*(msg.data[4])))			#L2
		gripperClose = int(round(100*(msg.data[5])))			#R2
		gripperState =  gripperOpen + gripperClose
		wrist_rotation = msg.data[3]      						#-1, 0, or 1, D-pad left/right
		
		#Max values
		if(wrist_extension == 1):
			wrist_extension = -75
		elif(wrist_extension == -1):
			wrist_extension = 75
		elif(wrist_extension == 0):
			wrist_extension = 0

		if(wrist_rotation == 1):
			wrist_rotation = -85
		elif(wrist_rotation == -1):
			wrist_rotation = 85
		elif(wrist_rotation == 0):
			wrist_rotation = 0

		Arm = ""
		#Formatting serial output string
		if(shoulder_rotation != shoulder_rotation_prev):
			output = "ARM:" + "1" + "," + str(shoulder_rotation) + " "
			ser.write(output.encode())
			shoulder_rotation_prev = shoulder_rotation
			Arm += output

		if(shoulder_extension != shoulder_extension_prev):
			output = "ARM:" + "2" + "," + str(shoulder_extension) + " "
			ser.write(output.encode())
			shoulder_extension_prev = shoulder_extension
			Arm += output

		if(elbow_extension != elbow_extension_prev):		
			output = "ARM:" + "3" + "," + str(elbow_extension) + " "
			ser.write(output.encode())
			elbow_extension_prev = elbow_extension
			Arm += output	

		if(wrist_extension != wrist_extension_prev):
			output = "ARM:" + "4" + "," + str(wrist_extension) + " "
			ser.write(output.encode())
			wrist_extension_prev = wrist_extension
			Arm += output		

		if(wrist_rotation != wrist_rotation_prev):
			output = "ARM:" + "5" + "," + str(wrist_rotation) + " "
			ser.write(output.encode())
			wrist_rotation_prev = wrist_rotation
			Arm += output
		
		if(gripperState != gripperState_prev):
			output = "ARM:" + "6" + "," + str(gripperState) + " "
			ser.write(output.encode())
			gripperState_prev = gripperState
			Arm += output
		
		#Print to Base Station terminal
		print(Arm)		
		
	#Sample Return
	elif(system==2):
		#If current = previous value, do not update value. Prevents flooding of system
		global sar_position_prev, drill_position_prev, spin_drill_prev, move_tray_prev, move_probe_prev
		sar_position = int(round(100*(msg.data[0])))	#Left stick
		drill_position = int(round(100*(msg.data[1])))	#Right stick
		spin_drill = int(round(100*(msg.data[5])))		#R2
		move_tray = int(msg.data[9])					#1 is right, -1 is left, D-Pad
		move_probe = int(msg.data[10])					#1 is up, -1 is down, D-Pad

		#Max values
		if(move_tray == 1):
			move_tray = "20"
		elif(move_tray == -1):
			move_tray = "-20"
		elif(move_tray == 0):
			move_tray = "0"

		if(move_probe == 1):
			move_probe = "20"
		elif(move_probe == -1):
			move_probe = "-20"
		elif(move_probe == 0):
			move_probe = "0"

		SR=""
		#Formatting serial output string
		if(sar_position != sar_position_prev):
			output = "SAR:" + "1" + "," + str(sar_position) + " "
			ser.write(output.encode())
			sar_position_prev = sar_position
			SR += output

		if(drill_position != drill_position_prev):
			output = "SAR:" + "2" + "," + str(drill_position) + " "
			ser.write(output.encode())
			drill_position_prev = drill_position
			SR += output

		if(spin_drill != spin_drill_prev):
			output = "SAR:" + "3" + "," + str(spin_drill) + " "
			ser.write(output.encode())
			spin_drill_prev = spin_drill
			SR += output

		if(move_tray != move_tray_prev):
			output = "SAR:" + "4" + "," + str(move_tray) + " "
			ser.write(output.encode())
			move_tray_prev = move_tray
			SR += output

		if(move_probe != move_probe_prev):
			output = "SAR:" + "5" + "," + str(move_probe) + " "
			ser.write(output.encode())
			move_probe_prev = move_probe
			SR += output

		#Print to Base Station terminal
		print(SR)

#Health check
def status_check(event):
    ser.write('C')

def listener():
    rospy.init_node('serial_communication')
    rospy.Subscriber("controls_input", Float32MultiArray, Drive)
    #rospy.Subscriber("drive_cmd", String, Drive)
    rospy.Timer(rospy.Duration(1), status_check)
    #rospy.Subsciber("Topic", Type, FunctionCall)	
    rospy.spin()

if __name__ == '__main__':
    listener()
