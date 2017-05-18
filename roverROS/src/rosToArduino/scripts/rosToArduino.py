#!/usr/bin/env python
import rospy
import serial
import time
ser = serial.Serial('/dev/ttyACM0')

ser.baudrate = 9600
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
system = 0

def Drive_auto(msg):
	rospy.loginfo(rospy.get_caller_id() + "Autonomy is sending: %s",msg.data)
	ser.write(msg.data.encode())
	
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
		
		#Serial Formatting: Conditionals on wether to include a '+' or not 
		if(int_linear >= 0):
			int_linear = "+" + str(int_linear)
		elif(int_linear < 0):
			int_linear = str(int_linear)
		if(int_angular >= 0):
			int_angular = "+" + str(int_angular)
		elif(int_angular < 0):
			int_angular = str(int_angular)
		
		#Formatting serial output string
		output = "DRV" +":" + int_linear + "," + int_angular + "\n"
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
		shoulder_rotation = -1*int(round(100*msg.data[12]))		#-1 to 1, Right Stick left/right									
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

		#Serial Formatting: Conditionals on wether to include a '+' or not		
		if(shoulder_rotation >= 0):
			shoulder_rotation_out = "+" + str(shoulder_rotation)
		elif(shoulder_rotation < 0):
			shoulder_rotation_out = str(shoulder_rotation)

		if(shoulder_extension >= 0):
			shoulder_extension_out = "+" + str(shoulder_extension)
		elif(shoulder_extension < 0):
			shoulder_extension_out = str(shoulder_extension)

		if(elbow_extension >= 0):
			elbow_extension_out = "+" + str(elbow_extension)
		elif(elbow_extension < 0):
			elbow_extension_out = str(elbow_extension)

		if(wrist_extension >= 0):
			wrist_extension_out = "+" + str(wrist_extension)
		elif(wrist_extension < 0):
			wrist_extension_out = str(wrist_extension)

		if(wrist_rotation >= 0):
			wrist_rotation = "+" + str(wrist_rotation)
		elif(wrist_rotation < 0):
			wrist_rotation = str(wrist_rotation)

		if(gripperState >= 0):
			gripperState = "+" + str(gripperState)
		elif(gripperState < 0):
			gripperState = str(gripperState)

		#Formatting serial output string
		output = "A" +"," + "1" + "," + shoulder_rotation_out
		ser.write(output.encode())
		Arm = output

		output = "A" +"," + "2" + "," + shoulder_extension_out
		ser.write(output.encode())
		Arm += output

		output = "A" +"," + "3" + "," + elbow_extension_out
		ser.write(output.encode())
		Arm += output	

		output = "A" +"," + "4" + "," + wrist_extension_out
		ser.write(output.encode())
		Arm += output		

		output = "A" +"," + "5" + "," + wrist_rotation
		ser.write(output.encode())
		Arm += output
		
		output = "A" +"," + "6" + "," + gripperState
		ser.write(output.encode())
		Arm += output
		
		#Print to Base Station terminal
		print(Arm)		
		
	#Sample Return
	elif(system==2):
		sar_position = int(round(100*(msg.data[0])))	#Left stick
		drill_position = int(round(100*(msg.data[1])))	#Right stick
		spin_drill = int(round(100*(msg.data[5])))		#R2
		move_tray = int(msg.data[9])					#1 is right, -1 is left, D-Pad
		move_probe = int(msg.data[10])					#1 is up, -1 is down, D-Pad

		#Max values
		if(move_tray == 1):
			move_tray = "+20"
		elif(move_tray == -1):
			move_tray = "-20"
		elif(move_tray == 0):
			move_tray = "+0"

		if(move_probe == 1):
			move_probe = "+20"
		elif(move_probe == -1):
			move_probe = "-20"
		elif(move_probe == 0):
			move_probe = "+0"

		#Serial Formatting: Conditionals on wether to include a '+' or not
		if(sar_position >= 0):
			sar_position = "+" + str(sar_position)
		elif(sar_position < 0):
			sar_position = str(sar_position)

		if(drill_position >= 0):
			drill_position = "+" + str(drill_position)
		elif(drill_position < 0):
			drill_position = str(drill_position)

		if(spin_drill >= 0):
			spin_drill = "+" + str(spin_drill)

		#Formatting serial output string
		output = "R" +"," + "1" + "," + sar_position
		ser.write(output.encode())
		SR = output

		output = "R" +"," + "2" + "," + drill_position
		ser.write(output.encode())
		SR += output

		output = "R" +"," + "3" + "," + spin_drill
		ser.write(output.encode())
		SR += output

		output = "R" +"," + "4" + "," + move_tray
		ser.write(output.encode())
		SR += output

		output = "R" +"," + "5" + "," + move_probe
		ser.write(output.encode())
		SR += output

		#Print to Base Station terminal
		print(SR)

#Health check
def status_check(event):
    ser.write('C')

def listener():
    rospy.init_node('rosToArduino')
    rospy.Subscriber("controls_input", Float32MultiArray, Drive)
    #rospy.Subscriber("drive_cmd", String, Drive)
    rospy.Timer(rospy.Duration(1), status_check)
    #rospy.Subsciber("Topic", Type, FunctionCall)	
    rospy.spin()

if __name__ == '__main__':
    listener()
