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
		shoulderExt = -1*int(round(100*msg.data[0]))   	#-1 to 1, Left Stick
		elbowExt = -1*int(round(100*msg.data[1]))	#-1 to 1, Right Stick
		wristExt = msg.data[2]				#-1, 0, or 1, D-pad up/down
		gripperOpen = int(round(-100*(msg.data[4])))	#L2
		gripperClose = int(round(100*(msg.data[5])))	#R2
		gripperState =  gripperOpen + gripperClose
		wristRot = msg.data[3]      			#-1, 0, or 1, D-pad left/right
		
		if(wristExt == 1):
			wristExt = -75
		elif(wristExt == -1):
			wristExt = 75
		elif(wristExt == 0):
			wristExt = 0

		if(wristRot == 1):
			wristRot = -85
		elif(wristRot == -1):
			wristRot = 85
		elif(wristRot == 0):
			wristRot = 0

		#String Formatting for Serial
		if(gripperState >= 0):
			gripperState = "+" + str(gripperState)
		elif(gripperState < 0):
			gripperState = str(gripperState)

		if(wristExt >= 0):
			wristExtOUT = "+" + str(wristExt)
		elif(wristExt < 0):
			wristExtOUT = str(wristExt)
 	
		if(wristRot >= 0):
			wristRot = "+" + str(wristRot)
		elif(wristRot < 0):
			wristRot = str(wristRot)

		if(shoulderExt >= 0):
			shoulderExtOUT = "+" + str(shoulderExt)
		elif(shoulderExt < 0):
			shoulderExtOUT = str(shoulderExt)

		if(elbowExt >= 0):
			elbowExtOUT = "+" + str(elbowExt)
		elif(elbowExt < 0):
			elbowExtOUT = str(elbowExt)

		#Formatting serial output string
		output = "A" +"," + "2" + "," + shoulderExtOUT
		ser.write(output.encode())
		Arm = output

		output = "A" +"," + "3" + "," + elbowExtOUT
		ser.write(output.encode())
		Arm += output	

		output = "A" +"," + "4" + "," + wristExtOUT
		ser.write(output.encode())
		Arm += output		

		output = "A" +"," + "5" + "," + wristRot
		ser.write(output.encode())
		Arm += output
		
		output = "A" +"," + "6" + "," + gripperState
		ser.write(output.encode())
		Arm += output
		
		print(Arm)
		#Gripper Limits .2 < x < .5 of 270		
		
	#Sample Return
	elif(system==2):
		sarPos = int(round(100*(msg.data[0])))		#L3
		drillPos = int(round(100*(msg.data[1])))	#R3
		spinDrill = int(round(100*(msg.data[5])))	#R2
		moveTray = int(msg.data[9])			#1 is right, -1 is left, D-Pad
		moveProbe = int(msg.data[10])		#1 is up, -1 is down, D-Pad

		if(sarPos >= 0):
			sarPos = "+" + str(sarPos)
		elif(sarPos < 0):
			sarPos = str(sarPos)

		if(drillPos >= 0):
			drillPos = "+" + str(drillPos)
		elif(drillPos < 0):
			drillPos = str(drillPos)

		if(spinDrill >= 0):
			spinDrill = "+" + str(spinDrill)

		if(moveTray == 1):
			moveTray = "+20"
		elif(moveTray == -1):
			moveTray = "-20"
		elif(moveTray == 0):
			moveTray = "+0"

		if(moveProbe == 1):
			moveProbe = "+20"
		elif(moveProbe == -1):
			moveProbe = "-20"
		elif(moveProbe == 0):
			moveProbe = "+0"

		if(len(moveProbe)==3):
			moveProbe = moveProbe[0] + '0' + moveProbe[1:]
		if(len(moveProbe)==2):
			moveProbe = moveProbe[0] + '00' + moveProbe[1:]
		output = "R" +"," + "5" + "," + moveProbe
		ser.write(output.encode())
		SR += output

		print(SR)

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
