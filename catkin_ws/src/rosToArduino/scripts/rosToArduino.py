#!/usr/bin/env python
import rospy
import serial
import time
ser = serial.Serial('/dev/ttyACM0')
ser1 = serial.Serial('/dev/ttyACM1')

ser.baudrate = 9600
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

system = 0


###-~-~-~-~ If no changes, no output, help prevent flooding system -~-~-~-~###
"""
	if(newValue!=oldValue):
		Write to Serial
	oldValue=newValue
"""

def Drive(msg):
	global system
	if(msg.data[6]==1):
		system += 1

	system = system%3

	if(system==0): 							#Drive
		linear = round(50*(msg.data[7]))
		angular = round(50*(msg.data[8]))
		int_linear = int(float(linear))
		int_angular = int(float(angular))

		if(msg.data[11]==1 and int_linear==50):		#Full Speed Forward
			int_linear=80
		if(msg.data[11]==1 and int_linear==-50):		#Full Speed Backwards
			int_linear=-80

		if(int_linear >= 0):
			int_linear = "+" + str(int_linear)
		elif(int_linear < 0):
			int_linear = str(int_linear)

		if(int_angular >= 0):
			int_angular = "+" + str(int_angular)
		elif(int_angular < 0):
			int_angular = str(int_angular)
	
		if(len(int_linear)==3):
			int_linear = int_linear[0] + '0' + int_linear[1:]
		if(len(int_angular)==3):
			int_angular = int_angular[0] + '0' + int_angular[1:]
		if(len(int_linear)==2):
			int_linear = int_linear[0] + '00' + int_linear[1:]
		if(len(int_angular)==2):
			int_angular = int_angular[0] + '00' + int_angular[1:]
		output = "D" +"," + int_linear + "," + int_angular
		ser.write(output.encode())
		print(output)
	
	#Arm
	elif(system==1):									
		shoulderExt = -1*int(round(100*msg.data[0]))   	#-1 to 1, Left Stick
		elbowExt = -1*int(round(100*msg.data[1]))		#-1 to 1, Right Stick
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

		if(len(shoulderExtOUT)==3):
			shoulderExtOUT = shoulderExtOUT[0] + '0' + shoulderExtOUT[1:]
		if(len(shoulderExtOUT)==2):
			shoulderExtOUT = shoulderExtOUT[0] + '00' + shoulderExtOUT[1:]

		if(len(elbowExtOUT)==3):
			elbowExtOUT = elbowExtOUT[0] + '0' + elbowExtOUT[1:]
		if(len(elbowExtOUT)==2):
			elbowExtOUT = elbowExtOUT[0] + '00' + elbowExtOUT[1:]

		if(len(wristRot)==3):
			wristRot = wristRot[0] + '0' + wristRot[1:]
		if(len(wristRot)==2):
			wristRot = wristRot[0] + '00' + wristRot[1:]

		if(len(wristExtOUT)==3):
			wristExtOUT = wristExtOUT[0] + '0' + wristExtOUT[1:]
		if(len(wristExtOUT)==2):
			wristExtOUT = wristExtOUT[0] + '00' + wristExtOUT[1:]

		if(len(gripperState)==3):
			gripperState = gripperState[0] + '0' + gripperState[1:]
		if(len(gripperState)==2):
			gripperState = gripperState[0] + '00' + gripperState[1:]

		
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

	
		if(len(sarPos)==3):
			sarPos = sarPos[0] + '0' + sarPos[1:]
		if(len(sarPos)==2):
			sarPos = sarPos[0] + '00' + sarPos[1:]
		output = "R" +"," + "1" + "," + sarPos
		ser.write(output.encode())
		SR = output

		if(len(drillPos)==3):
			drillPos = drillPos[0] + '0' + drillPos[1:]
		if(len(drillPos)==2):
			drillPos = drillPos[0] + '00' + drillPos[1:]
		output = "R" +"," + "2" + "," + drillPos
		ser.write(output.encode())
		SR += output

		if(len(spinDrill)==3):
			spinDrill = spinDrill[0] + '0' + spinDrill[1:]
		if(len(spinDrill)==2):
			spinDrill = spinDrill[0] + '00' + spinDrill[1:]
		output = "R" +"," + "3" + "," + spinDrill
		ser.write(output.encode())
		SR += output

		if(len(moveTray)==3):
			moveTray = moveTray[0] + '0' + moveTray[1:]
		if(len(moveTray)==2):
			moveTray = moveTray[0] + '00' + moveTray[1:]
		output = "R" +"," + "4" + "," + moveTray
		ser.write(output.encode())
		SR += output

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
    rospy.init_node('rosToArduino2')
    rospy.Subscriber("arm_cmd", Float32MultiArray, Drive)
    rospy.Timer(rospy.Duration(1), status_check)
    #rospy.Subsciber("Topic", Type, FunctionCall)	

    rospy.spin()

if __name__ == '__main__':
    listener()
