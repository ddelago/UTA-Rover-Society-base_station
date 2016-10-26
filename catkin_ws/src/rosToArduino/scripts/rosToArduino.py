#!/usr/bin/env python
import rospy
import serial
import time
ser = serial.Serial('/dev/ttyACM0')

ser.baudrate = 9600
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
#Global variables:
shoulderExt = 220
elbowExt = 220
wristExt = 45
system=0


##### If no changes, no output, help prevent flooding system? #####

def Drive(msg):
	global system, shoulderExt, elbowExt, wristExt
	if(msg.data[6]==1):
		system += 1

	system = system%3

	if(system==0):
		linear = round(100*(msg.data[7]))
		angular = round(100*(msg.data[8]))
		int_linear = int(float(linear))
		int_angular = int(float(angular))

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

	elif(system==1):
		shoulderAxis = msg.data[0]   	#-1 to 1
		elbowAxis = msg.data[1]		#-1 to 1
		wristExtD = msg.data[2]		#1 or 0
		wristExtU = msg.data[3]		#1 or 0
		wristRotL = int(round(-100*(msg.data[4])))
		wristRotR = int(round(100*(msg.data[5])))
		wristNet =  wristRotL + wristRotR

		if(shoulderAxis>0):
			if(shoulderExt<=250):
				shoulderExt+=1
				if(shoulderAxis>.25):
					shoulderExt+=2
				if(shoulderAxis>.50):
					shoulderExt+=3
				if(shoulderAxis>.75):
					shoulderExt+=4
		if(shoulderAxis<0):
			if(shoulderExt>=75):
				shoulderExt-=1
				if(shoulderAxis<-.25):
					shoulderExt-=2
				if(shoulderAxis<-.50):
					shoulderExt-=3
				if(shoulderAxis<-.75):
					shoulderExt-=4
		
		if(elbowAxis>0):
			if(elbowExt<=250):
				elbowExt+=1
				if(elbowAxis>.25):
					elbowExt+=2
				if(elbowAxis>.50):
					elbowExt+=3
				if(elbowAxis>.75):
					elbowExt+=4
		if(elbowAxis<0):
			if(elbowExt>=75):
				elbowExt-=1
				if(elbowAxis<-.25):
					elbowExt-=2
				if(elbowAxis<-.50):
					elbowExt-=3
				if(elbowAxis<-.75):
					elbowExt-=4
		if(wristExtU==1.0):
			if(wristExt<=250):
				wristExt+=5
		if(wristExtD==1.0):
			if(wristExt>=45):
				wristExt-=5


		if(wristNet >= 0):
			wristNetOUT = "+" + str(wristNet)
		elif(wristNet < 0):
			wristNetOUT = str(wristNet)

		if(wristExt >= 0):
			wristExtOUT = "+" + str(wristExt)
		elif(wristExt < 0):
			wristExtOUT = str(wristExt)

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

		if(len(wristNetOUT)==3):
			wristNetOUT = wristNetOUT[0] + '0' + wristNetOUT[1:]
		if(len(wristNetOUT)==2):
			wristNetOUT = wristNetOUT[0] + '00' + wristNetOUT[1:]

		if(len(wristExtOUT)==3):
			wristExtOUT = wristExtOUT[0] + '0' + wristExtOUT[1:]
		if(len(wristExtOUT)==2):
			wristExtOUT = wristExtOUT[0] + '00' + wristExtOUT[1:]


		output = "A" +"," + "2" + "," + shoulderExtOUT
		ser.write(output.encode())

		output = "A" +"," + "3" + "," + elbowExtOUT
		ser.write(output.encode())
		
		output = "A" +"," + "4" + "," + wristExtOUT
		ser.write(output.encode())

		output = "A" +"," + "5" + "," + wristNetOUT
		ser.write(output.encode())

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


		if(len(drillPos)==3):
			drillPos = drillPos[0] + '0' + drillPos[1:]
		if(len(drillPos)==2):
			drillPos = drillPos[0] + '00' + drillPos[1:]
		output = "R" +"," + "2" + "," + drillPos
		ser.write(output.encode())


		if(len(spinDrill)==3):
			spinDrill = spinDrill[0] + '0' + spinDrill[1:]
		if(len(spinDrill)==2):
			spinDrill = spinDrill[0] + '00' + spinDrill[1:]
		output = "R" +"," + "3" + "," + spinDrill
		ser.write(output.encode())


		if(len(moveTray)==3):
			moveTray = moveTray[0] + '0' + moveTray[1:]
		if(len(moveTray)==2):
			moveTray = moveTray[0] + '00' + moveTray[1:]
		output = "R" +"," + "4" + "," + moveTray
		ser.write(output.encode())


		if(len(moveProbe)==3):
			moveProbe = moveProbe[0] + '0' + moveProbe[1:]
		if(len(moveProbe)==2):
			moveProbe = moveProbe[0] + '00' + moveProbe[1:]
		output = "R" +"," + "5" + "," + moveProbe
		ser.write(output.encode())


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
