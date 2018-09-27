import threading
# import DobotDllType as dType
import time
# import cv2 as cv
import numpy as np 
from numpy import *
import python_vision
from python_vision import Coordinates 
import socket

height = 480
width = 640


def main():

	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect(('localhost', 8002))

	python_vision.init()
	time.sleep(2)
	while(1):
		input()

		leftplanepos = python_vision.GetLeftPlaneCentralPoint()
		rightplanepos = python_vision.GetRightPlaneCentralPoint()
		leftangle = python_vision.GetLeftPlaneAngle()
		rightangle= python_vision.GetRightPlaneAngle()


		redpos = python_vision.GetRedPoints()
		redangle = python_vision.GetRedAngles()	
		greenpos = python_vision.GetGreenPoints()
		greenangle = python_vision.GetGreenAngles()
		number = python_vision.GetNumOfPoints()

		testcoordinates = Coordinates(number,redpos,greenpos,redangle,greenangle)


		infostring = ""
		infostring =  str(leftplanepos[0]) + ','+str(leftplanepos[1])+','+ str(leftplanepos[2])+ ','+ str(leftangle[0]) + ',' +str(rightplanepos[0]) + ','+str(rightplanepos[1])+','+ str(rightplanepos[2])+ ','+ str(rightangle[0])
		if(number[0] != 0):
			infostring = infostring+ ',' + str(testcoordinates.redcoordinates[0][0]) + ','+str(testcoordinates.redcoordinates[0][1])+','+ str(testcoordinates.redcoordinates[0][2])+ ','+str(testcoordinates.redangles[0])
			if(number[1] != 0) and (number[0] != 0):
				infostring = infostring + ','+ str(testcoordinates.greencoordinates[0][0]) + ','+str(testcoordinates.greencoordinates[0][1])+','+ str(testcoordinates.greencoordinates[0][2])+ ','+str(testcoordinates.greenangles[0])
		elif (number[1] != 0):
			infostring =  infostring+ ','+str(testcoordinates.greencoordinates[0][0]) + ','+str(testcoordinates.greencoordinates[0][1])+','+ str(testcoordinates.greencoordinates[0][2])+ ','+str(testcoordinates.greenangles[0])
		else:
			print("No iphone detected")

		print(infostring)
		infostring = str.encode(infostring)
		s.sendall(infostring)



	return



	# # while(1):

	# #Connect Dobot
	# state = dType.ConnectDobot(api, "", 115200)[0]
	# print("Connect status:",CON_STR[state])

	# if (state == dType.DobotConnect.DobotConnect_NoError):

	#     #Clean Command Queued
	#     dType.SetQueuedCmdClear(api)

	#     #Async Motion Params Setting
	#     # dType.SetHOMEParams(api, 250, 0, 50, 0, isQueued = 1)
	#     # dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
	#     # dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)

	#     dType.SetHOMEParams(api, 250, 0, 50, 0, isQueued = 0)
	#     dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 0)
	#     dType.SetPTPCommonParams(api, 100, 100, isQueued = 0)

	#     #Async Home
	#     # dType.SetHOMECmd(api, temp = 0, isQueued = 1)
	#     dType.SetHOMECmd(api, temp = 0, isQueued = 0)


	#     dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 200 , 200, 50, 0, isQueued = 0)
	#     dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 150, 200, 50, 0, isQueued = 0)

	#     # #Async PTP Motion
	#     # for i in range(0, 5):
	#     #     if i % 2 == 0:
	#     #         offset = 50
	#     #     else:
	#     #         offset = -50
	#     #     lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 200 , offset, offset, offset, isQueued = 1)[0]

	#     # Start to Execute Command Queued
	#     dType.SetQueuedCmdStartExec(api)

	#     # # Wait for Executing Last Command 
	#     # while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
	#     #     dType.dSleep(100)

	#     # Stop to Execute Command Queued
	#     dType.SetQueuedCmdStopExec(api)
	#     #Disconnect Dobot
	#     dType.DisconnectDobot(api)



if __name__ == "__main__":
    main()