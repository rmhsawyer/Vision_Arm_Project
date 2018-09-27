import numpy as np
import sys
import random

#try:
#    import thread
#except ImportError:
#    import _thread as thread

import threading
import os
import ctypes
from ctypes import *
dir_path = os.path.dirname(os.path.realpath(__file__))
lib = cdll.LoadLibrary(dir_path + '/build/librealsensedetection.so')
t1 = None
from numpy.ctypeslib import ndpointer

class CameraThread (threading.Thread):
	def __init__(self, threadID, name):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
	def run(self):
		lib.Run();

class Coordinates():
	def __init__(self,numberofpoints,redcoordinates, greencoordinates,redangles,greenangles):
		self.numberofpoints = numberofpoints
		redcoordinateslist = []
		greencoordinatelist = []
		redanglelist = []
		greenanglelist = []

		if(numberofpoints[0] != 0):
			for i in range(0,numberofpoints[0]):
				redcoordinateslist.append([redcoordinates[i*3],redcoordinates[i*3+1],redcoordinates[i*3+2]])
			for i in range(0,numberofpoints[0]):
				redanglelist.append(redangles[i])
				
		if(numberofpoints[1] != 0):
			for i in range(0,numberofpoints[1]):
				greencoordinatelist.append([greencoordinates[i*3],greencoordinates[i*3+1],greencoordinates[i*3+2]])
			for i in range(0,numberofpoints[1]):
				greenanglelist.append(greenangles[i])

		self.redcoordinates = redcoordinateslist
		self.greencoordinates = greencoordinatelist
		self.redangles = redanglelist;
		self.greenangles = greenanglelist;

	def __getredcoordinate__(self, key):
		try:
			return self.redcoordinates[key]
		except:
			IndexError 
			return 0

	def __getgreencoordinate__(self, key):
		try:
			return self.greencoordinates[key]
		except:
			IndexError 
			return 0


def init():
	thread1 = CameraThread(1, "camera_thread")
	thread1.start()

def GetPoints():
	"""Get object coordinates X, Y, Z """
	lib.GetCentralPoint.restype = ctypes.POINTER(ctypes.c_float * 3)

	pos = np.zeros((3), dtype=np.float)

	pospointer = lib.GetCentralPoint().contents

	for i in range(0, 3):
		pos[i] = pospointer[i]

	return pos

def GetNumOfPoints():
	"""Get Number of Red, Green, Blue Objects """
	lib.GetNumOfPoints.restype = ctypes.POINTER(ctypes.c_int * 3)

	Number = np.zeros((3), dtype=np.int)

	pospointer = lib.GetNumOfPoints().contents

	for i in range(0, 3):
		Number[i] = pospointer[i]

	return Number


def GetRedAngles():
	"""Get Red objects coordinates X, Y, Z """
	Number = GetNumOfPoints()
	lib.GetRedAngles.restype = ctypes.POINTER(ctypes.c_float * (Number[0]))

	angle = np.zeros((Number[0]), dtype=np.float)

	anglepointer = lib.GetRedAngles().contents
	for i in range(0,Number[0]):
		angle[i] = anglepointer[i]

	return angle

def GetGreenAngles():
	"""Get Red objects coordinates X, Y, Z """
	Number = GetNumOfPoints()
	lib.GetGreenAngles.restype = ctypes.POINTER(ctypes.c_float * (Number[1]))

	angle = np.zeros((Number[1]), dtype=np.float)

	anglepointer = lib.GetGreenAngles().contents
	for i in range(0,Number[1]):
		angle[i] = anglepointer[i]

	return angle



def GetRedPoints():
	"""Get Red objects coordinates X, Y, Z """
	Number = GetNumOfPoints()
	lib.GetRedCentralPoint.restype = ctypes.POINTER(ctypes.c_float * (Number[0]*3))

	pos = np.zeros((3*Number[0]), dtype=np.float)

	pospointer = lib.GetRedCentralPoint().contents
	for i in range(0,3*Number[0]):
		pos[i] = pospointer[i]

	return pos



def GetGreenPoints():
	"""Get Green object coordinates X, Y, Z """
	Number = GetNumOfPoints()
	lib.GetGreenCentralPoint.restype = ctypes.POINTER(ctypes.c_float * (Number[1]*3))

	pos = np.zeros((3*Number[1]), dtype=np.float)

	pospointer = lib.GetGreenCentralPoint().contents
	for i in range(0,3*Number[1]):
		pos[i] = pospointer[i]

	return pos



def GetIntrinsicsParmameters():
	""" Get camera intrinsics parameters: width, height, fx, fy, centralpoint X, centralpoint Y"""
	lib.GetIntrinsics.restype = ctypes.POINTER(ctypes.c_float * 6)
	Intrinsics = np.zeros((6), dtype = np.float)
	Intrinsicspointer = lib.GetIntrinsics().contents

	for i in range(0,6):
		Intrinsics[i] = Intrinsicspointer[i]

	return Intrinsics 




def GetLeftPlaneCentralPoint():
	"""Get object coordinates X, Y, Z """
	lib.GetLeftPlaneCentralPoint.restype = ctypes.POINTER(ctypes.c_float * 3)

	pos = np.zeros((3), dtype=np.float)

	pospointer = lib.GetLeftPlaneCentralPoint().contents

	for i in range(0, 3):
		pos[i] = pospointer[i]

	return pos



def GetRightPlaneCentralPoint():
	"""Get object coordinates X, Y, Z """
	lib.GetRightPlaneCentralPoint.restype = ctypes.POINTER(ctypes.c_float * 3)

	pos = np.zeros((3), dtype=np.float)

	pospointer = lib.GetRightPlaneCentralPoint().contents

	for i in range(0, 3):
		pos[i] = pospointer[i]

	return pos


def GetLeftPlaneAngle():
	"""Get object coordinates X, Y, Z """
	lib.GetLeftPlaneAngle.restype = ctypes.POINTER(ctypes.c_float * 1)

	angle = np.zeros((1), dtype=np.float)

	anglepointer = lib.GetLeftPlaneAngle().contents

	for i in range(0, 1):
		angle[i] = anglepointer[i]

	return angle


def GetRightPlaneAngle():
	"""Get object coordinates X, Y, Z """
	lib.GetRightPlaneAngle.restype = ctypes.POINTER(ctypes.c_float * 1)

	angle = np.zeros((1), dtype=np.float)

	anglepointer = lib.GetRightPlaneAngle().contents

	for i in range(0, 1):
		angle[i] = anglepointer[i]

	return angle





