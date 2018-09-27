import DobotDllType as dType
import threading
import cv2 as cv
import numpy as np 
from numpy import *
import python_vision
import gevent
from gevent import socket

height = 480
width = 640

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

#Load Dll
api = dType.load()
calibrationdata=[]
armcoordinates = []
rotation = []
translation = []
count = 0

# ok
# [[-2.64384674e-02 -7.40883742e-01  1.84169445e+00]
#  [-7.62176425e-01  2.60853795e-02  6.40229564e-01]
#  [ 1.41739050e-03  2.61182162e-03 -2.02362539e-01]] [[143.27534239]
#  [170.2307903 ]
#  [ 23.77720599]]





def Camera_Calibration(data):
    
    # get solvePnP parameters       
    coordinates = data[:,[0,1,2]]
    cameracoordinates = data[:,[3,4]]
    intrinsics = python_vision.GetIntrinsicsParmameters()
    intrinsicsmatrix = np.array([[intrinsics[2], 0 ,intrinsics[4]],
                                    [0 ,intrinsics[3],intrinsics[5]],
                                    [0,0,1]])
    dist_coef = np.zeros(4)

    # get transformation matrix
    _ret, rvec, tvec = cv.solvePnP(coordinates, cameracoordinates, intrinsicsmatrix, dist_coef)
    rotation, jacobian= cv.Rodrigues(rvec, jacobian= 0)
    translation = tvec

    return rotation,translation

def Get_Coordinates(x,y,z,rotation,translation):
    """convert camera coordinates to 3D camera coordinates, z represents  camera focal length"""
    rotation = mat(rotation)
    translation = mat(translation)
    try:
        if x != 0.0 and y != 0.0 and z != -1000:
            cordinates = mat([[x],[y],[z]]) 
            new_cordinates = rotation.I * (cordinates - translation)
            return new_cordinates
    except ValueError:
        print("Calibration points error!")




def Get_Calibration_data(calibrationdata,data):

    for i in range(0,3):
        calibrationdata.append(data[i])
    print(calibrationdata)
    return calibrationdata



def Get_Calibration_Points(calibrationdata,armcoordinates):
    """Collect arm and camera 3D coordinates"""

    numofpoints = 30
    data = np.zeros((numofpoints,6))
    for i in range(numofpoints):

        if i == 0:
            print("Start Calibration Process. Total points: %d" %numofpoints)
        print("Please move the object to the point %d and hit enter to record" % (i+1))
        cameracoordinates = [calibrationdata[i*3],calibrationdata[i*3+1],calibrationdata[i*3+2]]
        # cameracoordinates = python_vision.GetRedPoints()
        print("The camera coordinates are: ",cameracoordinates)
        print("Please move the arm to the object and hit enter")
        print("The arm coordinates are: ",armcoordinates[i*3],armcoordinates[i*3+1],armcoordinates[i*3+2])
        print("\r")

        data[i,[0,1,2]] = armcoordinates[i*3],armcoordinates[i*3+1],armcoordinates[i*3+2]
        data[i,[3,4,5]] = cameracoordinates[0],cameracoordinates[1],cameracoordinates[2]

    return data



def CameraToArm_Affine_Transformation(data):
    """Get affine transformation matrix"""
    Arm_Coordinates = data[:,[0,1,2]]
    Camera_Coordinates = data[:,[3,4,5]]
    retval, affine, inlier = cv.estimateAffine3D(Arm_Coordinates,Camera_Coordinates)
    try:
        if retval == 1:
            rotation = affine[:,[0,1,2]]
            translation = affine[:,[3]]
            return rotation,translation

    except ValueError:
        print("Calibration Points not correct. Please try again")
        pass



def server(port):
    try:
        s = socket.socket()
        s.bind(('0.0.0.0',port))
        s.listen(500)
        while True:
            cli, addr = s.accept()
            gevent.spawn(handle_request, cli)
    except Exception as ex:
        print(ex)

def handle_request(conn):
    count = 1
    try:
        while True:
            
            data = conn.recv(1024)
            if not data:
                print("No iphone detected")
                # conn.shutdown(socket.SHUT_RD)
                # conn.close()

            else:
                data = bytes.decode(data)
                data = data.split(',')
                data = list(map(float, data))
                print(data)
                print("Enter command: 1: To save calibration data; 2: To start Calculation; 3: To start manipulate")
                command = input()
                if command == '1':
                    print("Current data point: %d" %count)
                    count += 1
                    for i in range(8,11):
                        calibrationdata.append(data[i])
                    print(calibrationdata)
                    armdata = dType.GetPose(api)
                    for i in range(0,3):
                        armcoordinates.append(armdata[i])
                    print(armcoordinates)
                if command == '2':
                    data1 = Get_Calibration_Points(calibrationdata,armcoordinates)
                    print(data1)
                    rotation,translation = CameraToArm_Affine_Transformation(data1)
                    print(rotation,translation)
                if command == '3':
                    # rotation =  [[ 1.20579102e-02 ,-7.36993647e-01, 4.42460274e+00],
                    # [-7.46837788e-01 , 1.53891244e-02 ,-2.19670624e+00],
                    # [ 4.50816195e-04 , 4.13577669e-03 , 1.16035733e-01]]
                    # translation = [[331.11519432],[-47.19413349],[ 47.68995806]]



                    new_Redcoordinates = Get_Coordinates(data[8],data[9],data[10],rotation,translation)
                    new_Leftcoordinates = Get_Coordinates(data[0],data[1],data[2],rotation,translation)
                    new_Pinkcoordinates = Get_Coordinates(data[12],data[13],data[14],rotation,translation)
                    new_Rightcoordinates = Get_Coordinates(data[4],data[5],data[6],rotation,translation)
                    Diff_Red = data[11]-data[3]
                    Diff_Pink = data[15]-data[7]
                    print(new_Redcoordinates)
                    print(new_Pinkcoordinates)
                    print(new_Leftcoordinates)
                    print(new_Rightcoordinates)
                    print(Diff_Red)
                    print(Diff_Pink)
                    dType.SetQueuedCmdClear(api)
                    dType.SetHOMEParams(api, 250, 0, 50, 0, isQueued = 0)
                    dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 0)
                    dType.SetPTPCommonParams(api, 100, 100, isQueued = 0)
                    dType.SetPTPJumpParams(api, 50, 60, isQueued=0)
                    
                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, new_Redcoordinates.item(0) , new_Redcoordinates.item(1), 40 , 0, isQueued = 1)
                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, new_Redcoordinates.item(0) , new_Redcoordinates.item(1), new_Redcoordinates.item(2) , 0, isQueued = 1)

                    dType.SetEndEffectorSuctionCup(api, 1,1,isQueued = 1)
                    dType.SetWAITCmd(api, 1, isQueued=1)
                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, new_Redcoordinates.item(0) , new_Redcoordinates.item(1), 40 , Diff_Red, isQueued = 1)      
                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, new_Leftcoordinates.item(0), new_Leftcoordinates.item(1), 40, Diff_Red, isQueued = 1)
                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, new_Leftcoordinates.item(0), new_Leftcoordinates.item(1), new_Leftcoordinates.item(2)+10, Diff_Red, isQueued = 1)
                    dType.SetEndEffectorSuctionCup(api, 1,0,isQueued = 1)
                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, new_Leftcoordinates.item(0), new_Leftcoordinates.item(1), 40, 0, isQueued = 1)
                    # dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 200, 200, 50, 0, isQueued = 1)


                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, new_Pinkcoordinates.item(0) , new_Pinkcoordinates.item(1), 40 , 0, isQueued = 1)
                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, new_Pinkcoordinates.item(0) , new_Pinkcoordinates.item(1), new_Pinkcoordinates.item(2) , 0, isQueued = 1)

                    dType.SetEndEffectorSuctionCup(api, 1,1,isQueued = 1)
                    dType.SetWAITCmd(api, 1, isQueued=1)
                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, new_Pinkcoordinates.item(0) , new_Pinkcoordinates.item(1), 40 , Diff_Pink, isQueued = 1)      
                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, new_Rightcoordinates.item(0), new_Rightcoordinates.item(1), 40, Diff_Pink, isQueued = 1)
                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, new_Rightcoordinates.item(0), new_Rightcoordinates.item(1), new_Rightcoordinates.item(2)+10, Diff_Pink, isQueued = 1)
                    dType.SetEndEffectorSuctionCup(api, 1,0,isQueued = 1)
                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, new_Rightcoordinates.item(0), new_Rightcoordinates.item(1), 40, 0, isQueued = 1)
                    dType.SetHOMECmd(api, temp = 0, isQueued = 1)
                    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 150, 0, -10, 0, isQueued = 1)[0]


                    dType.SetQueuedCmdStartExec(api)
                    
                    while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
                        dType.dSleep(100)
                    dType.SetQueuedCmdStopExec(api)
                    # dType.DisconnectDobot(api)


    except OSError as e:
        print("client has been closed")
 
    except Exception as ex:
        print(ex)
    finally:
        conn.close()




def main():

    state = dType.ConnectDobot(api, "", 115200)[0]
    print("Connect status:",CON_STR[state])

    server(8002)

    # if (state == dType.DobotConnect.DobotConnect_NoError):
    #     pose = dType.GetPose(api)
    #     print(pose)
    #     #Clean Command Queued
    #     dType.SetQueuedCmdClear(api)

    #     #Async Motion Params Setting
    #     dType.SetHOMEParams(api, 150, 0, -10, 0, isQueued = 1)
    #     dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
    #     dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)
    #     dType.SetPTPJumpParams(api, 50, 70, isQueued=1)
    #     # dType.SetHOMEParams(api, 250, 0, 50, 0, isQueued = 0)
    #     # dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 0)
    #     # dType.SetPTPCommonParams(api, 100, 100, isQueued = 0)
    #     #Async Home
    #     dType.SetHOMECmd(api, temp = 0, isQueued = 1)
    #     # dType.SetHOMECmd(api, temp = 0, isQueued = 0)


    #     dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 144 , 0, -20, 0, isQueued = 1)
        
        
    #     # dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, -50, 200, -30, 0, isQueued = 1)
    #     # dType.SetEndEffectorSuctionCup(api, 1,1,isQueued = 1)
    #     # dType.SetWAITCmd(api, 3, isQueued=1)
    #     # dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, -50 , 200, 50, 0, isQueued = 1)
    #     # dType.SetEndEffectorSuctionCup(api, 0,0,isQueued = 1)
    #     # dType.SetWAITCmd(api, 3, isQueued=1)
    #     # lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 180, 200, 50, 0, isQueued = 1)[0]
       

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
    #     pose = dType.GetPose(api)
    #     print(pose)

    #     #Disconnect Dobot
    #     dType.DisconnectDobot(api)

if __name__ == "__main__":
    main()
