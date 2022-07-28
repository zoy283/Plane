# !/usr/bin/python
# -*- coding: utf-8 -*-
import math
from math import pi
import pyrealsense2 as rs
import serial
import numpy as np
import time as time1
from threading import Thread
import threading
import csv
import DataDeal265 as dd
import struct
import cv2
import imutils
import OpenVision as ov
import RPi.GPIO as GPIO

import sys
from color_shape import LastButNotLeast
from find_cam import find_cam

CopterTakingOff = 1
TargetPosition = [0.0, 0.0, 0.0]
filename = 'router.txt'
args = sys.argv  #执行“python arg.py 1 5”，args=['arg.py', '1', '5']


def location_control(routeList, routeNodeIndex, pix_x, pix_y):
    time = 0.5
    limit = 5
    range = 5
    x = float(routeList[routeNodeIndex][0])
    y = float(routeList[routeNodeIndex][1])
    z = float(routeList[routeNodeIndex][2])
    if pix_x > 160 + limit:
        pix_x = 160 + limit
    if pix_x < 160 - limit:
        pix_x = 160 - limit
    if pix_y > 120 + limit:
        pix_y = 120 + limit
    if pix_y < 120 - limit:
        pix_y = 120 - limit
    x_new = x - (pix_x - 160.0) * z * 0.1125 / 0.304 / 100
    y_new = y + (pix_y - 120.0) * z * 0.1125 / 0.304 / 100
    if (pix_x < 160 + limit and pix_x > 160 + limit and pix_y < 120 + limit
            and pix_y > 120 - limit):
        routeList.insert(routeNodeIndex + 1, [x, y, z, 0.1, 0, 0, 2])
        routeList.insert(routeNodeIndex + 1, [x, y, 0.8, 5, 0, 0, 0])
    else:
        routeList.insert(routeNodeIndex + 1, [x_new, y_new, z, time, 0, 0, 1])
    return routeList


def detect():
    global x_pix
    global y_pix
    global color
    global shape
    roi = [80, 100]
    while (True):
        ret, frame = cap.read()
        framecopy = frame.copy
        frame, color, shape, x_pix, y_pix = LastButNotLeast(
            frame, framecopy, roi)
        ov.show(frame)


def addRouter(routeList, routeNodeIndex, point):
    pointList = [[-0.5, -2.75], [-2, -1.25], [-2.75, -2], [-3.5, .25],
                 [-3.5, -.75], [-2.75, -.5], [-1.25, -.5], [-1.25, -2],
                 [-.5, -1.25], [-2, .25], [-2, -2.75], [-3.5, -1.25]]
    target_x1 = pointList[point - 1][0]
    target_y1 = pointList[point - 1][1]

    X = float(routeList[routeNodeIndex][0])
    Y = float(routeList[routeNodeIndex][1])
    Z = float(routeList[routeNodeIndex][2])
    time = 1

    symbol = 1
    if target_x1 - X < 0:
        symbol = -1
    dis_x = symbol * (target_x1 - X)
    count_1 = dis_x // 0.5
    for i in range(int(count_1)):
        X = X + symbol * 0.5
        routeList.insert(routeNodeIndex + i + 1, [X, Y, Z, 0, 0, 0, 0])
    X = X + symbol * (dis_x % 0.5)
    routeList.insert(routeNodeIndex + int(count_1) + 1, [X, Y, Z, 1, 0, 0, 0])

    if target_y1 - Y < 0:
        symbol = -1
    else:
        symbol = 1
    dis_y = symbol * (target_y1 - Y)
    count_2 = dis_y // 0.5
    for i in range(int(count_2)):
        Y = Y + symbol * 0.5
        routeList.insert(routeNodeIndex + int(count_1) + i + 2,
                         [X, Y, Z, 2, 0, 0, 0])
    Y = Y + symbol * (dis_y % 0.5)
    routeList.insert(routeNodeIndex + int(count_2) + int(count_1) + 2,
                     [X, Y, Z, 3, 0, 0, 1])

    return routeNodeIndex + int(count_2) + int(count_1) + 2


#定时更新路径点
def Router(name):
    global timer
    global routeList
    global routeNodeIndex
    global routeNodeNum
    global SendTargetPos
    global CopterLanding
    global LaserArray
    global LaserDistance
    global FlightMode
    global CopterTakingOff
    global routeStartFlag
    global route_flag
    global barcodeData
    global x_pix
    global y_pix
    global color_flagz
    global area
    global color
    if routeNodeIndex < routeNodeNum and routeStartFlag == True:
        # 第25个点（26行）将要赋值给目标,目前在第24个点（25）行的位置上
        if routeNodeIndex > 1 and FlightMode == 1:
            location_control(routeList, routeNodeIndex - 1, x_pix, y_pix)
            routeNodeNum = len(routeList)
        elif FlightMode == 2:
            port = serial.Serial(port="/dev/ttyUSB0",
                                 baudrate=115200,
                                 stopbits=1,
                                 parity=serial.PARITY_NONE,
                                 timeout=1000)
            port.write("1".encode('UTF-8'))
        FlightMode = int(routeList[routeNodeIndex][6])
        TargetPosition[0] = float(routeList[routeNodeIndex][0])
        TargetPosition[1] = float(routeList[routeNodeIndex][1])
        TargetPosition[2] = float(routeList[routeNodeIndex][2])
        time = float(routeList[routeNodeIndex][3])
        LaserArray = int(routeList[routeNodeIndex][4])
        LaserDistance = float(routeList[routeNodeIndex][5])

        print(
            "route node %d: x : %.2f , y : %.2f , z : %.2f , time : %.1f s ,Arrray : %d , Dis : %.1f , S : %d"
            % (routeNodeIndex, TargetPosition[0], TargetPosition[1],
               TargetPosition[2], time, LaserArray, LaserDistance, FlightMode))
        SendTargetPos = 1
        routeNodeIndex = routeNodeIndex + 1
        timer = threading.Timer(time, Router, ["Router"])
        timer.start()
    else:
        #SendTargetPos = 0
        CopterTakingOff = 1
        CopterLanding = 1
        routeNodeIndex = 1
        print("Landing")
        timer.cancel()


# 字符串对比
def StrComparison(str1, str2):
    n = len(str1)
    res = []
    for x in str1:
        if x in str2:
            res.append(x)
    #print (n)
    return (n - len(res))


#串口通信线程
def PortCom1(port):
    global pipe
    global cfg
    global SendTargetPos
    global CopterLanding
    global CopterTakingOff
    global _265Ready
    global GetOnceCmd
    global routeNodeIndex
    global routeStartFlag

    while (True):
        #         size=port.inWaiting()
        #         if(size!=0):
        response = port.readline()
        print(response)
        if (response != None):
            port.flushInput()
            CmdStr1 = str(b'Start265\n')
            CmdStr2 = str(b'Departures\n')
            CmdStr3 = str(b'Refresh265\n')
            CMD = str(response)
            #刷新265
            if ((StrComparison(CMD, CmdStr1) <= 1) and GetOnceCmd == False):
                print(StrComparison(CMD, CmdStr1), response, CMD)
                # Declare RealSense pipeline, encapsulating the actual device and sensors
                pipe = rs.pipeline()
                #                 try:
                #                     pipe.stop()
                #                 except:
                #                     print("Error1")
                # Build config object and request pose data
                cfg = rs.config()
                cfg.enable_stream(rs.stream.pose, rs.format.any, framerate=200)
                # Start streaming with requested config
                pipe.start(cfg)
                dd.initData()
                SendTargetPos = 0
                CopterLanding = 0
                _265Ready = True
                GetOnceCmd = True
                routeStartFlag = True

            elif ((StrComparison(CMD, CmdStr2) <= 1) and CopterTakingOff == 1):
                print(StrComparison(CMD, CmdStr2), response, CMD)
                print("Get!")
                Router("first")
                CopterTakingOff = 0

            elif (StrComparison(CMD, CmdStr3) <= 1):
                _265Ready = False
                GetOnceCmd = False
                routeNodeIndex = 1
                CopterTakingOff = 1
                routeStartFlag = False
                print("ReStart!")
                print(StrComparison(CMD, CmdStr3), response, CMD)
                try:
                    pipe.stop()
                    time1.sleep(1.0)
                except:
                    print("Error2")
            response = 0
            CMD = 0
        time1.sleep(0.02)


if __name__ == '__main__':
    global routeList
    global routeNodeIndex
    global routeNodeNum
    global SendTargetPos
    global CopterLanding
    global LaserArray
    global LaserDistance
    global FlightMode
    global pipe
    global _265Ready
    global GetOnceCmd
    global CheckSum
    #-------------测试------------
    global color_flag
    global route_flag
    global x_point
    global y_point
    global barcodeData
    global countDrop
    global color
    #-----------------------------

    port_1 = serial.Serial(port="/dev/ttyAMA0",
                           baudrate=230400,
                           stopbits=1,
                           parity=serial.PARITY_NONE,
                           timeout=1000)
    port_2 = serial.Serial(port="/dev/ttyUSB0",
                           baudrate=115200,
                           stopbits=1,
                           parity=serial.PARITY_NONE,
                           timeout=1000)

    ov.init()
    board_cam = find_cam(b"mmal service 16.1 (platform:bcm2835-v4l2):")
    front_cam = find_cam(b"USB Camera:  USB GS CAM (usb-3f980000.usb-1.1.3):")
    cap = cv2.VideoCapture(board_cam)
    cap.set(3, 320)
    cap.set(4, 240)
    routeStartFlag = True
    #串口1通信线程
    thread_Serial = Thread(target=PortCom1, args=(port_1, ))
    thread_Serial.start()
    #摄像头
    thread_Laser = Thread(target=detect, args=())
    thread_Laser.start()
    #导入路径文件
    routeCsv = csv.reader(open(filename))
    routeList = list(routeCsv)
    routeNodeNum = len(routeList)
    routeNodeIndex = 1
    #插入指定路径点
    point_1 = int(args[1])
    point_2 = int(args[2])
    print(point_1, point_2)
    routeNodeIndex = addRouter(routeList, routeNodeIndex, point_1)
    routeNodeIndex = addRouter(routeList, routeNodeIndex, point_2)
    routeNodeNum = len(routeList)
    print(routeList)
    #输出路径点个数
    print("route nodes num is : " + str(routeNodeNum - 1))
    routeNodeIndex = 1
    _265Ready = False
    GetOnceCmd = False
    CheckSum = 0
    dataBuf = [0] * 65
    qifei_flag = 1
    count = 0

    try:
        while (True):
            if _265Ready:
                # Wait for the next set of frames from the camera
                frames = pipe.wait_for_frames()
                # Fetch pose frame
                pose = frames.get_pose_frame()
                if pose:
                    # Print some of the pose data to the terminal
                    data = pose.get_pose_data()
                    dataBuf, pos_X, pos_Y, pos_Z, Euler = dd.solveData(data)
                    if (SendTargetPos == 1):
                        posX = TargetPosition[0]
                        posY = TargetPosition[1]
                        posZ = TargetPosition[2]

                        dataBuf[43] = 0x20
                        posX_buf = struct.pack("f", posX)
                        dataBuf[44] = posX_buf[0]
                        dataBuf[45] = posX_buf[1]
                        dataBuf[46] = posX_buf[2]
                        dataBuf[47] = posX_buf[3]
                        posY_buf = struct.pack("f", posY)
                        dataBuf[48] = posY_buf[0]
                        dataBuf[49] = posY_buf[1]
                        dataBuf[50] = posY_buf[2]
                        dataBuf[51] = posY_buf[3]
                        posZ_buf = struct.pack("f", posZ)
                        dataBuf[52] = posZ_buf[0]
                        dataBuf[53] = posZ_buf[1]
                        dataBuf[54] = posZ_buf[2]
                        dataBuf[55] = posZ_buf[3]

                        dataBuf[56] = LaserArray
                        Laser_Dis = struct.pack("f", LaserDistance)
                        dataBuf[57] = Laser_Dis[0]
                        dataBuf[58] = Laser_Dis[1]
                        dataBuf[59] = Laser_Dis[2]
                        dataBuf[60] = Laser_Dis[3]
                        if LaserDistance != 0:
                            LaserDistance = 0
                        dataBuf[61] = FlightMode

                    if CopterLanding == 1:
                        dataBuf[62] = 0xA5
                    else:
                        dataBuf[62] = 0x00

                    for i in range(0, 62):
                        CheckSum = CheckSum + dataBuf[i]

                    dataBuf[63] = 0xAA
                    dataBuf[64] = CheckSum & 0x00ff

                    print(
                        "\rrpy_rad[0]:{:.2f},rpy_rad[1]:{:.2f},rpy_rad[2]:{:.2f} ,X:{:.2f},Y:{:.2f},Z:{:.2f} "
                        .format(Euler[0] * 57.3, Euler[1] * 57.3,
                                Euler[2] * 57.3, pos_X, pos_Y, pos_Z),
                        end="")

                    if qifei_flag == 1:
                        GPIO.output(29, GPIO.LOW)
                        count = count + 1
                    if count == 200:
                        count = 0
                        qifei_flag = 0
                        dataBuf[0] = 0x55
                        dataBuf[1] = 0xAA
                        dataBuf[2] = 0x20
                        dataBuf[63] = 0xAA
                        dataBuf[64] = 0x00
                    port.write(dataBuf)
                    CheckSum = 0
#                     pipe.stop()
            else:
                dataBuf[0] = 0x55
                dataBuf[1] = 0xAA
                dataBuf[2] = 0xFF
                dataBuf[63] = 0xAA
                dataBuf[64] = 0x00
                port.write(dataBuf)
                time1.sleep(0.1)
    finally:
        print("some erro")
#         pipe.stop()