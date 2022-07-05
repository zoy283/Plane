# !/usr/bin/python
# -*- coding: utf-8 -*-
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
from pyzbar import pyzbar
import argparse
from check_color import check_color

from add_round_route import add_round_route
from find_cam import find_cam
from find_stick import find_stick

CopterTakingOff = 1
TargetPosition = [0.0, 0.0, 0.0]
filename = 'router.txt'
front_cam = find_cam(b"USB Camera:  USB GS CAM (usb-3f980000.usb-1.1.3):")
board_cam = find_cam(b"mmal service 16.1 (platform:bcm2835-v4l2):")


def detect():
    global x_pix
    global color_flagz
    global area
    judgeCount = 0
    while (True):
        ret, frame = cap.read()
        frame, x_pix, width, color_flagz, area = check_color(frame, "green")
        ov.show(frame)


def laser_init():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(29, GPIO.OUT)
    GPIO.output(29, GPIO.HIGH)
    GPIO.setup(40, GPIO.OUT)
    GPIO.output(40, GPIO.LOW)


# LED闪烁函数 number闪烁次数 color灯光颜色
def flicker(number, color):
    for i in range(number):
        if (color == 'red'):
            GPIO.output(28, GPIO.LOW)
            time1.sleep(0.5)
            GPIO.output(28, GPIO.HIGH)
            time1.sleep(0.5)
        elif (color == 'green'):
            GPIO.output(29, GPIO.LOW)
            time1.sleep(0.5)
            GPIO.output(29, GPIO.HIGH)
            time1.sleep(0.5)


def decode():
    global barcodeData
    cap = cv2.VideoCapture(1)
    cap.set(3, 320)
    cap.set(4, 240)
    flag = 1
    # 加载输入图像
    while (True):
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        barcodes = pyzbar.decode(frame)
        for barcode in barcodes:
            # 提取二维码的位置,然后用边框标识出来在视频中
            (x, y, w, h) = barcode.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            imgRoi = frame[y:y + h, x:x + w]
            cv2.imwrite("5_30.jpg", imgRoi)
            # 字符串转换
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type
            if flag == 1:
                flicker(int(barcodeData))
                flag = 0
            # 在图像上面显示识别出来的内容
            text = "{}".format(barcodeData)
            cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 255, 0), 2)
            # 打印识别后的内容


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
    global color_flagz
    global area

    if routeNodeIndex < routeNodeNum and routeStartFlag == True:
        # 第25个点（26行）将要赋值给目标,目前在第24个点（25）行的位置上

        if (routeNodeIndex > 1):
            if color_flagz == 1:
                find_stick(routeList, routeNodeIndex, x_pix, area)
                print("area: ", area)
                routeNodeNum = len(routeList)
            if route_flag == 3:
                routeList = add_round_route(routeList, routeNodeIndex, 0.5,
                                            'clock')
                routeNodeNum = len(routeList)
                route_flag = 4
        TargetPosition[0] = float(routeList[routeNodeIndex][0])
        TargetPosition[1] = float(routeList[routeNodeIndex][1])
        TargetPosition[2] = float(routeList[routeNodeIndex][2])
        time = float(routeList[routeNodeIndex][3])
        LaserArray = int(routeList[routeNodeIndex][4])
        LaserDistance = float(routeList[routeNodeIndex][5])
        FlightMode = int(routeList[routeNodeIndex][6])

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
def PortCom(port):
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
    #-----------------------------

    port = serial.Serial(port="/dev/ttyAMA0",
                         baudrate=230400,
                         stopbits=1,
                         parity=serial.PARITY_NONE,
                         timeout=1000)
    laser_init()
    ov.init()
    cap = cv2.VideoCapture(front_cam)
    cap.set(3, 320)
    cap.set(4, 240)
    route_flag = 1
    routeStartFlag = True
    barcodeData = 0
    countDrop = 0
    #串口通信线程
    thread_Serial = Thread(target=PortCom, args=(port, ))
    thread_Serial.start()
    #     #激光
    thread_Laser = Thread(target=detect, args=())
    thread_Laser.start()
    #导入路径文件
    routeCsv = csv.reader(open(filename))
    routeList = list(routeCsv)
    routeNodeNum = len(routeList)
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