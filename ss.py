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
from pyzbar import pyzbar
import argparse
from check_color import check_color

CopterTakingOff = 1
TargetPosition = [0.0, 0.0, 0.0]
filename = 'router.txt'


# def check_color(frame, color):
#     global color_flag
#     global x_point
#     global y_point
#     color_range = [[156, 43, 46], [180, 255, 255], [35, 43, 46],
#                    [77, 255, 255], [100, 43, 46], [124, 255, 255]]  # 红 绿 蓝
#     frame = imutils.resize(frame, width=160)
#     frame_new = frame[55:65, 75:85]
#     hsv = cv2.cvtColor(
#         frame_new, cv2.COLOR_BGR2HSV)  # opencv是以BGR格式读取图片的，所以要将得到的RGB值倒着输入。

#     if color == "red":
#         lower = np.array(color_range[0])
#         upper = np.array(color_range[1])
#     elif color == "green":
#         lower = np.array(color_range[2])
#         upper = np.array(color_range[3])
#     else:
#         lower = np.array(color_range[4])
#         upper = np.array(color_range[5])

#     mask_color = cv2.inRange(hsv, lower, upper)
#     mask_color = cv2.medianBlur(mask_color, 3)  # ksize: 滤波模板的尺寸大小，必须是大于1的奇数
#     cnt_color = cv2.findContours(mask_color.copy(), cv2.RETR_EXTERNAL,
#                                  cv2.CHAIN_APPROX_SIMPLE)
#     #  cv2.RETR_EXTERNAL     表示只检测外轮廓
#     #   cv2.CHAIN_APPROX_SIMPLE     压缩水平方向，垂直方向，对角线方向的元素，只保留该方向的终点坐标
#     cnt_color = cnt_color[0] if imutils.is_cv2() else cnt_color[1]
#     cv2.rectangle(frame, (75, 55), (85, 65), (255, 0, 0), thickness=1)
#     if len(cnt_color) > 0:
#         area = [cv2.contourArea(i) for i in cnt_color]
#         index = np.argmax(area)
#         rect_red = cv2.minAreaRect(cnt_color[index])
#         if (rect_red[1][0] > 5) and (rect_red[1][1] > 5):
#             box_red = np.int0(cv2.boxPoints(rect_red))
#             box_red[0][0] = box_red[0][0] + 75
#             box_red[1][0] = box_red[1][0] + 75
#             box_red[2][0] = box_red[2][0] + 75
#             box_red[3][0] = box_red[3][0] + 75
#             box_red[0][1] = box_red[0][1] + 55
#             box_red[1][1] = box_red[1][1] + 55
#             box_red[2][1] = box_red[2][1] + 55
#             box_red[3][1] = box_red[3][1] + 55
#             cv2.drawContours(frame, [box_red], 0, (0, 0, 255), 2)
#             color_flag = 1
#     else:
#         color_flag = 0
#     return frame


def detect():
    global x_pix
    global color_flagz
    global area
    
    while (True):
        ret, frame = cap.read()
        frame, x_pix, width, color_flagz,area=check_color(frame, "green")
#         ov.show(frame)


def laser_init():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(10, GPIO.OUT)
    GPIO.output(10, GPIO.LOW)
    GPIO.setup(29, GPIO.OUT)
    GPIO.output(29, GPIO.HIGH)


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

def findStick(routeList, routeNodeIndex, x, area):
    global route_flag
    X = float(routeList[routeNodeIndex][0])
    Y = float(routeList[routeNodeIndex][1])
    Z = float(routeList[routeNodeIndex][2])

    time = 0.5
    minarea = 1400
    maxarea = 1800
    minY = 75  # 左右范围
    maxY = 85
    step = 0.05 # 左右步进
    step1 = 0.05 # 前后步进
    # 水平调整
    if route_flag == 1:
        if x < minY:
            if minY-x>20:
                step=0.05
            else:
                step=0.02
            y_new = Y - step
            routeList.insert(routeNodeIndex + 1, [X, y_new, Z, time, 0, 0, 0])
        elif x > maxY:
            if x-maxY>20:
                step=0.05
            else:
                step=0.02
            y_new = Y + step
            routeList.insert(routeNodeIndex + 1, [X, y_new, Z, time, 0, 0, 0])
        else :
            route_flag == 1
    # 前后调整
    if route_flag == 2:
        if area < minarea:
            x_new = X - step1
            routeList.insert(routeNodeIndex + 1, [x_new, Y, Z, time, 0, 0, 0])           
        elif area > maxarea:
            x_new = X + step1
            routeList.insert(routeNodeIndex + 1, [x_new, Y, Z, time, 0, 0, 0])

    return routeList
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


def addCircleRoute(routeList, routeNodeIndex, radius, direction):
    radianList = [
        1 / 6 * pi,
        2 / 6 * pi,
        3 / 6 * pi,
        4 / 6 * pi,
        5 / 6 * pi,
        pi,
        7 / 6 * pi,
        8 / 6 * pi,
        10 / 6 * pi,
        11 / 6 * pi,
    ]
    x = float(routeList[routeNodeIndex][0])
    y = float(routeList[routeNodeIndex][1])
    z = float(routeList[routeNodeIndex][2])
    # 生成顺时针路径
    if (direction == 'clock'):
        for i in range(len(radianList)):
            x_new = round(x + radius * (-1 + math.cos(radianList[i])), 2)
            y_new = round(y + radius * (math.sin(radianList[i])), 2)
            routeList.insert(routeNodeIndex + 1, [x_new, y_new, z, 2, 0, 0, 0])
    # 生成逆时针路径
    elif (direction == 'cntclock'):
        for i in range(len(radianList)):
            x_new = round(
                x + radius *
                (-1 + math.cos(radianList[len(radianList) - 1 - i])), 2)
            y_new = round(
                y + radius * (math.sin(radianList[len(radianList) - 1 - i])),
                2)
            routeList.insert(routeNodeIndex + 1, [x_new, y_new, z, 2, 0, 0, 0])
    # print(circleList)
    routeList.insert(routeNodeIndex + len(radianList)+1, [x, y, z, 2, 0, 0, 0])
    return routeList


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

#         if (routeNodeIndex == 2):
#             routeList = addCircleRoute(routeList, routeNodeIndex, 0.7, 'clock')
#             print("\nafter add: " + str(routeList))
#             routeNodeNum = len(routeList)
        if (routeNodeIndex>1):
            if color_flagz == 1:
                findStick(routeList, routeNodeIndex, x_pix,area)
                routeNodeNum = len(routeList)

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
    #-----------------------------

    port = serial.Serial(port="/dev/ttyAMA0",
                         baudrate=230400,
                         stopbits=1,
                         parity=serial.PARITY_NONE,
                         timeout=1000)
    laser_init()
    ov.init()
    cap = cv2.VideoCapture(0)
    cap.set(3, 320)
    cap.set(4, 240)
    route_flag = 1  #按路径飞则True
    routeStartFlag = True
    barcodeData = 0

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
                        count = count + 1
                        print(count)
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