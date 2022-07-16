import csv
import math
from time import sleep
import time
import cv2
import numpy as np
from check_color import check_color
import imutils

def check_color(frame, color):
    x = 0
    y = 0
    width = 0
    color_flag=0
    area = 0
    color_range = [[156, 43, 46], [180, 255, 255], [41, 19, 22],
                   [96, 255, 255], [100, 43, 46], [124, 255, 255]]  # 红 绿 蓝
    frame = imutils.resize(frame, width=160)
    frame_new = frame
    hsv = cv2.cvtColor(
        frame_new, cv2.COLOR_BGR2HSV)  # opencv是以BGR格式读取图片的，所以要将得到的RGB值倒着输入。

    if color == "red":
        lower = np.array(color_range[0])
        upper = np.array(color_range[1])
    elif color == "green":
        lower = np.array(color_range[2])
        upper = np.array(color_range[3])
    else:
        lower = np.array(color_range[4])
        upper = np.array(color_range[5])

    mask_color = cv2.inRange(hsv, lower, upper)
    mask_color = cv2.medianBlur(mask_color, 3)  # ksize: 滤波模板的尺寸大小，必须是大于1的奇数
    cnt_color = cv2.findContours(mask_color.copy(), cv2.RETR_EXTERNAL,
                                 cv2.CHAIN_APPROX_SIMPLE)
    #  cv2.RETR_EXTERNAL     表示只检测外轮廓
    #   cv2.CHAIN_APPROX_SIMPLE     压缩水平方向，垂直方向，对角线方向的元素，只保留该方向的终点坐标
    cnt_color = cnt_color[1] if imutils.is_cv2() else cnt_color[0]
    cv2.rectangle(frame, (75, 55), (85, 65), (255, 0, 0), thickness=1)
    if len(cnt_color) > 0:
        area = [cv2.contourArea(i) for i in cnt_color]
        index = np.argmax(area)
        rect_red = cv2.minAreaRect(cnt_color[index])
        if (rect_red[1][0] > 0) and (rect_red[1][1] > 0):#   rect_red[1][1] height  rect_red[1][0] with rect[0][0], rect[0][1]中心点
            box_red = np.int0(cv2.boxPoints(rect_red))
            # box_red[0][0] = box_red[0][0] + 75
            # box_red[1][0] = box_red[1][0] + 75
            # box_red[2][0] = box_red[2][0] + 75
            # box_red[3][0] = box_red[3][0] + 75
            # box_red[0][1] = box_red[0][1] + 55
            # box_red[1][1] = box_red[1][1] + 55
            # box_red[2][1] = box_red[2][1] + 55
            # box_red[3][1] = box_red[3][1] + 55
            cv2.drawContours(frame, [box_red], 0, (0, 0, 255), 2)
            color_flag = 1
            x = rect_red[0][0]
            y = rect_red[0][1]
            width = rect_red[1][0]
            area = rect_red[1][0] * rect_red[1][1]
    else:
        color_flag = 0
    return frame,x,y,width,color_flag,area

def find_point(routeList, routeNodeIndex, x, y):
    global route_flag
    X = float(routeList[routeNodeIndex][0])
    Y = float(routeList[routeNodeIndex][1])
    Z = float(routeList[routeNodeIndex][2])
    distance = 0.01
    time = 0.5
    # 水平调整
    if abs(80 - x) > 5 :
        x_new = X + (80 - x)*distance
    if abs(60 - y) > 5 :
        y_new = Y + (y - 60)*distance
    routeList.insert(routeNodeIndex + 1, [x_new, y_new, Z, time, 0, 0, 0])

    return routeList


if __name__ == '__main__':
    global route_flag
    route_flag = 2
    routeCsv = csv.reader(open('router.txt'))
    routeList = list(routeCsv)
    routeNodeNum = len(routeList)
    #输出路径点个数
    print("route nodes num is : " + str(routeNodeNum - 1))
    routeNodeIndex = 1
    # cap = cv2.VideoCapture(0)
    # cap.set(3, 320)
    # cap.set(4, 240)
    # ret,frame = cap.read()
    frame = cv2.imread("point.png")
    cv2.namedWindow('frame', 1)
    cv2.resizeWindow("frame", 400, 200)
    frame, x, y, width, color_flagz, area = check_color(frame, "red")
    if color_flagz == 1:
        find_point(routeList, routeNodeIndex, x, y)
        routeNodeNum = len(routeList)
        print(str(routeList[routeNodeIndex+1]))

    cv2.namedWindow("input image", cv2.WINDOW_AUTOSIZE)
    cv2.imshow("frame", frame)
    cv2.waitKey(0)
