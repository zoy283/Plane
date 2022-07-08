import csv
import math
from time import sleep
import time
from turtle import color
import cv2
from check_color import check_color


def find_stick(routeList, routeNodeIndex, x, area):
    global route_flag
    global color
    X = float(routeList[routeNodeIndex][0])
    Y = float(routeList[routeNodeIndex][1])
    Z = float(routeList[routeNodeIndex][2])

    time = 0.5
    minY = 75  # 左右范围
    maxY = 85
    step = 0.05  # 左右步进
    # 水平调整
    if route_flag == 1:
        if x < minY:
            if minY - x > 20:
                step = 0.05
            else:
                step = 0.02
            y_new = Y - step
            routeList.insert(routeNodeIndex + 1, [X, y_new, Z, time, 0, 0, 0])
        elif x > maxY:
            if x - maxY > 20:
                step = 0.05
            else:
                step = 0.02
            y_new = Y + step
            routeList.insert(routeNodeIndex + 1, [X, y_new, Z, time, 0, 0, 0])
        else:
            route_flag == 2
            color ="red"
            # routeList.insert(routeNodeIndex + 1, [0, Y, Z, time, 0, 0, 0])
    return routeList


if __name__ == '__main__':
    global route_flag
    route_flag = 1
    routeCsv = csv.reader(open('router.txt'))
    routeList = list(routeCsv)
    routeNodeNum = len(routeList)
    #输出路径点个数
    print("route nodes num is : " + str(routeNodeNum - 1))
    routeNodeIndex = 1
    cap = cv2.VideoCapture(0)
    cap.set(3, 320)
    cap.set(4, 240)
    while True:
        # ret,frame = cap.read()
        frame = cv2.imread("green.png")
        cv2.namedWindow('frame', 1)
        cv2.resizeWindow("frame", 400, 200)
        frame, x, width, color_flagz, area = check_color(frame, "green")
        if color_flagz == 1:
            find_stick(routeList, routeNodeIndex, x, area)
            routeNodeNum = len(routeList)

        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break
        print("area1 : ", area, route_flag)
