import csv
import math
from time import sleep
import time
import cv2
from check_color import check_color


def findStick(routeList, routeNodeIndex, x):
    global route_flag
    X = float(routeList[routeNodeIndex][0])
    Y = float(routeList[routeNodeIndex][1])
    Z = float(routeList[routeNodeIndex][2])

    time = 0.7
    minwidth = 20
    maxwidth = 20
    minY = 75  # 左右范围
    maxY = 85
    step = 0.2 # 步进
    # 水平调整
    if route_flag == 1:
        if x < minY:
            y_new = Y - step
        elif x > maxY:
            y_new = Y + step
        else:
            y_new = Y
        routeList.insert(routeNodeIndex + 1, [X, y_new, Z, time, 0, 0, 0])
    # # 前后调整
    # if route_flag == 2:
    #     if width < minwidth:
    #         x_new = x - 0.2
    #     elif width > maxwidth:
    #         x_new = x + 0.2
    #     routeList.insert(routeNodeIndex + 1, [x_new, Y, Z, time, 0, 0, 0])

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
        frame = cv2.imread("gree.png")
        frame, x, width, color_flagz,area = check_color(frame, "green")
        if color_flagz == 1:
            findStick(routeList, routeNodeIndex, x)
            routeNodeNum = len(routeList)
        
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break
        print("route nodes num is : " + str(routeList[routeNodeIndex]))
        print("area1 : " ,area)
        time.sleep(0.2)
