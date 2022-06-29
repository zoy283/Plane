import csv
from math import pi
import math


def findStick(routeList, routeNodeIndex, x, width):
    global route_flag
    X = float(routeList[routeNodeIndex][0])
    Y = float(routeList[routeNodeIndex][1])
    Z = float(routeList[routeNodeIndex][2])

    time = 0.7
    minwidth = 20
    maxwidth = 20
    minY = 70
    maxY = 90
    # 水平调整
    if route_flag == 1:
        if x < minY:
            y_new = X - 0.2
        elif x > maxY:
            y_new = X + 0.2
        else:
            route_flag = 2
        routeList.insert(routeNodeIndex + 1, [X, y_new, Z, time, 0, 0, 0])
    # 前后调整
    if route_flag == 2:
        if width < minwidth:
            x_new = x - 0.2
        elif width > maxwidth:
            x_new = x + 0.2
        routeList.insert(routeNodeIndex + 1, [x_new, Y, Z, time, 0, 0, 0])

    return routeList


if __name__ == '__main__':
    routeCsv = csv.reader(open('router.txt'))
    routeList = list(routeCsv)
    routeNodeNum = len(routeList)
    #输出路径点个数
    print("route nodes num is : " + str(routeNodeNum - 1))
    routeNodeIndex = 1

    print(routeList)
