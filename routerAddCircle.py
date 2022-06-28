import csv
from math import pi
import math

from matplotlib.pyplot import tick_params


def addCircleRoute(routeList, routeNodeIndex, radius, direction):
    # radianList = [
    #     1 / 6 * pi,
    #     2 / 6 * pi,
    #     3 / 6 * pi,
    #     4 / 6 * pi,
    #     5 / 6 * pi,
    #     pi,
    #     7 / 6 * pi,
    #     8 / 6 * pi,
    #     10 / 6 * pi,
    #     11 / 6 * pi,
    # ]
    time = 1
    diff = 14
    radianList = []
    for i in range(1, diff):
        radianList.append(i / diff * 2 * pi)
    x = float(routeList[routeNodeIndex][0])
    y = float(routeList[routeNodeIndex][1])
    z = float(routeList[routeNodeIndex][2])
    # 生成顺时针路径
    if (direction == 'clock'):
        for i in range(len(radianList)):
            x_new = round(x + radius * (-1 + math.cos(radianList[i])), 2)
            y_new = round(y + radius * (math.sin(radianList[i])), 2)
            routeList.insert(routeNodeIndex + 1,
                             [x_new, y_new, z, time, 0, 0, 0])
    # 生成逆时针路径
    elif (direction == 'cntclock'):
        for i in range(len(radianList)):
            x_new = round(
                x + radius *
                (-1 + math.cos(radianList[len(radianList) - 1 - i])), 2)
            y_new = round(
                y + radius * (math.sin(radianList[len(radianList) - 1 - i])),
                2)
            routeList.insert(routeNodeIndex + 1,
                             [x_new, y_new, z, time, 0, 0, 0])
    # print(circleList)
    # 返回原点
    routeList.insert(routeNodeIndex + len(radianList) + 1,
                     [x, y, z, time, 0, 0, 0])
    return routeList


if __name__ == '__main__':
    routeCsv = csv.reader(open('router.txt'))
    routeList = list(routeCsv)
    routeNodeNum = len(routeList)
    #输出路径点个数
    print("route nodes num is : " + str(routeNodeNum - 1))
    routeNodeIndex = 1

    routeList = addCircleRoute(routeList, routeNodeIndex, 0.5, 'cntclock')
    print(routeList)
