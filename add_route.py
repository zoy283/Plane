import csv
import math
""" 
调用
from add_route import add_route

routeList, routeNodeNum = add_route(routeList, routeNodeNum, 8) #point参数为0时规划回起点路径
"""


def add_route(routeList, routeNodeNum, point):
    pointList = [[0, 0], [-0.5, -2.75], [-2, -1.25], [-2.75, -2], [-3.5, .25],
                 [-3.5, -2.75], [-2.75, -.5], [-1.25, -.5], [-1.25, -2],
                 [-.5, -1.25], [-2, .25], [-2, -2.75], [-3.5, -1.25]]
    target_x = pointList[point][0]
    target_y = pointList[point][1]

    x = float(routeList[routeNodeNum][0])
    y = float(routeList[routeNodeNum][1])
    z = float(routeList[routeNodeNum][2])
    time = 2

    length = round(((target_x - x)**2 + (target_y - y)**2)**0.5, 2)
    step = math.floor(length / 0.5)
    step_x = (target_x - x) / step
    step_y = (target_y - y) / step
    print(target_x, target_y, length, step, step_x, step_y)
    for i in range(1, step):
        new_x = round(x + step_x * i, 2)
        new_y = round(y + step_y * i, 2)
        routeList.insert(routeNodeNum + step, [new_x, new_y, z, time, 0, 0, 0])
    if (point > 0):
        routeList.insert(routeNodeNum + step + 1,
                         [target_x, target_y, z, time, 0, 0, 0])
        routeList.insert(routeNodeNum + step + 2,
                         [target_x, target_y, 0.8, time, 0, 0, 0])
        routeList.insert(routeNodeNum + step + 3,
                         [target_x, target_y, 0.8, 8, 0, 0, 0])
        routeList.insert(routeNodeNum + step + 4,
                         [target_x, target_y, z, time, 0, 0, 0])
    else:
        routeList.insert(routeNodeNum + step + 1,
                         [target_x, target_y, z, time, 0, 0, 0])
    routeNodeNum = len(routeList) - 1
    return routeList, routeNodeNum


if __name__ == '__main__':
    routeCsv = csv.reader(open('router.txt'))
    routeList = list(routeCsv)
    routeNodeNum = len(routeList) - 1
    #输出路径点个数
    print("route nodes num is : " + str(routeNodeNum))
    routeNodeIndex = 1

    routeList, routeNodeNum = add_route(routeList, routeNodeNum, 8)
    routeList, routeNodeNum = add_route(routeList, routeNodeNum, 0)
    print(routeList)
    print('Down')