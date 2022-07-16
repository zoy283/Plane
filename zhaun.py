import csv
import math


def yaw_rotate(x, y, theta):

    # theta = theta * math.pi / 180
    x1 = x * math.cos(theta) + y * math.sin(theta)
    y1 = y * math.cos(theta) - x * math.sin(theta)

    return x1, y1


def yaw_rotate1(routeList, routeNodeIndex, theta):

    time = 0.2
    angle = 10 * math.pi / 180

    X = float(routeList[routeNodeIndex][0])
    Y = float(routeList[routeNodeIndex][1])
    Z = float(routeList[routeNodeIndex][2])

    count = theta // 10
    for i in range(int(count)):
        X,Y=yaw_rotate(X,Y,angle)
        routeList.insert(routeNodeIndex + 1, [X, Y, Z, time, 0, angle, 0])
    X, Y = yaw_rotate(X, Y, (theta % 10) * math.pi / 180)
    
    routeList.insert(routeNodeIndex+count+1,
                     [X, Y, Z, time, 0, (theta % 10) * math.pi / 180, 0])
    return routeList

if __name__ == '__main__':
    routeCsv = csv.reader(open('router.txt'))
    routeList = list(routeCsv)
    routeNodeNum = len(routeList)
    routeNodeIndex = 1

    # x1, y1 = yaw_rotate(1, 2, 90)
    # print(x1, y1)

    yaw_rotate1(routeList, routeNodeIndex, 90)
    routeNodeNum = len(routeList)
    for i in range(routeNodeNum):
        print(str(routeList[i]))