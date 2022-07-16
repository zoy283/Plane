import numpy as np
import imutils
import time
import cv2
import math
from IPython import display


def nothing(x):
    pass


def hough_detect(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # param1 --> Canny边缘检测的最大阈值
    # param2 --> 越大Hough圆的检测要求越高
    # maxRadius --> 0(等于图像的尺寸)
    img = cv2.medianBlur(gray, 7)  # 进行中值模糊，去噪点
    circles = cv2.HoughCircles(img,
                               cv2.HOUGH_GRADIENT,
                               1,
                               120,
                               param1=100,
                               param2=30,
                               minRadius=20,
                               maxRadius=0)
    circles = np.uint16(np.around(circles))
    maxradius = max(circles[0, :, 2])
    if circles is not None:
        for i in circles[0, :]:
            if (i[2] == maxradius):
                # draw the outer circle
                cv2.circle(frame, (i[0], i[1]), i[2], (255, 0, 0), 2)
                # draw the center of the circle
                cv2.circle(frame, (i[0], i[1]), 2, (255, 0, 0), 3)
                Circles_x = int(i[0])
                Circles_y = int(i[1])
                print("检测到霍夫圆")
    cv2.imshow("ss", frame)
    return Circles_x, Circles_y


# 初始化右侧图像显示功能,优化了传输,
def hough_control(routeList, routeNodeIndex, pix_x, pix_y):
    time = 0.5
    limit = 5
    x = float(routeList[routeNodeIndex][0])
    y = float(routeList[routeNodeIndex][1])
    z = float(routeList[routeNodeIndex][2])
    if pix_x > 80 + limit:
        pix_x = 80 + limit
    if pix_x < 80 - limit:
        pix_x = 80 - limit
    if pix_y > 60 + limit:
        pix_y = 60 + limit
    if pix_y < 60 - limit:
        pix_y = 60 - limit
    x_new = x + (pix_x - 80.0) * z * 0.225 / 0.304 / 100
    y_new = y + (pix_y - 60.0) * z * 0.225 / 0.304 / 100
    routeList.insert(routeNodeIndex + 1, [x_new, y_new, z, time, 0, 0, 0])
    return routeList


if __name__ == '__main__':
    # 读入图片
    frame = cv2.imread("hough.jpg")
    cv2.imshow("src", frame)
    cv2.namedWindow('img2', 1)
    cv2.resizeWindow("img2", 500, 500)  #创建一个500*500大小的窗口

    cv2.createTrackbar('param1', 'img2', 50, 1000, nothing)
    cv2.createTrackbar('param2', 'img2', 100, 1000, nothing)
    cv2.createTrackbar('minRadius', 'img2', 0, 100, nothing)
    cv2.createTrackbar('maxRadius', 'img2', -1, 200, nothing)
    while (1):
        frame = cv2.imread("hough.jpg")
        frame = imutils.resize(frame, width=320, height=240)
        param1 = cv2.getTrackbarPos('param1', 'img2')
        param2 = cv2.getTrackbarPos('param2', 'img2')
        minRadius = cv2.getTrackbarPos('minRadius', 'img2')
        maxRadius = cv2.getTrackbarPos('maxRadius', 'img2')

        # b, g, r = cv2.split(frame)    # 分离三个颜色
        # # r = np.int16(r)             # 将红色与蓝色转换为int16，为了后期做差
        # # b = np.int16(b)
        # # r_minus_b = r - b           # 红色通道减去蓝色通道，得到r_minus_b
        # # r_minus_b = (r_minus_b + abs(r_minus_b)) / 2    # r_minus_b中小于0的全部转换为0
        # # r_minus_b = np.uint8(r_minus_b)
        # cv2.imshow("r", r)
        # cv2.imshow("b", b)
        # cv2.imshow("g", g)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow("gray", gray)

        # ret,binary = cv2.threshold(gray,120,120,cv2.THRESH_BINARY)
        # cv2.imshow("binary",binary)

        # param1 --> Canny边缘检测的最大阈值
        # param2 --> 越大Hough圆的检测要求越高
        # maxRadius --> 0(等于图像的尺寸)
        # img = cv2.medianBlur(gray, 7)  # 进行中值模糊，去噪点
        gaussian = cv2.GaussianBlur(gray, (7, 7), 0)
        cv2.imshow("gaussian", gaussian)
        edges = cv2.Canny(gaussian, 70, 120, apertureSize=3)
        cv2.imshow("edges", edges)
        # img = binary
        # cv2.imshow("zhognzhi", img)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT_ALT, 20, 120,
                                   param1, 0.7, minRadius, maxRadius)
        print('circles.shape:', circles.shape)
        circles = np.uint16(np.around(circles))
        # maxradius=max(circles[0,:,2])
        if circles is not None:
            for cir in circles:
                i = cir[0]
                print('circle.shape:', i.shape, 'circle:', i)
                # if(i[2]==maxradius):
                #     draw the outer circle
                cv2.circle(frame, (i[0], i[1]), i[2], (255, 0, 0), 2)
                # draw the center of the circle
                cv2.circle(frame, (i[0], i[1]), 2, (255, 0, 0), 3)

                # Circles_x = int(i[0])
                # Circles_y = int(i[1])
        cv2.imshow("img2", frame)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:  #esc exit
            break

    cv2.waitKey(0)
    cv2.destroyAllWindows()