# -*- coding:utf-8 -*-
import numpy as np
import imutils
import time
import cv2
import math
from IPython import display


# 增加了底层驱动，可以直接通过cv2的0号设备读取摄像头
print("start reading video...")
print("start working")
# 初始化右侧图像显示功能,优化了传输,
# 增加了imshow('name',frame)函数，与cv2的imshow保持一致
# 无需指定fps

def line_detect(image,orignal):
    Horizontal_line = 0
    vertical_line = 0
    # 检测白线    这里是设置检测直线的条件，可以去读一读HoughLinesP()函数，然后根据自己的要求设置检测条件
    image1 = image[0:60,:]
    image2 = image[60:120,:]
    lines_1 = cv2.HoughLinesP(image1, 1, np.pi / 180, 40,minLineLength=10,maxLineGap=10)
    # 对通过霍夫变换得到的数据进行遍历
    lines_2 = cv2.HoughLinesP(image2, 1, np.pi / 180, 40,minLineLength=10,maxLineGap=10)

    for line in lines_1:
        # newlines1 = lines[:, 0, :]
        x1,y1,x2,y2 = line[0]   #两点确定一条直线，这里就是通过遍历得到的两个点的数据 （x1,y1）(x2,y2)
        # 转换为浮点数，计算斜率
        print("x1=%s,x2=%s,y1=%s,y2=%s" % (x1, x2, y1, y2))
        if abs(y2 - y1) < 10 :
            print("直线是水平的")
            cv2.line(orignal,(x1,y1),(x2,y2),(0,0,255),2)     #在原图上画线
            Horizontal_line = Horizontal_line + 1
            Angle_1 = 0
        elif abs(x2 -x1) < 10 :
            continue
        else:
            cv2.line(orignal,(x1,y1),(x2,y2),(0,0,255),2)     #在原图上画线
            x1 = float(x1)
            x2 = float(x2)
            y1 = float(y1)
            y2 = float(y2)
            k = -(y2 - y1) / (x2 - x1)             # 计算斜率
            Angle_1 = np.arctan(k) * 57.29577                 # 求反正切，再将得到的弧度转换为度
            print("直线倾斜角度为：" + str(Angle_1) + "度")
    for line in lines_2:
        # newlines1 = lines[:, 0, :]
        x1,y1,x2,y2 = line[0]   #两点确定一条直线，这里就是通过遍历得到的两个点的数据 （x1,y1）(x2,y2)
        # 转换为浮点数，计算斜率
        print("x1=%s,x2=%s,y1=%s,y2=%s" % (x1, x2, y1, y2))
        if abs(x2 -x1) < 10 :
            print("直线是垂直的")
            cv2.line(orignal,(x1,y1+60),(x2,y2+60),(0,0,255),2)     #在原图上画线
            vertical_line = vertical_line + 1
            Angle_2 = 90
        elif abs(y2 - y1) < 10 :
            continue
        else:
            cv2.line(orignal,(x1,y1+60),(x2,y2+60),(0,0,255),2)     #在原图上画线
            x1 = float(x1)
            x2 = float(x2)
            y1 = float(y1)
            y2 = float(y2)
            k = -(y2 - y1) / (x2 - x1)             # 计算斜率
            Angle_2 = np.arctan(k) * 57.29577                 # 求反正切，再将得到的弧度转换为度
            print("直线倾斜角度为：" + str(Angle_2) + "度")    
    
    if(Horizontal_line == 1 and vertical_line == 2):
        Angle = 90
    else :
        Angle = Angle_2 + Angle_1
    cv2.imshow("line_detect",orignal)
    print("旋转角" + str(Angle))            
    print("水平线数量：" + str(Horizontal_line))    
    print("垂直线数量：" + str(vertical_line))  

    return Angle


def black_line(img):
    centers = []
    LineOffset = 0
    thrd_set = 80
    ret, frame = cv2.threshold(img, thrd_set, 255, cv2.THRESH_BINARY)
    kernel_size = 5
    frame = cv2.GaussianBlur(frame, (kernel_size, kernel_size), 0)
    frame = cv2.Canny(frame, 600, 500)  # Canny边缘检测300-500
    cv2.imshow("Canny",frame)
    line_detect(frame,img)
    for i in range(5):
        line = frame[ :,10 + i * 30]  # 取出一行的像素值
        white_count = np.sum(line == 255)  # 找到像素值为255的像素点的数量
        white_index = np.where(line == 255)  # 找到像素值为255的像素点的位置
        print(white_index)
        if white_count != 0:  # 找到白线
            if white_count == 2:
                if (np.abs(white_index[0][white_count - 1] - white_index[0][0]) >= 10 and np.abs(
                        white_index[0][white_count - 1] - white_index[0][0]) <= 60):
                    center = (white_index[0][white_count - 1] + white_index[0][0]) / 2  # 记录中心位置
                    centers.append(center)  # 列表
                    cv2.circle(frame, (10 + i * 30, int(center)), 4, (160, 180, 200), 2)  # 画点函数
            elif white_count > 2:
                for j in range(white_count - 1):
                    if (np.abs(white_index[0][j] - white_index[0][j + 1]) >= 10 and np.abs(
                            white_index[0][j] - white_index[0][j + 1]) < 60):
                        center = (white_index[0][j] + white_index[0][j + 1]) / 2
                        centers.append(center)
                        cv2.circle(frame, (10 + i * 30, int(center)), 4, (160, 180, 200), 2)
        for i in range(11):
            line = frame[10 + i * 10,: ]  # 取出一行的像素值
            white_count = np.sum(line == 255)  # 找到像素值为255的像素点的数量
            white_index = np.where(line == 255)  # 找到像素值为255的像素点的位置
            if white_count != 0:  # 找到白线
                if white_count == 2:
                    if (np.abs(white_index[0][white_count - 1] - white_index[0][0]) >= 10 and np.abs(
                            white_index[0][white_count - 1] - white_index[0][0]) <= 60):
                        center = (white_index[0][white_count - 1] + white_index[0][0]) / 2  # 记录中心位置
                        centers.append(center)  # 列表
                        cv2.circle(frame, (int(center),10 + i * 10), 4, (160, 180, 200), 2)  # 画点函数
                elif white_count > 2:
                    for j in range(white_count - 1):
                        if (np.abs(white_index[0][j] - white_index[0][j + 1]) >= 10 and np.abs(
                                white_index[0][j] - white_index[0][j + 1]) < 60):
                            center = (white_index[0][j] + white_index[0][j + 1]) / 2
                            centers.append(center)
                            cv2.circle(frame, (int(center),10 + i * 10), 4, (160, 180, 200), 2)
    centersCnt = len(centers)
    if (centersCnt == 0):  # 前方未检测到可用线
        Speed = 0
    else:
        Speed = 15  # e.g.:15=0.15m/s
        if centersCnt > 1:
            LineOffset = (centers[0] + centers[1]) / 2
        else:
            LineOffset = centers[0]
    cv2.imshow("1",frame)
    X = 76
    Y = int(LineOffset)
    print("LineOffset is %d " % (LineOffset))
    centers = []

def black_line_Next(img):
    centers = []
    thrd_set = 80
    ret, frame = cv2.threshold(img, thrd_set, 255, cv2.THRESH_BINARY)
    kernel_size = 5
    frame = cv2.GaussianBlur(frame, (kernel_size, kernel_size), 0)
    frame = cv2.Canny(frame, 600, 500)  # Canny边缘检测300-500
    cv2.imshow("Canny",frame)
    for i in range(3):  # 取图像上半部分
        line1 = frame[:, 10 + i * 30]  # 取出一行的像素值
        white_count = np.sum(line1 == 255)  # 找到像素值为255的像素点的数量
        white_index = np.where(line1 == 255)  # 找到像素值为255的像素点的位置
        if white_count != 0:  # 找到白线
            for j in range(white_count - 1):
                if (np.abs(white_index[0][j] - white_index[0][j + 1]) >= 20 and np.abs(
                        white_index[0][j] - white_index[0][j + 1]) < 60):
                    line1_exist = True
        line1 = frame[10 + i * 20,: ]  # 取出一列的像素值
        white_count = np.sum(line1 == 255)  # 找到像素值为255的像素点的数量
        white_index = np.where(line1 == 255)  # 找到像素值为255的像素点的位置
        if white_count != 0:  # 找到白线
            for j in range(white_count - 1):
                if (np.abs(white_index[0][j] - white_index[0][j + 1]) >= 20 and np.abs(
                        white_index[0][j] - white_index[0][j + 1]) < 60):
                    line1_exist = True
    for i in range(3):  # 取图像下半部分
        line2 = frame[:, 60 + i * 30]  # 取出一行的像素值
        white_count = np.sum(line2 == 255)  # 找到像素值为255的像素点的数量
        white_index = np.where(line2 == 255)  # 找到像素值为255的像素点的位置
        if white_count != 0:  # 找到白线
            for j in range(white_count - 1):
                if (np.abs(white_index[0][j] - white_index[0][j + 1]) >= 20 and np.abs(
                        white_index[0][j] - white_index[0][j + 1]) < 60):
                    line2_exist = True
if __name__ == '__main__':
    # 读入图片
    src = cv2.imread("line.png")
    # 设置窗口大小
    cv2.namedWindow("input image", cv2.WINDOW_AUTOSIZE)
    # 显示原始图片
    cv2.imshow("input image", src)
    # 调用函数
    black_line(src)
    cv2.waitKey(0)