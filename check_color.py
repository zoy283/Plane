import cv2
import numpy as np
import imutils

def check_color(frame, color):
    x = 0
    width = 0
    color_flag=0
    area = 0
    color_range = [[156, 43, 46], [180, 255, 255], [35, 43, 46],
                   [77, 255, 255], [100, 43, 46], [124, 255, 255]]  # 红 绿 蓝
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
    # cv2.rectangle(frame, (75, 55), (85, 65), (255, 0, 0), thickness=1)
    if len(cnt_color) > 0:
        area = [cv2.contourArea(i) for i in cnt_color]
        index = np.argmax(area)
        rect_red = cv2.minAreaRect(cnt_color[index])
        if (rect_red[1][0] > 5) and (rect_red[1][1] > 5):#   rect_red[1][1] height  rect_red[1][0] with rect[0][0], rect[0][1]中心点
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
            width = rect_red[1][0]
            area = rect_red[1][0] * rect_red[1][1]
    else:
        color_flag = 0
    return frame,x,width,color_flag,area