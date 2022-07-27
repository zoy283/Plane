import collections
import math

import cv2
import numpy as np


def stackImages(scale, imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]),
                                                None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank] * rows
        hor_con = [imageBlank] * rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None, scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver


#颜色字典
def ColorList():
    dict = collections.defaultdict(list)
 
    # #黑色
    # lower_black = np.array([0, 0, 0])
    # upper_black = np.array([180, 255, 46])
    # color_list = []
    # color_list.append(lower_black)
    # color_list.append(upper_black)
    # dict['black'] = color_list
 
    # #灰色
    # lower_gray = np.array([0, 0, 46])
    # upper_gray = np.array([180, 43, 220])
    # color_list = []
    # color_list.append(lower_gray)
    # color_list.append(upper_gray)
    # dict['gray']=color_list
 
    # #白色
    # lower_white = np.array([0, 0, 221])
    # upper_white = np.array([180, 30, 255])
    # color_list = []
    # color_list.append(lower_white)
    # color_list.append(upper_white)
    # dict['white'] = color_list
 
    # #粉色
    # lower_pink = np.array([156, 43, 46])
    # upper_pink = np.array([180, 255, 255])
    # color_list = []
    # color_list.append(lower_pink)
    # color_list.append(upper_pink)
    # dict['pink']=color_list
 
    #红色
    lower_red = np.array([0, 43, 46])
    upper_red = np.array([10, 255, 255])
    color_list = []
    color_list.append(lower_red)
    color_list.append(upper_red)
    dict['red'] = color_list
 
    # #橙色
    # lower_orange = np.array([11, 43, 46])
    # upper_orange = np.array([25, 255, 255])
    # color_list = []
    # color_list.append(lower_orange)
    # color_list.append(upper_orange)
    # dict['orange'] = color_list
 
    # #黄色
    # lower_yellow = np.array([26, 43, 46])
    # upper_yellow = np.array([34, 255, 255])
    # color_list = []
    # color_list.append(lower_yellow)
    # color_list.append(upper_yellow)
    # dict['yellow'] = color_list
 
    # #绿色
    # lower_green = np.array([35, 43, 46])
    # upper_green = np.array([77, 255, 255])
    # color_list = []
    # color_list.append(lower_green)
    # color_list.append(upper_green)
    # dict['green'] = color_list
 
    # #青色
    # lower_cyan = np.array([78, 43, 46])
    # upper_cyan = np.array([99, 255, 255])
    # color_list = []
    # color_list.append(lower_cyan)
    # color_list.append(upper_cyan)
    # dict['cyan'] = color_list
 
    #蓝色
    lower_blue = np.array([100, 43, 46])
    upper_blue = np.array([124, 255, 255])
    color_list = []
    color_list.append(lower_blue)
    color_list.append(upper_blue)
    dict['blue'] = color_list
 
    # # 紫色
    # lower_purple = np.array([125, 43, 46])
    # upper_purple = np.array([155, 255, 255])
    # color_list = []
    # color_list.append(lower_purple)
    # color_list.append(upper_purple)
    # dict['purple'] = color_list
 
    return dict

#颜色判断
def findColor(imgcut):
    img_hsv=cv2.cvtColor(imgcut,cv2.COLOR_BGR2HSV)
    color_dict=ColorList()
    #print(color_dict)
    color_most=0
    color_now=None
    for color in color_dict:
        #二值化 和颜色字典比较 在上下限之间的像素变为255，之外的所有像素变为0
        color_cmp=cv2.inRange(img_hsv,color_dict[color][0],color_dict[color][1])
        #膨胀 使颜色分割成块并更突出
        color_boom = cv2.dilate(color_cmp,None,iterations=1)
        #取出每一小块
        contours,hierarchy=cv2.findContours(color_boom.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        color_area=0
        for img in contours:
            color_area+=cv2.contourArea(img)
        if(color_area>color_most):
            color_most=color_area
            color_now=color
    return color_now

#计算斜率
def k_count(x1,y1,x2,y2):
    if((x2-x1)==0):
        x2+=0.01
    k=(y2-y1)/(x2-x1)
    if (k==0):
        k+=0.01
    return k

#计算角度
def angle_count(k1,k2):
    angle=math.atan2((k2-k1),(1+k1*k2))
    angle=angle*180/math.pi
    return abs(angle)

#图形处理
def LastButNotLeast(imginit,imgcopy):
    # 灰度化
    img_Gray = cv2.cvtColor(imginit, cv2.COLOR_BGR2GRAY)
    # 高斯平滑
    img_Blur = cv2.GaussianBlur(img_Gray, (3, 3), 1)
    # 边缘检测
    img_Canny = cv2.Canny(img_Blur, 50, 50)
    #得到图片中所有图形的轮廓
    #findContours(image, mode, method[, contours[, hierarchy[, offset]]]) -> contours, hierarchy
    contours,hierarchy=cv2.findContours(img_Canny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    #参数：输入图像，霍夫梯度法，分辨率，最小距离，检测方法的对应的参数*2，半径
    for img in contours:    
        #计算面积 太小就不算了
        area=cv2.contourArea(img)
        if area>30:
            perimeter=cv2.arcLength(img,True)
            #折线化
            side=cv2.approxPolyDP(img,0.01*perimeter,True)
            #print(side)
            #计算有几条线
            sideNum=len(side)
            #print(sideNum)
            #计算边长
            length=[]
            k=[]
            for i in range(0,sideNum):
                if(i+1<sideNum):                
                    l=((side[i][0][0]-side[i+1][0][0])**2+(side[i][0][1]-side[i+1][0][1])**2)**(1/2)
                    ktemp=k_count(side[i][0][0],side[i][0][1],side[i+1][0][0],side[i+1][0][1])
                else:
                    l=((side[i][0][0]-side[0][0][0])**2+(side[i][0][1]-side[0][0][1])**2)**(1/2)
                    ktemp=k_count(side[i][0][0],side[i][0][1],side[0][0][0],side[0][0][1])
                length.append(l)
                k.append(ktemp)
            #print(length)
            #计算角度
            angle=[]
            for i in range(0,sideNum):
                if(i+1<sideNum):
                    ang=angle_count(k[i],k[i+1])
                else:
                    ang=angle_count(k[i],k[0])
                angle.append(ang)
            #print(angle)
            #形状判断
            #三角形
            if sideNum==3:
                tag="triangle"
            #其他四边形
            elif sideNum==4:
                tag="rectangular"
                # flag=0
                #菱形
                # err=5
                # if(length[1]-err<=length[0]<=length[1]+err):
                #     if(length[2]-err<=length[1]<=length[2]+err):
                #         if(length[3]-err<=length[2]<=length[3]+err):
                #             if(length[0]-err<=length[3]<=length[0]+err):
                #                 tag="diamond"
                #                 flag=1
                #矩形
                # if(89<angle[0]<91):
                #     if(89<angle[1]<91):
                #         if(89<angle[2]<91):
                #             if(89<angle[3]<91):
                #                 tag="rectangular"
                                # if(flag==1):
                                    # tag="square"
            # elif sideNum==5:
            #     tag="pentagon"
            # elif sideNum==6:
            #     tag="hexagon"
            # elif sideNum==10:
            #     tag="five-pointed star"
            elif sideNum>10:
                tag="circle"
            else:
                tag="None"                
            #定个位
            x,y,wide,high=cv2.boundingRect(side)
            #裁剪中心位置
            x0=int(x+(wide/2))
            y0=int(y+(high/2))
            err=25
            imgCut=imginit[(y0-err):(y0+err),(x0-err):(x0+err)]
            color=findColor(imgCut)
            #添加标签
            cv2.rectangle(imgcopy, (x-5, y-5), (x + wide+5, y + high+5), (0, 235, 6), 2)
            cv2.putText(imgcopy, tag,(x, y), cv2.FONT_HERSHEY_TRIPLEX, 0.5,(0,0, 255), 1)
            cv2.putText(imgcopy, color,(x+10, y+10), cv2.FONT_HERSHEY_TRIPLEX, 0.3,(0, 255, 255), 1)

if __name__ == "__main__":
    # path='shapes3.png'
    #path=r"D:\test.png"
    cap=cv2.VideoCapture(0)
    cap.set(3,320)
    cap.set(4,240)
    while (True):
        ret,img=cap.read()
        while(np.all(img==None)):
            print("无法读取图片")
        img_Copy=img.copy()
        
        #图像处理函数
        LastButNotLeast(img,img_Copy)

        #输出结果
        imgBlank = np.zeros_like(img)
        imgStack = stackImages(1.5, ([img, img_Copy]))

        cv2.imshow("Stack1", imgStack)

        cv2.waitKey(0)