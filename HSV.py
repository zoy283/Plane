import cv2
import numpy as np

def nothing(x):
    pass
#通过Opencv读取图片信息
#src = cv2.imread('image.jpg')
img = cv2.imread('red4.png')
rows,cols,channels = img.shape
cv2.imshow("src", img)
cv2.namedWindow('img2',1)
cv2.resizeWindow("img2", 500, 500) #创建一个500*500大小的窗口

# 创建6个滑条用来操作HSV3个分量的上下截取界限
cv2.createTrackbar('Hlow','img2',139,180,nothing)
cv2.createTrackbar('Hup','img2',180,180,nothing)
cv2.createTrackbar('Slow','img2',36,255,nothing)
cv2.createTrackbar('Sup','img2',255,255,nothing)
cv2.createTrackbar('Vlow','img2',31,255,nothing)
cv2.createTrackbar('Vup','img2',255,255,nothing)

# lower_red = np.array([55,30,30])
# upper_red = np.array([99,255,255])
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
while(1):
    # mask = cv2.inRange(hsv, lower_red, upper_red)
    #将制定像素点的数据设置为0, 要注意的是这三个参数对应的值是Blue, Green, Red。
    hlow = cv2.getTrackbarPos('Hlow', 'img2')
    hup = cv2.getTrackbarPos('Hup', 'img2')
    slow = cv2.getTrackbarPos('Slow', 'img2')
    sup = cv2.getTrackbarPos('Sup', 'img2')
    vlow = cv2.getTrackbarPos('Vlow', 'img2')
    vup = cv2.getTrackbarPos('Vup', 'img2')
    lower = np.array([hlow, slow, vlow])
    upper = np.array([hup, sup, vup])
    mask = cv2.inRange(hsv, lower, upper)
    img2 = cv2.bitwise_and(img, img, mask=mask)

   # cv2.imshow("src", src)
    cv2.imshow("img2", img2)
    k = cv2.waitKey(1)&0xFF
    if k == 27: #esc exit
        print(lower,upper)
        break
#cv2.waitKey(0)
cv2.destroyAllWindows()