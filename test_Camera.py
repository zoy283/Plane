import cv2
import numpy as np
import sys

from check_color import check_color

arg = sys.argv
# print(arg)
cap=cv2.VideoCapture(0)
cap.set(3,320)
cap.set(4,240)
#cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
while (True):
    ret,frame=cap.read()
    if ret == True:
        frame,x,width,color_flag = check_color(frame, "red")
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()
