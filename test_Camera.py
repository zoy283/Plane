import cv2
import numpy as np
import sys

arg = sys.argv
# print(arg)
# cap=cv2.VideoCapture(int(arg[1]))
cap=cv2.VideoCapture(0)
cap.set(3,320)
cap.set(4,240)
#cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
while (True):
    ret,frame=cap.read()
    if ret == True:
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xff == 27: # ESC
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()
