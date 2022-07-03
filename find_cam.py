from subprocess import PIPE, Popen
import cv2

"""
在VIO中调用方法
from find_cam import find_cam

global front_cam = find_cam(b"USB Camera:  USB GS CAM (usb-3f980000.usb-1.1.3):")
global board_cam = find_cam(b"mmal service 16.1 (platform:bcm2835-v4l2):")

cap = cv2.VideoCapture(front_cam/board_cam)
"""


# 返回摄像头ID
def find_cam(cam):
    cmd = ["/usr/bin/v4l2-ctl", "--list-devices"]
    out, err = Popen(cmd, stdout=PIPE, stderr=PIPE).communicate()
    # out = b'bcm2835-codec (platform:bcm2835-codec):\n\t/dev/video10\n\t/dev/video11\n\t/dev/video12\n\nmmal service 16.1 (platform:bcm2835-v4l2):\n\t/dev/video2\n\nUSB Camera:  USB GS CAM (usb-3f980000.usb-1.1.3):\n\t/dev/video0\n\t/dev/video1'
    for i in out.split(b"\n\n"):
        array = i.split(b"\n\t")
        if (cam == array[0]):
            return array[1].decode('utf8')
    return None


if __name__ == "__main__":
    front_cam = find_cam(b"USB Camera:  USB GS CAM (usb-3f980000.usb-1.1.3):")
    board_cam = find_cam(b"mmal service 16.1 (platform:bcm2835-v4l2):")
    print(front_cam, board_cam)
    cap = cv2.VideoCapture(front_cam)
    cap.set(3, 320)
    cap.set(4, 240)
    while (True):
        ret, frame = cap.read()
        if ret == True:
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xff == 27:  # ESC
                break
        else:
            break
    cap.release()
    cv2.destroyAllWindows()