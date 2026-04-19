import time
import sys
import os
import cv2
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from Func import * 
from task.VV_file_final.final_fun import self_function
shareLib = LoadShareLib()
fun = Func(shareLib)
f = self_function()


if __name__ == "__main__":
    #middle
    f.MoveARM(True, 8, 20, 60, -185, 0, 12, 45, 9)
    time.sleep(1)
    cam = cv2.VideoCapture(0)
    success, frame = cam.read()
    cv2.imwrite('/home/pi/PickPro/python/task/VV_file_final/cap_photo/' + 'M' + '.jpg', frame)
    cam.release()

    #left
    f.MoveARM(False, 8, 20, 60, -185, 60, 12, 50, 9)
    time.sleep(1)
    cam = cv2.VideoCapture(0)
    success, frame = cam.read()
    cv2.imwrite('/home/pi/PickPro/python/task/VV_file_final/cap_photo/' + 'L' + '.jpg', frame)
    cam.release()
    
    #right
    f.MoveARM(False, 8, 20, 60, -185, -45, 12, 50, 9)
    time.sleep(1)
    cam = cv2.VideoCapture(0)
    success, frame = cam.read()
    cv2.imwrite('/home/pi/PickPro/python/task/VV_file_final/cap_photo/' + 'R' + '.jpg', frame)
    cam.release()