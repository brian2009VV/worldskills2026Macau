import cv2
import time
import os

if __name__ == "__main__":
    f = os.listdir('/home/pi/PickPro/python/task/VV_file_final/cap_photo')
    num = 57
    for i in f: num += 1
    
    i = 1
    while True:
        k = input()
        camera = cv2.VideoCapture(0)
        success, frame = camera.read()
        if not success: break
    
        cv2.imwrite('/home/pi/PickPro/python/task/VV_file_final/cap_photo/' +  str(num + i) + '.jpg', frame)
        print(frame.shape)
        
        i += 1
        camera.release()

        