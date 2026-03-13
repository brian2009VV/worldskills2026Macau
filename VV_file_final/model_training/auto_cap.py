import cv2
import time

if __name__ == "__main__":
    camera = cv2.VideoCapture(0)
    for i in range(10):
        success, frame = camera.read()
        if not success: break

        cv2.imwrite('/home/pi/PickPro/python/task/VV_file_final/cap_photo/' +  str(i) + '.jpg', frame)
        print(frame.shape)

        time.sleep(3)

        