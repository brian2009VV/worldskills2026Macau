import cv2
import time 
from ultralytics import YOLO

model = YOLO('/home/pi/PickPro/python/task/VV_file_final/model/best.pt')

while True:
    print("hihi")
    k = input()
    camera = cv2.VideoCapture(0)
    success, frame = camera.read()
    if not success: break
    boxes = model(frame)[0].boxes
    for cls, conf, xyxy in zip(boxes.cls, boxes.conf, boxes.xyxy):
        if float(conf) < 0.1: continue
        x1, y1, x2, y2 = map(int, xyxy)
        cv2.putText(frame, str(int(cls)) + " " + str(round(float(conf), 3)), [x1 + 5, y1 + 20], cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 255, 0), 2, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    cv2.imwrite('/home/pi/PickPro/python/task/VV_file_final/cap_photo/' + 'ans' + '.jpg', frame)
    
    camera.release()
