import time
import sys
import os
import cv2
from ultralytics import YOLO
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from Func import * 
from task.VV_file_final.final_fun import self_function
shareLib = LoadShareLib()
fun = Func(shareLib)
f = self_function()
model_path = '/home/pi/PickPro/python/task/VV_file_final/model/best.pt'

def ReARM(TF):
    f.MoveARM(TF, 5, 10, 50, 0, 0, 1, 0, 9)

def GETModelTrainingObjectonGroundXY(modelpath, max_count):
    model = YOLO(modelpath)
    camera = cv2.VideoCapture(0)
    count = 0
    last_cls = -1
    x1, y1, x2, y2 = 0, 0, 0, 0
    while camera.isOpened():
        success, frame = camera.read()
        if not success: continue
        boxes = model(frame)[0].boxes
        max_conf = 0
        max_conf_cls = -1
        x1pi, y1pi, x2pi, y2pi = 0, 0, 0, 0
        for cls, conf, xyxy in zip(boxes.cls, boxes.conf, boxes.xyxy):
            if float(conf) > max_conf:
                max_conf_cls = int(cls)
                max_conf = float(conf)
                x1pi, y1pi, x2pi, y2pi = map(int, xyxy)
        
        if max_conf_cls == -1: continue

        if last_cls == -1 and max_conf_cls != -1:
            last_cls = max_conf_cls
            count = 1
            x1, y1, x2, y2 = x1pi, y1pi, x2pi, y2pi
            continue

        if max_conf_cls == last_cls:
            count += 1
            x1, y1, x2, y2 = x1pi, y1pi, x2pi, y2pi
        else:
            last_cls = max_conf_cls
            count = 1
            x1, y1, x2, y2 = x1pi, y1pi, x2pi, y2pi

        if count >= max_count: break

    return (last_cls, (x1, y1, x2, y2))


if __name__ == "__main__":
    #resetARM
    f.MoveARMXYZAC(True, (5, 10), (0, 35, 40), 90, 10)
    
    #get cam result
    model_result = GETModelTrainingObjectonGroundXY(model_path, 3)
    print(model_result)

    #calculate realXYZ
    CAMXYZ = (0, 32, 50)
    objectX = (model_result[1][0] + model_result[1][2]) // 2
    objectY = (model_result[1][1] + model_result[1][3]) // 2
    ox, oy, oz = f.GETrealXYZthroughcamfaceonground(CAMXYZ, (objectX, objectY), (640, 480))
    print(ox, oy, oz)
    
    #ClampObject
    f.MoveARMXYZAC(False, (5, 10), (ox, oy, oz + 5), 90, 15)
    f.MoveARMXYZAC(False, (5, 10), (ox, oy, oz + 5), 90, 1)
    time.sleep(1)
    ReARM(False)


