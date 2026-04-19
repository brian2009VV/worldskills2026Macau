import time
import sys
import os
import cv2
from ultralytics import YOLO

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
from Func import *
from task.VV_file_final.flash.quick_fun import quick_function
shareLib = LoadShareLib()
fun = Func(shareLib)
f = quick_function()
model_path = '/home/pi/PickPro/python/task/VV_file_final/model/best.pt'

CAMXYZ50 = [0, 35, 43]
CAMXYZ00 = [0, 28, 46]

M = []
v_f = 50
v_h = 40
MIN_HOLE_L = 40
deta_a = 12.5
X_range = (10, 190)
Y_range = (30, 70)
disLO = 20
wall_len = 200
Wall = [[-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]]

def reset_arm(TF):
    f.MoveARM(TF, 8, 20, 50, 0, 0, 5, 0, 7)

def find_max_length_list(target, arr):
    MAX = 0
    l = -1
    r = -1
    ll = -1
    rr = -1
    for i in range(len(arr)):
        if l == -1 and arr[i] == target:
            l = i
            r = i
        elif arr[i] == target and l != -1: r = i
        else:
            if r - l + 1 >= MAX:
                ll = l
                rr = r
                MAX = r - l + 1
            l = -1
            r = -1

    if l != -1:
        if r - l + 1 >= MAX:
            ll = l
            rr = r

    return (ll, rr)

def write_Wall(n, x, times):
    time.sleep(0.1)
    WXY = []
    Alist = []
    l_a = 999999
    r_a = 0

    for i in range(times):
        time.sleep(0.1)
        data = f.GETLidarDataAL()
        Alist = []
        for i in data: Alist.append(i[0])

        if len(Alist) == 0: continue

        lpi = 1
        rpi = 1
        ll = 0
        rr = 1
        MAX_L = 0
        for i in range(1, len(Alist)):
            if abs(Alist[i - 1] - Alist[i]) <= deta_a:
                rr = i
            else:
                if MAX_L <= rr - ll + 1:
                    MAX_L = rr - ll + 1
                    lpi = ll
                    rpi = rr
                ll = i
                rr = i

        if MAX_L <= rr - ll + 1:
            lpi = ll
            rpi = rr

        print(lpi, rpi)
        l_a = min(Alist[lpi], l_a)
        r_a = max(Alist[rpi], r_a)

        t = f.GETLidarDataXY(data, (x, 0), 20)
        for j in t: WXY.append(j)

    print(l_a, r_a)
    print(WXY)

    W1 = []
    l = 0
    count = 0
    for i in WXY:
        if i[0] <= X_range[0] or i[0] >= X_range[1]: continue
        if i[1] <= Y_range[0] or i[1] >= Y_range[1]: continue
        W1.append(i)
        l += i[1]
        count += 1

    W1.sort(key = lambda x: x[0])

    if count == 0: return False
    l = l / count
    l = l - disLO
    
    V = [round(x - l * 1 / math.tan(l_a * math.pi / 180), 1), round(x - l * 1 / math.tan(r_a * math.pi / 180), 1)]
    V[0] = max(X_range[0], V[0])
    V[1] = min(X_range[1], V[1])

    W = []
    l = -1
    r = -1
    for i in W1:
        if l == -1: 
            l = i[0]
            r = i[0]
        else:
            if i[0] - r >= MIN_HOLE_L:
                W.append([l, r])
                l = -1
                r = -1
            else: r = i[0]
    
    if l != -1: W.append([l, r])
    
    print(W)
    for i in W:
        if i[0] % 10 == i[1] % 10: continue
        if i[0] % 10 > 6.5: i[0] = (i[0] // 10 + 1) * 10 + 0.1
        if i[1] % 10 < 4.5: i[1] = (i[1] // 10 - 1) * 10 + 9.9
    
    if V[0] % 10 > 6.5: V[0] = (V[0] // 10 + 1) * 10 + 0.1
    if V[1] % 10 < 4.5: V[1] = (V[1] // 10 - 1) * 10 + 9.9

    for i in range(int(V[0] // 10), int(V[1] // 10) + 1): Wall[n][i] = 0
    for i in W: 
        for j in range(int(i[0] // 10), int(i[1] // 10) + 1): Wall[n][j] = 1

    print(W)
    print(Wall[n])

def check_hole(n):
    count = 0
    for i in Wall[n]:
        if i == 1:
            count += 1
            break

    if not count: return (6, 12)

    l, r = find_max_length_list(0, Wall[n])
    
    print(l, r)

    if l == -1:
        l, r = find_max_length_list(-1, Wall[n])
        return (l, r)

    if r - l + 1 >= 7: 
        if l == 0: return (r - 6, r)
        elif r == len(Wall[n]) - 1: return (l, l + 6)
        else:
            if Wall[n][l - 1] == 1: return (l, l + 6)
            elif Wall[n][r + 1] == 1: return (r - 6, r)
            else: return (l, r)

    if Wall[n][l - 1] == -1:
        l -= 1
        while l >= 0:
            if Wall[n][l] == -1: Wall[n][l] = 0
            if Wall[n][l] == 1: break
            if r - l + 1 >= 7: 
                print(Wall[n])
                return (l, r)
            l -= 1

    if Wall[n][r + 1] == -1:
        r += 1
        while r < len(Wall[n]):
            if Wall[n][r] == -1: Wall[n][r] = 0
            if Wall[n][r] == 1: break
            if r - l + 1 >= 7: 
                print(Wall[n])
                return (l, r)
            r += 1
    return (l, r)

def go_through_hole(L, R, x):
    m_point = ( L * 10 + (R + 1) * 10 ) / 2
    M.append(m_point)
    print(m_point)
    
    if m_point == 95 or m_point == 105:
        f.RelativeXYW([90, 0, 0], v_f, 5, 1.5)
        return 100
    
    elif m_point < 95:
        f.RelativeXYW([0, 0, -90], v_h, 5, 1.5)
        #f.CalibrateFRONT(m_point - disLO)
        f.CalibrateFRONT(wall_len // 2 - disLO)
        f.RelativeXYW([abs(x - m_point) + 5, 0, 0, 0], v_h, 2, 1.5)
        f.RelativeXYW([0, 0, 90], v_h, 5, 1.5)
        f.RelativeXYW([90, 0, 0], v_f, 5, 1.5)
    else:
        f.RelativeXYW([0, 0, 90], v_h, 5, 1.5)
        #f.CalibrateFRONT(wall_len - m_point - disLO)
        f.CalibrateFRONT(wall_len // 2 - disLO)
        f.RelativeXYW([abs(x - m_point) + 5, 0, 0, 0], v_h, 2, 1.5)
        f.RelativeXYW([0, 0, -90], v_h, 5, 1.5)
        f.RelativeXYW([90, 0, 0], v_f, 5, 1.5)

    return m_point

def get_model_object_realxyz(correct_pre, CAMXYZ, gamma, sita):
    model = YOLO(model_path)
    
    R = []
    camera = cv2.VideoCapture(0)
    success, frame = camera.read()
    boxes = model(frame)[0].boxes

    for cls, conf, xyxy in zip(boxes.cls, boxes.conf, boxes.xyxy):
        if float(conf) >= correct_pre:
            x1, y1, x2, y2 = map(int, xyxy)
            OXY = (x1 / 2 + x2 / 2, y1 / 2 + y2 / 2)
            ROXYZ = f.GETrealXYZ(CAMXYZ, OXY, gamma, sita)
            R.append((int(cls), round(float(conf), 1), ROXYZ))
            cv2.putText(frame, str(int(cls)) + " " + str(ROXYZ[0]) + " " + str(ROXYZ[1]) + " " + str(ROXYZ[2]), [x1 + 5, y1 + 20], cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 255, 0), 2, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    
    cv2.imwrite('/home/pi/PickPro/python/task/VV_file_final/cap_photo/' + 'ans' + '.jpg', frame)
    camera.release()

    return R

if __name__ == "__main__":
    f.OFFStartLED()