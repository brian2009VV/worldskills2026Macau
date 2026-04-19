import time
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
from Func import *
from task.VV_file_final.flash.quick_fun import quick_function
shareLib = LoadShareLib()
fun = Func(shareLib)
f = quick_function()

Wall = [[-2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -2],
        [-2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -2],
        [-2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -2],
        [-2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -2]]

#Related to functions#####################################################################################
disLO = 20                             #Distance between radar and chassis center
wall_len = 200                         #Total length of the wall
min_hole_len = 40                      #Determine the maximum distance between the wall and the hole
deta_a = 12.5                          #Error tolerance value for determining the visible range
X_range = (10, 190)                    #X-axis range of radar data
Y_range = (30, 60)                     #Y-axis range of radar data
l_edge = 4.5                           #Left edge decision threshold
r_edge = 6.5                           #Right edge decision threshold
max_hole_boards = 7                    #The max number of the boards length of hole
min_hole_boards = 4                    #The minimum number of board lengths required to determine hole 
min_outlier_num = 0                    #Minimum acceptable number of outliers
l_safety_len = 25                      #Left safety distance
r_safety_len = 25                      #Right safety distance
max_repeated_count = 3                 #Maxium repeated number of "m_point"
#Related to movement#######################################################################################
v_f = 50
v_h = 50
###########################################################################################################

def count_digit_times_function(n : int, target : int, l : int, r : int) -> int:
    t = 0
    for i in range(l, r + 1):
        if Wall[n][i] == target: t += 1

    return t

def get_data_to_wall(n : int, x : int, times : int) -> bool:
    WXY = []
    l = 0
    l_a, r_a = 999999, 0

    for i in range(times):
        time.sleep(0.1)
        data = f.GETLidarDataAL()
        Alist = []
        for i in data: Alist.append(i[0])
        if len(Alist) == 0: continue
        
        lpi, rpi = 0, 0
        ll, rr = 0, 1
        max_l = 0
        for i in range(1, len(Alist)):
            if abs(Alist[i - 1] - Alist[i]) <= deta_a:
                rr = i
            else:
                if max_l <= rr - ll + 1:
                    max_l = rr - ll + 1
                    lpi = ll
                    rpi = rr
                ll = i
                rr = i

        if max_l <= rr - ll + 1:
            lpi = ll
            rpi = rr

        l_a = min(Alist[lpi], l_a)
        r_a = max(Alist[rpi], r_a)

        t = f.GETLidarDataXY(data, (x, 0), 20)
        for j in t:
            if j[0] <= X_range[0] or j[0] >= X_range[1]: continue
            if j[1] <= Y_range[0] or j[1] >= Y_range[1]: continue
            WXY.append(j)
            l += j[1]

    WXY.sort(key = lambda x: x[0])

    if len(WXY) == 0: return False
    l = l / len(WXY)
    l = l - disLO
    
    V = [round(x - l * 1 / math.tan(l_a * math.pi / 180), 1), round(x - l * 1 / math.tan(r_a * math.pi / 180), 1)]
    V[0] = max(X_range[0], V[0])
    V[1] = min(X_range[1], V[1])

    W = []
    l, r = -1, -1
    for i in WXY:
        if l == -1: 
            l = i[0]
            r = i[0]
        else:
            if i[0] - r >= min_hole_len:
                W.append([l, r])
                l = -1
                r = -1
            else: r = i[0]
    
    if l != -1: W.append([l, r])

    for i in W:
        if i[0] % 10 == i[1] % 10: continue
        if i[0] % 10 > r_edge: i[0] = (i[0] // 10 + 1) * 10 + 0.1
        if i[1] % 10 < l_edge: i[1] = (i[1] // 10 - 1) * 10 + 9.9
    
    if V[0] % 10 > r_edge: V[0] = (V[0] // 10 + 1) * 10 + 0.1
    if V[1] % 10 < l_edge: V[1] = (V[1] // 10 - 1) * 10 + 9.9

    for i in range(int(V[0] // 10), int(V[1] // 10) + 1): Wall[n][i] = 0
    for i in W: 
        for j in range(int(i[0] // 10), int(i[1] // 10) + 1): Wall[n][j] = 1

    return True

def guess_hole_function(n : int, cur_hole_boards : int) -> list:
    if cur_hole_boards < min_hole_boards: return []

    res = []
    l, r = -1, -1
    W = Wall[n]
    for i in range(len(W)):
        if W[i] != 1:
            if l == -1:
                l = i
                r = i
            else: r = i
        else:
            l, r = -1, -1

        if l != -1:
            if r - l + 1 >= cur_hole_boards:
                res.append([r + 1 - cur_hole_boards, r])

    if len(res) == 0: res = guess_hole_function(n, cur_hole_boards - 1)

    res.sort(key = lambda x : (-count_digit_times_function(n, 0, x[0], x[1]), -(x[1] - x[0])))
    
    return res

def check_front_hole(x : int, times : int) -> bool:
    W = []

    for i in range(times):
        time.sleep(0.1)
        data = f.GETLidarDataAL()
        t = f.GETLidarDataXY(data, (x, 0), 20)
        for j in t:
            if j[0] <= X_range[0] or j[0] >= X_range[1]: continue
            if j[1] <= Y_range[0] or j[1] >= Y_range[1]: continue
            W.append(j)

        W.sort(key = lambda x: x[0])

    count = 0
    for i in W:
        if i[0] <= x + r_safety_len and i[0] >= x - l_safety_len: count += 1

    if count <= min_outlier_num: return True
    else: 
        T = [0, 0, 0]
        for i in W:
            if i[0] <= x + r_safety_len and i[0] >= x - l_safety_len: 
                t = int(i[0] // 10 - x // 10 + 1)
                if t >= 0 and t <= len(T) - 1:
                    T[t] = 1
        
        count = 0
        for i in T:
            if i == 1: count += 1
        
        if count == len(T): f.CalibrateFRONT(20)

        return False

def go_throgh_hole(n : int, x : int) -> None:
    count = -1
    last_m = 0
    while True:
        get_data_to_wall(n, x, 5)
        re = guess_hole_function(n, max_hole_boards)

        if len(re) == 0 or count >= max_repeated_count:
            print(len(re))
            print(count)
            for i in range(1, 19): Wall[n][i] = -1
            if x < wall_len // 2:
                f.RelativeXYW([0, 0, -90], v_h, 2, 1.5)
                f.CalibrateFRONT(20)
                f.RelativeXYW([0, 0, 90], v_h, 2, 1.5)
                x = 40
            else:
                f.RelativeXYW([0, 0, 90], v_h, 2, 1.5)
                f.CalibrateFRONT(20)
                f.RelativeXYW([0, 0, -90], v_h, 2, 1.5)
                x = 160

            count = -1
            last_m = 0
            continue

        l, r = re[0][0], re[0][1]
        m_point = (l * 10 + r * 10) / 2
        m_point = min(X_range[1] - 15, m_point)
        m_point = max(X_range[0] + 15, m_point)

        if count == -1:
            count = 1
            last_m = m_point
        else:
            if last_m == m_point: count += 1
            else: count = -1

        print(Wall[n])
        print(m_point)

        if m_point == x: pass
        elif m_point > x: 
            f.RelativeXYW([0, m_point - x + 5, 0], v_h, 2, 1.5)
            x = m_point
        else: 
            f.RelativeXYW([0, m_point - x - 5, 0], v_h, 2, 1.5)
            x = m_point

        if check_front_hole(x, 3):
            f.RelativeXYW([90, 0, 0], v_f, 2, 1.5)
            return x

if __name__ == "__main__":
    f.ZeroXYW()
    while True:
        Wall = [[-2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -2],
        [-2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -2],
        [-2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -2],
        [-2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -2]]

        f.RelativeXYW([0, 0, -180], v_h, 5, 1.5)
        f.CalibrateFRONT(20)
        f.RelativeXYW([0, 0, -90], v_h, 5, 1.5)
        f.CalibrateFRONT(20)

        f.WAITPUSHStartLED()
        time.sleep(0.5)
        f.RelativeXYW([0, 0, -90], v_h, 5, 1.5)

        x = 160
        for i in range(4): x = go_throgh_hole(i, x)

        f.CalibrateFRONT(20)
        f.RelativeXYW([0, 0, -90], v_h, 5, 1.5)
        f.CalibrateFRONT(20)
        f.RelativeXYW([0, 0, -90], v_h, 5, 1.5)



