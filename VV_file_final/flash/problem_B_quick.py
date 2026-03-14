import time
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
from Func import *
from task.VV_file_final.flash.quick_fun import quick_function

M = []
shareLib = LoadShareLib()
fun = Func(shareLib)
f = quick_function()
MIN_HOLE_L = 60
X_range = (10, 190)
Y_range = (25, 60)
disLO = 20
wall_len = 200
#Wpi = []
Wall = [[-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]]

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
        ll = l
        rr = r
    return (ll, rr)

def write_Wall(n, x):
    WXY = []
    left_a = 99999
    right_a = 0
    for i in range(10):
        data = f.GETLidarDataAL()
        left_a = min(data[0][0], left_a)
        right_a = max(data[-1][0], right_a)
        t = f.GETLidarDataXY(data, (x, 0), 20)
        for j in t:
            WXY.append(j)
        time.sleep(0.1)

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

    l = l / count
    l = l - disLO
    V = (round(x - l * math.cos(left_a * math.pi / 180), 1), round(x - l * math.cos(right_a * math.pi / 180), 1))

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

    for i in W:
        if i[0] % 10 == i[1] % 10: continue
        if i[0] % 10 > 6: i[0] = (i[0] // 10 + 1) * 10 + 0.1
        if i[1] % 10 < 4: i[1] = (i[1] // 10 - 1) * 10 + 9.9

    for i in range(int(V[0] // 10), int(V[1] // 10) + 1): Wall[n][i] = 0
    for i in W: 
        for j in range(int(i[0] // 10), int(i[1] // 10) + 1): Wall[n][j] = 1

    print(W)
    print(Wall[n])


def check_hole_middle(n):
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

    if r - l + 1 >= 7: return (l, r)

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

def go_through_hole(k, L, R):
    m_point = ( L * 10 + (R + 1) * 10 ) / 2
    M.append(m_point)
    print(m_point)
    
    if m_point == 95 or m_point == 105:
        f.RelativeXYW([85, 0, 0], 30, 5, 1.5)
        return 100
    
    elif m_point < 95:
        f.RelativeXYW([0, 0, -90], 30, 5, 1.5)
        f.CalibrateFRONT(m_point - disLO)
        f.RelativeXYW([0, 0, 90], 30, 5, 1.5)
        f.RelativeXYW([85, 0, 0], 30, 5, 1.5)
    else:
        f.RelativeXYW([0, 0, 90], 30, 5, 1.5)
        f.CalibrateFRONT(wall_len - m_point - disLO)
        f.RelativeXYW([0, 0, -90], 30, 5, 1.5)
        f.RelativeXYW([85, 0, 0], 30, 5, 1.5)

    return m_point
    

if __name__ == "__main__":
    f.RelativeXYW([0, 0, -180], 30, 5, 1.5)
    f.CalibrateFRONT(20)
    f.RelativeXYW([0, 0, -90], 30, 5, 1.5)
    f.CalibrateFRONT(20)

    f.RelativeXYW([0, 0, -90], 30, 5, 1.5)
    write_Wall(0, 160)
    f.RelativeXYW([0, 0, -90], 30, 5, 1.5)
    #f.CalibrateFRONT(wall_len // 2 - disLO)
    f.RelativeXYW([65, 0, 0], 30, 5, 1.5)
    f.RelativeXYW([0, 0, 90], 30, 5, 1.5)
    write_Wall(0, 100)
    L, R = check_hole_middle(0)
    x = go_through_hole(0, L, R)
    
    for i in range(3):
        write_Wall(i + 1, x)

        if x == 100:
            pass
        elif x > 100:
            f.RelativeXYW([0, 0, -90], 30, 5, 1.5)
            f.RelativeXYW([x - 100 + 5, 0, 0], 30, 5, 1.5)
            f.RelativeXYW([0, 0, 90], 30, 5, 1.5)
        else:
            f.RelativeXYW([0, 0, 90], 30, 5, 1.5)
            f.RelativeXYW([100 - x + 5, 0, 0], 30, 5, 1.5)
            f.RelativeXYW([0, 0, -90], 30, 5, 1.5)
        write_Wall(i + 1, 100)
        L, R = check_hole_middle(i + 1)
        x = go_through_hole(i + 1, L, R)
    
        print(M)
        
    #print(check_hole_middle(0))



    



