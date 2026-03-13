import time
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from Func import * 
from task.VV_file_final.final_fun import self_function
shareLib = LoadShareLib()
fun = Func(shareLib)
f = self_function()
wall_len = 200
disLO = 20
t_err = 0.7
dx_err = 10
l_err = 20
disback = 20
ALLWall = []

def go_through_hole(start_x):
    SD = f.WallModeling(start_x, 1, wall_len, 60, 40, 40, disLO, t_err, dx_err, l_err)
    WandX = f.DetermineHOLEinHorizontalWall(SD, 80, 40, wall_len)
    
    #Calibration Back
    if start_x <= 100:
        f.RelativeXYW([0, 0, 90], 15, 5, 1.5)
        f.CalibrateFRONT(20)
        f.RelativeXYW([0, 0, 90], 15, 5, 1.5)
        f.CalibrateFRONT(disback)
        f.RelativeXYW([0, 0, 180], 15, 5, 1.5)
    else:
        f.RelativeXYW([0, 0, -90], 15, 5, 1.5)
        f.CalibrateFRONT(20)
        f.RelativeXYW([0, 0, -90], 15, 5, 1.5)
        f.CalibrateFRONT(disback)
        f.RelativeXYW([0, 0, 180], 15, 5, 1.5)

    # Go Through The Hole
    Hx = WandX[1]
    if Hx <= wall_len // 2:
        f.RelativeXYW([0, 0, -90], 15, 5, 1.5)
        f.CalibrateFRONT(20)
        f.RelativeXYW([20 + disLO - Hx, 0, 0], 15, 5, 1.5)
        f.RelativeXYW([0, 0, 90], 15, 5, 1.5)
    else:
        f.RelativeXYW([0, 0, 90], 15, 5, 1.5)
        f.CalibrateFRONT(20)
        f.RelativeXYW([-(wall_len - Hx - 20 - disLO), 0, 0], 15, 5, 1.5)
        f.RelativeXYW([0, 0, -90], 15, 5, 1.5)

    f.RelativeXYW([85, 0, 0], 15, 3, 1.5)

    ALLWall.append(WandX)
    return WandX[1]


if __name__ == "__main__":
    f.ZeroXYW()

    #first wall
    f.RelativeXYW([0, 0, -180], 15, 5, 1.5)
    f.CalibrateFRONT(disback)
    f.RelativeXYW([0, 0, 180], 15, 5, 1.5)

    x = go_through_hole(150)

    #second to forth wall
    for i in range(3): x = go_through_hole(x)

    #end point
    f.RelativeXYW([0, 0, -90], 15, 5, 1.5)
    f.CalibrateFRONT(20)

    for i in ALLWall: print(i)