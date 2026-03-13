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

if __name__ == "__main__":
    SD = f.WallModeling(70, 200, 30, 25, 20, 0.5, 10, 20)
    WandX = f.DetermineHOLEinHorizontalWall(SD, 80, 40, 200)
    
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

    f.RelativeXYW([80, 0, 0], 15, 5, 1.5)

    print(WandX)