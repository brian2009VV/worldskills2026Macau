import time
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from Func import * 
from task.VV_file_final.final_fun import self_function
shareLib = LoadShareLib()
fun = Func(shareLib)
f = self_function()

if __name__ == "__main__":
    SD = f.WallModeling(150, 1, 200, 60, 40, 40, 20, 0.7, 10, 30)
    print(SD)
    print(f.DetermineHOLEinHorizontalWall(SD, 80, 40, 200))