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
    RD = f.TURNandGETLidarDataXY(45, (40, 0), 20)
    #RD = f.GETLidarDataAL()
    for i in RD: print(i)