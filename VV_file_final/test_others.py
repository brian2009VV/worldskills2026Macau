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
    #fun.ResetTurnFun(speed = 5)
    fun.ResetLiftFun(speed = 12)
    #fun.RotatingServoCtrl(val = 0, cnt = 5)
    #fun.ClampServoCtrl(val = 10, cnt = 5)
    #fun.RaiseServoCtrl(val = 0, cnt = 1)
    #fun.TelescopicServoCtrl(val = 5, cnt = 5)
    #f.RelativeXYW([50, 0, 0], 40, 5, 1.5)