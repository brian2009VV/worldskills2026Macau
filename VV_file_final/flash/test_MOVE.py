import time
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
from Func import *
from task.VV_file_final.flash.quick_fun import quick_function
shareLib = LoadShareLib()
fun = Func(shareLib)
f = quick_function()

if __name__ == "__main__":
    f.RelativeXYW([90, 0, 0], 30, 5, 1.5)