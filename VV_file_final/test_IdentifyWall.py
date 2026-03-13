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
    SD = f.IDENTIFYWallinInterval(-45, (40, 0), 20, 0.5, 10, (0, 200), (0, 9999999))
    print(SD)