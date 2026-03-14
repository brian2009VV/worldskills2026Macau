import time
import sys
import os
import math
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Func import * 
shareLib = LoadShareLib()
fun = Func(shareLib)

class quick_function:
    def __init__(self):
        self.port_handler = None
        self.packet_handler = None
    
    def ZeroXYW(self):
        fun.ZeroOdom()
    
    def CalibrateFRONT(self, dis):
        if dis >= 20:
            fun.LidarCalibFun(dis, d_err = 0.5, d_err_cnt = 5, angle_err = 0.5, angle_err_cnt = 5, left_right_e = 1)
        else:
            fun.LidarCalibFun(20, d_err = 0.5, d_err_cnt = 5, angle_err = 0.5, angle_err_cnt = 5, left_right_e = 1)
            fun.USCalibFun(dis, angle_e = 1, dis_e = 0.5, left_right_e = 2, CNT = 5)
        
    def RelativeXYW(self, dp, v, p_e, a_e):
        dx = dp[0]
        dy = dp[1]
        dw = dp[2]

        d = math.sqrt(dx ** 2 + dy ** 2)

        op = fun.ReadCurPose()

        if dx == 0:
            t = math.pi / 2
        else:
            t = math.atan(dy / dx)

        if dy > 0:
            a = op.theta_ * math.pi / 180 + t
        elif dy < 0:
            a = op.theta_ * math.pi / 180 + t + math.pi
        else:
            if dx >= 0:
                a = op.theta_ * math.pi / 180 + t
            else:
                a = op.theta_ * math.pi / 180 + t + math.pi

        nx = int(op.x_ + d * math.cos(a))
        ny = int(op.y_ + d * math.sin(a))
        nw = int(op.theta_ + dw)

        nw = nw % 360

        fun.TrackingPointFun(Pose(nx, ny, nw), v, p_e, a_e)
        time.sleep(1)

        return([nx, ny, nw])    

    def GETLidarDataAL(self):
        fun.StartTestIO()
        LAD = fun.ShareLib.LidarAngleData.read()
        LRD = fun.ShareLib.LidarRangeData.read()
        fun.BreakTestIO()

        LD = []

        for i in range(len(LAD)):
            k = []

            if math.isnan(LRD[i]): LRD[i] = 0.0

            LAD[i] = round(LAD[i] * 180 / math.pi, 1)
            LRD[i] = round(LRD[i] * 100, 1)

            if abs(LAD[i]) > 180: LAD[i] = 0.0
            elif abs(LRD[i]) > 9999999: LRD[i] = 0.0 

            if LRD[i] == 0.0: continue

            if LAD[i] >= 20 and LAD[i] <= 160:
                k.append(LAD[i])
                k.append(LRD[i])
                LD.append(k)

        LD = sorted(LD, key = lambda x: x[0])
        return LD
    
    def GETLidarDataXY(self, dataA, LXY, disLO):
        dataXY = []
        for i in dataA:
            l = i[1]
            angle = i[0] / 180 * math.pi
            x = LXY[0] - l * math.cos(angle)
            y = LXY[1] + l * math.sin(angle)

            x = round(x, 1)
            y = round(y, 1)

            dataXY.append([x, y + disLO])

        return dataXY
