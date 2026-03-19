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
    
    def MoveARM(self, NeedtoReset, speedTurn, speedLeft, armHigh, turnAngle, rotateAngle, clampVal, raiseAngle, telescopicVal):
        Max_arm_high = 86.5

        Reset_ARM_LEFT = [
        (fun.ResetLiftFun, (speedLeft,)), 
        (fun.LiftCtrlFun, (-2,)),      
        ]
        Reset_TurnandLeft = [
        (fun.run_in_order, (Reset_ARM_LEFT,)),   
        (fun.ResetTurnFun, (speedTurn,)),
        ]

        if abs(turnAngle) <= 5 and NeedtoReset:
            Gototarget = [
            (fun.LiftCtrlFun, (-(Max_arm_high - armHigh), 10, 3,),),
            (fun.RaiseServoCtrl, (raiseAngle, 5,)),    
            (fun.RotatingServoCtrl, (rotateAngle, 5,)),
            (fun.ClampServoCtrl, (clampVal, 5,)),
            (fun.TelescopicServoCtrl, (telescopicVal, 5,)),
            ]
        else:
            Gototarget = [
            (fun.LiftCtrlFun, (-(Max_arm_high - armHigh), 10, 3,),),
            (fun.TurnCtrlFun, (turnAngle, 10, 3,),),
            (fun.RaiseServoCtrl, (raiseAngle, 5,)),    
            (fun.RotatingServoCtrl, (rotateAngle, 5,)),
            (fun.ClampServoCtrl, (clampVal, 5,)),
            (fun.TelescopicServoCtrl, (telescopicVal, 5,)),
            ]

        if NeedtoReset: results1 = fun.run_in_threads(Reset_TurnandLeft)
        results2 = fun.run_in_threads(Gototarget)
        return True
    
    def GETrealXYZ(self, CAMXYZ, OXY, gamma, sita):
        #cam turn left: gamma < 0
        #cam turn right: gamma > 0
        framesize = (640, 480)
        camangle = (66, 53)
        alpha = camangle[0] / 2 / 180 * math.pi
        beta = camangle[1] / 2  / 180 * math.pi
        sita = sita / 180 * math.pi

        z = CAMXYZ[2]

        l = z / math.cos(sita)
        lpi = z / math.cos(sita - beta) * math.cos(beta)
        print(l, lpi)

        w = lpi * math.tan(alpha)
        h = lpi * math.tan(beta)

        Rx = w * (OXY[0] - framesize[0] / 2) / (framesize[0] / 2)
        Ry = h * (framesize[1] / 2 - OXY[1]) / (framesize[1] / 2)
        print(OXY[0], OXY[1], w, h, Rx, Ry)

        deta = math.atan((l - lpi) / h)
        print(deta * 180 / math.pi)
        
        ypi = (Ry + h) * math.cos(deta) + z * math.tan(sita - beta)
        xpi = Rx
        k = (Ry + h) * math.sin(deta)
        print(k)

        x = xpi * z / (z - k)
        y = ypi * z / (z - k)

        print(x, y, z)
        return(x, y, z)
        '''
        print(l, lpi)
        w = l * math.tan(alpha / 180 * math.pi)
        wpi = lpi * math.tan(alpha / 180 * math.pi)

        hpi = lpi * math.tan(beta / 180 * math.pi)
        h = (hpi ** 2 + (lpi - l) ** 2) ** 0.5

        print(h)

        y = (framesize[1] - OXY[1]) / (framesize[1] / 2) * h

        print(y)
        k = h * wpi / (w - wpi)

        wpipi = (k + y) / k * wpi

        x = (OXY[0] - framesize[0] / 2) / (framesize[0] / 2) * wpipi
        y += z * math.tan((sita - beta) / 180 * math.pi)
        '''
        print(x, y, z)
        r = (x ** 2 + y ** 2) ** 0.5
        
        if x == 0: threta = math.pi / 2
        else: threta = math.atan(y / x)
        
        print(threta * 180 / math.pi)

        if threta < 0: threta = math.pi + threta

        threta = threta - gamma * math.pi / 180

        x = r * math.cos(threta)
        y = r * math.sin(threta)

        x += CAMXYZ[0]
        y += CAMXYZ[1]
        
        x = round(x, 1)
        y = round(y, 1)
        z = round(z, 1)

        

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
