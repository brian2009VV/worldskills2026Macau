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
    
    def MoveARMXYZAC(self, ResetTF, Resetspeed, XYZ, A, C):
        #XYZ origin is the centre of the robot
        disOT = 10
        disTele = 16
        disClamp = 18
        disCL = 6.5

        ypi = disClamp * math.cos(A * math.pi / 180)
        zpi = disClamp * math.sin(A * math.pi / 180)

        x = XYZ[0]
        y = XYZ[1] - disOT - ypi
        z = XYZ[2] + zpi + disCL
        
        if y == 0: alpha = math.pi / 2
        else: alpha = math.atan(x / y)

        Turnangle = (-math.pi + alpha) * 180 / math.pi

        Rotateangle = alpha * 180 / math.pi

        l = (x ** 2 + y ** 2) ** 0.5 - disTele

        l = min(l, 9)
        l = max(-2, l)
        
        self.MoveARM(ResetTF, Resetspeed[0], Resetspeed[1], z, Turnangle, Rotateangle, C, 90 - A, l)
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
        
        r = (x ** 2 + y ** 2) ** 0.5
        
        if x == 0: threta = math.pi / 2
        else: threta = math.atan(y / x)
        
        print(threta * 180 / math.pi)

        if x < 0 and y > 0: threta = math.pi + threta
        elif x < 0 and y < 0: threta = math.pi + threta
        elif x > 0 and y < 0: threta = 2 * math.pi + threta

        threta = (threta - gamma * math.pi / 180) % (2 * math.pi)

        x = r * math.cos(threta)
        y = r * math.sin(threta)

        x += CAMXYZ[0]
        y += CAMXYZ[1]
        
        x = round(x, 1)
        y = round(y, 1)

        return (x, y, 0)
        

    def RelativeXYW(self, dp, v, p_e, a_e):
        dx = dp[0]
        dy = dp[1]
        dw = dp[2]

        d = math.sqrt(dx ** 2 + dy ** 2)

        op = fun.ReadCurPose()

        if dx == 0:
            if dy >= 0: t = math.pi / 2
            else: t = -math.pi / 2
        else:
            if dy >= 0:
                if dx >= 0: t = math.atan(dy / dx)
                else: t = math.pi + math.atan(dy / dx)
            else:
                if dx >= 0: t = math.atan(dy / dx)
                else: t = -math.pi + math.atan(dy / dx)

        a = op.theta_ * math.pi / 180 + t

        nx = int(op.x_ + d * math.cos(a))
        ny = int(op.y_ + d * math.sin(a))
        nw = int(op.theta_ + dw)

        nw = nw % 360

        if dx == 0 and dy == 0: fun.Rotate(dw)
        else: fun.TrackingXYFun(Pose(nx, ny, nw), v, p_e, a_e)

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
