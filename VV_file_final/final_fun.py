import time
import sys
import os
import math
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Func import * 
shareLib = LoadShareLib()
fun = Func(shareLib)

class self_function:
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

    def GETrealXYZthroughcamfaceonground(self, CAMXYZ, objectXY, framesize):
        #XYZ origin is the centre of the robot
        wideangle = 66
        verticalangle = 53

        camx = CAMXYZ[0]
        camy = CAMXYZ[1]
        camz = CAMXYZ[2]

        realwide = camz * math.tan(wideangle / 2 * math.pi / 180)
        realhigh = camz * math.tan(verticalangle / 2 * math.pi / 180)

        fx = -(framesize[0] / 2 - objectXY[0])
        fy = framesize[1] / 2 - objectXY[1]

        realx = fx / framesize[0] * realwide
        realy = fy / framesize[1] * realhigh
        realz = 0

        return (round(camx + realx, 1), round(camy + realy, 1), round(realz, 1))

    def MoveARM(self, NeedtoReset, speedTurn, speedLeft, armHigh, turnAngle, rotateAngle, clampVal, raiseAngle, telescopicVal):
        #Common situation: speedTrun = 5, speedLeft = 10
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
        disOT = 12
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

        if l < 0 or l > 9:
            print("fail to get to point")
            return False
        
        self.MoveARM(ResetTF, Resetspeed[0], Resetspeed[1], z, Turnangle, Rotateangle, C, 90 - A, l)
        return True
        
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
        time.sleep(1)
        fun.StartTestIO()
        LAD = fun.ShareLib.LidarAngleData.read()
        LRD = fun.ShareLib.LidarRangeData.read()
        fun.BreakTestIO()
        time.sleep(1)

        LD = []

        for i in range(len(LAD)):
            k = []

            if math.isnan(LRD[i]): LRD[i] = 0.0

            LAD[i] = round(LAD[i] * 180 / math.pi, 1)
            LRD[i] = round(LRD[i] * 100, 1)

            if abs(LAD[i]) > 180: LAD[i] = 0.0
            elif abs(LRD[i]) > 9999999: LRD[i] = 0.0 

            if LRD[i] == 0.0: continue

            if LAD[i] >= 30 and LAD[i] <= 150:
                k.append(LAD[i])
                k.append(LRD[i])
                LD.append(k)

        LD = sorted(LD, key = lambda x: x[0])
        return LD
    
    def TURNandGETLidarDataXY(self, TA, LXY, disLO):
        #turn left: TA < 0, turn right: TA > 0
        self.RelativeXYW([0, 0, TA], 15, 5, 1)
        dataA = self.GETLidarDataAL()
        self.RelativeXYW([0, 0, -TA], 15, 5, 1)

        dataXY = []
        for i in dataA:
            l = i[1]
            angle = (i[0] + TA) / 180 * math.pi
            x = LXY[0] - l * math.cos(angle)
            y = LXY[1] + l * math.sin(angle)
            
            x = x + disLO * math.sin(TA / 180 * math.pi)
            y = y + disLO * math.cos(TA / 180 * math.pi)

            x = round(x, 1)
            y = round(y, 1)

            dataXY.append([x, y])

        return dataXY

    def IDENTIFYWallinInterval(self, Turnangle, LXY, disLO, t_err, dx_err, Xrange, Yrange):
        WD = self.TURNandGETLidarDataXY(Turnangle, LXY, disLO)

        WallData = []
        for i in WD:
            if i[0] >= Xrange[0] and i[0] <= Xrange[1] and i[1] >= Yrange[0] and i[1] <= Yrange[1]: WallData.append(i)
            
        W = []
        if len(WallData) < 3: return W

        r = 0
        while r < len(WallData):
            k = 0
            t = []
            for i in range(r + 2, len(WallData)):
                dy1 = WallData[i - 1][1] - WallData[i - 2][1]
                dx1 = WallData[i - 1][0] - WallData[i - 2][0]

                dy2 = WallData[i][1] - WallData[i - 1][1]
                dx2 = WallData[i][0] - WallData[i - 1][0]

                r = i + 1
                if dx1 == 0 or dx2 == 0: continue

                if abs(dy1 / dx1 - dy2 / dx2) <= t_err and abs(dx1) <= dx_err and abs(dx2) <= dx_err:
                    k = (dy1 / dx1 + dy2 / dx2) / 2
                    t.append((WallData[i - 2][0], WallData[i - 2][1]))
                    break

            if len(t) == 0: break

            while r < len(WallData):
                dy = WallData[r][1] - WallData[r - 1][1]
                dx = WallData[r][0] - WallData[r - 1][0]

                if dx == 0: break
                if abs(dy / dx - k) <= t_err and abs(dx) <= dx_err: 
                    k = (k + dy / dx) / 2
                    r += 1
                else: 
                    if r + 1 < len(WallData):
                        dypi = WallData[r + 1][1] - WallData[r - 1][1]
                        dxpi = WallData[r + 1][0] - WallData[r - 1][0]

                        if dxpi == 0: break
                        if abs(dypi / dxpi - k) <= t_err and abs(dxpi) <= dx_err:
                            k = (k + dypi / dxpi) / 2
                            r += 2
                        else: break
                    else: break

            t.append((WallData[r - 1][0], WallData[r - 1][1]))
            t.sort(key = lambda t : t[0])
            W.append(t)

        return W

    def WallIntervalMerging(self, WM, l_err, t_err, dx_err):
        #Merge Overlapping Intervals
        i = 1
        n = len(WM)
        while i < n:
            j = i - 1
            c = 0
            while j >= 0:
                if WM[j][1][0] >= WM[i][0][0]:
                    k1 = (WM[j][0][1] - WM[j][1][1]) / (WM[j][0][0] - WM[j][1][0])
                    k2 = (WM[i][0][1] - WM[i][1][1]) / (WM[i][0][0] - WM[i][1][0])
                    ypi = k1 * (WM[i][0][0] - WM[j][0][0]) + WM[j][0][1]
                    if abs(ypi - WM[i][0][1]) <= l_err and abs(k1 - k2) <= t_err:
                        if WM[i][1][0] > WM[j][1][0]:
                            tt = [WM[j][0], WM[i][1]]
                            WM.append(tt)
                            del(WM[j])
                            del(WM[i - 1])
                        else: del(WM[i])
                        WM.sort(key = lambda t: t[0][0])
                        n -= 1
                        c = 1
                        break
                j -= 1

            if not c: i += 1

        #Merge Adjacent Intervals
        i = 1
        n = len(WM)

        while i < n:
            k1 = (WM[i - 1][0][1] - WM[i - 1][1][1]) / (WM[i - 1][0][0] - WM[i - 1][1][0])
            k2 = (WM[i][0][1] - WM[i][1][1]) / (WM[i][0][0] - WM[i][1][0])

            ypi = k1 * (WM[i][0][0] - WM[i - 1][0][0]) + WM[i - 1][0][1]

            if WM[i][0][0] >= WM[i - 1][1][0] and abs(k1 - k2) <= t_err and abs(WM[i - 1][1][0] - WM[i][0][0]) <= dx_err and abs(ypi - WM[i][0][1]) <= l_err:
                WM.append([WM[i - 1][0], WM[i][1]])
                del(WM[i])
                del(WM[i - 1])
                WM.sort(key = lambda t: t[0][0])
                n -= 1

            else: i += 1

        return WM

    def WallModeling(self, start_x, walk_times, wall_len, Turnangle, dL, dR, disLO, t_err, dx_err, l_err):
        #Common situation: walk_times = 1, wall_len = 200, Turnangle = 60, dL = 40, dR = 40, disLO = 20, t_err = 0.5, dx_err = 10, l_err = 20
        WD = []
        
        if start_x <= wall_len / 2:
            self.RelativeXYW([0, 0, -90], 15, 5, 1.5)
            k = -(wall_len - dL - dR) // walk_times
            self.CalibrateFRONT(20)
            x = 20 + disLO
            self.RelativeXYW([x - dL, 0, 0], 15, 5, 1.5)
            self.RelativeXYW([0, 0, 90], 15, 5, 1.5)

        else:
            self.RelativeXYW([0, 0, 90], 15, 5, 1.5)
            k =  (wall_len - dL - dR) // walk_times
            self.CalibrateFRONT(20)
            x = wall_len - 20 - disLO
            self.RelativeXYW([(wall_len - dR) - x, 0, 0], 15, 5, 1.5)
            self.RelativeXYW([0, 0, -90], 15, 5, 1.5)

        for i in range(walk_times + 1):
            if i == 0:
                if k < 0:
                    D = self.IDENTIFYWallinInterval(-Turnangle, (x, 0), disLO, t_err, dx_err, (0, wall_len), (0, 9999999))
                    WD = WD + D
                    D = self.IDENTIFYWallinInterval(0, (x, 0), disLO, t_err, dx_err, (0, wall_len), (0, 9999999))
                    WD = WD + D
                    D = self.IDENTIFYWallinInterval(Turnangle, (x, 0), disLO, t_err, dx_err, (0, wall_len), (0, 9999999))
                    WD = WD + D
                else:
                    D = self.IDENTIFYWallinInterval(Turnangle, (x, 0), disLO, t_err, dx_err, (0, wall_len), (0, 9999999))
                    WD = WD + D
                    D = self.IDENTIFYWallinInterval(0, (x, 0), disLO, t_err, dx_err, (0, wall_len), (0, 9999999))
                    WD = WD + D
                    D = self.IDENTIFYWallinInterval(-Turnangle, (x, 0), disLO, t_err, dx_err, (0, wall_len), (0, 9999999))
                    WD = WD + D
            
            elif i == walk_times:
                if k < 0:
                    D = self.IDENTIFYWallinInterval(Turnangle, (x, 0), disLO, t_err, dx_err, (0, wall_len), (0, 9999999))
                    WD = WD + D
                    D = self.IDENTIFYWallinInterval(0, (x, 0), disLO, t_err, dx_err, (0, wall_len), (0, 9999999))
                    WD = WD + D
                    D = self.IDENTIFYWallinInterval(-Turnangle, (x, 0), disLO, t_err, dx_err, (0, wall_len), (0, 9999999))
                    WD = WD + D
                else:
                    D = self.IDENTIFYWallinInterval(-Turnangle, (x, 0), disLO, t_err, dx_err, (0, wall_len), (0, 9999999))
                    WD = WD + D
                    D = self.IDENTIFYWallinInterval(0, (x, 0), disLO, t_err, dx_err, (0, wall_len), (0, 9999999))
                    WD = WD + D
                    D = self.IDENTIFYWallinInterval(Turnangle, (x, 0), disLO, t_err, dx_err, (0, wall_len), (0, 9999999))
                    WD = WD + D
            else:
                D = self.IDENTIFYWallinInterval(0, (x, 0), disLO, t_err, dx_err, (0, wall_len), (0, 9999999))
                WD = WD + D
                
            if i <= walk_times - 2:
                self.RelativeXYW([0, 0, -90], 15, 5, 1.5)
                self.RelativeXYW([k, 0, 0], 15, 5, 1.5)
                self.RelativeXYW([0, 0, 90], 15, 5, 1.5)

                x += -k

            if i == walk_times - 1:
                if k < 0:
                    self.RelativeXYW([0, 0, 90], 15, 5, 1.5)
                    self.CalibrateFRONT(dR - disLO)
                    self.RelativeXYW([0, 0, -90], 15, 5, 1.5)

                    x += -k
                else:
                    self.RelativeXYW([0, 0, -90], 15, 5, 1.5)
                    self.CalibrateFRONT(dL - disLO)
                    self.RelativeXYW([0, 0, 90], 15, 5, 1.5)

                    x += -k
                
            time.sleep(0.5)
        
        WD.sort(key = lambda t: t[0][0])
        WD = self.WallIntervalMerging(WD, l_err, t_err, dx_err)

        return WD

    def DetermineHOLEinHorizontalWall(self, WM, MAX_W_D, MIN_H_LEN, wall_len):
        #WM need to be the list form of WallModeling function returned
        #Common situations: MAX_W_D = 80, MIN_H_LEN = 40, wall_len = 200
        #Interval Mapping
        W = []
        WM.sort(key = lambda t: (t[0][1] + t[1][1]) / 2)
        for i in range(len(WM)):
            if len(W) == 0: W.append(WM[i])
            else:
                n = len(W)
                for j in range(n):
                    kpi = (WM[i][0][1] - WM[i][1][1]) / (WM[i][0][0] - WM[i][1][0])
                    y1 = kpi * (W[j][0][0] - WM[i][0][0]) + WM[i][0][1]
                    y2 = kpi * (W[j][1][0] - WM[i][0][0]) + WM[i][0][1]
                    y1 = round(y1, 1)
                    y2 = round(y2, 1)
                    if WM[i][0][0] < W[j][0][0]:
                        if WM[i][1][0] < W[j][0][0]: 
                            W.append(WM[i])
                            break
                        elif WM[i][1][0] <= W[j][1][0] and WM[i][1][0] >= W[j][0][0]: 
                            W.append([WM[i][0], (W[j][0][0], y1)])
                            break
                        else: 
                            W.append([WM[i][0], (W[j][0][0], y1)])
                            WM[i] = [(W[j][1][0], y2), WM[i][1]]
                            if j + 1 == n: W.append(WM[i])
                    elif WM[i][0][0] >= W[j][0][0] and WM[i][0][0] <= W[j][1][0]:
                        if WM[i][1][0] <= W[j][1][0]: break
                        else: 
                            WM[i] = [(W[j][1][0], y2), WM[i][1]]
                            if j + 1 == n: W.append(WM[i])
                    else:
                        if j + 1 == n: W.append(WM[i])

                W.sort(key = lambda t: t[0][0])
                
        #Unreasonable Interval Merging
        i = 1
        n = len(W)
        while i < n:
            d1 = (W[i][0][1] + W[i][1][1]) / 2
            d2 = (W[i - 1][0][1] + W[i - 1][1][1]) / 2
            if d1 > MAX_W_D and d2 > MAX_W_D:
                tt = [(W[i - 1][0][0], round(max(d1, d2), 1)), (W[i][1][0], round(max(d1, d2), 1))]
                W.insert(i - 1, tt)
                del (W[i])
                del (W[i])
                n -= 1
            else:
                i += 1

        # Check Hole
        # 1. Gaps Between Intervals
        if W[0][0][0] >= MIN_H_LEN:
            return (W, round(W[0][0][0] / 2, 1))
        elif wall_len - W[-1][1][0] >= MIN_H_LEN:
            return (W, round((wall_len + W[-1][1][0]) / 2, 1))
        else:
            for i in range(1, len(W)):
                x1 = W[i - 1][1][0]
                x2 = W[i][0][0]
                if x2 - x1 >= MIN_H_LEN: return (W, round((x1 + x2) / 2, 1))

        # 2. Check Intervals
        count = 0
        Wpi = W.copy()
        if len(Wpi) == 1: return -1
        while count < len(Wpi):
            m = -1
            t = -1
            for i in range(len(Wpi)):
                d = (Wpi[i][0][1] + Wpi[i][1][1]) / 2
                if m < d:
                    t = i
                    m = d
            count += 1

            if t == 0:
                if Wpi[t + 1][0][0] >= MIN_H_LEN: return (W, round(Wpi[t + 1][0][0] / 2, 1))
            elif t == len(Wpi) - 1:
                if wall_len - Wpi[t - 1][1][0] >= MIN_H_LEN: return (W, round((wall_len + Wpi[t - 1][1][0]) / 2, 1))
            else:
                if Wpi[t + 1][0][0] - Wpi[t - 1][1][0] >= MIN_H_LEN: return (W, round((Wpi[t + 1][0][0] + Wpi[t - 1][1][0]) / 2, 1))

            Wpi.append([(W[t][0][0], -1), (W[t][1][0], -1)])
            del(Wpi[t])
            Wpi.sort(key=lambda t: t[0][0])

        return -1
