import time
import sys
import os
import cv2
from ultralytics import YOLO

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
from Func import *
from task.VV_file_final.flash.quick_fun import quick_function

shareLib = LoadShareLib()
fun = Func(shareLib)
f = quick_function()

Wall = [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0]]

# Related to functions#####################################################################################
model_path = '/home/pi/PickPro/python/task/VV_file_final/model/best.pt'  # Model path
model_correct_p = 0.7  # Required model accuracy
disLO = 20  # Distance between radar and chassis center
disOO = 30 # Distance between object and the centre of the robot
wall_len = 200  # Total length of the wall
layer_wide = 80  # The wide of one layer
hole_len = 70  # The size of the hole
obj_m_err = 5  # The width of the holes on both sides of obj_m is not included.
max_next_layer_object_dis = 110  # The maxium distance of the object in the next layer
camxyz40 = (0, 35, 39)  # The coordinates of the camera when it is raised 40 degrees
camxyz50 = (0, 35, 43)  # The coordinates of the camera when it is raised 50 degrees
camxyz00 = (-2, 28, 46)  # The coordinates of the camera when it is vertical to the ground
calibration_threshold = 3  # Allowable error between object and true coordinates
max_mis_detect_num = 3  # Maximum number of detecting object
max_calibration_times = 3  # Maximum number of calibrations
min_repeated_obj_dis = 10  # The smallest distance between two objects

# Related to movement#######################################################################################
v_f = 30  # The robot's forward speed
v_h = 30  # The speed of the robot's lateral movement
X_move_range = (40, 160)  # Robot's range of motion
f_dis = 90  # Distance when robot go through the hole

# Related to robot arm######################################################################################
object_size = 3
reset_turn_speed = 5  # Rotation speed when resetting the robotic arm
reset_left_speed = 12  # Ascent speed when resetting the robotic arm
reset_arm_pose = [60, 0, 0, 12, 90, -2]  # The pose of the robotic arm when the robot moves forward
check_arm_pose_l = [60, -188, 45, 15, 50, 9]
check_arm_pose_m = [60, -188, 0, 15, 50, 9]
check_arm_pose_d = [60, -188, 0, 15, 40, 9]
check_arm_pose_r = [60, -188, -45, 15, 50, 9]
look_down_arm_pose = [60, -188, 0, 12, 0, 9]
place_object_pose = [70, 0, 0, 12, 0, -2]

############################################################################################################
def find_hole_x(n: int) -> float:
    for i in range(len(Wall[n])):
        if Wall[n][i] == 0: return (i * 10 + hole_len / 2)
    return 0

def del_samilar_dis_digit_in_list(l: list) -> list:
    res = []
    for i in l:
        if len(res) == 0: res.append(i)
        else: 
            can = 1
            for j in res:
                if ((j[0] - i[0]) ** 2 + (j[1] - i[1]) ** 2) ** 0.5 <= min_repeated_obj_dis:
                    can = 0
                    break

            if can: res.append(i)

    return res

def reset_arm(TF: bool) -> None:
    f.MoveARM(TF, reset_turn_speed, reset_left_speed, reset_arm_pose[0], reset_arm_pose[1], reset_arm_pose[2], reset_arm_pose[3], reset_arm_pose[4], reset_arm_pose[5])

def ctrl_arm_check_pose(TF: bool, check_pose: list) -> None:
    f.MoveARM(TF, reset_turn_speed, reset_left_speed, check_pose[0], check_pose[1], check_pose[2], check_pose[3], check_pose[4], check_pose[5])

def ctrl_arm_look_down(TF: bool) -> None:
    f.MoveARM(TF, reset_turn_speed, reset_left_speed, look_down_arm_pose[0], look_down_arm_pose[1], look_down_arm_pose[2], look_down_arm_pose[3], look_down_arm_pose[4], look_down_arm_pose[5])

def get_model_object_realxyz(correct_pre: float, camxyz: tuple, gamma: float, sita: float) -> list:
    model = YOLO(model_path)

    R = []
    camera = cv2.VideoCapture(0)
    success, frame = camera.read()
    if not success: return R

    boxes = model(frame)[0].boxes
    for cls, conf, xyxy in zip(boxes.cls, boxes.conf, boxes.xyxy):
        if float(conf) >= correct_pre:
            x1, y1, x2, y2 = map(int, xyxy)
            oxy = (x1 / 2 + x2 / 2, y1 / 2 + y2 / 2)
            roxyz = f.GETrealXYZ(camxyz, oxy, gamma, sita)
            R.append((int(cls), round(float(conf), 1), roxyz))
            cv2.putText(frame, str(int(cls)) + " (" + str(roxyz[0]) + ", " + str(roxyz[1]) + ", " + str(roxyz[2]) + ")", [x1 + 5, y1 + 20], cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 255, 0), 2, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    cv2.imwrite('/home/pi/PickPro/python/task/VV_file_final/cap_photo/' + 'ans' + '.jpg', frame)
    camera.release()

    return R

def dynamic_calibration_object() -> None:
    count_cal = 0
    count_det = 0
    while True:
        if count_det >= max_mis_detect_num: break
        if count_cal >= max_calibration_times: break

        ctrl_arm_look_down(False)
        E = get_model_object_realxyz(model_correct_p, camxyz00, 0, 0)
        if len(E) == 0: 
            count_det += 1
            continue
        
        E.sort(key = lambda t : -t[1])

        if abs(disOO - E[0][2][1]) <= calibration_threshold: break
        else:
            reset_arm(False)
            print("hihiihii       ", E[0][2][1] - disOO)
            count_cal += 1
            f.RelativeXYW([-30, 0, 0], v_f, 2, 1.5)
            f.RelativeXYW([30 + E[0][2][1] - disOO, 0, 0], v_f, 2, 1.5)

def three_consecutive_checks(TF: bool, x: float) -> list:
    rres = []
    ctrl_arm_check_pose(False, check_arm_pose_l)
    res = get_model_object_realxyz(model_correct_p, camxyz50, -abs(check_arm_pose_l[2]), check_arm_pose_l[4])
    for i in res: rres.append(i)

    ctrl_arm_check_pose(False, check_arm_pose_m)
    res = get_model_object_realxyz(model_correct_p, camxyz50, abs(check_arm_pose_m[2]), check_arm_pose_m[4])
    for i in res: rres.append(i)

    ctrl_arm_check_pose(False, check_arm_pose_d)
    res = get_model_object_realxyz(model_correct_p, camxyz40, abs(check_arm_pose_d[2]), check_arm_pose_d[4])
    for i in res: rres.append(i)

    ctrl_arm_check_pose(False, check_arm_pose_r)
    res = get_model_object_realxyz(model_correct_p, camxyz50, abs(check_arm_pose_r[2]), check_arm_pose_r[4])
    for i in res: rres.append(i)

    reset_arm(TF)

    obj = []
    for i in rres:
        if i[2][1] > max_next_layer_object_dis: continue
        obj.append((x + i[2][0], i[2][1]))

    obj.sort(key = lambda t : t[0])
    return obj

def go_forward_and_clamp_objects(x: float, ob_x_list: list, is_travel_vertically: bool) -> int:
    for i in range(len(ob_x_list)):
        reset_arm(False)

        if not is_travel_vertically:
            if ob_x_list[i] < x:
                if ob_x_list[i] + disOO > X_move_range[1] or ob_x_list[i] + disOO < X_move_range[0]: continue
            else:
                if ob_x_list[i] - disOO > X_move_range[1] or ob_x_list[i] - disOO < X_move_range[0]: continue

        if not is_travel_vertically: 
            if x > ob_x_list[i]: f.CalibrateFRONT(x - disLO)
            else: f.CalibrateFRONT(wall_len - x - disLO)

        if not is_travel_vertically:
            if x > ob_x_list[i]: 
                f.RelativeXYW([x - ob_x_list[i] - disOO, 0, 0], v_f, 2, 1.5)
                x = ob_x_list[i] + disOO
            else: 
                f.RelativeXYW([ob_x_list[i] - x - disOO, 0, 0], v_f, 2, 1.5)
                x = ob_x_list[i] - disOO
        else:
            f.RelativeXYW([ob_x_list[i] - x - disOO, 0, 0], v_f, 2, 1.5)
            x = ob_x_list[i] - disOO

        dynamic_calibration_object()

        E = []
        count = 0
        while True:
            if count >= max_mis_detect_num: break
            E = get_model_object_realxyz(model_correct_p, camxyz00, 0, 0)
            if len(E) == 0: count += 1
            else: break

        if len(E) == 0: continue

        E.sort(key = lambda t : -t[1])

        f.MoveARMXYZAC(False, (reset_turn_speed, reset_left_speed), (E[0][2][0], E[0][2][1] + 3, E[0][2][2] + 5), 90, 18)
        f.MoveARMXYZAC(False, (reset_turn_speed, reset_left_speed), (E[0][2][0], E[0][2][1] + 3, E[0][2][2] + 5), 90, object_size)
        time.sleep(2)
        f.MoveARMXYZAC(False, (reset_turn_speed, reset_left_speed), (E[0][2][0], E[0][2][1] + 3, E[0][2][2] + 20), 90, object_size)
        f.MoveARM(False, reset_turn_speed, reset_left_speed, place_object_pose[0], place_object_pose[1], place_object_pose[2], object_size, place_object_pose[4], place_object_pose[5])
        f.MoveARM(False, reset_turn_speed, reset_left_speed, place_object_pose[0], place_object_pose[1], place_object_pose[2], place_object_pose[3], place_object_pose[4], place_object_pose[5])

    reset_arm(False)
    return x

def clamp_all_objects_in_one_layer(x: float, obj_l_x: list, obj_r_x: list) -> float:
    if x < wall_len // 2:
        if len(obj_l_x) != 0:
            f.RelativeXYW([0, 0, -90], v_h, 2, 1.5)
            x = go_forward_and_clamp_objects(x, obj_l_x, False)
            f.RelativeXYW([0, 0, -180], v_h, 2, 1.5)
        else:
            f.RelativeXYW([0, 0, 90], v_h, 2, 1.5)

        if len(obj_r_x) != 0:
            x = go_forward_and_clamp_objects(x, obj_r_x, False)

            f.RelativeXYW([0, 0, -180], v_h, 2, 1.5)
            f.RelativeXYW([x - wall_len // 2 + 5, 0, 0], v_h, 2, 1.5)
            f.CalibrateFRONT(wall_len // 2 - disLO)
            f.RelativeXYW([0, 0, 90], v_h, 2, 1.5)

        else:
            f.RelativeXYW([wall_len // 2 - x + 5, 0, 0], v_h, 2, 1.5)
            f.CalibrateFRONT(wall_len // 2 - disLO)
            f.RelativeXYW([0, 0, -90], v_h, 2, 1.5)

    else:
        if len(obj_r_x) != 0:
            f.RelativeXYW([0, 0, 90], v_h, 2, 1.5)
            x = go_forward_and_clamp_objects(x, obj_r_x, False)
            f.RelativeXYW([0, 0, -180], v_h, 2, 1.5)
        else:
            f.RelativeXYW([0, 0, -90], v_h, 2, 1.5)

        if len(obj_l_x) != 0:
            x = go_forward_and_clamp_objects(x, obj_l_x, False)

            f.RelativeXYW([0, 0, -180], v_h, 2, 1.5)
            f.RelativeXYW([wall_len // 2 - x + 5, 0, 0], v_h, 2, 1.5)
            f.CalibrateFRONT(wall_len // 2 - disLO)
            f.RelativeXYW([0, 0, -90], v_h, 2, 1.5)

        else:
            f.RelativeXYW([x - wall_len // 2 + 5, 0, 0], v_h, 2, 1.5)
            f.CalibrateFRONT(wall_len // 2 - disLO)
            f.RelativeXYW([0, 0, 90], v_h, 2, 1.5)

    x = wall_len // 2
    return x

if __name__ == "__main__":
    #reset everything
    f.ZeroXYW()
    reset_arm(True)
    f.RelativeXYW([0, 0, -180], v_h, 5, 1.5)
    f.CalibrateFRONT(20)
    f.RelativeXYW([0, 0, -90], v_h, 5, 1.5)
    f.CalibrateFRONT(20)
    
    f.WAITPUSHStartLED()
    time.sleep(0.5)
    reset_arm(True)
    
    x = 160
    #check and clamp object in layer one
    f.RelativeXYW([0, 0, -180], v_h, 5, 1.5)
    ctrl_arm_check_pose(False, check_arm_pose_m)
    res = get_model_object_realxyz(model_correct_p, camxyz50, 0, 50)

    obj_x = []
    for i in res:
        if abs(i[2][0]) <= layer_wide // 2 + 10: obj_x.append(x - i[2][1])
    obj_x.sort(reverse = True)

    if len(res) != 0: x = go_forward_and_clamp_objects(x, obj_x, False)
    else: reset_arm(False)

    f.RelativeXYW([x - wall_len // 2, 0, 0], v_f, 2, 1.5)
    f.CalibrateFRONT(wall_len // 2 - disLO)
    x = wall_len // 2

    f.RelativeXYW([0, 0, 90], v_h, 2, 1.5)

    #go through layer one to five and clamp object in layer two to five
    for i in range(4):
        obj = three_consecutive_checks(False, x)
        hole_x = find_hole_x(i)
        obj_l, obj_m, obj_r = [], [], []
        for i in obj:
            if i[0] < hole_x - hole_len / 2 + obj_m_err: obj_l.append(i)
            elif i[0] >= hole_x - hole_len / 2 + obj_m_err and i[0] <= hole_x + hole_len / 2 - obj_m_err: obj_m.append(i)
            else: obj_r.append(i)

        obj_l = del_samilar_dis_digit_in_list(obj_l)
        obj_m = del_samilar_dis_digit_in_list(obj_m)
        obj_r = del_samilar_dis_digit_in_list(obj_r)

        obj_l_x, obj_m_x, obj_r_x = [], [], []
        for i in obj_l: obj_l_x.append(i[0])
        for i in obj_m: obj_m_x.append(i[1])
        for i in obj_r: obj_r_x.append(i[0])

        obj_l_x.sort(reverse = True)
        obj_m_x.sort()
        obj_r_x.sort()

        print(obj_l_x, obj_m_x, obj_r_x)

        if hole_x < x: f.RelativeXYW([0, hole_x - x - 5, 0], v_h, 2, 1.5)
        else: f.RelativeXYW([0, hole_x - x + 5, 0], v_h, 2, 1.5)

        f.RelativeXYW([0, 0, -90], v_h, 2, 1.5)
        f.CalibrateFRONT(hole_x - disLO)
        f.RelativeXYW([0, 0, 90], v_h, 2, 1.5)
        x = hole_x

        if len(obj_m_x) == 0: f.RelativeXYW([f_dis, 0, 0], v_f, 2, 1.5)
        else:
            dy = go_forward_and_clamp_objects(0, obj_m_x, True)
            print(dy)
            f.RelativeXYW([f_dis - dy, 0, 0], v_f, 2, 1.5)

        x = clamp_all_objects_in_one_layer(x, obj_l_x, obj_r_x)

    #go to end point
    f.CalibrateFRONT(20)
    f.RelativeXYW([0, 0, -90], v_h, 5, 1.5)
    f.CalibrateFRONT(20)
    f.OFFStartLED()



