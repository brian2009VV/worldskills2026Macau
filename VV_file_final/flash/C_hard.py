import time
import sys
import os

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
disLO = 20  # Distance between radar and chassis center
disOO = 30 # Distance between object and the centre of the robot
wall_len = 200  # Total length of the wall
layer_wide = 80  # The wide of one layer
hole_len = 70  # The size of the hole
obj_m_err = 10  # The width of the holes on both sides of obj_m is not included.
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
reset_left_speed = 20  # Ascent speed when resetting the robotic arm
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

def reset_arm(TF: bool) -> None:
    f.MoveARM(TF, reset_turn_speed, reset_left_speed, reset_arm_pose[0], reset_arm_pose[1], reset_arm_pose[2], reset_arm_pose[3], reset_arm_pose[4], reset_arm_pose[5])

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
    f.RelativeXYW([0, 0, -90], v_h, 5, 1.5)

    for i in range(4):
        hole_x = find_hole_x(i)
        f.RelativeXYW([0, 0, -90], v_h, 5, 1.5)
        f.RelativeXYW([x - hole_x, 0, 0], v_h, 5, 1.5)
        f.RelativeXYW([0, 0, 90], v_h, 5, 1.5)

        f.RelativeXYW([0, 0, -90], v_h, 5, 1.5)
        f.CalibrateFRONT(hole_x - disLO)
        f.RelativeXYW([0, 0, 90], v_h, 5, 1.5)

        f.RelativeXYW([f_dis, 0, 0], v_f, 2, 1.5)
        x = hole_x

    #go to end point
    f.CalibrateFRONT(20)
    f.RelativeXYW([0, 0, -90], v_h, 5, 1.5)
    f.CalibrateFRONT(20)
    f.OFFStartLED()



