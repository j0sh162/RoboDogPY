from RoboDogPIN import TaskSpaceManipulator
from read_write_csv import save_parameters
import numpy as np


def get_error(target_com, curr_com):
    return np.absolute(target_com - curr_com)

"""
eps = 10**-4
YAY 5 _ 946
YAY 10 _ 460
"""
def run_experiment_p_control(kp, eps, target_com=[0, 0.01, -0.036]):
    kd = 0
    task_space = TaskSpaceManipulator("go1_description/urdf/go1.urdf", kp, kd)
    start_com = task_space.calc_com().tolist()
    task_space.set_target(target_com)
    print("Start loop kp = ", kp)
    diff_x = 100
    diff_y = 100
    diff_z = 100
    # or ((diff_x > eps) and (diff_y > eps) and (diff_z > eps)):
    for i in range(200000):
        task_space.p_control_iterate()
        curr_com = task_space.calc_com()
        save_parameters(i, kp, kd, start_com, target_com, curr_com, "no_resolve")
        diff_x = abs(curr_com[0] - target_com[0])
        diff_y = abs(curr_com[1] - target_com[1])
        diff_z = abs(curr_com[2] - target_com[2])
        if (diff_x <= eps) and (diff_y <= eps) and (diff_z <= eps):
            print("YAY", kp, "_", i)
            break
        if i % 10000 == 0:
            print("Progress i = ", i)
    print("End loop kp = ", kp)

def run_experiment_damped(kp, eps, damping_factor, target_com=[0, 0.01, -0.036]):
    kd = 0
    task_space = TaskSpaceManipulator("go1_description/urdf/go1.urdf", kp, kd)
    start_com = task_space.calc_com()
    task_space.set_target(target_com)
    print("Start loop kp = ", kp)
    diff_x = 100
    diff_y = 100
    diff_z = 100
    # or ((diff_x > eps) and (diff_y > eps) and (diff_z > eps)):
    for i in range(200000):
        task_space.damped_least_sqaures_iterate(damping_factor)
        curr_com = task_space.calc_com()
        save_parameters(i, kp, kd, start_com, target_com, curr_com, "damped_least_squares", damping_factor)
        diff_x = abs(curr_com[0] - target_com[0])
        diff_y = abs(curr_com[1] - target_com[1])
        diff_z = abs(curr_com[2] - target_com[2])
        if (diff_x <= eps) and (diff_y <= eps) and (diff_z <= eps):
            print("YAY", kp, "_", i)
            break
        if i % 10000 == 0:
            print("Progress i = ", i)
    print("End loop kp = ", kp)

def run_experiment_null_space(kp, eps, alpha, target_com=[0, 0.01, -0.036]):
    kd = 0
    task_space = TaskSpaceManipulator("go1_description/urdf/go1.urdf", kp, kd)
    start_com = task_space.calc_com()
    task_space.set_target(target_com)
    print("Start loop kp = ", kp)
    diff_x = 100
    diff_y = 100
    diff_z = 100
    # or ((diff_x > eps) and (diff_y > eps) and (diff_z > eps)):
    for i in range(200000):
        task_space.null_space_iterate(alpha)
        curr_com = task_space.calc_com()
        save_parameters(i, kp, kd, start_com, target_com, curr_com, "null_space_projection", alpha)
        diff_x = abs(curr_com[0] - target_com[0])
        diff_y = abs(curr_com[1] - target_com[1])
        diff_z = abs(curr_com[2] - target_com[2])
        if (diff_x <= eps) and (diff_y <= eps) and (diff_z <= eps):
            print("YAY", kp, "_", i)
            break
        if i % 10000 == 0:
            print("Progress i = ", i)
    print("End loop kp = ", kp)

if __name__ == "__main__":

    print("=================Start main loop=================")
    run_experiment_p_control(5, 10**-4)
    run_experiment_p_control(10, 10**-4)
    run_experiment_p_control(15, 10**-4)
    print("=================Finish main loop=================")

    # do target coms: 1 good, 1 singularity
    # singularity - PD controls fails, Damped/Null space - dont
    # only PD fails - singularity (target com not reached) (crazy values)
    # all 3 fails -- target com is completely unreachable (stucky wucky)


    #progress_com.append(task_space.calc_com())


        # task_space.damped_least_sqaures_iterate(damping_factor=0.1)
        # task_space.null_space_iterate(alpha = 0.1) # alpha - null space ??? velocity/ secondary obj velocity

