from RoboDogPIN import TaskSpaceManipulator
from read_write_csv import save_parameters
import numpy as np


def get_error(target_com, curr_com):
    return np.absolute(target_com - curr_com)

if __name__ == "__main__":


    print("=================Start main loop=================")

    # kP values: 5, 10, 15, 50, 100, 150
    # kp = 1
    # kd = 0
    # task_space = TaskSpaceManipulator("go1_description/urdf/go1.urdf", kp, kd)
    # start_com = task_space.calc_com()
    # target_com = [0, 0.01, -0.036]
    # task_space.set_target(target_com)
    # print("Start loop kp = ", kp)
    # for i in range(200000):
    #     task_space.p_control_iterate()
    #     save_parameters(i, kp, kd, start_com, target_com, task_space.calc_com(), "no_resolve")
    #     if i % 10000 == 0:
    #         print("Progress i = ", i)
    # print("End loop kp = ", kp)
    
    # kp = 3
    # kd = 0
    # task_space = TaskSpaceManipulator("go1_description/urdf/go1.urdf", kp, kd)
    # start_com = task_space.calc_com()
    # target_com = [0, 0.01, -0.036]
    # task_space.set_target(target_com)
    # print("Start loop kp = ", kp)
    # for i in range(200000):
    #     task_space.p_control_iterate()
    #     save_parameters(i, kp, kd, start_com, target_com, task_space.calc_com(), "no_resolve")
    #     if i % 10000 == 0:
    #         print("Progress i = ", i)
    # print("End loop kp = ", kp)

    """
    eps = 10**-4
    YAY 5 _ 946
    YAY 10 _ 460
    """


    eps = 10**-4
    kp = 5
    kd = 0
    task_space = TaskSpaceManipulator("go1_description/urdf/go1.urdf", kp, kd)
    start_com = task_space.calc_com()
    target_com = [0, 0.01, -0.036]
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

    kp = 10
    kd = 0
    task_space = TaskSpaceManipulator("go1_description/urdf/go1.urdf", kp, kd)
    start_com = task_space.calc_com()
    target_com = [0, 0.01, -0.036]
    task_space.set_target(target_com)
    print("Start loop kp = ", kp)
    diff_x = 100
    diff_y = 100
    diff_z = 100
    # or ((diff_x > eps) and (diff_y > eps) and (diff_z > eps)):
    for i in range(200000):
        task_space.p_control_iterate()
        curr_com = task_space.calc_com()
        save_parameters(i, kp, kd, start_com, target_com, task_space.calc_com(), "no_resolve")
        diff_x = abs(curr_com[0] - target_com[0])
        diff_y = abs(curr_com[1] - target_com[1])
        diff_z = abs(curr_com[2] - target_com[2])
        if (diff_x <= eps) and (diff_y <= eps) and (diff_z <= eps):
            print("YAY", kp, "_", i)
            break
        if i % 10000 == 0:
            print("Progress i = ", i)
    print("End loop kp = ", kp)

    # kp = 15
    # kd = 0
    # task_space = TaskSpaceManipulator("go1_description/urdf/go1.urdf", kp, kd)
    # start_com = task_space.calc_com()
    # target_com = [0, 0.01, -0.036]
    # task_space.set_target(target_com)
    # print("Start loop kp = ", kp)
    # for i in range(200000):
    #     task_space.p_control_iterate()
    #     save_parameters(i, kp, kd, start_com, target_com, task_space.calc_com(), "no_resolve")
    #     if i % 10000 == 0:
    #         print("Progress i = ", i)
    # print("End loop kp = ", kp)

    # kp = 15
    # kd = 0
    # task_space = TaskSpaceManipulator("go1_description/urdf/go1.urdf", kp, kd)
    # start_com = task_space.calc_com()
    # target_com = [0, 0.01, -0.036]
    # task_space.set_target(target_com)
    # print("Start loop kp = ", kp)
    # for i in range(200000):
    #     task_space.p_control_iterate()
    #     save_parameters(i, kp, kd, start_com, target_com, task_space.calc_com(), "no_resolve")
    #     if i % 10000 == 0:
    #         print("Progress i = ", i)
    # print("End loop kp = ", kp)

    # kp = 35
    # kd = 0
    # task_space = TaskSpaceManipulator("go1_description/urdf/go1.urdf", kp, kd)
    # start_com = task_space.calc_com()
    # target_com = [0, 0.01, -0.036]
    # task_space.set_target(target_com)
    # print("Start loop kp = ", kp)
    # for i in range(200000):
    #     task_space.p_control_iterate()
    #     save_parameters(i, kp, kd, start_com, target_com, task_space.calc_com(), "no_resolve")
    #     if i % 10000 == 0:
    #         print("Progress i = ", i)
    # print("End loop kp = ", kp)

    # kp = 50
    # kd = 0
    # task_space = TaskSpaceManipulator("go1_description/urdf/go1.urdf", kp, kd)
    # start_com = task_space.calc_com()
    # target_com = [0, 0.01, -0.036]
    # task_space.set_target(target_com)
    # print("Start loop kp = ", kp)
    # for i in range(200000):
    #     task_space.p_control_iterate()
    #     save_parameters(i, kp, kd, start_com, target_com, task_space.calc_com(), "no_resolve")
    #     if i % 10000 == 0:
    #         print("Progress i = ", i)
    # print("End loop kp = ", kp)

    # kp = 75
    # kd = 0
    # task_space = TaskSpaceManipulator("go1_description/urdf/go1.urdf", kp, kd)
    # start_com = task_space.calc_com()
    # target_com = [0, 0.01, -0.036]
    # task_space.set_target(target_com)
    # print("Start loop kp = ", kp)
    # for i in range(200000):
    #     task_space.p_control_iterate()
    #     save_parameters(i, kp, kd, start_com, target_com, task_space.calc_com(), "no_resolve")
    #     if i % 10000 == 0:
    #         print("Progress i = ", i)
    # print("End loop kp = ", kp)

    # kp = 100
    # kd = 0
    # task_space = TaskSpaceManipulator("go1_description/urdf/go1.urdf", kp, kd)
    # start_com = task_space.calc_com()
    # target_com = [0, 0.01, -0.036]
    # task_space.set_target(target_com)
    # print("Start loop kp = ", kp)
    # for i in range(200000):
    #     task_space.p_control_iterate()
    #     save_parameters(i, kp, kd, start_com, target_com, task_space.calc_com(), "no_resolve")
    #     if i % 10000 == 0:
    #         print("Progress i = ", i)
    # print("End loop kp = ", kp)

    # kp = 150
    # kd = 0
    # task_space = TaskSpaceManipulator("go1_description/urdf/go1.urdf", kp, kd)
    # start_com = task_space.calc_com()
    # target_com = [0, 0.01, -0.036]
    # task_space.set_target(target_com)
    # print("Start loop kp = ", kp)
    # for i in range(200000):
    #     task_space.p_control_iterate()
    #     save_parameters(i, kp, kd, start_com, target_com, task_space.calc_com(), "no_resolve")
    #     if i % 10000 == 0:
    #         print("Progress i = ", i)
    # print("End loop kp = ", kp)

    # kp = 200
    # kd = 0
    # task_space = TaskSpaceManipulator("go1_description/urdf/go1.urdf", kp, kd)
    # start_com = task_space.calc_com()
    # target_com = [0, 0.01, -0.036]
    # task_space.set_target(target_com)
    # print("Start loop kp = ", kp)
    # for i in range(200000):
    #     task_space.p_control_iterate()
    #     save_parameters(i, kp, kd, start_com, target_com, task_space.calc_com(), "no_resolve")
    #     if i % 10000 == 0:
    #         print("Progress i = ", i)
    # print("End loop kp = ", kp)


    print("=================Finish main loop=================")

    # do target coms: 1 good, 1 singularity
    # singularity - PD controls fails, Damped/Null space - dont
    # only PD fails - singularity (target com not reached) (crazy values)
    # all 3 fails -- target com is completely unreachable (stucky wucky)



    

    #progress_com.append(task_space.calc_com())


        # task_space.damped_least_sqaures_iterate(damping_factor=0.1)
        # task_space.null_space_iterate(alpha = 0.1) # alpha - null space ??? velocity/ secondary obj velocity

