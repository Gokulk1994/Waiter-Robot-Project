import sys

sys.path.append('../robotics-course/build')
import numpy as np
import libry as ry
import time
import tkinter as tk

from food_order.user_interactions import user_order
from food_order.user_gui import GuiInput
from perception.cv_operations import CV_Perception
from optimization.komo_optimization import KomoOperations
from simulation.Simulations import Environment
from enumeration.enum_classes import *

import  cv2 as cv

root = tk.Tk()
gui = GuiInput(root)

while True:
    root.update()
    if gui.get_order_stat():
        break

root.update()

model_path = "models/Restaurant.g"
env = Environment(model_path)
S = env.S
C = env.C
RealWorld = env.RealWorld
V = env.V
[start_R_gripper_pos, J] = C.evalFeature(ry.FS.position, ["R_gripper"])
[start_grip_quat, J] = C.evalFeature(ry.FS.quaternion, ["R_gripperCenter"])


def finite_state_machine():
    pass


def simulate_obj_motion(env, sim, config):
    p_glass = sim.getPosition()
    r_glass = sim.getQuaternion()

    config.setPosition(p_glass)
    config.setQuaternion(r_glass)

    env.V.recopyMeshes(env.C)
    env.V.setConfiguration(env.C)


def panda_to_item(env, gripper, gripper_cen, target_pos, target_object, vector_target, is_close, fingers_opt=None):
    tau = .005

    sim = env.RealWorld.frame(target_object)
    config = env.C.getFrame(target_object)

    komo_steps = 1
    progress = ProgressState.Init
    move_tray = False

    for t in range(50):
        print("-------------------------------------------")
        print("time : ", t)
        # time.sleep(0.01)

        q = env.S.get_q()
        simulate_obj_motion(env, sim, config)

        if is_close:
            Check_Conditon = not (env.S.getGripperIsGrasping(gripper))
        else:
            Check_Conditon = env.S.getGripperIsGrasping(gripper)

        if Check_Conditon:
            if progress == ProgressState.Init:
                [grip_y, _] = env.C.evalFeature(ry.FS.position, [gripper])
                distance = np.linalg.norm(target_pos - grip_y)
                print("Distance to Goal : ", distance)
                if distance < 0.06:

                    print("Target position reached at :", t)
                    if is_close:
                        print("Gripper Closing in progress...")
                        env.S.closeGripper(gripper)
                    else:
                        print("Gripper Opening in progress...")
                        env.S.openGripper(gripper)

                    progress = ProgressState.InProgress

        else:
            if progress == ProgressState.InProgress:
                print("Gripper operation done")
                progress = ProgressState.Finished
            else:
                print("gripper operation not performed")

        if progress != ProgressState.Finished:
            komo_op = KomoOperations(env.C, komo_steps, tau)
            komo_optimized = komo_op.move_to_position(gripper_cen, target_pos, vector_target, fingers_opt)

            for i in range(komo_steps):
                print("Length of komo config", len(komo_optimized.getConfiguration(i)))
                env.C.setFrameState(komo_optimized.getConfiguration(i))
                q = env.C.getJointState()
                env.S.step(q, tau, ry.ControlMode.position)
        else:
            if move_tray == False:
                print("moving to tray")
                vector_target = [[1, 0, 0],
                                 ["None"],
                                 [0, 1, 0]
                                 ]
                target_pos = env.get_position("tray")
                target_pos += [0, 0, 0.12]
                is_close = False
                progress = ProgressState.Init
                move_tray = True
            else:
                env.S.step(q, tau, ry.ControlMode.position)

    if progress != ProgressState.Init:
        # print(komo_optimized.getReport())

        # env.V = komo_optimized.view()
        # env.V.playVideo()
        pass

    if progress != ProgressState.Finished:
        print("Insufficient Iterations. Process Terminated before Gripper is closed")
        return False, env
    return True, env


def move_pr2_user():
    pass


def move_pr2_kitchen():
    pass


def place_order_panda():
    pass


def clean_table():
    pass


def run_empty_steps(S, num_iter=20, tau=0.001):
    for i in range(20):
        time.sleep(0.05)
        S.step(S.get_q(), 0.001, ry.ControlMode.position)


def __grab_and_place(komo_op, object_name, object_pos, tray_pos, gripper_name, gripper_center,
                   fingers_opt, offsets1, offsets2, vector_target1, vector_target2):
    global S, C, V, RealWorld

    tau = 0.005
    target_object = object_name
    countbreak = 0
    sim_glass = RealWorld.frame(object_name)
    config_glass = C.getFrame(object_name)
    cup_pos = object_pos

    target_pos = tray_pos
    #kitchen_cam = CV_Perception(env, "kitchen_camera")
    progress = ProgressState.Init
    position_steps = 8
    # komo_op = KomoOperations(env.C, position_steps, tau)
    for t in range(170):
        # time.sleep(0.1)
        # grab sensor readings from the simulation
        q = S.get_q()
        p_glass = sim_glass.getPosition()
        r_glass = sim_glass.getQuaternion()
        config_glass.setPosition(p_glass)
        config_glass.setQuaternion(r_glass)

        if t % 10 == 0:
            [rgb, depth] = S.getImageAndDepth()
            #kitchen_cam.update_rgb_image()

        V.recopyMeshes(C)
        V.setConfiguration(C)

        if not S.getGripperIsGrasping(gripper_name):
            [y, J] = C.evalFeature(ry.FS.position, [gripper_name])
            distance = np.linalg.norm(y - cup_pos)
            # print(distance)
            if distance < 0.06 and progress == ProgressState.Init:
                progress = ProgressState.Started
                # print(S.getGripperWidth("R_gripper"))
                S.closeGripper(gripper_name)
                print("graspping", t)
                #print(S.getGripperWidth("R_gripper"))
        else:
            [y, J] = C.evalFeature(ry.FS.position, [gripper_name])
            distance = np.linalg.norm(y - target_pos)
            # print(distance)
            if distance < 0.06 and progress == ProgressState.Started:
                S.openGripper(gripper_name)
                print("opening")
                progress = ProgressState.InProgress

        if progress == ProgressState.Init:
            print("move to object called")
            komo = komo_op.move_to_position(gripper_center, cup_pos, offsets1,
                                            vector_target1, fingers_opt, False)
            komo.optimize()
        elif progress == ProgressState.Started and S.getGripperIsGrasping(gripper_name):
            komo = komo_op.move_to_position(gripper_center, target_pos, offsets2,
                                            vector_target2, fingers_opt, False)
            komo.optimize()
            # v1 = komo.view()
            # v1.playVideo()

        if progress == ProgressState.Init or (progress == ProgressState.Started and S.getGripperIsGrasping(gripper_name)):
            for i in range(position_steps):
                time.sleep(2)
                print(i)
                C.setFrameState(komo.getConfiguration(i))
                # V.setConfiguration(C)
                # V.recopyMeshes(C)
                q = C.getJointState()
                S.step(q, tau, ry.ControlMode.position)
            # V.setConfiguration(C)
            # V.recopyMeshes(C)
        else:
            S.step(q, tau, ry.ControlMode.position)
        if progress == ProgressState.InProgress and not S.getGripperIsGrasping(gripper_name) :
            countbreak += 1
            if countbreak > 30:
                break


def grab_from_shelf_arm(object_name, obj_pos, tray_pos):
    fingers_opt = ['R_finger1', 'R_finger2']
    gripper_name = "R_gripper"
    gripper_center = "R_gripperCenter"
    offsets1 = [[0.6, 0, 0], [0.3, 0, 0], [0.2, 0, 0]]
    offsets2 = [[0, 0.4, 0.1], [0, 0.3, 0.05], [0, 0.2, 0.04]]
    vector_target1 = [[0, -1, 0],
                      ["None"],
                      [1, 0, 0]]
    vector_target2 = [[1, 0, 0],
                      ["None"],
                      [0, 1, 0]]
    # from perception
    obj_pos = obj_pos + [0.08, 0, 0]
    shelf_gripper_komo = KomoOperations(env.C)
    __grab_and_place(shelf_gripper_komo, object_name, obj_pos, tray_pos, gripper_name,
                     gripper_center, fingers_opt, offsets1, offsets2, vector_target1,
                     vector_target2)
    move_back_shelf_arm(shelf_gripper_komo, target_object)


def grab_from_green_arm(object_name, obj_pos, table_pos):
    fingers_opt = ['1_finger1', '1_finger2']
    gripper_name = "1_gripper"
    gripper_center = "1_gripperCenter"

    offsets1 = [[0, -0.4, 0], [0, -0.3, 0], [0, -0.2, 0]]
    offsets2 = [[0, -0.4, 0.1], [0, -0.3, 0.05], [0, -0.25, 0.05]]
    vector_target1 = [[1, 0, 0],
                      ["None"],
                      [0, -1, 0]]
    vector_target2 = [[1, 0, 0],
                      ["None"],
                      [0, -1, 0]]
    obj_pos = obj_pos + [0, -0.08, 0]

    green_gripper_komo = KomoOperations(env.C)
    __grab_and_place(green_gripper_komo, object_name, obj_pos, table_pos, gripper_name,
                   gripper_center, fingers_opt, offsets1, offsets2, vector_target1, vector_target2)
    move_back_green_arm(green_gripper_komo, target_object)


def __move_back(komo_op, position_steps, reference_object, offset, gripper_center, fingers_opt, vector_target):
    global S, C, V, RealWorld
    state = 0
    tau = 0.005

    sim_glass = RealWorld.frame(reference_object)
    config_glass = C.getFrame(reference_object)

    for t in range(30):
        # time.sleep(0.1)
        # grab sensor readings from the simulation
        q = S.get_q()
        p_glass = sim_glass.getPosition()
        r_glass = sim_glass.getQuaternion()
        config_glass.setPosition(p_glass)
        config_glass.setQuaternion(r_glass)
        V.recopyMeshes(C)
        V.setConfiguration(C)
        if state != 1:
            komo = komo_op.gripper_move_back_position(gripper_center, p_glass + offset,
                                                      5, vector_target, fingers_opt)
            komo.optimize()
            for i in range(position_steps):
                time.sleep(2)
                print(i)
                C.setFrameState(komo.getConfiguration(i))
                q = C.getJointState()
                S.step(q, tau, ry.ControlMode.position)
            state = 1
        else:
            S.step(q, tau, ry.ControlMode.position)


def move_back_green_arm(gripper_komo, reference_object):
    fingers_opt = ['1_finger1', '1_finger2']
    gripper_center = "1_gripperCenter"
    offset = [0, -0.3, 0]
    vector_target = [[1, 0, 0],
                     ["None"],
                     [0, -1, 0]]
    __move_back(gripper_komo, 5, reference_object, offset, gripper_center,
              fingers_opt, vector_target)


def move_back_shelf_arm(shelf_gripper_komo, reference_object):
    fingers_opt = ['R_finger1', 'R_finger2']
    gripper_center = "R_gripperCenter"
    offset = [0, 0.3, 0]
    vector_target = [[1, 0, 0],
                     ["None"],
                     [0, 1, 0]]
    __move_back(shelf_gripper_komo, 5, reference_object, offset, gripper_center, fingers_opt, vector_target)


def __update_config_reference(sim_obj1, config_obj1, sim_obj2, config_obj2):
    if sim_obj1:
        p_glass = sim_obj1.getPosition()
        r_glass = sim_obj1.getQuaternion()
        config_obj1.setPosition(p_glass)
        config_obj1.setQuaternion(r_glass)
    if sim_obj2:
        p_glass = sim_obj2.getPosition()
        r_glass = sim_obj2.getQuaternion()
        config_obj2.setPosition(p_glass)
        config_obj2.setQuaternion(r_glass)
    V.recopyMeshes(C)
    V.setConfiguration(C)
    return


def __move_pr2(pr2_komo, dest_pos, vector_target, start_tau_param, incr_tau_param, ref_objects=[],
             max_iterations=1500):
    print("moving pr2 to desired position")
    count_break = 0
    state = 0
    sim_obj1 = None
    sim_obj2 = None
    config_obj1 = None
    config_obj2 = None
    tau = 0.005

    if len(ref_objects) == 2:
        sim_obj1 = RealWorld.frame(ref_objects[0])
        config_obj1 = C.getFrame(ref_objects[0])
        sim_obj2 = RealWorld.frame(ref_objects[1])
        config_obj2 = C.getFrame(ref_objects[1])
    elif len(ref_objects) == 1:
        sim_obj1 = RealWorld.frame(ref_objects[0])
        config_obj1 = C.getFrame(ref_objects[0])

    for t in range(max_iterations):
        q = S.get_q()
        __update_config_reference(sim_obj1, config_obj1, sim_obj2, config_obj2)
        [y, J] = C.evalFeature(ry.FS.position, ["base_footprint"])
        distance = np.linalg.norm(y - dest_pos)
        #print(distance)
        if distance < 0.8:
            state = 1
            count_break += 1
        if state != 1 or count_break < 9:
            start_tau_param = start_tau_param + incr_tau_param
            komo = pr2_komo.move_pr2_position(dest_pos, vector_target, start_tau_param)
            komo.optimize()
            C.setFrameState(komo.getConfiguration(0))
            q = C.getJointState()
        # elif state == 1 and count_break > 9:
        #     break
        S.step(q, tau, ry.ControlMode.position)


'''
ref_objects are frame names of the max 2 object which are currently being served
'''


def move_pr2_table(dest_pos, ref_objects):
    vector_target = [[-1, 0, 0],
                     [0, -1, 0],
                     ["None"]]
    start_tau_param = 0.0000001
    incr_tau_param = 0.0000005
    pr2_komo = KomoOperations(env.C)
    __move_pr2(pr2_komo, dest_pos, vector_target, start_tau_param, incr_tau_param, ref_objects)


if __name__ == '__main__':
    state = State.Load_env

    # create environment

    # set mass of dynamic objects
    env.add_dyna_mass(100.0)

    # receive order

    user_ip = True

    # get gui order

    table_1, table_2 = gui.get_order()

    table_1_order = []
    table_2_order = []

    # Activate Kitchen camera
    if user_ip:

        kitchen_cam = CV_Perception(env, "table_camera")

        percept_red_table_pos = kitchen_cam.get_target_pos(ItemColor(1)) + [0.0231, -0.011, 0]
        percept_red_table_pos[2] = 0

        print(percept_red_table_pos)
        print(env.get_position("coffe_table"))

        percept_green_table_pos = kitchen_cam.get_target_pos(ItemColor(2)) + [0,-0.01, 0]
        percept_green_table_pos[2] = 0

        print(percept_green_table_pos)
        print(env.get_position("coffe_table_1"))

        kitchen_cam = CV_Perception(env, "kitchen_camera")
        # print(ItemColor(int(order)))
        for i in table_1:
            target = kitchen_cam.get_target_pos(ItemColor(i.value)) + [0.067,0,-0.013]
            table_1_order.append(target)
        for i in table_2:
            target = kitchen_cam.get_target_pos(ItemColor(i.value)) + [0.0477, 0.0065, -0.0312]
            table_2_order.append(target)

        cv.destroyAllWindows()
    else:
        percept_green_table_pos = np.array([-0.18, -1.8, 0])
        percept_red_table_pos   = np.array([-0.18, 0.6, 0])

    run_empty_steps(env.S, num_iter=20, tau=0.001)
    # Pick an Object : Close gripper
    percept_green_table_pos += [0.82, -0.95, 0]
    print("green table pos :", percept_green_table_pos)

    fingers_opt = ['R_finger1', 'R_finger2']

    current_serving_objects = []

    target_object = "dyna_coffe_mug"
    current_serving_objects.append(target_object)

    target_pos = np.array([2.35, 2.1, 0.783])
    # from perception

    if user_ip:
        object_pos = table_1_order[0]
    else:
        print("enable user input to get real world position")
        config_glass = C.getFrame(target_object)
        object_pos = config_glass.getPosition()

    grab_from_shelf_arm(target_object, object_pos, target_pos)

    target_object = "green_glass"
    current_serving_objects.append(target_object)
    target_pos = np.array([2.14, 2.1, 0.77])

    # from perception
    if user_ip:
        object_pos = table_2_order[0]
    else:
        print("enable user input to get real world position")
        config_glass = C.getFrame(target_object)
        object_pos = config_glass.getPosition()

    grab_from_shelf_arm(target_object, object_pos, target_pos)

    # near red table position from perception

    move_pr2_table(percept_green_table_pos, current_serving_objects)

    # placing on dining table second item goes first if serving 2 items to same table
    target_object = "green_glass"
    config_glass = C.getFrame(target_object)
    object_pos = config_glass.getPosition()

    # from perception
    target_pos =  np.array([-1.2, 0.7, 0.639])
    grab_from_green_arm(target_object, object_pos, target_pos)


    position_steps = 8
    target_object = "green_glass"
    target_pos = np.array([1.9, 2.1, 0.783])

    #grab_and_place(target_object, target_pos)

    position_steps = 5
    #move_back(position_steps, target_object, target_pos)

    print("press any key to exit")
    while True:
        key = input()
        if key:
            break
