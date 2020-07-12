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

import cv2 as cv

root = tk.Tk()
gui = GuiInput(root)

while True:
    root.update()
    if gui.get_order_stat():
        break

root.update()
root.quit()

model_path = "models/Restaurant.g"
env = Environment(model_path)
S = env.S
C = env.C
RealWorld = env.RealWorld
V = env.V


def finite_state_machine():
    pass


def simulate_obj_motion(env, sim, config):
    p_glass = sim.getPosition()
    r_glass = sim.getQuaternion()

    config.setPosition(p_glass)
    config.setQuaternion(r_glass)

    env.V.recopyMeshes(env.C)
    env.V.setConfiguration(env.C)


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


def __grab_and_place(komo_op, object_name, object_pos, dest_pos, gripper_name, gripper_center,
                     fingers_opt, offsets1, offsets2, vector_target1, vector_target2):
    global S, C, V, RealWorld

    tau = 0.005
    countbreak = 0
    sim_glass = RealWorld.frame(object_name)
    config_glass = C.getFrame(object_name)
    cup_pos = object_pos

    target_pos = dest_pos
    # kitchen_cam = CV_Perception(env, "kitchen_camera")
    progress = ProgressState.Init
    position_steps = 8
    # komo_op = KomoOperations(env.C, position_steps, tau)
    for t in range(700):
        # time.sleep(0.1)
        # grab sensor readings from the simulation
        q = S.get_q()
        p_glass = sim_glass.getPosition()
        r_glass = sim_glass.getQuaternion()
        config_glass.setPosition(p_glass)
        config_glass.setQuaternion(r_glass)

        if t % 10 == 0:
            [rgb, depth] = S.getImageAndDepth()
            # kitchen_cam.update_rgb_image()

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
                # print(S.getGripperWidth("R_gripper"))
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

        if progress == ProgressState.Init or (
                progress == ProgressState.Started and S.getGripperIsGrasping(gripper_name)):
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
        if progress == ProgressState.InProgress and not S.getGripperIsGrasping(gripper_name):
            countbreak += 1
            if countbreak > 30:
                print("t", t)
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
    sim_glass = RealWorld.frame(object_name)
    obj_pos = sim_glass.getPosition()
    obj_pos = obj_pos + [0.08, 0, 0]
    shelf_gripper_komo = KomoOperations(env.C)
    __grab_and_place(shelf_gripper_komo, object_name, obj_pos, tray_pos, gripper_name,
                     gripper_center, fingers_opt, offsets1, offsets2, vector_target1,
                     vector_target2)
    move_back_shelf_arm(shelf_gripper_komo, object_name)
    # widen_gripper(gripper_name)


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

    sim_glass = RealWorld.frame(object_name)
    # obj_pos = sim_glass.getPosition()
    obj_pos = obj_pos + [0, -0.07, 0]

    green_gripper_komo = KomoOperations(env.C)
    __grab_and_place(green_gripper_komo, object_name, obj_pos, table_pos, gripper_name,
                     gripper_center, fingers_opt, offsets1, offsets2, vector_target1, vector_target2)
    move_back_green_arm(green_gripper_komo, object_name)


def grab_from_red_arm(object_name, obj_pos, table_pos):
    fingers_opt = ['L_finger1', 'L_finger2']
    gripper_name = "L_gripper"
    gripper_center = "L_gripperCenter"

    offsets1 = [[0, -0.4, 0], [0, -0.3, 0], [0, -0.2, 0]]
    offsets2 = [[0, -0.4, 0.1], [0, -0.3, 0.05], [0, -0.25, 0.05]]
    vector_target1 = [[1, 0, 0],
                      ["None"],
                      [0, -1, 0]]
    vector_target2 = [[1, 0, 0],
                      ["None"],
                      [0, -1, 0]]

    sim_glass = RealWorld.frame(object_name)
    # obj_pos = sim_glass.getPosition()
    obj_pos = obj_pos + [0, -0.07, 0]

    green_gripper_komo = KomoOperations(env.C)
    __grab_and_place(green_gripper_komo, object_name, obj_pos, table_pos, gripper_name,
                     gripper_center, fingers_opt, offsets1, offsets2, vector_target1, vector_target2)
    move_back_red_arm(green_gripper_komo, object_name)


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


def move_back_red_arm(gripper_komo, reference_object):
    fingers_opt = ['L_finger1', 'L_finger2']
    gripper_center = "L_gripperCenter"
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
        # print(distance)
        if distance < 0.8:
            state = 1
            count_break += 1
        if state != 1 or count_break < 9:
            start_tau_param = start_tau_param + incr_tau_param
            komo = pr2_komo.move_pr2_position(dest_pos, vector_target, start_tau_param)
            komo.optimize()
            C.setFrameState(komo.getConfiguration(0))
            q = C.getJointState()
        elif state == 1 and count_break > 50:
            break
        S.step(q, tau, ry.ControlMode.position)


'''
ref_objects are frame names of the max 2 object which are currently being served
'''


def move_pr2_table(dest_pos, ref_objects):
    vector_target = [[-1, 0, 0],
                     [0, -1, 0],
                     ["None"]]
    start_tau_param = 0.00001
    # start_tau_param = 0.001
    incr_tau_param = 0.0000005
    pr2_komo = KomoOperations(env.C)
    __move_pr2(pr2_komo, dest_pos, vector_target, start_tau_param, incr_tau_param, ref_objects)


def widen_gripper(gripper):
    print("widening gripper", S.getGripperWidth(gripper))
    for i in range(100):
        S.openGripper(gripper)
        S.step(S.get_q(), 0.005, ry.ControlMode.position)
        if abs(S.getGripperWidth(gripper)) > 0.015:
            break


def move_pr2_offset(offset, ref_objects):
    [base_pos, J] = C.evalFeature(ry.FS.position, ["base_footprint"])
    print("inside pr2 offset", base_pos)
    dest_pos = base_pos + offset
    move_pr2_table(dest_pos, ref_objects)
    [base_pos, J] = C.evalFeature(ry.FS.position, ["base_footprint"])
    print("after moving pr2 offset", base_pos)


def move_pr2_shelf(dest_pos, ref_objects):
    print("moving pr2 back to shelf")
    vector_target = [[0, 1, 0],
                     [-1, 0, 0],
                     ["None"]]
    start_tau_param = 0.001
    incr_tau_param = 0
    pr2_komo = KomoOperations(env.C)
    __move_pr2(pr2_komo, dest_pos, vector_target, start_tau_param, incr_tau_param, ref_objects)


def table_cam_pos():
    kitchen_cam = CV_Perception(env, "table_camera")

    red_table_pos = kitchen_cam.get_target_pos(ItemColor(1)) + [0.0231, -0.011, 0]
    red_table_pos[2] = 0

    print("red table with offset ", red_table_pos)
    print(env.get_position("coffe_table"))

    green_table_pos = kitchen_cam.get_target_pos(ItemColor(2)) + [0, -0.01, 0] + [0.82, -0.95, 0]
    ## offset [0.82, -0.95, 0]
    green_table_pos[2] = 0

    print("green table with offset", green_table_pos)
    print(env.get_position("coffe_table_1"))

    return red_table_pos, green_table_pos

def get_kitchen_cam_pos(order, position, offset):
    kitchen_cam = CV_Perception(env, "kitchen_camera")
    order_pos = kitchen_cam.get_target_pos(ItemColor(order.value), position)

    if order_pos != []:
        order_pos += offset

    return order_pos


def teleport_obj(object_frame, position):
    frame = RealWorld.getFrame(object_frame)
    c_frame = C.getFrame(object_frame)
    frame.setPosition(position)
    S.setState(RealWorld.getFrameState())
    S.step([], 0.001, ry.ControlMode.none)
    p_glass = frame.getPosition()
    r_glass = frame.getQuaternion()
    c_frame.setPosition(p_glass)
    c_frame.setQuaternion(r_glass)


[initial_base_pos, J] = C.evalFeature(ry.FS.position, ["base_footprint"])

if __name__ == '__main__':
    state = State.Load_env

    coffe_list = ["dyna_coffee_1", "dyna_coffee_2", "dyna_coffee_3"]
    sprite_list = ["sprite_1", "sprite_2", "sprite_3"]
    cola_list = ["cola_1", "cola_2", "cola_3"]

    item_to_list_map = {Items.Coffee: coffe_list,
                        Items.Sprite: sprite_list,
                        Items.Cola: cola_list
                        }

    dining_table_to_location_map = {"table1": [-1.2, -1, 0.639],
                                    "table2": [1.9, 2.1, 0.783]
                                    }

    # create environment

    # set mass of dynamic objects
    env.add_dyna_mass(100.0)

    # receive order
    user_ip = True
    if user_ip:
        percept_red_table_pos, percept_green_table_pos = table_cam_pos()
    else:
        percept_green_table_pos = np.array([-0.18, -1.8, 0])
        percept_red_table_pos = np.array([-0.18, 0.6, 0])

    table_to_pr2_pos_map = {"table1": percept_red_table_pos,
                            "table2": percept_green_table_pos
                            }

    # get gui order
    table_1, table_2 = gui.get_order()
    table_1_order = []
    table_2_order = []

    item_stack = []

    object_pos_in_shelf = [[1.4, 2.5, 0.63], [1.6, 2.8, 0.6]]  # calculate using shelf position
    camera_offset = [[0.067, 0, -0.013], [0.0477, 0.0065, -0.0312]]

    # Create item stack from both table orders
    for order_1, order_2 in zip(table_1, table_2):
        if order_1 != Items.Invalid:
            item_stack.append(['table1,', order_1])
        if order_2 != Items.Invalid:
            item_stack.append(['table2,', order_2])

    # loop until all items are delivered to the dining table
    while len(item_stack):
        process_list = []
        Full_Order_Details = []
        target_name_list = []

        object_pos_in_tray = [[2.35, 2.1, 0.783], [2.14, 2.1, 0.77]]  # calculate using tray position

        if len(item_stack) == 1:
            process_list.append(item_stack.pop(0))
        else:  # must be between 2-6
            process_list.append(item_stack.pop(0))
            process_list.append(item_stack.pop(0))

        # get object frame, teleport and get item position from camera perception
        for i, item in enumerate(process_list):
            order = item_to_list_map[item[1]].pop(0)
            target_name_list.append([item[0], order])
            teleport_obj(order, object_pos_in_shelf[i])

            order_pos = RealWorld.frame(order).getPosition()
            print("Real world pos of ", order, " is : ", order_pos)

            run_empty_steps(env.S, num_iter=20, tau=0.001)
            obj_pos = get_kitchen_cam_pos(item[1], i, camera_offset[i])
            print("Camera pos of ", order, " is : ", obj_pos)
            Full_Order_Details.append([item[0], order, obj_pos])
        exit()
        cv.destroyAllWindows()
        current_serving_objects = []

        # Grab from shelf and place it in the Tray
        for table_id, target_object, obj_position in Full_Order_Details:
            print(table_id, target_object, obj_position)
            current_serving_objects[table_id].append(target_object)
            target_pos = object_pos_in_tray.pop(0)
            print("Start moving from shelf to tray...")
            grab_from_shelf_arm(target_object, obj_position, target_pos)
            print("Done")

        # move first order
        order_to_table = Full_Order_Details.pop(0)
        target_pos = dining_table_to_location_map[order_to_table[0]]
        target_object = order_to_table[1]
        object_pos = order_to_table[2]

        if order_to_table[0] == "table1":
            print("Move Pr2 to dining table1")
            move_pr2_table(percept_green_table_pos, current_serving_objects)

            print("grasping and placing order in table 1")
            grab_from_green_arm(target_object, object_pos, target_pos)

        elif order_to_table[0] == "table2":
            print("Move Pr2 to dining table1")
            move_pr2_table(percept_red_table_pos, current_serving_objects)

            print("grasping and placing order in table 1")
            grab_from_red_arm(target_object, object_pos, target_pos)

        else:  # impossible case.. only 2 table exists.. helpful during None expectation
            print("Invalid table ID received.. Stopping execution")
            exit()

        # check if second order is available and process item2
        if len(Full_Order_Details):
            order_to_table_new = Full_Order_Details.pop(0)
            target_pos_new = dining_table_to_location_map[order_to_table_new[0]]
            target_object_new = order_to_table_new[1]
            object_pos_new = order_to_table_new[2]

            # 1st order : table1 and 2nd order : table1
            if order_to_table[0] == "table1" and order_to_table_new[0] == "table1":
                print("Same order in table 1, move side for panda to pick")
                move_pr2_offset([0., -1.1, 0], current_serving_objects)

                print("grasping and placing order in table 1")
                grab_from_green_arm(target_object_new, object_pos_new, target_pos_new)

            # 1st order : table2 and 2nd order : table2
            elif order_to_table[0] == "table2" and order_to_table_new[0] == "table2":
                print("Same order in table 2, move side for panda to pick")
                move_pr2_offset([0., -1.1, 0], current_serving_objects)

                print("grasping and placing order in table 2")
                grab_from_red_arm(target_object_new, object_pos_new, target_pos_new)

            # 1st order : table1 and 2nd order : table2
            elif order_to_table[0] == "table1" and order_to_table_new[0] == "table2":
                print("Diff order, move side from table1 to table2")
                move_pr2_offset([0.35, 2.45, 0], current_serving_objects)

                print("grasping and placing order in table 1")
                grab_from_red_arm(target_object_new, object_pos_new, target_pos_new)

            else:  # impossible case.. only 2 table exists.. helpful during None expection
                print("Invalid table ID received.. Stopping execution")
                exit()

        move_pr2_shelf(initial_base_pos + [0, 0.8, 0], [])

    print("press any key to exit")
    while True:
        key = input()
        if key:
            break
