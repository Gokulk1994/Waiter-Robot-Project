import sys

sys.path.append('../robotics-course/build')
import numpy as np
import libry as ry
import time

from food_order.user_interactions import user_order
from perception.cv_operations import CV_Perception
from optimization.komo_optimization import KomoOperations
from simulation.Simulations import Environment
from enumeration.enum_classes import *

model_path = "models/Restaurant.g"
env = Environment(model_path)
S = env.S
C = env.C
RealWorld = env.RealWorld
V = env.V
[start_R_gripper_pos,J] = C.evalFeature(ry.FS.position, ["R_gripper"])
[start_grip_quat,J] = C.evalFeature(ry.FS.quaternion, ["R_gripperCenter"])

def finite_state_machine():
    pass


def simulate_obj_motion(env, sim, config):
    p_glass = sim.getPosition()
    r_glass = sim.getQuaternion()

    config.setPosition(p_glass)
    config.setQuaternion(r_glass)

    env.V.recopyMeshes(env.C)
    env.V.setConfiguration(env.C)


def panda_to_item(env, gripper, gripper_cen, target_pos, target_object, vector_target, is_close, fingers_opt = None):
    tau = .005

    sim = env.RealWorld.frame(target_object)
    config = env.C.getFrame(target_object)

    komo_steps = 1
    progress = ProgressState.Init
    move_tray = False

    for t in range(50):
        print("-------------------------------------------")
        print("time : ", t)
        #time.sleep(0.01)

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
                print("Length of komo config" , len(komo_optimized.getConfiguration(i)))
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
        #print(komo_optimized.getReport())

        #env.V = komo_optimized.view()
        #env.V.playVideo()
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

def grab_and_place(object_name, tray_pos):
    global S,C,V,RealWorld
    tau = 0.005
    target_object = object_name
    countbreak = 0
    sim_glass = RealWorld.frame(target_object)
    config_glass = C.getFrame(target_object)

    cup_pos = config_glass.getPosition() + [0.08, 0, 0]

    target_pos = tray_pos
    count = 11
    steps = 1
    closed = 0
    position_steps = 8
    komo_op = KomoOperations(env.C, position_steps, tau)
    for t in range(170):
        # time.sleep(0.1)
        S.getGripperIsGrasping("R_gripper")
        # grab sensor readings from the simulation
        q = S.get_q()
        p_glass = sim_glass.getPosition()
        r_glass = sim_glass.getQuaternion()
        config_glass.setPosition(p_glass)
        config_glass.setQuaternion(r_glass)
        V.recopyMeshes(C)
        V.setConfiguration(C)

        if not S.getGripperIsGrasping("R_gripper"):
            [y, J] = C.evalFeature(ry.FS.position, ["R_gripper"])
            distance = np.linalg.norm(y - cup_pos)
            # print(distance)
            if distance < 0.06 and count > 10 and closed == 0:
                closed = 1
                # print(S.getGripperWidth("R_gripper"))
                S.closeGripper("R_gripper")
                print("graspping", t)
                print(S.getGripperWidth("R_gripper"))
        else:
            # print("while closing ",S.getGripperWidth("R_gripper"))
            [y, J] = C.evalFeature(ry.FS.position, ["R_gripper"])
            distance = np.linalg.norm(y - target_pos)
            # print(distance)
            count += 1
            if distance < 0.06 and count > 10 and closed == 1:
                S.openGripper("R_gripper")
                print("opening")
                closed = 2

        if closed == 0:
            position_steps = 8
            print("move to object called")
            # komo = move_to_position("R_gripperCenter", [1.83, -1.23, 0.80])
            komo = komo_op.check_move_to_pos_close("R_gripperCenter", cup_pos, [],position_steps)
        elif closed == 1 and S.getGripperIsGrasping("R_gripper"):
            [grip_quat, J] = C.evalFeature(ry.FS.quaternion, ["R_gripperCenter"])
            position_steps = 8
            komo = komo_op.check_move_to_pos_open("R_gripperCenter", target_pos, grip_quat,position_steps)
            # v1 = komo.view()
            # v1.playVideo()

        if closed == 0 or (closed == 1 and S.getGripperIsGrasping("R_gripper")):
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
            countbreak += 1
            S.step(q, tau, ry.ControlMode.position)
            if countbreak > 10:
                break



def move_back(position_steps,target_object,target_pos):
    global S, C, V, RealWorld
    target_pos = start_R_gripper_pos
    tau  = 0.005
    state = 0
    komo_op = KomoOperations(env.C, position_steps, tau)
    sim_glass = RealWorld.frame(target_object)
    config_glass = C.getFrame(target_object)
    for t in range(50):
        #time.sleep(0.1)
        #grab sensor readings from the simulation
        q = S.get_q()
        p_glass = sim_glass.getPosition()
        r_glass = sim_glass.getQuaternion()
        config_glass.setPosition(p_glass)
        config_glass.setQuaternion(r_glass)
        V.recopyMeshes(C)
        V.setConfiguration(C)
        [y,J] = C.evalFeature(ry.FS.position, ["R_gripperCenter"])
        if state != 1:
            [start_grip_quat,J] = C.evalFeature(ry.FS.quaternion, ["R_gripperCenter"])
            komo = komo_op.move_back_position("R_gripperCenter", p_glass+[0,0.3,0], start_grip_quat,position_steps)
            for i in range(position_steps):
                time.sleep(2)
                print(i)
                C.setFrameState( komo.getConfiguration(i))
                q = C.getJointState()
                S.step(q, tau, ry.ControlMode.position)
            state=1
        else:
            S.step(q, tau, ry.ControlMode.position)

if __name__ == '__main__':
    state = State.Load_env

    # create environment


    # set mass of dynamic objects
    env.add_dyna_mass(100.0)

    # receive order

    user_ip = True

    # Activate Kitchen camera
    if user_ip:
        order_obj = user_order()
        order = order_obj.receive_order_user()

        kitchen_cam = CV_Perception(env, "kitchen_camera")
        #print(ItemColor(int(order)))
        target = kitchen_cam.get_target_pos(ItemColor(int(order)))
        print(target)
        print(env.get_position("dyna_coffe_mug"))
    else:
        target = env.get_position("dyna_coffe_mug") + [0.08,0,0]

    run_empty_steps(env.S, num_iter=20, tau=0.001)
    # Pick an Object : Close gripper

    fingers_opt = ['R_finger1','R_finger2']

    tau = .005
    target_object = "dyna_coffe_mug"
    target_pos = np.array([2.15, 2.1, 0.783])

    grab_and_place(target_object, target_pos)

    position_steps = 5
    move_back(position_steps, target_object,target_pos )

    position_steps = 8
    target_object = "green_glass"
    target_pos = np.array([1.9, 2.1, 0.783])

    grab_and_place(target_object, target_pos)

    position_steps = 5
    move_back(position_steps, target_object,target_pos)

    print("press any key to exit")
    while True:
        key = input()
        if key:
            break


"""

    if target is not None:
        print("moving to cup")
        target_vector = [[0, -1, 0], ["None"], [1, 0, 0]]
        status, env_new = panda_to_item(env, "R_gripper", "R_gripperCenter", target, "dyna_coffe_mug", target_vector, True, fingers_opt)
"""
"""  
    if status == True:
        print("moving to tray")
        target_vector = [[1, 0, 0],
                         ["None"],
                         [0, 1, 0]
                         ]
        target = env.get_position("tray")
        target += [0,0,0.12]
        status, env_new_1 = panda_to_item(env_new, "R_gripper", "R_gripperCenter", target, "dyna_coffe_mug",target_vector, False)
"""



    # get_order_panda(env.S, env.V, env.C, env.RealWorld)
