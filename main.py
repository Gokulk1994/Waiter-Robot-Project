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
    tau = .001

    sim = env.RealWorld.frame(target_object)
    config = env.C.getFrame(target_object)

    komo_steps = 8
    progress = ProgressState.Init

    for t in range(170):
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

                if distance < 0.06:
                    print("Gripper Closing in progress...")

                    if is_close:
                        env.S.closeGripper(gripper)
                    else:
                        env.S.openGripper(gripper)

                    progress = ProgressState.InProgress

        else:
            if progress == ProgressState.InProgress:
                print("Gripper operation done")
                progress = ProgressState.Finished
            else:
                print("gripper operation not performed")
            break

        if progress != ProgressState.Finished:
            komo_op = KomoOperations(env.C, komo_steps)
            komo_optimized = komo_op.move_to_position(gripper_cen, target_pos, vector_target, fingers_opt)

            for i in range(komo_steps):
                env.C.setFrameState(komo_optimized.getConfiguration(i))
                q = env.C.getJointState()
                env.S.step(q, tau, ry.ControlMode.position)

    if progress != ProgressState.Init:
        print(komo_optimized.getReport())

        env.V = komo_optimized.view()
        env.V.playVideo()

    if progress != ProgressState.Finished:
        print("Insufficient Iterations. Process Terminated before Gripper is closed")
        return False
    return True


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


if __name__ == '__main__':
    state = State.Load_env

    # create environment
    model_path = "models/Restaurant.g"
    env = Environment(model_path)

    # set mass of dynamic objects
    env.add_dyna_mass()

    # receive order

    user_ip = False

    # Activate Kitchen camera
    if user_ip:
        order_obj = user_order()
        order = order_obj.receive_order_user()

        kitchen_cam = CV_Perception(env, "kitchen_camera")
        print(ItemColor(int(order)))
        target = kitchen_cam.get_target_pos(ItemColor(int(order)))
        print(target)
        print(env.get_position("dyna_coffe_mug"))
    else:
        target = env.get_position("dyna_coffe_mug") + [0.08,0,0]

    run_empty_steps(env.S, num_iter=20, tau=0.001)
    # Pick an Object : Close gripper

    fingers_opt = ['R_finger1','R_finger2']

    if target is not None:
        print("moving to cup")
        target_vector = [[0, -1, 0],
                         ["None"],
                         [1, 0, 0]
                         ]
        status = panda_to_item(env, "R_gripper", "R_gripperCenter", target, "dyna_coffe_mug", target_vector, True, fingers_opt)

    if status == True:
        print("moving to tray")
        target_vector = [[1, 0, 0],
                         ["None"],
                         [0, 1, 0]
                         ]
        target = env.get_position("tray")
        target += [0,0,0.15]
        status = panda_to_item(env, "R_gripper", "R_gripperCenter", target, "dyna_coffe_mug",target_vector, False)



    print("object gripping status :", status)
    print("press any key to exit")
    while True:
        key = input()
        if key:
            break

    # get_order_panda(env.S, env.V, env.C, env.RealWorld)
