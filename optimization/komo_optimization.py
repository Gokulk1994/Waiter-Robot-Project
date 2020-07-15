import sys

sys.path.append('../robotics-course/build')
import libry as ry
import time
import numpy as np


class KomoOperations:
    def __init__(self, C, position_steps=1, tau=.005):
        self.C = C

        [self.base_pos, J] = self.C.evalFeature(ry.FS.position, ["base_footprint"])
        [self.start_endeffwork_pos, J] = C.evalFeature(ry.FS.position, ["endeffWorkspace"])
        [self.start_pr2L_pos, J] = self.C.evalFeature(ry.FS.position, ["pr2L"])
        [self.start_pr2R_pos, J] = self.C.evalFeature(ry.FS.position, ["pr2R"])
        [self.start_qItself, J]  = self.C.evalFeature(ry.FS.qItself, [])
        [self.start_endeffwork_pos, J] = self.C.evalFeature(ry.FS.position, ["endeffWorkspace"])
        [self.start_R_gripper_pos, J] = self.C.evalFeature(ry.FS.position, ["R_gripper"])
        [self.start_grip_quat, J] = self.C.evalFeature(ry.FS.quaternion, ["R_gripperCenter"])
        [self.start_qItself, J] = self.C.evalFeature(ry.FS.qItself, [])

        self.position_steps = position_steps
        self.tau = tau
        self.vectors = [ry.FS.vectorX, ry.FS.vectorY, ry.FS.vectorZ]

    def update_pr2_params(self):
        [self.base_pos, J] = self.C.evalFeature(ry.FS.position, ["base_footprint"])
        [self.start_pr2L_pos, J] = self.C.evalFeature(ry.FS.position, ["pr2L"])
        [self.start_pr2R_pos, J] = self.C.evalFeature(ry.FS.position, ["pr2R"])

    def get_default_komo(self, position_steps=8, tau=.005):
        komo = self.C.komo_path(1., position_steps, tau, True)
        komo.clearObjectives()
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
        return komo

    def immobilize_pr2(self, komo):
        self.update_pr2_params()
        komo.addObjective([], ry.FS.position, ['base_footprint'], ry.OT.eq, [1e2], target=self.base_pos)
        komo.addObjective([], ry.FS.position, ['pr2L'], ry.OT.eq, [1e2], target=self.start_pr2L_pos)
        komo.addObjective([], ry.FS.position, ['pr2R'], ry.OT.eq, [1e2], target=self.start_pr2R_pos)
        komo.addObjective([], ry.FS.position, ['endeffWorkspace'], ry.OT.eq, [1e2], target=self.start_endeffwork_pos)
        komo.addObjective([], ry.FS.qItself, ['torso_lift_joint'], ry.OT.eq, [1e1], order=1)
        return komo

    def immobilize_fingers(self, komo, finger1, finger2):
        komo.addObjective([], ry.FS.qItself, [finger1], ry.OT.eq, [1e1], order=1)
        komo.addObjective([], ry.FS.qItself, [finger2], ry.OT.eq, [1e1], order=1)
        return komo


    def move_to_position(self, gripper, target_position, offsets, vector_target=None,
                         fingers_opt=None, no_time_step=True):
        print("Komo call: move to position")

        if no_time_step:
            komo = self.get_default_komo(position_steps=1, tau=0.015)
            komo.addObjective([1.], ry.FS.position, [gripper], ry.OT.eq, [1e2], target=target_position)
        else:
            komo = self.get_default_komo()
            komo.addObjective([0., .25], ry.FS.position, [gripper], ry.OT.eq, [1e2], target=target_position + offsets[0])
            komo.addObjective([.25, .5], ry.FS.position, [gripper], ry.OT.eq, [1e2], target=target_position + offsets[1])
            komo.addObjective([.5, .75], ry.FS.position, [gripper], ry.OT.eq, [1e2], target=target_position + offsets[2])
            komo.addObjective([1.], ry.FS.position, [gripper], ry.OT.eq, [1e2], target=target_position)

        for i, target_vec in enumerate(vector_target):
            if target_vec != ["None"]:
                komo.addObjective([], self.vectors[i], [gripper], ry.OT.eq, [1e3], target=target_vec)

        komo.addObjective(time=[1.], feature=ry.FS.qItself, type=ry.OT.eq, order=1)

        if fingers_opt is not None:
            for finger in fingers_opt:
                komo.addObjective([], ry.FS.qItself, [finger], ry.OT.eq, [1e2], order=1)

        komo_final = self.immobilize_pr2(komo)
        return komo_final

    def gripper_move_back_position(self, gripper, position, position_steps, vector_target=None,
                                   fingers_opt=None):
        print("Komo call: move gripper initial position")
        start_time = time.time()

        komo = self.get_default_komo(position_steps)
        komo.addObjective(time=[.25, 1.], feature=ry.FS.qItself, type=ry.OT.eq, target=self.start_qItself)
        komo.addObjective(time=[1.], feature=ry.FS.qItself, type=ry.OT.eq, order=1)

        for i, target_vec in enumerate(vector_target):
            if target_vec != ["None"]:
                komo.addObjective([0., 0.25], self.vectors[i], [gripper], ry.OT.eq, [1e2], target=target_vec)

        komo.addObjective([0., .25], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=position)

        if fingers_opt is not None:
            for finger in fingers_opt:
                [open_finger_pos, J] = self.C.evalFeature(ry.FS.qItself, [finger])
                komo.addObjective([0, .25], ry.FS.qItself, [finger], ry.OT.eq, target=open_finger_pos)

        komo_final = self.immobilize_pr2(komo)
        print("komo opt time ", (time.time() - start_time))
        return komo_final

    def move_back_position_orig(self, start_qItself, position_steps):
        print("Komo call: move gripper original position")

        komo = self.get_default_komo(position_steps)
        komo.addObjective(time=[], feature=ry.FS.qItself, type=ry.OT.eq, target=start_qItself)
        komo.addObjective(time=[1.], feature=ry.FS.qItself, type=ry.OT.eq, order=1)
        komo_final = self.immobilize_pr2(komo)
        return komo_final


    def move_pr2_position(self, target_pos, vector_target, start_komo_tau=0.001):
        komo = self.get_default_komo(position_steps=1, tau=start_komo_tau)
        #komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
        komo.addObjective(time=[], feature=ry.FS.qItself, type=ry.OT.eq, order=1)
        komo.addObjective([], ry.FS.qItself, ["torso_lift_joint"], type=ry.OT.eq, order=1)
        komo.addObjective([], ry.FS.position, ['base_footprint'], ry.OT.eq, [1e2], target=target_pos)

        for i, target_vec in enumerate(vector_target):
            if target_vec != ["None"]:
                komo.addObjective([], self.vectors[i], ['base_footprint'], ry.OT.eq, [1e3], target=target_vec)
        return komo