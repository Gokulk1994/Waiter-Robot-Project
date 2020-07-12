import sys

sys.path.append('../robotics-course/build')
import libry as ry
import time


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
        #komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

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

    # def move_to_position(self, gripper, target_position, vector_target=None,
    #                      optimize=True, fingers_opt=None, no_time_step=True):
    #     print("Komo call: move to position")
    #
    #     start_time = time.time()
    #
    #     komo = self.get_default_komo()
    #
    #     if no_time_step:
    #         komo.addObjective([], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=target_position)
    #     else:
    #         komo.addObjective([0, 0.25], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=target_position + [0.15, 0, 0])
    #         komo.addObjective([.26, .5], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=target_position + [0.1, 0, 0])
    #         komo.addObjective([.51, .75], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=target_position + [0.05, 0, 0])
    #         komo.addObjective([0.76, 1.], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=target_position)
    #
    #     for i, target_vec in enumerate(vector_target):
    #         if target_vec != ["None"]:
    #             komo.addObjective([], self.vectors[i], [gripper], ry.OT.eq, [1e2], target=target_vec)
    #
    #     komo.addObjective(time=[1.], feature=ry.FS.qItself, type=ry.OT.eq, order=1)
    #
    #     if fingers_opt is not None:
    #         for finger in fingers_opt:
    #             komo.addObjective([], ry.FS.qItself, [finger], ry.OT.eq, [1e1], order=1)
    #     komo_final = self.immobilize_pr2(komo)
    #
    #     komo_final.optimize()
    #     print("komo opt time ", (time.time() - start_time))
    #     return komo_final

    # offsets contain 3 lists with offset for first 3 coordinates
    # offsets = [[0.6, 0, 0], [0.3, 0, 0], [0.2, 0, 0]]
    def move_to_position(self, gripper, target_position, offsets, vector_target=None,
                         fingers_opt=None, no_time_step=True):
        print("Komo call: move to position")
        start_time = time.time()
        komo = self.get_default_komo()

        if no_time_step:
            komo.addObjective([], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=target_position)
        else:
            komo.addObjective([0., .25], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=target_position + offsets[0])
            komo.addObjective([.25, .5], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=target_position + offsets[1])
            komo.addObjective([.5, .75], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=target_position + offsets[2])
            komo.addObjective([0.75, 1], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=target_position)

        for i, target_vec in enumerate(vector_target):
            if target_vec != ["None"]:
                komo.addObjective([], self.vectors[i], [gripper], ry.OT.eq, [1e2], target=target_vec)

        komo.addObjective(time=[1.], feature=ry.FS.qItself, type=ry.OT.eq, order=1)

        if fingers_opt is not None:
            for finger in fingers_opt:
                komo.addObjective([], ry.FS.qItself, [finger], ry.OT.eq, [1e1], order=1)
        komo_final = self.immobilize_pr2(komo)
        print("komo opt time ", (time.time() - start_time))
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

    def move_pr2_position(self, target_pos, vector_target, start_komo_tau=0.001):
        start_time = time.time()
        komo = self.get_default_komo(position_steps=1, tau=start_komo_tau)
        #komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
        komo.addObjective(time=[], feature=ry.FS.qItself, type=ry.OT.eq, order=1)
        komo.addObjective([], ry.FS.qItself, ["torso_lift_joint"], type=ry.OT.eq, order=1)
        komo.addObjective([], ry.FS.position, ['base_footprint'], ry.OT.eq, [1e2], target=target_pos)

        for i, target_vec in enumerate(vector_target):
            if target_vec != ["None"]:
                komo.addObjective([], self.vectors[i], ['base_footprint'], ry.OT.eq, [1e3], target=target_vec)
        #print("komo opt time ", (time.time() - start_time))
        return komo


    def check_move_to_pos_close(self, gripper, position, targetQuat, position_steps=8, useQuat=False, align=True):
        komo = self.C.komo_path(1., position_steps, self.tau, True)
        komo.clearObjectives()
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
        # komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);
        komo.addObjective([0., .25], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=position + [0.6, 0, 0])
        komo.addObjective([.25, .5], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=position + [0.3, 0, 0])
        komo.addObjective([.5, .75], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=position + [0.2, 0, 0])
        komo.addObjective([0.75, 1.], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=position)

        if useQuat:
            komo.addObjective([], ry.FS.quaternion, [gripper], ry.OT.eq, [1e1], target=targetQuat)

        if align:
            komo.addObjective([], ry.FS.vectorZ, [gripper], ry.OT.eq, [1e2], target=[1, 0, 0])
            komo.addObjective([], ry.FS.vectorX, [gripper], ry.OT.eq, [1e2], target=[0, -1, 0])

        komo.addObjective(time=[1.], feature=ry.FS.qItself, type=ry.OT.eq, order=1)

        komo.addObjective([], ry.FS.qItself, ['R_finger1'], ry.OT.eq, [1e1], order=1)
        komo.addObjective([], ry.FS.qItself, ['R_finger2'], ry.OT.eq, [1e1], order=1)

        komo.addObjective([], ry.FS.position, ['base_footprint'], ry.OT.eq, [1e2], target=self.base_pos)
        komo.addObjective([], ry.FS.position, ['pr2L'], ry.OT.eq, [1e2], target=self.start_pr2L_pos)
        komo.addObjective([], ry.FS.position, ['pr2R'], ry.OT.eq, [1e2], target=self.start_pr2R_pos)
        komo.addObjective([], ry.FS.position, ['endeffWorkspace'], ry.OT.eq, [1e2], target=self.start_endeffwork_pos)
        komo.optimize()
        return komo

    def check_move_to_pos_open(self,gripper, position, targetQuat, position_steps=8, align=True, useQuat=False):
        komo = self.C.komo_path(1., position_steps, self.tau, True)
        komo.clearObjectives()
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
        komo.addObjective([0., .25], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=position + [0, 0.4, 0.1])
        komo.addObjective([.25, .5], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=position + [0, 0.3, 0.05])
        komo.addObjective([.5, .75], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=position + [0, 0.2, 0.04])
        komo.addObjective([.75, 1], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=position)
        if useQuat:
            komo.addObjective([], ry.FS.quaternion, [gripper], ry.OT.eq, [1e1], target=targetQuat)

        if align:
            komo.addObjective([], ry.FS.vectorZ, [gripper], ry.OT.eq, [1e2], target=[0, 1, 0])
            komo.addObjective([], ry.FS.vectorX, [gripper], ry.OT.eq, [1e2], target=[1, 0, 0])

        komo.addObjective(time=[1.], feature=ry.FS.qItself, type=ry.OT.eq, order=1)
        komo.addObjective([], ry.FS.qItself, ['R_finger1'], ry.OT.eq, [1e1], order=1)
        komo.addObjective([], ry.FS.qItself, ['R_finger2'], ry.OT.eq, [1e1], order=1)
        komo.addObjective([], ry.FS.position, ['base_footprint'], ry.OT.eq, [1e2], target=self.base_pos)
        komo.addObjective([], ry.FS.position, ['pr2L'], ry.OT.eq, [1e2], target=self.start_pr2L_pos)
        komo.addObjective([], ry.FS.position, ['pr2R'], ry.OT.eq, [1e2], target=self.start_pr2R_pos)
        komo.addObjective([], ry.FS.qItself, ["torso_lift_joint"], type=ry.OT.eq, order=1)
        komo.optimize()
        return komo

    def move_back_position(self, gripper, position, targetQuat, position_steps, align=True):
        [open_finger_1, J] = self.C.evalFeature(ry.FS.qItself, ["R_finger1"])
        [open_finger_2, J] = self.C.evalFeature(ry.FS.qItself, ["R_finger2"])
        komo = self.C.komo_path(1., position_steps, self.tau, True)
        komo.clearObjectives()
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
        komo.addObjective(time=[.25, 1.], feature=ry.FS.qItself, type=ry.OT.eq, target=self.start_qItself)
        komo.addObjective(time=[1.], feature=ry.FS.qItself, type=ry.OT.eq, order=1)
        if align:
            komo.addObjective([0., 0.25], ry.FS.vectorZ, [gripper], ry.OT.eq, [1e2], target=[0, 1, 0])
            komo.addObjective([0., 0.25], ry.FS.vectorX, [gripper], ry.OT.eq, [1e2], target=[1, 0, 0])
            komo.addObjective([0., .25], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=position)
            komo.addObjective([0, .25], ry.FS.qItself, ['R_finger1'], ry.OT.eq, target=open_finger_1)
            komo.addObjective([0, .25], ry.FS.qItself, ['R_finger2'], ry.OT.eq, target=open_finger_2)
        komo.addObjective([], ry.FS.position, ['base_footprint'], ry.OT.eq, [1e2], target=self.base_pos)
        komo.addObjective([], ry.FS.position, ['pr2L'], ry.OT.eq, [1e2], target=self.start_pr2L_pos)
        komo.addObjective([], ry.FS.position, ['pr2R'], ry.OT.eq, [1e2], target=self.start_pr2R_pos)
        komo.addObjective([], ry.FS.position, ['endeffWorkspace'], ry.OT.eq, [1e2], target=self.start_endeffwork_pos)
        komo.addObjective([], ry.FS.qItself, ['torso_lift_joint'], ry.OT.eq, [1e1], order=1)
        komo.optimize()
        return komo
"""
    def move_back_position(self, gripper, position, targetQuat, position_steps, align=True):
        komo = self.C.komo_path(1., position_steps, self.tau, True)
        komo.clearObjectives()
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
        komo.addObjective(time=[.25, 1.], feature=ry.FS.qItself, type=ry.OT.eq, target=self.start_qItself)
        komo.addObjective(time=[1.], feature=ry.FS.qItself, type=ry.OT.eq, order=1)
        if align:
            komo.addObjective([0., 0.25], ry.FS.vectorZ, [gripper], ry.OT.eq, [1e2], target=[0, 1, 0])
            komo.addObjective([0., 0.25], ry.FS.vectorX, [gripper], ry.OT.eq, [1e2], target=[1, 0, 0])
            komo.addObjective([0., .25], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=self.glass_pos + [0, 0.3, 0])
            komo.addObjective([0, .25], ry.FS.qItself, ['R_finger1'], ry.OT.eq, target=self.open_finger_1)
            komo.addObjective([0, .25], ry.FS.qItself, ['R_finger2'], ry.OT.eq, target=self.open_finger_2)
        komo.addObjective([], ry.FS.position, ['base_footprint'], ry.OT.eq, [1e2], target=self.base_pos)
        komo.addObjective([], ry.FS.position, ['pr2L'], ry.OT.eq, [1e2], target=self.start_pr2L_pos)
        komo.addObjective([], ry.FS.position, ['pr2R'], ry.OT.eq, [1e2], target=self.start_pr2R_pos)
        komo.addObjective([], ry.FS.position, ['endeffWorkspace'], ry.OT.eq, [1e2], target=self.satrt_endeffwork_pos)
        komo.addObjective([], ry.FS.qItself, ['torso_lift_joint'], ry.OT.eq, [1e1], order=1)
        # komo.addObjective([], ry.FS.transVelocities, ['torso_lift_joint'], ry.OT.eq, [1e1], order=1)
        komo.optimize()
        return komo
"""