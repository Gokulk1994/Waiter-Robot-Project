import sys

sys.path.append('../robotics-course/build')
import libry as ry
import time


class KomoOperations:
    def __init__(self, C, position_steps=1, tau=0.01):
        self.C = C

        [self.base_pos, J] = self.C.evalFeature(ry.FS.position, ["base_footprint"])
        [self.start_pr2L_pos, J] = self.C.evalFeature(ry.FS.position, ["pr2L"])
        [self.start_pr2R_pos, J] = self.C.evalFeature(ry.FS.position, ["pr2R"])

        self.position_steps = position_steps
        self.tau = tau
        self.vectors = [ry.FS.vectorX, ry.FS.vectorY, ry.FS.vectorZ]

    def update_pr2_params(self):
        [self.base_pos, J] = self.C.evalFeature(ry.FS.position, ["base_footprint"])
        [self.start_pr2L_pos, J] = self.C.evalFeature(ry.FS.position, ["pr2L"])
        [self.start_pr2R_pos, J] = self.C.evalFeature(ry.FS.position, ["pr2R"])

    def get_default_komo(self):

        komo = self.C.komo_path(1., self.position_steps, self.tau, True)
        komo.clearObjectives()
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

        return komo

    def immobilize_pr2(self, komo):
        self.update_pr2_params()
        komo.addObjective([], ry.FS.position, ['base_footprint'], ry.OT.eq, [1e2], target=self.base_pos)
        komo.addObjective([], ry.FS.position, ['pr2L'], ry.OT.eq, [1e2], target=self.start_pr2L_pos)
        komo.addObjective([], ry.FS.position, ['pr2R'], ry.OT.eq, [1e2], target=self.start_pr2R_pos)
        return komo

    def immobilize_fingers(self, komo, finger1, finger2):
        komo.addObjective([], ry.FS.qItself, [finger1], ry.OT.eq, [1e1], order=1)
        komo.addObjective([], ry.FS.qItself, [finger2], ry.OT.eq, [1e1], order=1)
        return komo

    def move_to_position(self, gripper, target_position, vector_target=None,
                         optimize=True, fingers_opt=None, no_time_step=True):
        print("Komo call: move to position")

        start_time = time.time()

        komo = self.get_default_komo()

        if no_time_step:
            komo.addObjective([], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=target_position)
        else:
            komo.addObjective([0, 0.25], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=target_position + [0.15, 0, 0])
            komo.addObjective([.26, .5], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=target_position + [0.1, 0, 0])
            komo.addObjective([.51, .75], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=target_position + [0.05, 0, 0])
            komo.addObjective([0.76, 1.], ry.FS.position, [gripper], ry.OT.eq, [1e3], target=target_position)

        for i, target_vec in enumerate(vector_target):
            if target_vec != ["None"]:
                komo.addObjective([], self.vectors[i], [gripper], ry.OT.eq, [1e2], target=target_vec)

        komo.addObjective(time=[1.], feature=ry.FS.qItself, type=ry.OT.eq, order=1)

        komo_final = self.immobilize_pr2(komo)

        if fingers_opt is not None:
            for finger in fingers_opt:
                komo_final.addObjective([], ry.FS.qItself, [finger], ry.OT.eq, [1e1], order=1)

        if optimize:
            komo_final.optimize()
        print("komo opt time ", (time.time() - start_time))
        return komo_final

    def komo_optimize(self, komo):
        komo.optimize()
        return komo
